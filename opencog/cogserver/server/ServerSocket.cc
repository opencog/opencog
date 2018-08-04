/*
 * opencog/cogserver/server/ServerSocket.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2010 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
 *
 * Written by Welter Luigi <welter@vettalabs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <mutex>

#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <opencog/cogserver/server/ServerSocket.h>

using namespace opencog;

ServerSocket::ServerSocket(void) :
    _socket(nullptr)
{
}

ServerSocket::~ServerSocket()
{
    logger().debug("ServerSocket::~ServerSocket()");

    SetCloseAndDelete();
    delete _socket;
    _socket = nullptr;
}

void ServerSocket::Send(const std::string& cmd)
{
    OC_ASSERT(_socket, "Use of socket after it's been closed!\n");

    boost::system::error_code error;
    boost::asio::write(*_socket, boost::asio::buffer(cmd),
                       boost::asio::transfer_all(), error);

    // The most likely cause of an error is that the remote side has
    // closed the socket, even though we still had stuff to send.
    // I beleive this is a ENOTCON errno, maybe others as well.
    // (for example, ECONNRESET `Connection reset by peer`)
    // Don't log these harmless errors.
    if (error.value() != boost::system::errc::success and
        error.value() != boost::asio::error::not_connected and
        error.value() != boost::asio::error::broken_pipe and
        error.value() != boost::asio::error::bad_descriptor and
        error.value() != boost::asio::error::connection_reset)
        logger().warn("ServerSocket::Send(): %s on thread 0x%x\n"
                      "Attempted to send: %s",
             error.message().c_str(), pthread_self(), cmd.c_str());
}

// As far as I can tell, boost::asio is not actually thread-safe,
// in particular, when closing and destroying sockets.  This strikes
// me as incredibly stupid -- a first-class reason to not use boost.
// But whatever.  Hack around this for now.
static std::mutex _asio_crash;

// This is called in a different thread than the thread that is running
// the handle_connection() method. It's purpose in life is to terminate
// the connection -- it does so by closing the socket. Sometime later,
// the handle_connection() method notices that it's closed, and exits
// it's loop, thus ending the thread that its running in.
void ServerSocket::SetCloseAndDelete()
{
    std::lock_guard<std::mutex> lock(_asio_crash);
    logger().debug("ServerSocket::SetCloseAndDelete()");
    try
    {
        _socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        _socket->close();
    }
    catch (const boost::system::system_error& e)
    {
        if (e.code() != boost::asio::error::not_connected and
            e.code() != boost::asio::error::bad_descriptor)
        {
            logger().error("ServerSocket::handle_connection(): Error closing socket: %s", e.what());
        }
    }
}

typedef boost::asio::buffers_iterator<
    boost::asio::streambuf::const_buffers_type> bitter;

// Some random RFC 854 characters
#define IAC 0xff  // Telnet Interpret As Command
#define IP 0xf4   // Telnet IP Interrupt Process
#define AO 0xf5   // Telnet AO Abort Output
#define EL 0xf8   // Telnet EL Erase Line
#define WILL 0xfb // Telnet WILL
#define DO 0xfd   // Telnet DO
#define TIMING_MARK 0x6 // Telnet RFC 860 timing mark
#define TRANSMIT_BINARY 0x0 // Telnet RFC 856 8-bit-clean
#define CHARSET 0x2A // Telnet RFC 2066


// Goal: if the user types in a ctrl-C or a ctrl-D, we want to
// react immediately to this. A ctrl-D is just the ascii char 0x4
// while the ctrl-C is wrapped in a telnet "interpret as command"
// IAC byte secquence.  Basically, we want to forward all IAC
// sequences immediately, as well as the ctrl-D.
//
// Currently not implemented, but could be: support for the arrow
// keys, which generate the sequence 0x1b 0x5c A B C or D.
//
std::pair<bitter, bool>
match_eol_or_escape(bitter begin, bitter end)
{
    bool telnet_mode = false;
    bitter i = begin;
    while (i != end)
    {
        unsigned char c = *i++;
        if (IAC == c) telnet_mode = true;
        if (('\n' == c) ||
            (0x04 == c) || // ASCII EOT End of Transmission (ctrl-D)
            (telnet_mode && (c <= 0xf0)))
        {
            return std::make_pair(i, true);
        }
    }
    return std::make_pair(i, false);
}

void ServerSocket::set_connection(boost::asio::ip::tcp::socket* sock)
{
    if (_socket) delete _socket;
    _socket = sock;
}

void ServerSocket::handle_connection(void)
{
    logger().debug("ServerSocket::handle_connection()");
    OnConnection();
    boost::asio::streambuf b;
    while (true)
    {
        try
        {
            boost::asio::read_until(*_socket, b, match_eol_or_escape);
            std::istream is(&b);
            std::string line;
            std::getline(is, line);
            if (not line.empty() and line[line.length()-1] == '\r') {
                line.erase(line.end()-1);
            }
            OnLine(line);
        }
        catch (const boost::system::system_error& e)
        {
            if (e.code() == boost::asio::error::eof) {
                break;
            } else if (e.code() == boost::asio::error::connection_reset) {
                break;
            } else if (e.code() == boost::asio::error::not_connected) {
                break;
            } else {
                logger().error("ServerSocket::handle_connection(): Error reading data. Message: %s", e.what());
            }
        }
    }

    // If the data sent to us is not new-line terminated, then
    // there may still be some bytes sitting in the buffer. Get
    // them and forward them on.  These are typically scheme
    // strings issued from netcat, that simply did not have
    // newlines at the end.
    std::istream is(&b);
    std::string line;
    std::getline(is, line);
    if (not line.empty() and line[line.length()-1] == '\r') {
        line.erase(line.end()-1);
    }
    if (not line.empty())
        OnLine(line);

    logger().debug("ServerSocket::exiting handle_connection()");

    // In the standard scenario, ConsoleSocket inherits from this, and
    // so deleting this will cause the ConsoleSocket dtor to run. This
    // will, in turn, try to delete the shell, which will typically
    // stall until the current evaluation is done. If the current
    // evaluation is an infinite loop, then it will hang forever, and
    // gdb will show a stack trace stuck in GenericShell::while_not_done()
    // This is perfectly normal, and nothing can be done about it; we
    // can't kill it without hurting users who launch long-running but
    // finite commands via netcat. Nor can we magically unwind all the
    // C++ state and stacks, to leave only some very naked evaluator
    // running. The hang here, in the dtor, while_not_done(), really
    // must be thought of as the normal sync point for completion.
    delete this;
}
