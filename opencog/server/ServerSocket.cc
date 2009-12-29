/*
 * opencog/server/ServerSocket.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
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

#include <opencog/server/ServerSocket.h>
#include <opencog/util/Logger.h>
#include <boost/bind.hpp>


using namespace opencog; 

ServerSocket::ServerSocket(boost::asio::io_service& _io_service)
    : io_service(_io_service), socket(io_service), 
      lineProtocol(true), closed(false)
{
}

ServerSocket::~ServerSocket() {
   logger().debug("ServerSocket::~ServerSocket()");
}

tcp::socket& ServerSocket::getSocket()
{
    return socket;
}

void ServerSocket::Send(const std::string& cmd)
{
    boost::system::error_code error;
    boost::asio::write(socket, boost::asio::buffer(cmd), boost::asio::transfer_all(), error);
    if (error && !closed) {
        logger().error("ServerSocket::Send(): Error transfering data.");
    }
}

void ServerSocket::SetCloseAndDelete()
{
    logger().debug("ServerSocket::SetCloseAndDelete()");
    closed = true;
    socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    socket.close();
}

void ServerSocket::SetLineProtocol(bool val)
{
    lineProtocol = val;
}

bool ServerSocket::LineProtocol()
{
    return lineProtocol;
}

void ServerSocket::handle_connection(ServerSocket* ss)
{
    logger().debug("ServerSocket::handle_connection()");
    ss->OnConnection();
    boost::asio::streambuf b;
    for (;;) 
    {
        try {
            if (ss->LineProtocol()) 
            {
                //logger().debug("%p: ServerSocket::handle_connection(): Called read_until", ss);
                boost::asio::read_until(ss->getSocket(), b, boost::regex("\n"));
                //logger().debug("%p: ServerSocket::handle_connection(): returned from read_until", ss);
                std::istream is(&b);
                std::string line;
                std::getline(is, line); 
                if (!line.empty() && line[line.length()-1] == '\r') {
                    line.erase(line.end()-1);
                }
                //logger().debug("%p: ServerSocket::handle_connection(): Got new line: %s", ss, line.c_str());
                ss->OnLine(line);
            }
            else {
                boost::array<char, 128> buf;
                boost::system::error_code error;
                size_t len = ss->getSocket().read_some(boost::asio::buffer(buf), error);
                if (error == boost::asio::error::eof)
                    break; // Connection closed cleanly by peer.
                else if (error)
                    throw boost::system::system_error(error); // Some other error.

                ss->OnRawData(buf.data(), len);
            }
        } catch (boost::system::system_error& e) {
            if (ss->isClosed()) {
                break;
            } else if (e.code() == boost::asio::error::eof) {
                break;
            } else {
                logger().error("ServerSocket::handle_connection(): Error reading data. Message: %s", e.what());
            }
        }
    }
    delete ss;
}

void ServerSocket::start()
{
    logger().debug("ServerSocket::start()");
    connectionThread = boost::thread(boost::bind(&handle_connection, this));
}

bool ServerSocket::isClosed()
{
    return closed;
}

