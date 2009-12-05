/*
 * opencog/server/ConsoleSocket.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Andre Senna <senna@vettalabs.com>
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

#include <string>

#include <opencog/server/CogServer.h>
#include <opencog/server/ConsoleSocket.h>
#include <opencog/server/Request.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>

using namespace opencog;

ConsoleSocket::ConsoleSocket(boost::asio::io_service& _io_service) : 
    ServerSocket(_io_service),
    IHasMimeType("text/plain")
{
    SetLineProtocol(true);
    _shell = NULL;
}

ConsoleSocket::~ConsoleSocket()
{
    logger().debug("[ConsoleSocket] destructor");

    // If there's a shell, let them know that we are going away.
    // This wouldn't be needed if we had garbage collection.
    if (_shell) _shell->socketClosed();
}

void ConsoleSocket::OnConnection()
{
    logger().debug("[ConsoleSocket] OnConnection");
    if (!isClosed()) Send(config()["PROMPT"]);
}

tcp::socket& ConsoleSocket::getSocket()
{
    return ServerSocket::getSocket();
}

void ConsoleSocket::OnLine(const std::string& line)
{
    // If a shell processor has been designated, then defer all
    // processing to the shell.  In particular, avoid as much overhead
    // as possible, since the shell needs to be able to handle a
    // high-speed data feed with as little getting in the way as
    // possible.
    if (_shell) {
        _shell->eval(line, this);
        return;
    }

    logger().debug("[ConsoleSocket] OnLine [%s]", line.c_str());

    // parse command line
    std::list<std::string> params;
    tokenize(line, std::back_inserter(params), " \t\v\f");
    logger().debug("params.size(): %d", params.size());
    if (params.empty()) {
        // return on empty/blank line
        OnRequestComplete();
        return;
    }
    std::string cmdName = params.front();
    params.pop_front();

    CogServer& cogserver = static_cast<CogServer&>(server());
    _request = cogserver.createRequest(cmdName);
    if (_request == NULL) {
        char msg[256];
        snprintf(msg, 256, "command \"%s\" not found\n", cmdName.c_str());
        logger().error(msg);
        Send(msg);

        // try to send "help" command response
        _request = cogserver.createRequest("help");
        if (_request == NULL) {
            // no help request; just terminate the request
            OnRequestComplete();
            return;
        }
    }

    _request->setSocket(this);
    _request->setParameters(params);

    if (LineProtocol()) {
        // We only add the command to the processing queue
        // if it hasn't disabled the line protocol
        cogserver.pushRequest(_request);

        // Force a drain of all outstanding requests. This is because
        // any one of the requests could be a request to enter shell mode,
        // in which case, we *must* enter shell mode before handling any
        // additional input from the socket (since that input is almost
        // surely intended for the shell, and not for the cogserver).
#if 0 
       // Welter's comment: Calling this method here is causing concurrency problems 
       // since this runs in a separate thread (for socket handler) instead of main 
       // thread (where server loop runs).
       // TODO: Find another way to avoid next line to be handled by cogserver
       // shell when the previous command is for entering scheme (or any other)
       // shell. Perhaps we should create Disable/Enable methods for the
       // ConsoleSocket so it is disabled here and re-enabled
       // when the request is executed. 
        cogserver.processRequests();
#endif
        if (_request->isShell()) cogserver.processRequests();
    } else {
        // reset input buffer
        _buffer.clear();
    }
}

/**
 * Buffer up incoming raw data until an end-of-transmission (EOT)
 * is received. After the EOT, dispatch the buffered data as a single
 * request.
 *
 * Note that there is just one ConsoleSocket per client, so the buffer
 * is not shared.
 *
 * XXX the data buffer is currently just a string buffer, so any
 * null char in the data stream will terminate the string. So
 * this method isn't really for "raw" data containing null chars.
 * This should be fixed someday, as needed.
 */
void ConsoleSocket::OnRawData(const char *buf, size_t len)
{
    char* _tmp = strndup(buf, len); 
    logger().debug("[ConsoleSocket] OnRawData local buffer [%s]", _tmp);
    free(_tmp);

    _buffer.append(buf, len);
    logger().debug("[ConsoleSocket] OnRawData: global buffer:\n%s\n", _buffer.c_str());
    size_t buffer_len = _buffer.length();
    bool rawDataEnd = false;
    if (buffer_len > 1 && (_buffer.at(buffer_len-1) == 0xa)) {
        if (_buffer.at(buffer_len-2) == 0x4) {
            rawDataEnd = true;
        } else if (buffer_len > 2 && 
                   (_buffer.at(buffer_len-2) == 0xd) && 
                   (_buffer.at(buffer_len-3) == 0x4)) {
            rawDataEnd = true;
        } 
    }
    if (rawDataEnd) {
        logger().debug("[ConsoleSocket] found EOT; dispatching");
        // found the EOT pattern. dispatch the request and reset
        // the socket's line protocol flag
        _request->addParameter(_buffer);
        CogServer& cogserver = static_cast<CogServer&>(server());
        cogserver.pushRequest(_request);
        SetLineProtocol(true);
    }
}

void ConsoleSocket::OnRequestComplete()
{
    logger().debug("[ConsoleSocket] OnRequestComplete");
    Send(config()["PROMPT"]);
}

void ConsoleSocket::SetShell(GenericShell *g)
{
    _shell = g;
}
