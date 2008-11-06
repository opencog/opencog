/*
 * opencog/server/ServerSocket.cc
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
#include <sstream>

#include <opencog/server/CogServer.h>
#include <opencog/server/ConsoleSocket.h>
#include <opencog/server/Request.h>
#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>

using namespace opencog;

ConsoleSocket::ConsoleSocket(ISocketHandler &handler)
    : TcpSocket(handler), IHasMimeType("text/plain")
{
    SetLineProtocol(true);
    _shell = NULL;
}

ConsoleSocket::~ConsoleSocket()
{
}

void ConsoleSocket::OnAccept()
{   
    logger().debug("[ConsoleSocket] OnAccept");
    if (Detach() == false) {
        logger().error("Unable to detach socket.");
        return;
    }
}

void ConsoleSocket::OnDetached()
{
    logger().debug("[ConsoleSocket] OnDetached");
    Send(config()["PROMPT"]);
    SetNonblocking(true);
}

void ConsoleSocket::OnLine(const std::string& line)
{
    // If a shell processor has been designated, then defer all
    // processing to the shell.  In particular, avoid as much overhead
    // as possible, since the shell needs to be able to handle a
    // high-speed data feeed with as little getting in the way as
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
        // we only add the command to the processing queue
        // if it hasn't disabled the line protocol
        cogserver.pushRequest(_request);
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

    _buffer.write(buf, len);
    logger().debug("[ConsoleSocket] OnRawData: global buffer:\n%s\n", _buffer.str().c_str());
    char lastchars[3];
    _buffer.seekg((long) _buffer.tellp() - 3L);
    _buffer.read(lastchars, 3);
    logger().debug("[ConsoleSocket] last 3 chars: %x, %x, %x", lastchars[0], lastchars[1], lastchars[2]);
    if (_buffer.good()) {
        if ((lastchars[0] == 0x4 && lastchars[1] == 0xd && lastchars[2] == 0xa) ||
            (lastchars[1] == 0x4 && lastchars[2] == 0xa)) {

            logger().debug("[ConsoleSocket] found EOT; dispatching");
            // found the EOT pattern. dispatch the request and reset
            // the socket's line protocol flag
            _request->addParameter(_buffer.str());
            CogServer& cogserver = static_cast<CogServer&>(server());
            cogserver.pushRequest(_request);
            SetLineProtocol(true);
        }
    } else {
        logger().error("unable to retrieve last 3 chars (buffer contents=[%s])", _buffer.str().c_str());
        return;
    }
}

void ConsoleSocket::OnRequestComplete()
{
    Send(config()["PROMPT"]);
}

void ConsoleSocket::SetShell(GenericShell *g)
{
    _shell = g;
}
