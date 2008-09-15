/*
 * opencog/server/SimpleNetworkServer.cc
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

#include "SimpleNetworkServer.h"

#include <Sockets/SocketHandler.h>
#include <Sockets/ListenSocket.h>

#include <opencog/util/platform.h>
#include <opencog/server/CommandRequest.h>
#include <opencog/server/ServerSocket.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

using namespace opencog;

bool SimpleNetworkServer::stopListenerThreadFlag = false;

SimpleNetworkServer::~SimpleNetworkServer()
{
    if (! stopListenerThreadFlag) {
        stopListenerThreadFlag = true;
        pthread_join(socketListenerThread, NULL);
    }
}

SimpleNetworkServer::SimpleNetworkServer(CogServer *cs, int pn)
{
    started = false;
    stopListenerThreadFlag = false;
    portNumber = pn;
    cogServer = cs;
    shell_mode = false;

    // XXX Fixme, this is the wrong prompt to display -- 
    // when the shell server is in a mode other than the 
    // normal opencog mode. The prompt needs to be taken 
    // from the command processor, rather than here.
    prompt = "opencog> ";
}

/**
 * Convert a command line, received over a socket from some client,
 * into a request for the CogServer.
 */
void SimpleNetworkServer::processCommandLine(CallBackInterface *callBack,
                                             const std::string &cmdLine)
{
    std::string command;
    std::queue<std::string> args;

    // Special handling for command escapes
    // Note that "data" will have LF or CRLF because the socket code
    // has already been put into "raw" mode when "data" shows up.
    // However, other things, like "scm", will not have CRLF because
    // the socket code will be in line mode.
    if (cmdLine.substr(0, 5) == "data\n") {
        command = "data";
        args.push(cmdLine.substr(5));
    } else if (cmdLine.substr(0, 6) == "data\r\n") {
        command = "data";
        args.push(cmdLine.substr(6));
    } else if (cmdLine.substr(0, 3) == "scm") {
        command = "scm";
        shell_mode = true;
    } else if (cmdLine.substr(0, 1) == "") {
        command = "scm-exit";
        shell_mode = false;
    } else if (cmdLine.substr(0, 1) == ".") {
        command = "scm-exit";
        shell_mode = false;
    } else if (shell_mode) {
        // In shell mode, do *not* parse the command line!
        command = cmdLine;
    } else {
        // XXX FIXME: this is really the wrong place to do command-line
        // parsing. This needs to be moved to the simple cog server,
        // so that the silly "shell_mode" crap above can be removed.
        parseCommandLine(cmdLine, command, args);
    }

    CommandRequest *request = new CommandRequest(callBack, command, args);
    cogServer->pushRequest(request);
}

std::string SimpleNetworkServer::getCommandPrompt()
{
    return prompt;
}

void SimpleNetworkServer::start()
{
    if (started) {
        throw new RuntimeException(NULL, "Cannot restart SimpleNetworkServer");
    }

    ServerSocket::setMaster(this);

    pthread_attr_init(&socketListenerAttr);
    pthread_attr_setscope(&socketListenerAttr, PTHREAD_SCOPE_PROCESS);
    pthread_attr_setinheritsched(&socketListenerAttr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setdetachstate(&socketListenerAttr, PTHREAD_CREATE_DETACHED);

    pthread_create(&socketListenerThread,
                   &socketListenerAttr,
                   SimpleNetworkServer::portListener,
                   &portNumber);

    started = true;
}


void *SimpleNetworkServer::portListener(void *arg)
{
    int port = *((int*) arg);

    SocketHandler socketHandler;
    ListenSocket<ServerSocket> listenSocket(socketHandler);

    try {
        // we throw an exception ourselves because csockets may
        // be compiled with exceptions disabled
        if (listenSocket.Bind(port)) throw new Exception("bind error");
    } catch (Exception) {
        logger().error("Unable to bind to port %d. Aborting.", port);
        std::exit(1);
    }

    socketHandler.Add(&listenSocket);

    socketHandler.Select(0, 200);

    while (!stopListenerThreadFlag) {
        if (socketHandler.GetCount() == 0) {
            throw new RuntimeException(NULL, "NetworkElement - Bind to port %d is broken.", port);
        }
        // poll for 200 millsecs
        socketHandler.Select(0, 200 * 1000);
    }

    return NULL;
}

/**
 * parseCommandLine -- split string into space-separated tokens
 * @line -- input string
 * @command -- output, contains first non-whitespace part of input string
 * @args -- output, queue of space-separated tokens split from the input string.
 *
 * XXX ?? what is the purpose of this?? gnu getopt is an better/more general
 * way to get args from a command line.
 *
 * Anyway, command line processing should be moved to the 
 * CommandRequestProcessor since it is not correct to always process all
 * input streams (some input streams are not commands, they're data. 
 * XXX Fix this!
 */
void SimpleNetworkServer::parseCommandLine(const std::string &_line,
        std::string &command,
        std::queue<std::string> &args)
{
    std::string::size_type pos1, pos2;
    std::string line;

    // Trim off leading whitespace.
    line = _line;
    int nw = line.find_first_not_of(" \t\v\f");
    if (0 < nw) line = line.substr(nw);

    pos1 = line.find(' ', 0);
    if (pos1 == line.npos) {
        command.assign(line);
        return;
    }
    command.assign(line.substr(0, pos1));

    while (pos1 != line.npos) {
        pos2 = line.find(' ', pos1 + 1);
        if (pos2 == line.npos) {
            args.push(line.substr(pos1 + 1));
        } else {
            args.push(line.substr(pos1 + 1, pos2 - pos1 - 1));
        }
        pos1 = pos2;
    }
}
