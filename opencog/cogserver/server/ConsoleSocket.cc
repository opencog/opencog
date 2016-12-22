/*
 * opencog/cogserver/server/ConsoleSocket.cc
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

#include <opencog/util/Config.h>
#include <opencog/util/Logger.h>
#include <opencog/util/misc.h>

#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/ConsoleSocket.h>
#include <opencog/cogserver/server/Request.h>

using namespace opencog;

ConsoleSocket::ConsoleSocket(void)
{
    _use_count = 0;
    _shell = nullptr;
}

ConsoleSocket::~ConsoleSocket()
{
    logger().debug("[ConsoleSocket] destructor");

    // We need the use-count and the condition variables because
    // somehow the design of either this subsystem, or boost:asio
    // is broken. Basically, the boost::asio code calls this destructor
    // for ConsoleSocket while there are still requests outstanding
    // in another thread.  We have to stall the destructor until all
    // the in-flight requests are complete; we use the condition
    // variable to do this. But really, something somewhere is broken
    // or mis-designed. Not sure what/where; this code is too complicated.
    //
    // Some details: basically, the remote end of the socket "fires and
    // forgets" a bunch of commands, and then closes the socket before
    // these requests have completed.  boost:asio notices that the
    // remote socket has closed, and so decides its a good day to call
    // destructors. But of course, its not ...
    std::unique_lock<std::mutex> lck(_mtx);
    while (_use_count) _cv.wait(lck);

    // If there's a shell, kill it.
    if (_shell) delete _shell;

    logger().debug("[ConsoleSocket] destructor finished");
}

// Some random RFC 854 characters
#define IAC 0xff  // Telnet Interpret As Command
#define IP 0xf4   // Telnet IP Interrupt Process
#define AO 0xf5   // Telnet AO Abort Output
#define EL 0xf8   // Telnet EL Erase Line
#define WILL 0xfb // Telnet WILL
#define SB 0xfa   // Telnet SB subnegotiation start
#define SE 0xf0   // Telnet SE subnegotiation end
#define DO 0xfd   // Telnet DO
#define DONT 0xfe   // Telnet DONT
#define CHARSET 0x2a // Telnet RFC 2066 charset

void ConsoleSocket::OnConnection()
{
    logger().debug("[ConsoleSocket] OnConnection");

#ifdef NOT_RIGHT_NOW
    // Crude attempt to negotiate for a utf-8 clean channel.
    // Using RFC 2066 protocols.  Not robust.  We're just praying
    // for non-garbled UTF-8 goodness, here.

    // Anyway, this won't work for netcat, socat, because they'll
    // just pass all this crap back to the user, and we don't want
    // that.  I'm not sure how to tell if we're talking to a true
    // RFC telnet.
    char utf_plz[20];
    utf_plz[0] = IAC;
    utf_plz[1] = WILL;
    utf_plz[2] = CHARSET;
    utf_plz[3] = 0;
    Send(utf_plz);
    utf_plz[1] = DO;
    Send(utf_plz);

    utf_plz[0] = IAC;
    utf_plz[1] = SB;
    utf_plz[2] = CHARSET;
    utf_plz[3] = '\02';
    utf_plz[4] = 'U';
    utf_plz[5] = 'T';
    utf_plz[6] = 'F';
    utf_plz[7] = '-';
    utf_plz[8] = '8';
    utf_plz[9] = IAC;
    utf_plz[10] = SE;
    utf_plz[11] = 0;
    Send(utf_plz);
#endif

    sendPrompt();
}

void ConsoleSocket::sendPrompt()
{
    // Prompt with ANSI color codes, if possible.
    if (config().get_bool("ANSI_ENABLED", true))
        Send(config().get("ANSI_PROMPT", "[0;32mopencog[1;32m> [0m"));
    else
        Send(config().get("PROMPT", "opencog> "));
}

void ConsoleSocket::OnLine(const std::string& line)
{
    // If a shell processor has been designated, then defer all
    // processing to the shell.  In particular, avoid as much overhead
    // as possible, since the shell needs to be able to handle a
    // high-speed data feed with as little getting in the way as
    // possible.
    if (_shell) {
        _shell->eval(line);
        return;
    }

    // Hmm. Looks like most telnet agents respond with an
    // IAC WONT CHARSET IAC DONT CHARSET
    // Any case, just ignore CHARSET RFC 2066 negotiation
    if (IAC == (line[0] & 0xff) and CHARSET == (line[2] & 0xff)) {
        return;
    }

    // If the command starts with an open-paren, or a semi-colon, assume
    // its a scheme command. Pop into the scheme shell, and try again.
    if (line[0] == '(' or line[0] == ';')
    {
        OnLine("scm");

        // Re-issue the command, but only if we sucessfully got a shell.
        // (We might not get a shell if scheme is not installed.)
        if (_shell) {
            OnLine(line);
            return;
        }
    }

    logger().debug("[ConsoleSocket] OnLine [%s]", line.c_str());

    // Parse command line
    std::list<std::string> params;
    tokenize(line, std::back_inserter(params), " \t\v\f");
    logger().debug("params.size(): %d", params.size());
    if (params.empty()) {
        // return on empty/blank line
        sendPrompt();
        return;
    }

    std::string cmdName = params.front();
    params.pop_front();

    CogServer& cogserver = static_cast<CogServer&>(server());
    Request* request = cogserver.createRequest(cmdName);

    // Command not found.
    if (nullptr == request)
    {
        char msg[256];
        snprintf(msg, 256, "command \"%s\" not found\n", cmdName.c_str());
        logger().warn(msg);
        Send(msg);

        // Try to send "help" command response
        request = cogserver.createRequest("help");
        if (nullptr == request)
        {
            // no help request; just terminate the request
            sendPrompt();
            return;
        }
    }

    request->set_console(this);
    request->setParameters(params);

    // Add the command to the processing queue.
    cogserver.pushRequest(request);

    if (request->isShell())
    {
        logger().debug("ConsoleSocket::OnLine() request %s is a shell",
                       line.c_str());

        // Force a drain of the request queue, because we *must* enter
        // shell mode before handling any additional input from the
        // socket (since all subsequent input will be for the new shell,
        // not for the cogserver command processor).
        cogserver.processRequests();
    }
}

void ConsoleSocket::OnRequestComplete()
{
    logger().debug("[ConsoleSocket] OnRequestComplete");

    // Shells will send their own prompt
    if (nullptr == _shell) sendPrompt();
}

void ConsoleSocket::Exit()
{
    logger().debug("[ConsoleSocket] ExecuteExitRequest");
    SetCloseAndDelete();
}

void ConsoleSocket::SendResult(const std::string& res)
{
    Send(res);
}

void ConsoleSocket::SetShell(GenericShell *g)
{
    _shell = g;
}
