/*
 * opencog/cogserver/server/BuiltinRequestsModule.h
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Gustavo Gama <gama@vettalabs.com>
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

#ifndef _OPENCOG_BUILTIN_REQUESTS_MODULE_H
#define _OPENCOG_BUILTIN_REQUESTS_MODULE_H

#include <opencog/cogserver/server/SleepRequest.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/cogserver/server/ListRequest.h>
#include <opencog/cogserver/server/LoadModuleRequest.h>
#include <opencog/cogserver/server/Module.h>
#include <opencog/cogserver/server/ShutdownRequest.h>
#include <opencog/cogserver/server/UnloadModuleRequest.h>
#include <opencog/cogserver/server/ListModulesRequest.h>

#include <opencog/util/Logger.h>
#include <opencog/cogserver/server/CogServer.h>

namespace opencog
{
/** \addtogroup grp_server
 *  @{
 */

class BuiltinRequestsModule : public Module
{

private:

    Factory<ListRequest, Request>         listFactory;
    Factory<SleepRequest, Request>        sleepFactory;
    Factory<ShutdownRequest, Request>     shutdownFactory;
    Factory<LoadModuleRequest, Request>   loadmoduleFactory;
    Factory<UnloadModuleRequest, Request> unloadmoduleFactory;
    Factory<ListModulesRequest, Request>  listmodulesFactory;

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "exit", do_exit,
       "Close the shell connection",
       "Usage: exit\n\n"
       "Close the shell TCP/IP connection.\n",
       false, true)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "quit", do_quit,
       "Close the shell connection",
       "Usage: quit\n\n"
       "Close the shell TCP/IP connection.\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "q", do_q,
       "Close the shell connection",
       "Usage: q\n\n"
       "Close the shell TCP/IP connection.\n",
       false, true)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "", do_ctrld,
       "Close the shell connection",
       "Usage: ^D\n\n"
       "Close the shell TCP/IP connection.\n",
       false, true)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, ".", do_dot,
       "Close the shell connection",
       "Usage: .\n\n"
       "Close the shell TCP/IP connection.\n",
       false, true)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "help", do_help,
       "List the available commands or print the help for a specific command",
       "Usage: help [<command>]\n\n"
       "If no command is specified, then print a menu of commands.\n"
       "Otherwise, print verbose help for the indicated command.\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "h", do_h,
       "List the available commands or print the help for a specific command",
       "Usage: h [<command>]\n\n"
       "If no command is specified, then print a menu of commands.\n"
       "Otherwise, print verbose help for the indicated command.\n",
       false, true)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "stats", do_stats,
       "Print some diagnostic statistics about the server.",
       "Usage: stats\n\n",
       false, false)

// I'm adding the agent control commands via the macro syntax
// (it's much more convenient than adding several new .cc/.h files). -- Jared Wigmore
DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-start", do_startAgents,
       "Start some agents",
       "Usage: agents-start {<agent type>[,<dedicated>[,<thread_name>]]} [...]\n\n"
       "Create new agent instances of the specified agent type(s), and start them.\n"
       "If <dedicated> is 'yes', the agent will run in a separate thread with\n"
       "optional name <thread_name>. If <thread_name> already exists, the agent\n"
       "will be added to that thread instead of a new thread.\n",
       false, false)

// Note: this command currently allows running multiple instances of the same Agent type
DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-stop", do_stopAgents,
       "Stop some agents running",
       "Usage: agents-stop <agent type> [...]\n\n"
       "Stops the agents of the specified classes (class IDs).\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-step", do_stepAgents,
       "Run a single cycle of an agent(s)",
       "Usage: agents-step <agent type> [...]\n"
       "With one or more agent types as arguments, runs a single cycle of those agents."
       "With no arguments, runs one cycle of each agent that has been started."
       "Only to be run when the agent loop is stopped. Uses the already-started instances"
       "if available.\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-stop-loop", do_stopAgentLoop,
       "Stop the agent loop",
       "Usage: agents-stop-loop\n\n"
       "Stop the agent loop (that is, stop running agents during the CogServer loop).\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-start-loop", do_startAgentLoop,
       "Start the agent loop",
       "Usage: agents-start-loop\n\n"
       "Start the agent loop (that is, start running agents during the CogServer loop).\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-list", do_listAgents,
       "List available agents",
       "Usage: agents-list\n\n"
       "List all the available agents from loaded modules.\n",
       false, false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-active", do_activeAgents,
       "List running agents",
       "Usage: agents-active\n\n"
       "List all the currently running agents, including their configuration parameters.\n",
       false, false)

    void registerAgentRequests();
    void unregisterAgentRequests();

public:
    static const char* id();
    BuiltinRequestsModule(CogServer&);
    virtual ~BuiltinRequestsModule();
    virtual void init();

}; // class

/** @}*/
}  // namespace

#endif // _OPENCOG_BUILTIN_REQUESTS_MODULE_H
