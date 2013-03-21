/*
 * opencog/server/BuiltinRequestsModule.h
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

#include <opencog/server/DataRequest.h>
#include <opencog/server/ExitRequest.h>
#include <opencog/server/SleepRequest.h>
#include <opencog/server/Factory.h>
#include <opencog/server/HelpRequest.h>
#include <opencog/server/ListRequest.h>
#include <opencog/server/LoadModuleRequest.h>
#include <opencog/server/LoadRequest.h>
#include <opencog/server/SaveRequest.h>
#include <opencog/server/Module.h>
#include <opencog/server/ShutdownRequest.h>
#include <opencog/server/UnloadModuleRequest.h>

#include <opencog/util/Logger.h>
#include <opencog/server/CogServer.h>

namespace opencog
{

class BuiltinRequestsModule : public Module
{

private:

    Factory<ListRequest, Request>         listFactory;
    Factory<DataRequest, Request>         dataFactory;
    Factory<HelpRequest, Request>         helpFactory;
    Factory<ExitRequest, Request>         exitFactory;
    Factory<SleepRequest, Request>        sleepFactory;
    Factory<ShutdownRequest, Request>     shutdownFactory;
    Factory<LoadModuleRequest, Request>   loadmoduleFactory;
    Factory<UnloadModuleRequest, Request> unloadmoduleFactory;

// I'm adding the agent control commands via the macro syntax
// (it's much more convenient than adding several new .cc/.h files). -- Jared Wigmore
DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-start", do_startAgents, 
       "Start some agents", 
       "Usage: agents-start <agent type> [...]\n\n"
       "Create new agent instances of the specified agent type(s), and start them.\n",
       false)

// Note: this command currently allows running multiple instances of the same Agent type
DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-stop", do_stopAgents, 
       "Stop some agents running",
       "Usage: agents-stop <agent type> [...]\n\n"
       "Stops the agents of the specified classes (class IDs).\n",
       false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-step", do_stepAgents, 
       "Run a single cycle of an agent(s)",
       "Usage: agents-step <agent type> [...]\n"
       "With one or more agent types as arguments, runs a single cycle of those agents."
       "With no arguments, runs one cycle of each agent that has been started."
       "Only to be run when the agent loop is stopped. Uses the already-started instances"
       "if available.\n", 
       false)       

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-stop-loop", do_stopAgentLoop, 
       "Stop the agent loop",
       "Usage: agents-stop-loop\n\n"
       "Stop the agent loop (that is, stop running agents during the CogServer loop).\n", 
       false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-start-loop", do_startAgentLoop, 
       "Start the agent loop",
       "Usage: agents-start-loop\n\n"
       "Start the agent loop (that is, start running agents during the CogServer loop).\n",
       false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-list", do_listAgents, 
       "List available agents",
       "Usage: agents-list\n\n"
       "List all the available agents from loaded modules.\n", 
       false)

DECLARE_CMD_REQUEST(BuiltinRequestsModule, "agents-active", do_activeAgents, 
       "List running agents",
       "Usage: agents-active\n\n"
       "List all the currently running agents, including their configuration parameters.\n", 
       false)

    void registerAgentRequests();
    void unregisterAgentRequests();
public:

    static inline const char* id() {
        static const char* _id = "opencog::BuiltinRequestsModule";
        return _id;
    }

    BuiltinRequestsModule();
    virtual ~BuiltinRequestsModule();
    virtual void init();

}; // class

}  // namespace

#endif // _OPENCOG_BUILTIN_REQUESTS_MODULE_H
