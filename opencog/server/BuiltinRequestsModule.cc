/*
 * opencog/server/BuiltinRequestsModule.cc
 *
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#include "BuiltinRequestsModule.h"

#include <opencog/server/CogServer.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()                   { return BuiltinRequestsModule::id(); }
extern "C" Module*     opencog_module_load()                 { return new BuiltinRequestsModule(); }
extern "C" void        opencog_module_unload(Module* module) { delete module; }

BuiltinRequestsModule::BuiltinRequestsModule()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerRequest(ListRequest::info().id,         &listFactory); 
    cogserver.registerRequest(LoadRequest::info().id,         &loadFactory);
    cogserver.registerRequest(SaveRequest::info().id,         &saveFactory);
    cogserver.registerRequest(DataRequest::info().id,         &dataFactory); 
    cogserver.registerRequest(HelpRequest::info().id,         &helpFactory); 
    cogserver.registerRequest(ExitRequest::info().id,         &exitFactory); 
    cogserver.registerRequest(ShutdownRequest::info().id,     &shutdownFactory); 
    cogserver.registerRequest(LoadModuleRequest::info().id,   &loadmoduleFactory);
    cogserver.registerRequest(UnloadModuleRequest::info().id, &unloadmoduleFactory);
    do_startAgents_register();
    do_stopAgents_register();
}

BuiltinRequestsModule::~BuiltinRequestsModule()
{
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.unregisterRequest(ListRequest::info().id);
    cogserver.unregisterRequest(LoadRequest::info().id);
    cogserver.unregisterRequest(SaveRequest::info().id);
    cogserver.unregisterRequest(DataRequest::info().id);
    cogserver.unregisterRequest(HelpRequest::info().id);
    cogserver.unregisterRequest(ExitRequest::info().id);
    cogserver.unregisterRequest(ShutdownRequest::info().id);
    cogserver.unregisterRequest(LoadModuleRequest::info().id);
    cogserver.unregisterRequest(UnloadModuleRequest::info().id);
    do_startAgents_unregister();
    do_stopAgents_unregister();
}

void BuiltinRequestsModule::init()
{
}

std::string BuiltinRequestsModule::do_startAgents(Request *dummy, std::list<std::string> args)
{   
    std::list<const char*> availableAgents = cogserver().agentIds();
 
    std::vector<std::string> agents;

    for (std::list<std::string>::const_iterator it = args.begin();
         it != args.end(); ++it) {
        std::string agent_type = *it;
        // check that this is a valid type; give an error and return otherwise
        if (availableAgents.end() ==
         find(availableAgents.begin(), availableAgents.end(), *it)) {
            std::ostringstream oss;
            oss << "Invalid Agent ID (" << *it << ")";
            return oss.str();
        }
        
        agents.push_back(agent_type);
     }

    for (std::vector<std::string>::const_iterator it = agents.begin();
         it != agents.end(); ++it) {
        cogserver().createAgent(*it, true);
    }
    
    return "Successfully started agents";
}

std::string BuiltinRequestsModule::do_stopAgents(Request *dummy, std::list<std::string> args)
{
    std::list<const char*> availableAgents = cogserver().agentIds();
 
    std::vector<std::string> agents;

    for (std::list<std::string>::const_iterator it = args.begin();
         it != args.end(); ++it) {
        std::string agent_type = *it;
        // check that this is a valid type; give an error and return otherwise
        if (availableAgents.end() ==
         find(availableAgents.begin(), availableAgents.end(), *it)) {
            std::ostringstream oss;
            oss << "Invalid Agent ID (" << *it << ")";
            return oss.str();
        }
        
        agents.push_back(agent_type);
     }

    // Not safe to run while the CogServer is running
    for (std::vector<std::string>::const_iterator it = agents.begin();
         it != agents.end(); ++it) {
        cogserver().destroyAllAgents(*it);
    }
    
    return "Successfully stopped agents";
}

