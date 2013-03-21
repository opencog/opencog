/*
 * opencog/server/BuiltinRequestsModule.cc
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
    cogserver.registerRequest(SleepRequest::info().id,        &sleepFactory); 
    cogserver.registerRequest(ShutdownRequest::info().id,     &shutdownFactory); 
    cogserver.registerRequest(LoadModuleRequest::info().id,   &loadmoduleFactory);
    cogserver.registerRequest(UnloadModuleRequest::info().id, &unloadmoduleFactory);
    registerAgentRequests();
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
    cogserver.unregisterRequest(SleepRequest::info().id);
    cogserver.unregisterRequest(ShutdownRequest::info().id);
    cogserver.unregisterRequest(LoadModuleRequest::info().id);
    cogserver.unregisterRequest(UnloadModuleRequest::info().id);
    unregisterAgentRequests();
}

void BuiltinRequestsModule::registerAgentRequests()
{
    do_startAgents_register();
    do_stopAgents_register();
    do_stepAgents_register();
    do_startAgentLoop_register();
    do_stopAgentLoop_register();
    do_listAgents_register();
    do_activeAgents_register();
}

void BuiltinRequestsModule::unregisterAgentRequests()
{
    do_startAgents_unregister();
    do_stopAgents_unregister();
    do_stepAgents_unregister();
    do_startAgentLoop_unregister();
    do_stopAgentLoop_unregister();
    do_listAgents_unregister();
    do_activeAgents_unregister();
}

void BuiltinRequestsModule::init()
{
}

std::string BuiltinRequestsModule::do_startAgents(Request *dummy, std::list<std::string> args)
{   
    std::list<const char*> availableAgents = cogserver().agentIds();
 
    std::vector<std::string> agents;

    if (args.size() == 0)
        return "Error: No agents to start specified";

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

    if (args.size() == 0)
        return "Error: No agents to stop specified";
            
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
    // doesn't give an error if there is no instance of that agent type running

    for (std::vector<std::string>::const_iterator it = agents.begin();
         it != agents.end(); ++it) {
        // Check if this Agent instance is also a module
        if (cogserver().getModule(*it) != NULL) {
            // delete the Agent instance if it is not
            cogserver().destroyAllAgents(*it);
        }  
    }
    
    return "Successfully stopped agents";
}

std::string BuiltinRequestsModule::do_stepAgents(Request *dummy, std::list<std::string> args)
{    
    std::vector<Agent*> agents = cogserver().runningAgents();
    
    if (args.size() == 0) {
        for (std::vector<Agent*>::const_iterator it = agents.begin();
             it != agents.end(); ++it) {
            (*it)->run(&cogserver());
        }
        return "Ran a step of each active agent";
    } else {
        std::list<std::string> unknownAgents;
        int numberAgentsRun = 0;
        for (std::list<std::string>::const_iterator it = args.begin();
             it != args.end(); ++it) {
            
            std::string agent_type = *it;

            // try to find an already started agent with that name
            std::vector<Agent*>::const_iterator tmp = agents.begin();
            for ( ; tmp!=agents.end() ; tmp++ ) if ( *it == (*tmp)->classinfo().id ) break;
            
            Agent* agent;
            if (agents.end() == tmp) {
                // construct a temporary agent
                agent = cogserver().createAgent(*it, false);
                if (agent) {
                    agent->run(&cogserver());
                    cogserver().destroyAgent(agent);
                    numberAgentsRun++;
                } else {
                    unknownAgents.push_back(*it);
                }
            } else {
                agent = *tmp;
                agent->run(&cogserver());
            }
        }
        std::stringstream returnMsg;
        for (std::list<std::string>::iterator it = unknownAgents.begin();
                it != unknownAgents.end(); ++it) {
            returnMsg << "Unknown agent " << *it << std::endl;
        }
        returnMsg << "Successfully ran a step of " << numberAgentsRun <<
            "/" << args.size() << " agents." << std::endl;
        return returnMsg.str();
    }
}

std::string BuiltinRequestsModule::do_stopAgentLoop(Request *dummy, std::list<std::string> args)
{
    cogserver().stopAgentLoop();
    
    return "Stopped agent loop";
}

std::string BuiltinRequestsModule::do_startAgentLoop(Request *dummy, std::list<std::string> args)
{
    cogserver().startAgentLoop();
    
    return "Started agent loop";
}

std::string BuiltinRequestsModule::do_listAgents(Request *dummy, std::list<std::string> args)
{
    std::list<const char*> agentNames = cogserver().agentIds();
    std::ostringstream oss;
    
    for (std::list<const char*>::const_iterator it = agentNames.begin();
         it != agentNames.end(); ++it) {
        oss << (*it) << std::endl;
    }
    
    return oss.str();
}

std::string BuiltinRequestsModule::do_activeAgents(Request *dummy, std::list<std::string> args)
{
    std::vector<Agent*> agents = cogserver().runningAgents();
    std::ostringstream oss;
    
    for (std::vector<Agent*>::const_iterator it = agents.begin();
         it != agents.end(); ++it) {
        oss << (*it)->to_string() << std::endl;
    }
    
    return oss.str();
}
