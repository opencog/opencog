/*
 * opencog/cogserver/server/BuiltinRequestsModule.cc
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

#include <iomanip>
#include <unistd.h>

#include <opencog/util/ansi.h>
#include <opencog/util/oc_assert.h>
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/ConsoleSocket.h>

#include "BuiltinRequestsModule.h"

using namespace opencog;

DECLARE_MODULE(BuiltinRequestsModule)

BuiltinRequestsModule::BuiltinRequestsModule(CogServer& cs) : Module(cs)
{
    _cogserver.registerRequest(ListRequest::info().id,         &listFactory);
    _cogserver.registerRequest(ShutdownRequest::info().id,     &shutdownFactory);
    _cogserver.registerRequest(LoadModuleRequest::info().id,   &loadmoduleFactory);
    _cogserver.registerRequest(UnloadModuleRequest::info().id, &unloadmoduleFactory);
    _cogserver.registerRequest(ListModulesRequest::info().id,  &listmodulesFactory);
    registerAgentRequests();
}

BuiltinRequestsModule::~BuiltinRequestsModule()
{
    unregisterAgentRequests();
    _cogserver.unregisterRequest(ListRequest::info().id);
    _cogserver.unregisterRequest(ShutdownRequest::info().id);
    _cogserver.unregisterRequest(LoadModuleRequest::info().id);
    _cogserver.unregisterRequest(UnloadModuleRequest::info().id);
    _cogserver.unregisterRequest(ListModulesRequest::info().id);
}

void BuiltinRequestsModule::registerAgentRequests()
{
    do_help_register();
    do_h_register();

    do_exit_register();
    do_quit_register();
    do_q_register();
    do_ctrld_register();
    do_dot_register();

    do_stats_register();

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
    do_help_unregister();
    do_h_unregister();

    do_exit_unregister();
    do_quit_unregister();
    do_q_unregister();
    do_ctrld_unregister();
    do_dot_unregister();

    do_stats_unregister();

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

// ====================================================================
// Various flavors of closing the connection
std::string BuiltinRequestsModule::do_exit(Request* req, std::list<std::string> args)
{
    ConsoleSocket* con = req->get_console();
    OC_ASSERT(con, "Bad request state");

    con->Exit();

    // After the exit, the pointer to the console will be invalid,
    // so zero it out now, to avoid a bad dereference.  (Note that
    // this call may trigger the ConsoleSocket dtor to run).
    req->set_console(nullptr);
    return "";
}

std::string BuiltinRequestsModule::do_quit(Request *req, std::list<std::string> args)
{
    return do_exit(req, args);
}

std::string BuiltinRequestsModule::do_q(Request *req, std::list<std::string> args)
{
    return do_exit(req, args);
}

std::string BuiltinRequestsModule::do_ctrld(Request *req, std::list<std::string> args)
{
    return do_exit(req, args);
}

std::string BuiltinRequestsModule::do_dot(Request *req, std::list<std::string> args)
{
    return do_exit(req, args);
}

// ====================================================================
// Various flavors of help
std::string BuiltinRequestsModule::do_help(Request *req, std::list<std::string> args)
{
    std::ostringstream oss;

    if (args.empty()) {
        std::list<const char*> commands = _cogserver.requestIds();

        size_t maxl = 0;
        std::list<const char*>::const_iterator it;
        for (it = commands.begin(); it != commands.end(); ++it) {
            size_t len = strlen(*it);
            if (len > maxl) maxl = len;
        }

        oss << "Available commands:" << std::endl;
        for (it = commands.begin(); it != commands.end(); ++it) {
            // Skip hidden commands
            if (_cogserver.requestInfo(*it).hidden) continue;
            std::string cmdname(*it);
            std::string ansi_cmdname;
            ansi_green(ansi_cmdname); ansi_bright(ansi_cmdname);
            ansi_cmdname.append(cmdname);
            ansi_off(ansi_cmdname);
            ansi_green(ansi_cmdname);
            ansi_cmdname.append(":");
            ansi_off(ansi_cmdname);
            size_t cmd_length = strlen(cmdname.c_str());
            size_t ansi_code_length = strlen(ansi_cmdname.c_str()) - cmd_length;
            oss << "  " << std::setw(maxl+ansi_code_length+2) << std::left << ansi_cmdname
                << _cogserver.requestInfo(*it).description << std::endl;
        }
    } else if (args.size() == 1) {
        const RequestClassInfo& cci = _cogserver.requestInfo(args.front());
        if (cci.help != "")
            oss << cci.help << std::endl;
    } else {
        oss << do_helpRequest::info().help << std::endl;
    }

    return oss.str();
}

std::string BuiltinRequestsModule::do_h(Request *req, std::list<std::string> args)
{
    return do_help(req, args);
}

// ====================================================================
// Print general info about server.
std::string BuiltinRequestsModule::do_stats(Request *req, std::list<std::string> args)
{
    std::ostringstream oss;
    ConsoleSocket* con = req->get_console();

    oss << "Console use-count = " << con->get_use_count() << "\n";
    oss << "Console max-open-sockets = " << con->get_max_open_sockets() << "\n";
    oss << "Console curr-open-sockets = " << con->get_num_open_sockets() << "\n";

    // count open file descs
    int nfd = 0;
    for (int j=0; j<4096; j++) {
       int fd = dup(j);
       if (fd < 0) continue;
       close(fd);
       nfd++;
    }
    oss << "Process num-open-fds = " << nfd << "\n";

    return oss.str();
}

// ====================================================================
// Various agents commands
std::string BuiltinRequestsModule::do_startAgents(Request *dummy, std::list<std::string> args)
{
    std::list<const char*> availableAgents = _cogserver.agentIds();

    std::vector<std::tuple<std::string, bool, std::string>> agents;

    if (args.empty())
        return "Error: No agents to start specified\n";

    for (std::list<std::string>::const_iterator it = args.begin();
         it != args.end(); ++it) {
        auto p1 = it->find(',');
        std::string agent_type = it->substr(0, p1);
        // check that this is a valid type; give an error and return otherwise
        if (availableAgents.end() ==
         find(availableAgents.begin(), availableAgents.end(), agent_type)) {
            std::ostringstream oss;
            oss << "Invalid Agent ID \"" << agent_type << "\"\n";
            return oss.str();
        }

        bool threaded = false;
        std::string thread_name;
        if (p1 != std::string::npos) {
            auto p2 = it->find(',', p1 + 1);
            auto dedicated_str = it->substr(p1 + 1, p2 - p1 - 1);
            threaded = (dedicated_str == "yes");
            if (!threaded && dedicated_str != "no")
                return "Invalid dedicated parameter: " + dedicated_str + '\n';

            if (p2 != std::string::npos)
                thread_name = it->substr(p2 + 1);
        }

        agents.push_back(std::make_tuple(agent_type, threaded, thread_name));
     }

    for (auto it = agents.cbegin();
         it != agents.cend(); ++it) {
        auto agent = _cogserver.createAgent(std::get<0>(*it));
        _cogserver.startAgent(agent, std::get<1>(*it), std::get<2>(*it));
    }

    return "Successfully started agents\n";
}

std::string BuiltinRequestsModule::do_stopAgents(Request *dummy, std::list<std::string> args)
{
    std::list<const char*> availableAgents = _cogserver.agentIds();

    std::vector<std::string> agents;

    if (args.empty())
        return "Error: No agents to stop specified\n";

    for (std::list<std::string>::const_iterator it = args.begin();
         it != args.end(); ++it) {
        std::string agent_type = *it;
        // check that this is a valid type; give an error and return otherwise
        if (availableAgents.end() ==
         find(availableAgents.begin(), availableAgents.end(), *it)) {
            std::ostringstream oss;
            oss << "Invalid Agent ID \"" << *it << "\"\n";
            return oss.str();
        }

        agents.push_back(agent_type);
    }

    // Doesn't give an error if there is no instance of that agent type
    // running.  TODO FIXME.  Should check.
    for (std::vector<std::string>::const_iterator it = agents.begin();
         it != agents.end(); ++it)
    {
        _cogserver.stopAllAgents(*it);
    }

    return "Successfully stopped agents\n";
}

std::string BuiltinRequestsModule::do_stepAgents(Request *dummy, std::list<std::string> args)
{
    AgentSeq agents = _cogserver.runningAgents();

    if (args.empty()) {
        for (AgentSeq::const_iterator it = agents.begin();
             it != agents.end(); ++it) {
            (*it)->run();
        }
        return "Ran a step of each active agent\n";
    } else {
        std::list<std::string> unknownAgents;
        int numberAgentsRun = 0;
        for (std::list<std::string>::const_iterator it = args.begin();
             it != args.end(); ++it) {

            std::string agent_type = *it;

            // try to find an already started agent with that name
            AgentSeq::const_iterator tmp = agents.begin();
            for ( ; tmp != agents.end() ; ++tmp ) if ( *it == (*tmp)->classinfo().id ) break;

            AgentPtr agent;
            if (agents.end() == tmp) {
                // construct a temporary agent
                agent = AgentPtr(_cogserver.createAgent(*it, false));
                if (agent) {
                    agent->run();
                    _cogserver.stopAgent(agent);
                    numberAgentsRun++;
                } else {
                    unknownAgents.push_back(*it);
                }
            } else {
                agent = *tmp;
                agent->run();
            }
        }
        std::stringstream returnMsg;
        for (std::list<std::string>::iterator it = unknownAgents.begin();
                it != unknownAgents.end(); ++it) {
            returnMsg << "Unknown agent \"" << *it << "\"" << std::endl;
        }
        returnMsg << "Successfully ran a step of " << numberAgentsRun <<
            "/" << args.size() << " agents." << std::endl;
        return returnMsg.str();
    }
}

std::string BuiltinRequestsModule::do_stopAgentLoop(Request *dummy, std::list<std::string> args)
{
    _cogserver.stopAgentLoop();

    return "Stopped agent loop\n";
}

std::string BuiltinRequestsModule::do_startAgentLoop(Request *dummy, std::list<std::string> args)
{
    _cogserver.startAgentLoop();

    return "Started agent loop\n";
}

std::string BuiltinRequestsModule::do_listAgents(Request *dummy, std::list<std::string> args)
{
    std::list<const char*> agentNames = _cogserver.agentIds();
    std::ostringstream oss;

    for (std::list<const char*>::const_iterator it = agentNames.begin();
         it != agentNames.end(); ++it) {
        oss << (*it) << std::endl;
    }

    return oss.str();
}

std::string BuiltinRequestsModule::do_activeAgents(Request *dummy, std::list<std::string> args)
{
    AgentSeq agents = _cogserver.runningAgents();
    std::ostringstream oss;

    for (AgentSeq::const_iterator it = agents.begin();
         it != agents.end(); ++it) {
        oss << (*it)->to_string() << std::endl;
    }

    return oss.str();
}
