/*
 * examples/modules/SingleAgentModule.cc
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

#include "SingleAgentModule.h"

#include <opencog/server/Factory.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()
{
    return SingleAgentModule::info().id.c_str();
}

extern "C" Module* opencog_module_load(CogServer& cogserver)
{
    cogserver.registerAgent(SingleAgentModule::info().id, &(SingleAgentModule::factory()));
    SingleAgentModulePtr agent = cogserver.createAgent<SingleAgentModule>(true);
    agent->name = "OriginalSingleAgentModule";
    return agent.get();
}

extern "C" void opencog_module_unload(Module* m)
{
    SingleAgentModule* agent = static_cast<SingleAgentModule*>(m);
    agent->stopAgent();
}

SingleAgentModule::SingleAgentModule(CogServer& cs) : Agent(cs, 100), Module(cs)
{
    logger().info("[SingleAgentModule] constructor (%s)", name.c_str());
}

SingleAgentModule::~SingleAgentModule()
{
    logger().info("[SingleAgentModule] destructor (%s)", name.c_str());
    Module::_cogserver.destroyAllAgents(SingleAgentModule::info().id);
}

void SingleAgentModule::stopAgent()
{
    AgentPtr age = std::dynamic_pointer_cast<Agent>(shared_from_this());
    Module::_cogserver.stopAgent(age);
}

void SingleAgentModule::run()
{
    logger().info("[SingleAgentModule] run (%s)", name.c_str());
}

void SingleAgentModule::init()
{
    logger().info("[TestModule] init (%s)", name.c_str());

    // Use static global vars -- the egent is destroyed when these go 
    // out of scope (during the C++ finalizer).
    static SingleAgentModulePtr a = Module::_cogserver.createAgent<SingleAgentModule>(true);
    a->name = "SingleAgentModule1";

    static SingleAgentModulePtr b = Module::_cogserver.createAgent<SingleAgentModule>(true);
    b->name = "SingleAgentModule2";

    static SingleAgentModulePtr c = Module::_cogserver.createAgent<SingleAgentModule>(true);
    c->name = "SingleAgentModule3";
}
