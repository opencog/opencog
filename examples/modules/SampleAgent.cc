/*
 * examples/modules/SampleAgent.cc
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

#include "SampleAgent.h"

#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/util/Logger.h>

using namespace opencog;

// load/unload functions for the Module interface
extern "C" const char* opencog_module_id()                  { return SampleModule::info().id.c_str(); }
extern "C" Module*     opencog_module_load()                { return new SampleModule(); }
extern "C" void        opencog_module_unload(Module* m)     { delete m; }

SampleModule::SampleModule()
{
    logger().info("[SampleModule] constructor");
}

SampleModule::~SampleModule()
{
    logger().info("[SampleModule] destructor");
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.destroyAllAgents(SampleAgent::info().id);
}

void SampleModule::init()
{
    logger().info("[SampleModule] init");
    CogServer& cogserver = static_cast<CogServer&>(server());
    cogserver.registerAgent(SampleAgent::info().id, &factory);
    cogserver.createAgent(SampleAgent::info().id, true);
}

SampleAgent::SampleAgent() : Agent(100)
{
    logger().info("[SampleAgent] constructor");
}

SampleAgent::~SampleAgent() {
    logger().info("[SampleAgent] destructor");
}

void SampleAgent::run(CogServer* server)
{
    logger().info("[SampleAgent] run");
}
