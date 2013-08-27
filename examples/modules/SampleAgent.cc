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
DECLARE_MODULE(SampleModule)

SampleModule::SampleModule(CogServer& cs) : Module(cs)
{
    logger().info("[SampleModule] constructor");
}

SampleModule::~SampleModule()
{
    logger().info("[SampleModule] destructor");
    _cogserver.destroyAllAgents(SampleAgent::info().id);
}

void SampleModule::init()
{
    logger().info("[SampleModule] init");
    _cogserver.registerAgent(SampleAgent::info().id, &factory);
    _cogserver.createAgent(SampleAgent::info().id, true);
}

SampleAgent::SampleAgent(CogServer& cs) : Agent(cs, 100)
{
    logger().info("[SampleAgent] constructor");
}

SampleAgent::~SampleAgent()
{
    logger().info("[SampleAgent] destructor");
}

void SampleAgent::run()
{
    logger().info("[SampleAgent] run");
}
