/*
 * opencog/learning/PatternMiner/DistributedPatternMinerServer.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
 * Sep 2015
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

#include "DistributedPatternMinerServer.h"
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/util/Logger.h>

using namespace opencog;

// load/unload functions for the Module interface
DECLARE_MODULE(DistributedPatternMinerServerModule)

DistributedPatternMinerServerModule::DistributedPatternMinerServerModule(CogServer& cs) : Module(cs)
{
    logger().info("[DistributedPatternMinerServerModule] constructor");
}

DistributedPatternMinerServerModule::~DistributedPatternMinerServerModule()
{
    logger().info("[DistributedPatternMinerServerModule] destructor");
    _cogserver.stopAllAgents(DistributedPatternMinerServer::info().id);
}

void DistributedPatternMinerServerModule::init()
{
    logger().info("[DistributedPatternMinerServerModule] init");
    _cogserver.registerAgent(DistributedPatternMinerServer::info().id, &factory);
    _cogserver.createAgent(DistributedPatternMinerServer::info().id, true);
}

DistributedPatternMinerServer::DistributedPatternMinerServer(CogServer& cs) : Agent(cs, 100)
{
//    if ( load_scm_file( cs.getAtomSpace(), "pm_test_corpus.scm" ) == 0  )
//        logger().info( "OAC::%s - Loaded pattern miner test corpus file: '%s'",
//                        __FUNCTION__,
//                       "pm_test_corpus.scm"
//                     );
//    else
//        logger().error( "OAC::%s - Failed to load pattern miner test corpus file: '%s'",
//                         __FUNCTION__,
//                        "pm_test_corpus.scm"
//                      );

    this->patternMiner = new DistributedPatternMiner(&(cs.getAtomSpace()));

    logger().info("[DistributedPatternMinerServer] constructor");

}

DistributedPatternMinerServer::~DistributedPatternMinerServer()
{
    logger().info("[DistributedPatternMinerServer] destructor");
}


void DistributedPatternMinerServer::run()
{
    // test, only run once
    static bool hasRun = false;
    if (hasRun)
        return;

    this->patternMiner->launchCentralServer();

    hasRun = true;

}
