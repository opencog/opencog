/*
 * opencog/learning/PatternMiner/TestPatternMinerAgent.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
 * April 17 2014
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

#include "TestPatternMinerAgent.h"
#include <opencog/server/CogServer.h>
#include <opencog/server/Factory.h>
#include <opencog/util/Logger.h>
#include <opencog/guile/load-file.h>

using namespace opencog;

// load/unload functions for the Module interface
DECLARE_MODULE(TestPatternMinerModule)

TestPatternMinerModule::TestPatternMinerModule(CogServer& cs) : Module(cs)
{
    logger().info("[TestPatternMinerModule] constructor");
}

TestPatternMinerModule::~TestPatternMinerModule()
{
    logger().info("[TestPatternMinerModule] destructor");
    _cogserver.destroyAllAgents(TestPatternMinerAgent::info().id);
}

void TestPatternMinerModule::init()
{   
    logger().info("[TestPatternMinerModule] init");
    _cogserver.registerAgent(TestPatternMinerAgent::info().id, &factory);
    _cogserver.createAgent(TestPatternMinerAgent::info().id, true);
}

TestPatternMinerAgent::TestPatternMinerAgent(CogServer& cs) : Agent(cs, 100)
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

    this->patternMiner = new PatternMiner(&(cs.getAtomSpace()), 3);

    logger().info("[TestPatternMinerAgent] constructor");

}

TestPatternMinerAgent::~TestPatternMinerAgent()
{
    logger().info("[TestPatternMinerAgent] destructor");
}

void TestPatternMinerAgent::selectSubSetFromCorpus()
{
    vector<string> topics;
    topics.push_back("Neurology");
    topics.push_back("Geology");
    topics.push_back("Physics");
    topics.push_back("Politics");
    topics.push_back("Music");
    topics.push_back("Biology");
    topics.push_back("Western_philosophy");
    topics.push_back("Chemistry");
    this->patternMiner->selectSubsetFromCorpus(topics, 2);
}

void TestPatternMinerAgent::run()
{
    // test, only run once
    static bool hasRun = false;
    if (hasRun)
        return;
    // selectSubSetFromCorpus();
    this->patternMiner->runPatternMiner(1);

    hasRun = true;

}
