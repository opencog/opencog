/*
 * opencog/learning/PatternMiner/PatternMinerAgent.cc
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

#include "PatternMinerAgent.h"
#include <opencog/cogserver/server/CogServer.h>
#include <opencog/cogserver/server/Factory.h>
#include <opencog/util/Logger.h>

using namespace opencog;

// load/unload functions for the Module interface
DECLARE_MODULE(PatternMinerModule)

PatternMinerModule::PatternMinerModule(CogServer& cs) : Module(cs)
{
    logger().info("[PatternMinerModule] constructor");
}

PatternMinerModule::~PatternMinerModule()
{
    logger().info("[PatternMinerModule] destructor");
    _cogserver.stopAllAgents(PatternMinerAgent::info().id);
}

void PatternMinerModule::init()
{
    logger().info("[PatternMinerModule] init");
    _cogserver.registerAgent(PatternMinerAgent::info().id, &factory);
    _cogserver.createAgent(PatternMinerAgent::info().id, true);
}

PatternMinerAgent::PatternMinerAgent(CogServer& cs) : Agent(cs, 100)
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

    this->patternMiner = new PatternMiner(&(cs.getAtomSpace()));

    logger().info("[PatternMinerAgent] constructor");

}

PatternMinerAgent::~PatternMinerAgent()
{
    logger().info("[PatternMinerAgent] destructor");
}

void PatternMinerAgent::selectSubSetFromCorpus()
{
    vector<string> topics;
   // topics.push_back("Neurology");
   // topics.push_back("Geology");
    topics.push_back("Physics");
    topics.push_back("Politics");
   // topics.push_back("Music");
   // topics.push_back("Biology");
    topics.push_back("Western_philosophy");
   // topics.push_back("Chemistry");
    this->patternMiner->selectSubsetFromCorpus(topics, 2);
}

void PatternMinerAgent::run()
{
    // test, only run once
    static bool hasRun = false;
    if (hasRun)
        return;
    // selectSubSetFromCorpus();
    this->patternMiner->runPatternMiner(1);

    hasRun = true;

}
