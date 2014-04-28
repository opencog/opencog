/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PatternMiningAgent.cc
 *
 * @author Shujing Ke <rainkekekeke@gmail.com>
 * @date   2014-04-11
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
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/util/Config.h>
#include <opencog/guile/load-file.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include "OAC.h"
#include "PatternMiningAgent.h"

using namespace opencog::oac;

PatternMiningAgent::~PatternMiningAgent()
{
}

PatternMiningAgent::PatternMiningAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void PatternMiningAgent::init()
{
    sleep(15);
    logger().debug( "PatternMiningAgent::%s - Initialize the Agent [cycle = %d]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");


    // create corpus AtomSpace
    corpusAtomSpace = new AtomSpace();

    // pre-load scm files:
//    SCM_PRELOAD           = scm/core_types.scm,
//                            scm/spacetime_types.scm,
//                            scm/nlp_types.scm,
//                            scm/attention_types.scm,
//                            scm/embodiment_types.scm,
//                            scm/persistence.scm,
//                            scm/utilities.scm,
//                            scm/file-utils.scm,
//                            scm/debug.scm,

    load_scm_file( *(this->corpusAtomSpace), "./scm/core_types.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/spacetime_types.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/nlp_types.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/attention_types.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/embodiment_types.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/persistence.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/utilities.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/file-utils.scm" );
    load_scm_file( *(this->corpusAtomSpace), "./scm/debug.scm" );

    // load test corpus
    if ( load_scm_file( *(this->corpusAtomSpace), "./pm_test_corpus.scm" ) == 0  )
        logger().info( "PatternMiningAgent::%s - Loaded pattern miner test corpus file: '%s'",
                        __FUNCTION__,
                       "pm_test_corpus.scm"
                     );
    else
        logger().error( "PatternMiningAgent::%s - Failed to load pattern miner test corpus file: '%s'",
                         __FUNCTION__,
                        "pm_test_corpus.scm"
                      );


    cout << "PatternMiningAgent: init: loaded test corpus into corpusAtomSpace \n ";

    // create a pattern miner
    this->patternMiner = new PatternMiner(corpusAtomSpace,config().get_int("PATTERN_MAX_GRAM"));

    // Avoid initialize during next cycle
    this->bInitialized = true;

    cout << "PatternMiningAgent: init finished!\n ";
}

void PatternMiningAgent::run()
{
    this->cycleCount = _cogserver.getCycleCount();

    logger().debug( "PatternMiningAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server!");

    // Initialize the Agent (demandList etc)
    if ( !this->bInitialized )
        this->init();

    // test, only run once
    static bool hasRun = false;
    if (hasRun)
        return;
    this->patternMiner->runPatternMiner();
    hasRun = true;

}
