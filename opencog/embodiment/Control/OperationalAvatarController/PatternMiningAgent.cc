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
    logger().debug( "PatternMiningAgent::%s - Initialize the Agent [cycle = %d]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Get AtomSpace

    this->patternMiner = new PatternMiner(&(oac->getAtomSpace()),config().get_bool("PATTERN_MAX_GRAM"));

    // Avoid initialize during next cycle
    this->bInitialized = true;
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
