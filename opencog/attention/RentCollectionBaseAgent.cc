/*
 * opencog/attention/WARentCollectionAgent.h
 *
 * Written by Misgana Bayetta
 * All Rights Reserved
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

#include <algorithm>
#include <math.h>
#include <time.h>

#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/attention/atom_types.h>

#include <opencog/atomspace/AtomSpace.h>

#include "RentCollectionBaseAgent.h"

//#define DEBUG
#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;

RentCollectionBaseAgent::RentCollectionBaseAgent(CogServer& cs) :
    Agent(cs)
{
    // init starting wages/rents. these should quickly change and reach
    // stable values, which adapt to the system dynamics
    STIAtomRent = config().get_int("ECAN_STARTING_ATOM_STI_RENT", 1);
    LTIAtomRent = config().get_int("ECAN_STARTING_ATOM_LTI_RENT", 1);

    targetSTI = config().get_int("TARGET_STI_FUNDS", 10000);
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER", 10000);
    targetLTI = config().get_int("TARGET_LTI_FUNDS", 10000);
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER", 10000);

    // Provide a logger
    setLogger(new opencog::Logger("RentCollectionAgent.log", Logger::FINE, true));
}

void RentCollectionBaseAgent::run()
{
    HandleSeq targetSet;
    selectTargets(targetSet);

    if (targetSet.size() == 0) return;

    collectRent(targetSet);
   
    std::this_thread::sleep_for(std::chrono::milliseconds(get_sleep_time()));
}

int RentCollectionBaseAgent::calculate_STI_Rent()
{
    int funds = _as->get_STI_funds();
    double diff  = targetSTI - funds;
    double ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -0.99);
    //printf("ndiff: %f   ",ndiff);
    //
    double res = STIAtomRent + (STIAtomRent * ndiff);

    if (res < 1)
        if ((rand() % 100) > (100 * res))
            res = 1;

    return floor(res);
}

int RentCollectionBaseAgent::calculate_LTI_Rent()
{
    int funds = _as->get_LTI_funds();
    double diff  = targetLTI - funds;
    double ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return LTIAtomRent + (LTIAtomRent * ndiff);
}
