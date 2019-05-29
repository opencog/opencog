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
#include <opencog/attentionbank/types/atom_types.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/bank/AttentionBank.h>
#include <opencog/cogserver/server/CogServer.h>

#include "RentCollectionBaseAgent.h"

//#define DEBUG
#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;

RentCollectionBaseAgent::RentCollectionBaseAgent(CogServer& cs) :
    Agent(cs), _atq(&cs.getAtomSpace())
{
    _bank = &attentionbank(_as);
    load_params();
    // Provide a logger
    setLogger(new opencog::Logger("RentCollectionAgent.log", Logger::FINE, true));
}

void RentCollectionBaseAgent::run()
{
    load_params();
    HandleSeq targetSet;
    selectTargets(targetSet);

    if (targetSet.size() == 0) return;

    collectRent(targetSet);
}

void RentCollectionBaseAgent::load_params(void)
{
    // init starting wages/rents. these should quickly change and reach
    // stable values, which adapt to the system dynamics
    STIAtomRent = std::stod(_atq.get_param_value(AttentionParamQuery::rent_starting_sti_rent));
    LTIAtomRent = std::stod(_atq.get_param_value(AttentionParamQuery::rent_starting_lti_rent));
    targetSTI = std::stod(_atq.get_param_value(AttentionParamQuery::rent_target_sti_funds));
    stiFundsBuffer = std::stod(_atq.get_param_value(AttentionParamQuery::rent_sti_funds_buffer));
    targetLTI = std::stod(_atq.get_param_value(AttentionParamQuery::rent_target_lti_funds));
    ltiFundsBuffer = std::stod(_atq.get_param_value(AttentionParamQuery::rent_lti_funds_buffer));
}

AttentionValue::sti_t RentCollectionBaseAgent::calculate_STI_Rent()
{
    double funds = _bank->getSTIFunds();
    double diff  = targetSTI - funds;

    if(diff <= 0)
        return 0;

    double ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -0.99);
    AttentionValue::sti_t res = STIAtomRent + (STIAtomRent * ndiff);

    return res;
}

AttentionValue::sti_t RentCollectionBaseAgent::calculate_LTI_Rent()
{
    double funds = _bank->getLTIFunds();
    double diff  = targetLTI - funds;
    double ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return LTIAtomRent + (LTIAtomRent * ndiff);
}
