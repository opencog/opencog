/*
 * opencog/attention/RentCollectionAgent.cc
 *
 * Written by Roman Treutlein
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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/bank/AttentionBank.h>
#include <opencog/attentionbank/types/atom_types.h>
#include <opencog/cogserver/server/CogServer.h>

#include "WARentCollectionAgent.h"
#include "AttentionParamQuery.h"

//#define DEBUG

using namespace opencog;

WARentCollectionAgent::WARentCollectionAgent(CogServer& cs):
                       RentCollectionBaseAgent(cs),
                       _sdac(&attentionbank(&cs.getAtomSpace()).getImportance())
{
    // READ SLEEPING TIME HERE
    _sti_rent = STIAtomRent;
    _lti_rent = LTIAtomRent;
}

void WARentCollectionAgent::selectTargets(HandleSeq &targetSetOut)
{
    Handle h = _bank->getRandomAtomNotInAF();
    if(h == Handle::UNDEFINED)
        return;
    targetSetOut.push_back(h);
}

void WARentCollectionAgent::collectRent(HandleSeq& targetSet)
{
    for (const Handle& h : targetSet) {
        AttentionValue::sti_t sti = get_sti(h);
        AttentionValue::lti_t lti = get_lti(h);
        
        float last_update_time = _sdac.elapsed_time(h);
        STIAtomRent = STIAtomRent * last_update_time;
        LTIAtomRent = LTIAtomRent * last_update_time;
        
        double stiRent = calculate_STI_Rent();
        double ltiRent = calculate_LTI_Rent();
        if (stiRent > sti)
            stiRent = sti;

        if (ltiRent > lti)
            ltiRent = lti;

        _bank->set_sti(h, sti - stiRent);
        _bank->set_lti(h, lti - ltiRent);
    }
}
