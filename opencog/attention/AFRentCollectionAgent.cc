/*
 * opencog/attention/AFRentCollectionAgent.cc
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

#include "AFRentCollectionAgent.h"
#include "AttentionParamQuery.h"
#include "AttentionStat.h"

#include <thread>
//#define DEBUG
#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;

AFRentCollectionAgent::AFRentCollectionAgent(CogServer& cs) : RentCollectionBaseAgent(cs)
{
}

AFRentCollectionAgent::~AFRentCollectionAgent() {
}

void AFRentCollectionAgent::selectTargets(HandleSeq &targetSetOut)
{
    std::back_insert_iterator<HandleSeq> out_hi(targetSetOut);
    attentionbank(_as).get_handle_set_in_attentional_focus(out_hi);
}

void AFRentCollectionAgent::collectRent(HandleSeq& targetSet)
{
    static bool first_time = true;
    if (first_time) {
        last_update = high_resolution_clock::now();
        first_time = false;
    }

    update_freq = std::stod(_atq.get_param_value(AttentionParamQuery::af_rent_update_freq));

    // calculate elapsed time Et
    microseconds elapsed_time = duration_cast<microseconds>
                                (high_resolution_clock::now() - last_update);

    if (elapsed_time.count() < 1000000/update_freq)
        return;

    double w = elapsed_time.count() * update_freq / 1000000;

    for (const Handle& h : targetSet) {
        AttentionValue::sti_t sti = get_sti(h);
        AttentionValue::lti_t lti = get_lti(h);
        AttentionValue::sti_t stiRent =  calculate_STI_Rent();
        stiRent *= w;
        AttentionValue::lti_t ltiRent =  calculate_LTI_Rent();
        ltiRent *= w;

        if (stiRent > sti) stiRent = sti;
        if (ltiRent > lti) ltiRent = lti;

        _bank->set_sti(h, sti - stiRent);
        _bank->set_lti(h, lti - ltiRent);

#ifdef LOG_AV_STAT
        atom_avstat[h].rent = (w * stiRent);
#endif

    }

    // update elapsed time
    last_update = high_resolution_clock::now();
}
