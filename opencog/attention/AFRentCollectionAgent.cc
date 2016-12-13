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
#include <opencog/attention/atom_types.h>

#include <opencog/atomspace/AtomSpace.h>

#include "AFRentCollectionAgent.h"

#include <thread>
//#define DEBUG
#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;

AFRentCollectionAgent::AFRentCollectionAgent(CogServer& cs) : RentCollectionBaseAgent(cs)
{
    update_cycle     = 1 / config().get_double("ECAN_AF_RENT_FREQUENCY", 2); // per second.
    last_update      = high_resolution_clock::now();
}

AFRentCollectionAgent::~AFRentCollectionAgent() {
}

void AFRentCollectionAgent::selectTargets(HandleSeq &targetSetOut)
{
    std::back_insert_iterator< std::vector<Handle> > out_hi(targetSetOut);
    _as->get_handle_set_in_attentional_focus(out_hi);
    /* Without adding this sleep code right below the above method call,
     * nlp-parse evaluation thread waits for minutes before it gets a chance to
     * run.
     */
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    //std::this_thread::sleep_for(std::chrono::seconds(1));
}

void AFRentCollectionAgent::collectRent(HandleSeq& targetSet)
{
    // calculate elapsed time Et
    seconds elapsed_time = duration_cast<seconds>
                           (high_resolution_clock::now() - last_update);
    if (elapsed_time.count() <  update_cycle )
        return;

    for (Handle& h : targetSet) {
        int sti = h->getAttentionValue()->getSTI();
        int lti = h->getAttentionValue()->getLTI();
        int stiRent = calculate_STI_Rent();
        int ltiRent = calculate_LTI_Rent();

        if (stiRent > sti) stiRent = sti;
        if (ltiRent > lti) ltiRent = lti;

        h->setSTI(sti - stiRent);  
        h->setLTI(lti - ltiRent);
    }

    //update elapsed time
    last_update = high_resolution_clock::now();
}

