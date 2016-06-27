/*
 * opencog/attention/AFRentCollectionAgent.cc
 *
 * Copyright (C) 2008 by OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>

#include "AFRentCollectionAgent.h"

//#define DEBUG

using namespace opencog;

AFRentCollectionAgent::AFRentCollectionAgent(CogServer& cs) : RentCollectionBase(cs),
Agent(cs) {
    set_sleep_time(500);
 }

AFRentCollectionAgent::~AFRentCollectionAgent() {
 }

void AFRentCollectionAgent::run()
{
    while (true) {
        a = &_cogserver.getAtomSpace();

        HandleSeq atoms;
        size_t size;

        std::back_insert_iterator< std::vector<Handle> > out_hi(atoms);

        a->get_handle_set_in_attentional_focus(out_hi);

        size = atoms.size();

        if (size == 0) continue;

        for (Handle& h : atoms) {
	    
            int sti = h->getAttentionValue()->getSTI();
            int lti = h->getAttentionValue()->getLTI();
            int stiRent = calculate_STI_Rent();
            int ltiRent = calculate_LTI_Rent();

            //printf("stiRent: %d ",stiRent);
            //printf("sti: %d ",sti);

            if (stiRent > sti)
                stiRent = sti;

            if (ltiRent > lti)
                ltiRent = lti;

            //printf("stiRent: %d \n",stiRent);

            h->setSTI(sti - stiRent);
            h->setLTI(lti - ltiRent);
        }

        std::cout << "[DEBUG] [AFRentCollectionAgent] sleeping for " << get_sleep_time() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(get_sleep_time()));
    }
}
