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

#include <chrono>
#include <thread>
//#define DEBUG
#ifdef DEBUG
#undef DEBUG
#endif

using namespace opencog;

AFRentCollectionAgent::AFRentCollectionAgent(CogServer& cs) : RentCollectionBaseAgent(cs)
{
    set_sleep_time(500);
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
}
