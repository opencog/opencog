/*
 * opencog/attention/FocusBoundaryUpdatingAgent.cc
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
#include <map>
#include <chrono>

#include <fstream>
#include <stdio.h>

#include <opencog/util/Config.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/attention/atom_types.h>

#define DEPRECATED_ATOMSPACE_CALLS
#include <opencog/atomspace/AtomSpace.h>
#include "FocusBoundaryUpdatingAgent.h"

//#define DEBUG

using namespace opencog;
using namespace std::chrono;

FocusBoundaryUpdatingAgent::FocusBoundaryUpdatingAgent(CogServer& cs) :
        Agent(cs)
{
    // Provide a logger
    setLogger(new opencog::Logger("FocusBoundaryUpdatingAgent.log", Logger::FINE, true));

    afbSize         = config().get_double("ECAN_AFB_SIZE");
    bottomBoundary  = config().get_int("ECAN_AFB_BOTTOM");
}

void FocusBoundaryUpdatingAgent::run()
{
        AttentionValue::sti_t afboundary = _as->get_attentional_focus_boundary();

        AttentionValue::sti_t maxsti = _as->get_max_STI();
        AttentionValue::sti_t minsti = _as->get_min_STI();

        AttentionValue::sti_t range = maxsti - minsti;

        double decay = 0.1;

        int newafb = maxsti - (range * afbSize);
        //int oldafb = afboundary;

        afboundary = newafb * decay + (1.0 - decay) * afboundary;

        afboundary = std::max(afboundary,bottomBoundary);

        //printf("NewAfb: %d OldAfb: %d Afb: %d \n",newafb,oldafb,afboundary);

        // Set the AF boundary
        _as->set_attentional_focus_boundary(afboundary);
}
