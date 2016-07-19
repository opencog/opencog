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
    log = NULL;
    setLogger(new opencog::Logger("FocusBoundaryUpdatingAgent.log", Logger::FINE,
    true));

    afbSize         = (float)(config().get_double("ECAN_AFB_SIZE"));
    bottomBoundary  = (AttentionValue::sti_t)(config().get_int("ECAN_AFB_BOTTOM"));
}

FocusBoundaryUpdatingAgent::~FocusBoundaryUpdatingAgent()
{
    if (log)
        delete log;
}

void FocusBoundaryUpdatingAgent::setLogger(Logger* _log)
{
    if (log)
        delete log;
    log = _log;
}

Logger* FocusBoundaryUpdatingAgent::getLogger()
{
    return log;
}

void FocusBoundaryUpdatingAgent::run()
{
        a = &_cogserver.getAtomSpace();
        AttentionValue::sti_t afboundary = a->get_attentional_focus_boundary();

        AttentionValue::sti_t maxsti = a->get_max_STI();
        AttentionValue::sti_t minsti = a->get_min_STI();

        AttentionValue::sti_t range = maxsti - minsti;

        float decay = 0.1;

        int newafb = maxsti - (range * afbSize);
        //int oldafb = afboundary;

        afboundary = newafb * decay + (1 - decay) * afboundary;

        afboundary = std::max(afboundary,bottomBoundary);

        //printf("NewAfb: %d OldAfb: %d Afb: %d \n",newafb,oldafb,afboundary);

        // Set the AF boundary
        a->set_attentional_focus_boundary(afboundary);

}
