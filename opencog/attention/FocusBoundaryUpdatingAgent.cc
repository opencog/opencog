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

    afbSize         = config().get_double("ECAN_AFB_SIZE", 0.2);
    decay           = config().get_double("ECAN_AFB_DECAY", 0.05);
    bottomBoundary  = config().get_int("ECAN_AFB_BOTTOM", 50);
    minAFSize = config().get_int("MIN_AF_SIZE", 100);
    maxAFSize = config().get_int("MAX_AF_SIZE", 500);

   _as->set_attentional_focus_boundary(bottomBoundary);
}

void FocusBoundaryUpdatingAgent::run()
{
        AttentionValue::sti_t afboundary = _as->get_attentional_focus_boundary();

       /*
        AttentionValue::sti_t maxsti = _as->get_max_STI();
        AttentionValue::sti_t minsti = _as->get_min_STI();

        AttentionValue::sti_t range = maxsti - minsti;

        int newafb = maxsti - (range * afbSize);
        //int oldafb = afboundary;

        afboundary = newafb * decay + (1.0 - decay) * afboundary;

        afboundary = std::max(afboundary,bottomBoundary);

        //printf("NewAfb: %d OldAfb: %d Afb: %d \n",newafb,oldafb,afboundary);
        */
       
        HandleSeq afset;
        _as->get_handle_set_in_attentional_focus(std::back_inserter(afset));
        
        if(afset.size() > minAFSize ){
            afboundary = get_cutoff(afset);
        }

        // Set the AF boundary
        _as->set_attentional_focus_boundary(afboundary);
       
}

/**
 * Finds the natural cut off STI value between the minimum and maximum AF set.
 * The natural cutoff here is assumed as the index where the biggest STI
 * difference occured in the decreasingly ordered set of atoms bn Min and Max AF
 * size.
 *
 * @param afset - Atoms in the AF
 * @return the natural cutoff STI value
 */
AttentionValue::sti_t FocusBoundaryUpdatingAgent::get_cutoff(HandleSeq& afset)
{
    auto getSTI = [](const Handle& h) { return h->getAttentionValue()->getSTI();};
    std::sort(afset.begin(), afset.end(),[&](const Handle& h1, const Handle& h2){
            return getSTI(h1) > getSTI(h2);
            });
    HandleSeq afAtoms = HandleSeq(afset.begin() + minAFSize, afset.begin() + maxAFSize);

    int cut_off_index = 0;
    int biggest_diff = -1 ; // diffs are always +ve. so, its okay to initialize it with -ve number.
    constexpr int DIFF_MAGNITUDE = 0.5; // make this a parameter
    for( HandleSeq::size_type i = 0 ; i < afAtoms.size()-1; i++ ){
        int diff = getSTI(afAtoms[i]) - getSTI(afAtoms[i+1]);
        if(diff > biggest_diff*(1 + DIFF_MAGNITUDE )) {  
            biggest_diff = diff;
            cut_off_index = i;
        }
    }

    return getSTI(afAtoms[cut_off_index]);
} 
