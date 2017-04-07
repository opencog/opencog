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
#include <opencog/attentionbank/AttentionBank.h>
#include "FocusBoundaryUpdatingAgent.h"

//#define DEBUG

using namespace opencog;
using namespace std::chrono;

FocusBoundaryUpdatingAgent::FocusBoundaryUpdatingAgent(CogServer& cs) :
    Agent(cs), _atq(&cs.getAtomSpace())
{
    // Provide a logger
    setLogger(new opencog::Logger("FocusBoundaryUpdatingAgent.log", Logger::FINE, true));

    _bank = &attentionbank(_as);
    _bank->setAttentionalFocusBoundary(bottomBoundary);
}

void FocusBoundaryUpdatingAgent::run()
{
        afbSize = std::stod(_atq.get_param_value(AttentionParamQuery::af_size));
        decay = std::stod(_atq.get_param_value(AttentionParamQuery::af_decay));
        bottomBoundary = std::stoi(_atq.get_param_value(AttentionParamQuery::af_bottom));
        minAFSize = std::stoi(_atq.get_param_value(AttentionParamQuery::af_min_size));
        maxAFSize = std::stoi(_atq.get_param_value(AttentionParamQuery::af_max_size));

        AttentionValue::sti_t afboundary = _bank->getAttentionalFocusBoundary();
        // Let there always be K top STI valued atoms in the AF.
        HandleSeq hseq = _bank->getTopSTIValuedHandles();
        if( hseq.size() > 0)
            //getTopSTIVlauedHandles function returns Handles in Increasing STI
            //order. 
             afboundary = _bank->get_sti(hseq[0]);
       /*
        AttentionValue::sti_t maxsti = _bank->get_max_STI();
        AttentionValue::sti_t minsti = _bank->get_min_STI();

        AttentionValue::sti_t range = maxsti - minsti;

        int newafb = maxsti - (range * afbSize);
        //int oldafb = afboundary;

        afboundary = newafb * decay + (1.0 - decay) * afboundary;

        afboundary = std::max(afboundary, bottomBoundary);

        //printf("NewAfb: %d OldAfb: %d Afb: %d \n",newafb,oldafb,afboundary);
        */
      
        // Make sure not too many atoms are in the AF.
        HandleSeq afset;
        _bank->get_handle_set_in_attentional_focus(std::back_inserter(afset));
        if(afset.size() > minAFSize ) {
            afboundary = get_cutoff(afset);
        }

        // Set the AF boundary
        _bank->setAttentionalFocusBoundary(afboundary);
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
    std::sort(afset.begin(), afset.end(),
            [&](const Handle& h1, const Handle& h2)->bool {
                return _bank->get_sti(h1) > _bank->get_sti(h2);
            });

    HandleSeq afAtoms = HandleSeq(afset.begin() + minAFSize, 
                        ((int)afset.size() > maxAFSize ? afset.begin() + maxAFSize : afset.end()));
    int cut_off_index = 0;
    int biggest_diff = -1 ; // diffs are always +ve. so, its okay to initialize it with -ve number.
    constexpr int DIFF_MAGNITUDE = 0.5; // make this a parameter
    for( int i = 0 ; i < (int)afAtoms.size()-1; i++ ){
        int diff = _bank->get_sti(afAtoms[i]) - _bank->get_sti(afAtoms[i+1]);
        if(diff > biggest_diff*(1 + DIFF_MAGNITUDE )) {  
            biggest_diff = diff;
            cut_off_index = i;
        }
    }

    return _bank->get_sti(afAtoms[cut_off_index]);
} 
