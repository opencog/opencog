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
        if( hseq.size() > 0){
            //getTopSTIVlauedHandles function returns Handles in Increasing STI
            //order. 
             afboundary = _bank->get_sti(hseq[0]);
             _bank->setAttentionalFocusBoundary(afboundary);
        }
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
            _bank->setAttentionalFocusBoundary(afboundary);
        }
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
    // A hack to bypass some atomic_dispatch segfault. I am hypothesizing
    // sorting might have failed due to change in STI in another thread while
    // std::sort wants to do it atomically.
    // so, since atomic sorting is not critical to our case, lets copy the sti
    // values and do the sorting and cutoff calculation in a non atomic way.
    std::vector<AttentionValue::sti_t> af_sti;
    for(const Handle& h: afset)
        af_sti.push_back(_bank->get_sti(h));
    
    std::sort(af_sti.begin(), af_sti.end(),
            [&](const AttentionValue::sti_t& sti1,
                const AttentionValue::sti_t& sti2)->bool {
            return sti1 > sti2;
            });

    auto afSTIValues = std::vector<AttentionValue::sti_t>(af_sti.begin() + minAFSize, 
            ((int)af_sti.size() > maxAFSize ? af_sti.begin() + maxAFSize : af_sti.end()));

    // If there is no natural boundary, find it amongst the minAFSize atoms.
    if(afSTIValues[0] == afSTIValues[afSTIValues.size()]){
        afSTIValues = std::vector<AttentionValue::sti_t>(af_sti.begin(), af_sti.begin() + minAFSize);
    }


    int cut_off_index = 0;
    AttentionValue::sti_t biggest_diff = -1.0f ; // diffs are always +ve. so, its okay to initialize it with -ve number.
    constexpr AttentionValue::sti_t DIFF_MAGNITUDE = 0.5f; // make this a parameter
    for( int i = 0 ; i < (int)afSTIValues.size()-1; i++ ){
        AttentionValue::sti_t diff = afSTIValues[i] - afSTIValues[i+1]; // 5 4 3 1
        if(diff > biggest_diff*(1 + DIFF_MAGNITUDE )) {  
            biggest_diff = diff;
            cut_off_index = i;
        }
    }
    return afSTIValues[cut_off_index];
} 
