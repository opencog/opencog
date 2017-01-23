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
#include <opencog/attention/atom_types.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attentionbank/AttentionBank.h>

#include "WARentCollectionAgent.h"
#include "AttentionParamQuery.h"

//#define DEBUG

using namespace opencog;

WARentCollectionAgent::WARentCollectionAgent(CogServer& cs) : RentCollectionBaseAgent(cs)
{
    // READ SLEEPING TIME HERE
    set_sleep_time(2000);
}

Handle WARentCollectionAgent::tournamentSelect(HandleSeq population){
    _tournamentSize = std::stoi(_atq.get_param_value(AttentionParamQuery::rent_tournament_size));

    int sz = (_tournamentSize >  population.size() ? population.size() : _tournamentSize);

    if (sz <= 0)
        throw RuntimeException(TRACE_INFO,"PopulationSize must be >0");

    Handle tournament[sz];

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,population.size()-1);

    for(int i = 0; i < sz ; i++){
        int idx = distribution(generator);
        tournament[i] = population[idx];
    }

    auto result = std::max_element(tournament, tournament + (sz - 1),
         [&](const Handle& h1, const Handle & h2) -> bool
    {
        return _bank->get_sti(h1) > _bank->get_sti(h2);
    });

    return *result;
}


void WARentCollectionAgent::selectTargets(HandleSeq &targetSetOut)
{
    AttentionValue::sti_t AFBoundarySTI = attentionbank(_as).getAttentionalFocusBoundary();
    AttentionValue::sti_t lowerSTI  = AFBoundarySTI - 15;

    std::default_random_engine generator;
    // randomly select a bin
    std::uniform_int_distribution<AttentionValue::sti_t> dist(lowerSTI,AFBoundarySTI);
    auto sti = dist(generator);

    HandleSeq candidates;
    attentionbank(_as).get_handles_by_AV(std::back_inserter(candidates),sti,sti+5);
    if(candidates.size() > 100) candidates.resize(100); //Resize to 100 elements.

    if (candidates.size() == 0) return;

    Handle h = tournamentSelect(candidates);
    targetSetOut.push_back(h);
}

void WARentCollectionAgent::collectRent(HandleSeq& targetSet)
{
    for (const Handle& h : targetSet) {
        int sti = _bank->get_sti(h);
        int lti = _bank->get_lti(h);
        int stiRent = calculate_STI_Rent();
        int ltiRent = calculate_LTI_Rent();

        if (stiRent > sti)
            stiRent = sti;

        if (ltiRent > lti)
            ltiRent = lti;

        _bank->set_sti(h, sti - stiRent);
        _bank->set_lti(h, lti - ltiRent);
    }
}
