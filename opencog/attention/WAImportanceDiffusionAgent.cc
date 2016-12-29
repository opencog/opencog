/*
 * opencog/attention/WAImportanceDiffusionAgent.cc
 *
 * Copyright (C) 2016 Opencog Foundation
 *
 * All Rights Reserved
 *
 * Writeen by Cosmo Harrigan
 * Written by Misgana Bayetta <misgana.bayetta@gmail.com>
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

#include <opencog/util/Config.h>
#include <opencog/attention/atom_types.h>
#include <opencog/attentionbank/AttentionBank.h>

#include "WAImportanceDiffusionAgent.h"

using namespace opencog;


WAImportanceDiffusionAgent::WAImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
    _tournamentSize = config().get_int("ECAN_DIFFUSION_TOURNAMENT_SIZE", 5);

    set_sleep_time(300);
}

WAImportanceDiffusionAgent::~WAImportanceDiffusionAgent()
{
}

void WAImportanceDiffusionAgent::run()
{
    spreadImportance();

    //some sleep code
    std::this_thread::sleep_for(std::chrono::milliseconds(get_sleep_time()));
}

Handle WAImportanceDiffusionAgent::tournamentSelect(HandleSeq population){
    int sz = (_tournamentSize >  population.size() ? population.size() : _tournamentSize);

    if (sz <= 0)
        throw RuntimeException(TRACE_INFO,"PopulationSize must be >0");

    Handle tournament[sz];
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,population.size()-1);

    for(int i = 0; i < sz; i++){
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

/*
 * Carries out the importance diffusion process, spreading STI along the
 * graph according to the configuration settings
 */
void WAImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq sourceVec = diffusionSourceVector();

    if (sourceVec.size() == 0)
        return;

    Handle target = tournamentSelect(sourceVec);

    // Check the decision function to determine if spreading will occur
    diffuseAtom(target);

    // Now, process all of the outstanding diffusion events in the diffusion
    // stack
    processDiffusionStack();
}

/*
 * Returns a vector of atom handles that will diffuse STI
 *
 * Calculated as all atoms in the attentional focus (nodes and links)
 * excluding any hebbian links
 */
HandleSeq WAImportanceDiffusionAgent::diffusionSourceVector(void)
{
    HandleSeq  sources;
    
    AttentionValue::sti_t AFBoundarySTI = _bank->getAttentionalFocusBoundary();
    AttentionValue::sti_t lowerSTI  =   AFBoundarySTI - 15;

    std::default_random_engine generator;
    // randomly select a bin 
    std::uniform_int_distribution<AttentionValue::sti_t> dist(lowerSTI,AFBoundarySTI);
    auto sti = dist(generator);
    _bank->get_handles_by_AV(std::back_inserter(sources),sti,sti+5);
    
    if(sources.size() > 100){ sources.resize(100); } //Resize to 100 elements.

#ifdef DEBUG
    std::cout << "Calculating diffusionSourceVector." << std::endl;
    std::cout << "AF Size before removing hebbian links: " <<
        sources.size() << "\n";
#endif

    // Remove the hebbian links
    auto it_end =
        std::remove_if(sources.begin(), sources.end(),
                [=](const Handle& h)
                {
                Type type = h->getType();

#ifdef DEBUG
                std::cout << "Checking atom of type: " <<
                classserver().getTypeName(type) << "\n";
#endif

                if (type == ASYMMETRIC_HEBBIAN_LINK ||
                    type == HEBBIAN_LINK ||
                    type == SYMMETRIC_HEBBIAN_LINK ||
                    type == INVERSE_HEBBIAN_LINK ||
                    type == SYMMETRIC_INVERSE_HEBBIAN_LINK)
                {
#ifdef DEBUG
                std::cout << "Atom is hebbian" << "\n";
#endif
                return true;
                }
                else
                {
#ifdef DEBUG
                    std::cout << "Atom is not hebbian" << "\n";
#endif
                    return false;
                }
                });
    sources.erase(it_end, sources.end());

#ifdef DEBUG
    std::cout << "AF Size after removing hebbian links: " <<
        sources.size() << "\n";
#endif

    return sources;
}



