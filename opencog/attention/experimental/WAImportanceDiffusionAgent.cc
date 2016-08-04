/*
 * WAImportanceDiffusionAgent.cc
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

#include "WAImportanceDiffusionAgent.h"

using namespace opencog;


WAImportanceDiffusionAgent::WAImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
    _tournamentSize = config().get_int("ECAN_DIFFUSION_TOURNAMENT_SIZE", 5);

    set_sleep_time(300);
}

void WAImportanceDiffusionAgent::run()
{
    spreadDecider->setFocusBoundary(0);
    spreadImportance();

    //some sleep code
    std::this_thread::sleep_for(std::chrono::milliseconds(get_sleep_time()));
}

Handle WAImportanceDiffusionAgent::tournamentSelect(HandleSeq population){

    Handle tournament[_tournamentSize];
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,population.size()-1);

    for(int i = 0; i < _tournamentSize; i++){
        int idx = distribution(generator);
        tournament[i] = population[idx];
    }

    auto result = std::max_element(tournament, tournament + (_tournamentSize - 1), []
                              (const Handle& h1, const Handle & h2)
    {
        return (h1->getSTI() > h2->getSTI());
    });

    return *result;
}

/*
 * Carries out the importance diffusion process, spreading STI along the
 * graph according to the configuration settings
 */
void WAImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq diffusionSourceVector = ImportanceDiffusionBase::diffusionSourceVector(false);

    if (diffusionSourceVector.size() == 0)
        return;

    Handle target = tournamentSelect(diffusionSourceVector);

    // Check the decision function to determine if spreading will occur
    if (spreadDecider->spreadDecision(target->getSTI())) {
#ifdef DEBUG
        std::cout << "Calling diffuseAtom." << std::endl;
#endif
        diffuseAtom(target);
    }
#ifdef DEBUG
    else
    {
        std::cout << "Did not call diffuseAtom." << std::endl;
    }
#endif

    // Now, process all of the outstanding diffusion events in the diffusion
    // stack
    processDiffusionStack();
}
