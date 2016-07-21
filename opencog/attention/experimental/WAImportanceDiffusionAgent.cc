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

#include <opencog/cogserver/server/CogServer.h>

#include "WAImportanceDiffusionAgent.h"

using namespace opencog;


WAImportanceDiffusionAgent::WAImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
    set_sleep_time(300);
}

WAImportanceDiffusionAgent::~WAImportanceDiffusionAgent(){

}

void WAImportanceDiffusionAgent::run()
{
    std::cout << "[DEBUG] [WAImportanceDiffusionAgent] started running.\n";

    spreadDecider->setFocusBoundary(0);
    spreadImportance();

    //some sleep code
    std::cout << "[DEBUG] [WAImportanceDiffusionAgent] sleeping for " <<  get_sleep_time() << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(get_sleep_time()));
}

/*
 * Carries out the importance diffusion process, spreading STI along the
 * graph according to the configuration settings
 */
void WAImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq diffusionSourceVector = ImportanceDiffusionBase::diffusionSourceVector(false);

    HandleSeq targetSet;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,diffusionSourceVector.size()-1);


    for (unsigned int i = 0; (i < SAMPLE_SIZE and i < diffusionSourceVector.size()) ; i++) {
        HandleSeq tmp;

        // Randomly sample 50% of the diffusionSourceVector
        for (unsigned int i = 0; i < diffusionSourceVector.size()/2; i++) {
            int idx = distribution(generator);
            tmp.push_back(diffusionSourceVector[idx]);
        }

        auto result = std::max_element(tmp.begin(), tmp.end(), []
                                  (const Handle& h1, const Handle & h2)
        {
            return (h1->getSTI() > h2->getSTI());
        });

        // Select the atoms with the highest STI amongst the samples
        targetSet.push_back(*result);

        // set diffusionSource vector to the current samples.
        diffusionSourceVector = tmp;
    }

    diffusionSourceVector = targetSet;

    // Calculate the diffusion for each source atom, and store the diffusion
    // event in a stack
    for (Handle atomSource : diffusionSourceVector)
    {
        // Check the decision function to determine if spreading will occur
        if (spreadDecider->spreadDecision(atomSource->getSTI())) {
#ifdef DEBUG
            std::cout << "Calling diffuseAtom." << std::endl;
#endif
            diffuseAtom(atomSource);
        }
#ifdef DEBUG
        else
        {
            std::cout << "Did not call diffuseAtom." << std::endl;
        }
#endif
    }

    // Now, process all of the outstanding diffusion events in the diffusion
    // stack
    processDiffusionStack();
}
