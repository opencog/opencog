/*
 * AFImportanceDiffusionAgent.cc
 *
 * Copyright (C) 2016 Opencog Foundation
 *
 * All Rights Reserved
 *
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

#include <chrono>
#include <thread>

#include <opencog/cogserver/server/CogServer.h>

#include "AFImportanceDiffusionAgent.h"

using namespace opencog;

AFImportanceDiffusionAgent::AFImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
   set_sleep_time(500);
}

void AFImportanceDiffusionAgent::run()
{
    spreadImportance();

    //some sleep code
    std::this_thread::sleep_for(std::chrono::milliseconds(get_sleep_time()));
}

/*
 * Carries out the importance diffusion process, spreading STI along the
 * graph according to the configuration settings
 */
void AFImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq diffusionSourceVector =  ImportanceDiffusionBase::diffusionSourceVector(true);

    // Calculate the diffusion for each source atom, and store the diffusion
    // event in a stack
    for (Handle atomSource : diffusionSourceVector)
    {
        diffuseAtom(atomSource);
    }

    // Now, process all of the outstanding diffusion events in the diffusion
    // stack
    processDiffusionStack();
}

/*
 * Diffuses importance from one atom to its non-hebbian incident atoms
 * and hebbian adjacent atoms
 */
void AFImportanceDiffusionAgent::diffuseAtom(Handle source)
{
    // (1) Find the incident atoms that will be diffused to
    HandleSeq incidentAtoms =
            ImportanceDiffusionBase::incidentAtoms(source);

    // (2) Find the hebbian adjacent atoms that will be diffused to
    HandleSeq hebbianAdjacentAtoms =
            ImportanceDiffusionBase::hebbianAdjacentAtoms(source);

    // (3) Calculate the probability vector that determines what proportion to
    //     diffuse to each incident atom
    std::map<Handle, double> probabilityVectorIncident =
            ImportanceDiffusionBase::probabilityVectorIncident(
                incidentAtoms);

#ifdef DEBUG
    std::cout << "Calculating diffusion for " << source << std::endl;
    std::cout << "Incident probability vector contains " <<
                 probabilityVectorIncident.size() << " atoms." << std::endl;
#endif

    // (4) Calculate the probability vector that determines what proportion to
    //     diffuse to each hebbian adjacent atom
    std::map<Handle, double> probabilityVectorHebbianAdjacent =
            ImportanceDiffusionBase::probabilityVectorHebbianAdjacent(
                source, hebbianAdjacentAtoms);

#ifdef DEBUG
    std::cout << "Hebbian adjacent probability vector contains " <<
                 probabilityVectorHebbianAdjacent.size() << " atoms." <<
                 std::endl;
#endif

    // (5) Combine the two probability vectors into one according to the
    //     configuration parameters
    std::map<Handle, double> probabilityVector = combineIncidentAdjacentVectors(
                probabilityVectorIncident, probabilityVectorHebbianAdjacent);

#ifdef DEBUG
    std::cout << "Probability vector contains " << probabilityVector.size() <<
                 " atoms." << std::endl;
#endif

    // (6) Calculate the total amount that will be diffused
    AttentionValue::sti_t totalDiffusionAmount =
            calculateDiffusionAmount(source);

#ifdef DEBUG
    std::cout << "Total diffusion amount: " << totalDiffusionAmount << std::endl;
#endif

    /* ===================================================================== */

    // If there is nothing to diffuse, finish
    if (totalDiffusionAmount == 0)
        return;

    // Used for calculating decay amount. Since we will be using per time unit
    // decay, we need to calculated elapsed time unit since last spreading for
    // adjusting the decay exponent.
    opencog::ecan::chrono_d elapsed_seconds;
    auto now = hr_clock::now();

    if (first_time) {
        elapsed_seconds = now - now;
        last_spreading_time = now;
    } else {
        elapsed_seconds = now - last_spreading_time;
    }

    // Perform diffusion from the source to each atom target
    typedef std::map<Handle, double>::iterator it_type;
    for(it_type iterator = probabilityVector.begin();
        iterator != probabilityVector.end(); ++iterator)
    {
        DiffusionEventType diffusionEvent;

        // Calculate the diffusion amount using the entry in the probability
        // vector for this particular target (stored in iterator->second)
        diffusionEvent.amount = (AttentionValue::sti_t)
                floor(totalDiffusionAmount * iterator->second);

        // TODO Use elapsed seconds and to calculate the diffusion amount.
        // (r_WA)^elapsed_seconds
        // Maintain STI freq bin.
        /*diffusionEvent.amount = (AttentionValue::sti_t)
                floor(pow ((totalDiffusionAmount * iterator->second),elapsed_seconds));*/

        diffusionEvent.source = source;
        diffusionEvent.target = iterator->first;

        // Add the diffusion event to a stack. The diffusion is stored in a
        // stack, so that all the diffusion events can be processed after the
        // diffusion calculations are complete. Otherwise, the diffusion
        // amounts will be calculated in a different way than expected.
        diffusionStack.push(diffusionEvent);
    }


    last_spreading_time = now;
    first_time = false;

    /* ===================================================================== */

    // TODO: What if the STI values of the atoms change during these updates?
    // This could go wrong if there were simultaneous updates in other threads.

    // TODO: Support inverse hebbian links
}
