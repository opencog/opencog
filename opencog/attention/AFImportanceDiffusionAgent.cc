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
#include <opencog/attentionbank/bank/AVUtils.h>

#include "AFImportanceDiffusionAgent.h"
#include "AttentionParamQuery.h"
#include "Attention.h"
#include "AttentionStat.h"

using namespace opencog;

AFImportanceDiffusionAgent::AFImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
}

void AFImportanceDiffusionAgent::run()
{
    // Reread param values for dynamically updating the values.
    maxSpreadPercentage = std::stod(_atq.get_param_value(
                                    AttentionParamQuery::dif_spread_percentage));
    hebbianMaxAllocationPercentage =std::stod(_atq.get_param_value(
                                    AttentionParamQuery::heb_max_alloc_percentage));
    spreadHebbianOnly = std::stoi(_atq.get_param_value(
                                  AttentionParamQuery::dif_spread_hebonly));

    spreadImportance();
}

/*
 * Carries out the importance diffusion process, spreading STI along the
 * graph according to the configuration settings
 */
void AFImportanceDiffusionAgent::spreadImportance()
{
    HandleSeq diffusionSourceVector =  ImportanceDiffusionBase::diffusionSourceVector();

    // Calculate the diffusion for each source atom, and store the diffusion
    // event in a stack
    for (Handle atomSource : diffusionSourceVector) diffuseAtom(atomSource);

    // Now, process all of the outstanding diffusion events in the diffusion
    // stack
    processDiffusionStack();
}

/*
 * Returns the total amount of STI that the atom will diffuse
 *
 * Calculated as the maximum spread percentage multiplied by the atom's STI
 */
AttentionValue::sti_t AFImportanceDiffusionAgent::calculateDiffusionAmount(Handle h)
{
    return (get_sti(h) * maxSpreadPercentage);
}
