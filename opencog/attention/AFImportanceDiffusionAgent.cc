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
#include "AttentionParamQuery.h"
#include "Attention.h"
#include "AttentionStat.h"

using namespace opencog;

#ifdef DEBUG
#undef DEBUG
#endif

AFImportanceDiffusionAgent::AFImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
   set_sleep_time(500);
}

void AFImportanceDiffusionAgent::run()
{
    // Read params
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
    updateMaxSpreadPercentage();

    return (AttentionValue::sti_t) round(_bank->get_sti(h) * maxSpreadPercentage);

    // TODO: Using integers for STI values can cause strange consequences.
    // For example, if the amount to diffuse is 0.4, it will become 0, causing
    // no diffusion to occur.
    //   * See: https://github.com/opencog/opencog/issues/676
}
