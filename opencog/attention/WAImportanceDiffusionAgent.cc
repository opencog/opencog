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
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/attention/atom_types.h>
#include <opencog/attentionbank/AttentionBank.h>
#include <opencog/attentionbank/StochasticImportanceDiffusion.h>

#include "AttentionUtils.h"
#include "WAImportanceDiffusionAgent.h"

using namespace opencog;


WAImportanceDiffusionAgent::WAImportanceDiffusionAgent(CogServer& cs) :
    ImportanceDiffusionBase(cs)
{
}

WAImportanceDiffusionAgent::~WAImportanceDiffusionAgent()
{
}

void WAImportanceDiffusionAgent::run()
{
    // Read params
    hebbianMaxAllocationPercentage =std::stod(_atq.get_param_value(
                                     AttentionParamQuery::dif_tournament_size));
    spreadImportance();
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

    Handle target = sourceVec[0];

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
    Handle h = _bank->getRandomAtomNotInAF();

    if(h == Handle::UNDEFINED){
        return HandleSeq{};
    }
    HandleSeq sources{h};
    removeHebbianLinks(sources);  //XXX Do wee need this?
    return sources;
}

/*
 * Returns the total amount of STI that the atom will diffuse
 *
 * Calculated as the maximum spread percentage multiplied by the atom's STI
 */

AttentionValue::sti_t WAImportanceDiffusionAgent::calculateDiffusionAmount(Handle h)
{
    static ecan::StochasticDiffusionAmountCalculator sdac(&_bank->getImportance());
    float current_estimate = sdac.diffused_value(h, maxSpreadPercentage);

    return get_sti(h) - current_estimate;
}
