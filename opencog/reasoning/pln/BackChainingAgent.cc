/*
 * opencog/reasoning/pln/BackChainingAgent.cc
 *
 * Copyright (C) 2009 by OpenCog Foundation
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
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

#include "BackChainingAgent.h"

#include <string>
#include <iostream>
#include <sstream>
#include <vector>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/util/mt19937ar.h>

#include "PLN.h"
#include "rules/RuleProvider.h"
#include "AtomSpaceWrapper.h"
#include "BackInferenceTreeNode.h"

using namespace opencog;
using namespace opencog::pln;

BackChainingAgent::BackChainingAgent(CogServer& cs) : Agent(cs), Bstate(), state(NULL), steps(0), maxSteps(500), stepsPerCycle(100)
{
    logger().info("[BackChainingAgent] constructor");
    // Starting values for rent and wage
    static const std::string defaultConfig[] = {
        "", ""
    };
    setParameters(defaultConfig);
    
    rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
    log = &logger();
}

BackChainingAgent::~BackChainingAgent()
{
    logger().info("[BackChainingAgent] destructor");
    delete rng;
}

opencog::RandGen* BackChainingAgent::getRandGen()
{
    return rng;
}

void BackChainingAgent::chooseTarget()
{
    log->info("=========== BackChainingAgent::chooseTarget =======");
    
    AtomSpace& a = atomspace();

    Handle h; // The target
    
    // Initial criterion for choosing targets: randomly choose a target with zero confidence
    HandleSeq hs;
    a.getHandleSet(back_inserter(hs), ATOM, true);

    //! @todo apply a filter
    
    if (hs.empty()) {
        log->error("[BackChainingAgent] no suitable atoms");
        Bstate.reset();
        state = NULL;
        return;
    }

    int chosen_int = getRandGen()->randint(hs.size());
    h = hs[chosen_int];
//    std::cout << "Atom: " << a.atomAsString(h);
    
    // Initially: Run one whole PLN inference
    // Later: Divide it into slices
    
    pHandleSeq fakeHandles = ASW()->realToFakeHandle(h);
    if (fakeHandles.empty()) {
        log->error("[BackChainingAgent] fakeHandles is empty");
        Bstate.reset();
        state = NULL;
        return;
    }
    pHandle fakeHandle = fakeHandles[0];    
    Btr<vtree> target(new vtree(fakeHandle));

    Bstate.reset(new BITNodeRoot(target, NULL));//new DefaultVariableRuleProvider));

    std::cout << "BITNodeRoot init ok\n";
    state = Bstate.get();

    steps = 0;
}

void BackChainingAgent::run()
{
    log->info("=========== BackChainingAgent::run =======");
    // if zero steps have been done, that means the target hasn't been set yet.
    if (steps >= maxSteps || steps == 0) {
        chooseTarget();
    }
    
    if (state != NULL) {
        int j = stepsPerCycle;
        state->infer(j, 0.000001f, 0.90f); // Sets j to the number of steps remaining for this cycle
        state->printResults();
        
        std::cout << "\n" << j << " $ remaining.\n";
        
        steps += (stepsPerCycle - j);

        std::cout << "steps: " << steps << std::endl;
        // If it didn't use all the steps, then it is finished
        if (j > 0) {
            // Indicate to use a new target next cycle.
            steps = maxSteps;
        }
    }
}

