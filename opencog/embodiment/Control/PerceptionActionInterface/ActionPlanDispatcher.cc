/*
 * opencog/embodiment/Control/OperationalPetController/ActionPlanDispatcher.cc
 *
 * Copyright (C) 2007-2008 TO_COMPLETE
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
#include "ActionPlanDispatcher.h"

using namespace PerceptionActionInterface;

ActionPlanDispatcher::ActionPlanDispatcher(PAI& _pai, const std::list<PetAction>&  _actionPlan) : 
                                            pai(_pai),  actionPlan(_actionPlan) {
    badlyFailed = false;
}
                                            
void ActionPlanDispatcher::dispatch() {
    try {
        planId = pai.createActionPlan();
        for (std::list<PetAction>::const_iterator itr = actionPlan.begin(); itr != actionPlan.end(); itr++) {
            pai.addAction(planId, *itr);
        }
        pai.sendActionPlan(planId); 
    } catch (...) {
        badlyFailed = true;
    }
}

bool ActionPlanDispatcher::done() {
    if (badlyFailed) return true; 
    return pai.isPlanFinished(planId);
}

bool ActionPlanDispatcher::failed() {
    if (badlyFailed) return true; 
    return pai.hasPlanFailed(planId);
}
