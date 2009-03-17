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
