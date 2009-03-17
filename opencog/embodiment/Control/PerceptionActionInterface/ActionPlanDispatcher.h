#ifndef ACTIONPLANDISPATCHER_H_
#define ACTIONPLANDISPATCHER_H_
/**
 * ActionPlanDispatcher.h
 * 
 * This class is useful for sending a list of actions (aka action plan)
 * and wait until the actions have finished or failed, despite of which individual 
 * action failures or completion.
 */
 
 #include "PAI.h"

namespace PerceptionActionInterface {
    
class ActionPlanDispatcher {

    PAI& pai;
    const std::list<PetAction>& actionPlan;
    ActionPlanID planId;
    bool badlyFailed;  

    public:
        ActionPlanDispatcher(PAI& _pai, const std::list<PetAction>&  _actionPlan);
        void dispatch();
        bool done();
        bool failed();
}; 

}
 
#endif /*ACTIONPLANDISPATCHER_H_*/
