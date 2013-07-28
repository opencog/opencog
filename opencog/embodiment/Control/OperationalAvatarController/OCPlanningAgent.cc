/*
 * @file opencog/embodiment/Control/OperationalAvatarController/OCPlanningAgent.cc
 *
 * @author Shujing KE <rainkekekeke@gmail.com>
 * @date 2013-07-27
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

#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/util/platform.h>

#include "OAC.h"
#include "OCPlanningAgent.h"
#include "Strips.h"
#include "PsiRuleUtil.h"

using namespace opencog::oac;

OCPlanningAgent::OCPlanningAgent()
{
    this->cycleCount = 0;

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void OCPlanningAgent::init(opencog::CogServer * server)
{
    logger().debug( "OCPlanningAgent::%s - Initializing the Agent [cycle = %d]",
                    __FUNCTION__,
                    this->cycleCount
                  );

    // Get OAC
    this->oac = dynamic_cast<OAC*>(server);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Avoid initialize during next cycle
    this->bInitialized = true;

    this->use_ocplanner = config().get_long("USE_OCPLANNDER");

    // if use ocplanner , create a ocplanner
    if (use_ocplanner)
    {
        ocplanner = new OCPlanner(&(oac->getAtomSpace()) ,oac);
        logger().debug("OCPlanningAgent::init - using OCPlanner! ");
    }

    this->currentOCPlanID = "";

    this->procedureExecutionTimeout = config().get_long("PROCEDURE_EXECUTION_TIMEOUT");
}

void OCPlanningAgent::getCurrentDemand()
{
    // Select the most important demand for now, by scm
    SchemeEval & evaluator = SchemeEval::instance();
    std::string scheme_expression,scheme_return_value;

    scheme_expression = "( update_selected_demand_goal )";

    // Run the Procedure that do planning
    scheme_return_value = evaluator.eval(scheme_expression);

    if ( evaluator.eval_error() )
    {
        logger().error( "OCPlanningAgent::%s -getCurrentDemand() failed '%s'",
                         __FUNCTION__,
                         scheme_expression.c_str()
                      );

        hSelectedDemandGoal = Handle::UNDEFINED;
    }

    hSelectedDemandGoal =  AtomSpaceUtil::getReference(oac->getAtomSpace(),
                                    oac->getAtomSpace().getHandle(CONCEPT_NODE,
                                                        "plan_selected_demand_goal"));

    // Update the pet's previously/ currently Demand Goal
    PsiRuleUtil::setCurrentDemandGoal(oac->getAtomSpace(), this->hSelectedDemandGoal);

}

void OCPlanningAgent::runOCPlanner()
{
    getCurrentDemand();

    logger().debug( "OCPlanningAgent::%s - do planning for the Demand Goal: %s [cycle = %d]",
                    __FUNCTION__,
                    oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal).c_str(),
                    this->cycleCount
                  );

    // todo currentOCPlanID = ocplanner->doPlanning();

    // if planning failed
    if (currentOCPlanID == "")
    {
        std::cout<<"OCPlanner can not find any suitable plan for the selected demand goal:"
                 <<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal).c_str()
                 <<std::endl;
    }
    else
    {
        std::cout<<"OCPlanner found a plan: "<< currentOCPlanID << " for the selected demand goal:"
                 <<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal).c_str()
                 <<std::endl;

        // get all the current plan action sequence, contain the actionId of each action.
        // ActionId is the handle of this action in the oac->getAtomSpace(), see PAI.
        this->current_actions = oac->getPAI().getActionSeqFromPlan(currentOCPlanID);

        oac->getPAI().sendActionPlan(currentOCPlanID);

        this->current_action = current_actions.front();
        this->current_actions.erase(current_actions.begin());
        this->timeStartCurrentAction = time(NULL);
    }
}

void OCPlanningAgent::run(opencog::CogServer * server)
{
    this->cycleCount = server->getCycleCount();

    logger().debug( "OCPlanningAgent::%s - Executing run %d times",
                     __FUNCTION__,
                     this->cycleCount
                  );

    if (this->currentOCPlanID != "")
    {
        if (! oac->getPAI().isPlanFinished(currentOCPlanID))
        {
            // the plan is still being execting

            // check if current action failed
            if (! oac->getPAI().isActionDone( this->current_action, oac->getPAI().getLatestSimWorldTimestamp()) )
            {

                // check if current action timeout
                if ( time(NULL) - this->timeStartCurrentAction >  this->procedureExecutionTimeout )
                {
                    std::cout<<"current action timeout [PlanId = "
                            <<this->currentOCPlanID<<", cycle = "<<this->cycleCount<<"] ... "
                            <<std::endl;


                    // add 'actionFailed' predicates for timeout actions
                    oac->getPAI().setPendingActionPlansFailed();

//                    // Stop the time out Action
//                    procedureInterpreter.stopProcedure(this->currentPlanId);

                    std::cout<<"Action status: timeout"<<std::endl;

                    // Now that this action failed, the following action sequence
                    // should be dropped.
                    this->current_actions.clear();

                  }
                   // If the Action is still running and is not time out, simply returns
                  else
                  {


                        std::cout<<"current action is still running [PlanId = "
                                <<this->currentOCPlanID<<", cycle = "<<this->cycleCount<<"] ... "
                                <<std::endl;
                  }
            }

//            if()
//            {
//                // Get next action from current plan
//                if ( !this->current_plan_actions.empty() ) {
//                    this->current_action = this->current_actions.back();
//                    this->current_plan_actions.pop_back();
//                }
//                else if ( !this->temp_action_list.empty() ) {
//                    this->getActions(oac->getAtomSpace(), this->temp_action_list.back(), this->current_plan_actions);
//                    this->temp_action_list.pop_back();

//                    this->current_action = this->current_actions.back();
//                    this->current_actions.pop_back();
//                }
//                else {
//                    logger().debug("PsiActionSelectionAgent::%s - "
//                                   "Failed to get any actions from the planner. Try planning next cycle [cycle = %d]",
//                                    __FUNCTION__, this->cycleCount
//                                  );
//                    return;
//                }
//            }

//              return;

//        }

//        if(oac->getPAI().hasPlanFailed(currentOCPlanID))
//        {
//            std::cout<<std::endl<<"OCPlanningAgent::Action execution failed! Plan ID = "<< this->currentOCPlanID
//                     <<std::endl<<"Selected Demand Goal [cycle = "<<this->cycleCount<<"]:"
//                     <<std::endl<<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal)<<std::endl;

//        }
//        else
//        {
//            std::cout<<std::endl<<"OCPlanningAgent::Action execution success! Plan ID = "<< this->currentOCPlanID
//                     <<std::endl<<"Selected Demand Goal [cycle = "<<this->cycleCount<<"]:"
//                     <<std::endl<<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal)<<std::endl;
//        }

//        this->currentOCPlanID = "";

     }
    }

    runOCPlanner();

}
