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

#include <boost/tokenizer.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>

#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/SpaceTime.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/util/platform.h>

#include "OAC.h"
#include "OCPlanningAgent.h"
#include "Strips.h"
#include "PsiRuleUtil.h"

using namespace opencog::oac;
using namespace std;

OCPlanningAgent::OCPlanningAgent(CogServer& cs) : Agent(cs)
{
    this->cycleCount = 0;

    startPlanning = false;

    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

OCPlanningAgent::~OCPlanningAgent()
{
    delete ocplanner;
}

void OCPlanningAgent::init()
{
    logger().debug( "OCPlanningAgent::%s - Initializing the Agent [cycle = %d]",
                    __FUNCTION__, this->cycleCount);

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    // Avoid initialize during next cycle
    this->bInitialized = true;

    ocplanner = new OCPlanner(&(oac->getAtomSpace()),oac->getPet().getPetId(),oac->getPet().getType());
    logger().debug("OCPlanningAgent::init - using OCPlanner! ");

    this->currentOCPlanID = "";

    this->current_actions.clear();
    this->current_action = Handle::UNDEFINED;
    this->current_step = -1;

    this->procedureExecutionTimeout = config().get_long("PROCEDURE_EXECUTION_TIMEOUT");

    // init the demanding goal list

    psi_demand_goal_list.clear();

    // Get demand names from the configuration file
    std::string demandNames = config()["PSI_DEMANDS"];

    // Process Demands one by one
    boost::tokenizer<> demandNamesTok (demandNames);

    std::string demandPredicateName;
    std::vector<Handle> outgoings;

    outgoings.clear();

    for ( boost::tokenizer<>::iterator iDemandName = demandNamesTok.begin();
          iDemandName != demandNamesTok.end();
          ++ iDemandName ) {

        demandPredicateName = (*iDemandName) + "DemandGoal";

        // Get EvaluationLink to the demand goal
        outgoings.clear();
        outgoings.push_back( oac->getAtomSpace().addNode(PREDICATE_NODE, demandPredicateName) );

        this->psi_demand_goal_list.push_back( oac->getAtomSpace().addLink(EVALUATION_LINK, outgoings) );

    }// for

    // Create an ReferenceLink holding all the demand goals (EvaluationLink)
    outgoings.clear();
    outgoings.push_back(oac->getAtomSpace().addNode(CONCEPT_NODE, "psi_demand_goal_list") );
    outgoings.push_back(oac->getAtomSpace().addLink(LIST_LINK, this->psi_demand_goal_list) );
    Handle referenceLink = AtomSpaceUtil::addLink(oac->getAtomSpace(), REFERENCE_LINK, outgoings, true);

    logger().debug("PsiActionSelectionAgent::%s - "
                   "Add the list of demand goals to AtomSpace: %s [cycle = %d]",
                   __FUNCTION__,
                   oac->getAtomSpace().atomAsString(referenceLink).c_str(), this->cycleCount
                  );


    std::cout<<"OCPlanningAgent finished init..."<<std::endl;
}


bool OCPlanningAgent::isMoveAction(string s)
{
    std::size_t pos1 = s.find("walk");
    std::size_t pos2 = s.find("jump_toward");
    if ( (pos1 != std::string::npos) || (pos2 != std::string::npos))
        return true;
    else
        return false;
}


void OCPlanningAgent::run()
{
    this->cycleCount = _cogserver.getCycleCount();

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(&_cogserver);
    OC_ASSERT(oac, "Did not get an OAC server");

    static int waitToStart =  0;

    if (!this->bInitialized)
        this->init();

    if (! startPlanning)
    {
        if ( ( oac->getPAI().hasFinishFistTimeWorldPercetption()) &&
             ( oac->getPsiDemandUpdaterAgent().get()->getHasPsiDemandUpdaterForTheFirstTime()))
        {
            // to wait for the PAI to finish more perception processing
            waitToStart ++;

            if (waitToStart > 10)
            {
                startPlanning = true;
                std::cout<<"OCPlanningAgent start!" <<std::endl;
            }
        }

        return;
    }

    logger().debug( "OCPlanningAgent::%s - Executing run %d times",
                     __FUNCTION__, this->cycleCount);

    // if the space map has not been set up yet, won't run the planner.
    if (! spaceServer().isLatestMapValid())
        return;


    if (this->currentOCPlanID != "") // Currently , there is one plan being executed
    {
        // check if current action failed
        if ( oac->getPAI().isActionFailed(this->currentOCPlanID, current_step))
        {
            std::cout<<"Current action "<< oac->getAtomSpace().atomAsString(this->current_action).c_str()
                     << " failed! [PlanId = "
                     <<this->currentOCPlanID<<", cycle = "<<this->cycleCount<<"] ... " <<std::endl;
        }
        else if ( oac->getPAI().isActionDone( this->currentOCPlanID, current_step) )
        {
            // the current action has been done successfully!
            std::cout<<"OCPlanningAgent::Action execution success! "<< oac->getAtomSpace().atomAsString(this->current_action).c_str()
                    << " [PlanId = " <<this->currentOCPlanID<<", cycle = "<<this->cycleCount<<"] ... " <<std::endl;

            // get next action in this plan
            if (this->current_actions.size() == (std::size_t)this->current_step)
            {
                // the current action is already the last action in this plan. so this plan is exectued successfully!
                std::cout<<std::endl<<"OCPlanningAgent::Action plan is executed successfully! Plan ID = "<< this->currentOCPlanID
                         <<std::endl<<"Selected Demand Goal [cycle = "<<this->cycleCount<<"]:"
                         <<std::endl<<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal)<<std::endl;

                // reset all the planning variables
                this->currentOCPlanID = "";
                this->current_action = Handle::UNDEFINED;
                this->current_actions.clear();
                this->current_step = -1;
            }
            else
            {
                // get the next action to execute it
                this->current_step ++;
                this->current_action = this->current_actions[this->current_step-1];
                this->timeStartCurrentAction = time(NULL);
            }
        }
        else
        {
            // the current action is still being executing...

            // check if current action timeout
            if ( time(NULL) - this->timeStartCurrentAction >  this->procedureExecutionTimeout )
            {
                    std::cout<<"current action timeout [PlanId = "
                            <<this->currentOCPlanID<<", cycle = "<<this->cycleCount<<"] ... "
                            <<std::endl;

                    // add 'actionFailed' predicates for timeout actions
                    oac->getPAI().setPendingActionPlansFailed();

                    // Now that this action failed, the following action sequence
                    // should be dropped.
                    this->current_actions.clear();
                    this->current_action = Handle::UNDEFINED;
                    this->currentOCPlanID = "";
                    this->current_step = -1;

              }
               // If the Action is still running and is not time out, simply returns
              else
              {
                    // check if current step , its previous step and its next step are all move to location, don't need to print out the message

                    if ( (this->current_step != 1) &&  ((std::size_t)this->current_step != this->current_actions.size()))
                    {
                        string cur_s = oac->getAtomSpace().atomAsString(current_actions[this->current_step-1]);
                        string pre_s = oac->getAtomSpace().atomAsString(current_actions[this->current_step-2]);
                        string next_s = oac->getAtomSpace().atomAsString(current_actions[this->current_step]);
                        if (isMoveAction(cur_s) && isMoveAction(pre_s) && isMoveAction(next_s))
                        {
                            return;
                        }
                    }

                    std::cout<<"current action "<< oac->getAtomSpace().atomAsString(this->current_action).c_str()
                             << " is still running [PlanId = "
                             <<this->currentOCPlanID<<", cycle = "<<this->cycleCount<<"] ... " <<std::endl;

                    return;
              }
        }
    }
    else
    {
        // currently, there is not any plan being executed.
        // run the planner to generate a new plan

#ifdef HAVE_GUILE
        // Select the most important demand for now, by scm
        SchemeEval* evaluator = new SchemeEval(&oac->getAtomSpace());
        std::string scheme_expression;

        scheme_expression = "( update_selected_demand_goal )";

        //scheme_expression = "( get_most_critical_demand_goal )";

        // Run the Procedure that do planning
        evaluator->eval(scheme_expression);

        if ( evaluator->eval_error() )
        {
            logger().error( "OCPlanningAgent::%s -getCurrentDemand() failed '%s'",
                             __FUNCTION__,
                             scheme_expression.c_str()
                          );

        }
        delete evaluator;
#endif // HAVE_GUILE

        hSelectedDemandGoal =  AtomSpaceUtil::getReference(oac->getAtomSpace(),
                                        oac->getAtomSpace().getHandle(CONCEPT_NODE,
                                                            "plan_selected_demand_goal"));

        // Update the pet's previously/ currently Demand Goal
        PsiRuleUtil::setCurrentDemandGoal(oac->getAtomSpace(), this->hSelectedDemandGoal);

        logger().debug( "OCPlanningAgent::%s - do planning for the Demand Goal: %s [cycle = %d]",
                        __FUNCTION__,
                        oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal).c_str(),
                        this->cycleCount
                      );

        std::cout<<"OCPlanner is doing planning for :"
                 <<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal).c_str()
                 <<std::endl;

        // the demand goal is something like "EnergyDemandGoal"
        currentOCPlanID = ocplanner->doPlanningForPsiDemandingGoal(this->hSelectedDemandGoal, &_cogserver);

        // if planning failed
        if (currentOCPlanID == "")
        {
            std::cout<<std::endl;
            std::cout<<"OCPlanner can not find any suitable plan for the selected demand goal:"
                     <<oac->getAtomSpace().atomAsString(this->hSelectedDemandGoal).c_str()
                     <<std::endl;
            std::cout<<std::endl;
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

            this->current_step = 1;
            this->current_action = current_actions[current_step - 1];
            this->timeStartCurrentAction = time(NULL);
        }
    }

}
