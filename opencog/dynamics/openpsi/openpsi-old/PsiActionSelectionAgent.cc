/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.cc
 *
 * @author Jinhua Chua <JinhuaChua@gmail.com>
 * @date 2011-12-28
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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

//#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/nlp/types/atom_types.h>
//#include <opencog/spacetime/atom_types.h>
//#include <opencog/spacetime/SpaceServer.h>
//#include <opencog/util/platform.h>
#include <opencog/util/random.h>
#include <opencog/server/CogServer.h>
#include <opencog/atoms/execution/Instantiator.h>
#include <opencog/util/Config.h>
#include <opencog/guile/SchemeEval.h>


#include "PsiActionSelectionAgent.h"
#include "PsiRuleUtil.h"

using namespace opencog;

//extern int currentDebugLevel;

PsiActionSelectionAgent::~PsiActionSelectionAgent()
{
    logger().debug("[PsiActionSelectionAgent] destructor");
}

PsiActionSelectionAgent::PsiActionSelectionAgent(CogServer& cs)
    : Agent(cs), atomspace(cs.getAtomSpace()),cycleCount(0)
{
    // Force the Agent initialize itself during its first cycle.
    this->forceInitNextCycle();
}

void PsiActionSelectionAgent::init()
{
    logger().debug("PsiActionSelectionAgent::%s - "
                   "Initializing the Agent [cycle = %d]",
                   __FUNCTION__, this->cycleCount);

    // Initialize the list of Demand Goals
    this->initDemandGoalList();

    // Initialize other members
    //this->currentSchemaId = 0;
    //this->procedureExecutionTimeout =
    //    config().get_long("PROCEDURE_EXECUTION_TIMEOUT");

    // Avoid initialize during next cycle
    this->bInitialized = true;
}

void PsiActionSelectionAgent::initDemandGoalList()
{
    logger().debug("PsiActionSelectionAgent::%s - "
                   "Initializing the list of Demand Goals (Final Goals) "
                   "[ cycle =%d ]",
                    __FUNCTION__, this->cycleCount);

    // Clear the old demand goal list
    this->psi_demand_goal_list.clear();

    // Get demand names from the configuration file
    std::string demandNames = config()["PSI_DEMANDS"];

    // Process Demands one by one
    boost::tokenizer<> demandNamesTok (demandNames);

    std::string demandPredicateName;
    HandleSeq outgoings;

    for ( boost::tokenizer<>::iterator iDemandName = demandNamesTok.begin();
          iDemandName != demandNamesTok.end();
          ++ iDemandName ) {

        demandPredicateName = (*iDemandName) + "DemandGoal";

        // Get EvaluationLink to the demand goal
        outgoings = { atomspace.add_node(PREDICATE_NODE, demandPredicateName) };

        this->psi_demand_goal_list.push_back(atomspace.add_link(EVALUATION_LINK,
                                                               outgoings));

    }// for

    // Create an ReferenceLink holding all the demand goals (EvaluationLink)
    outgoings = { atomspace.add_node(CONCEPT_NODE, "psi_demand_goal_list"),
                  atomspace.add_link(LIST_LINK, this->psi_demand_goal_list) };
    Handle referenceLink = AtomSpaceUtil::addLink(atomspace, REFERENCE_LINK,
                                                  outgoings, true);

    logger().debug("PsiActionSelectionAgent::%s - "
                   "Add the list of demand goals to AtomSpace: %s [cycle = %d]",
                   __FUNCTION__, atomspace.atom_as_string(referenceLink).c_str(),
                   this->cycleCount);
}

void PsiActionSelectionAgent::getActions(Handle hStep, HandleSeq & actions)
{
    opencog::Type atomType = atomspace.get_type(hStep);

    if (atomType == EXECUTION_LINK) {
        actions.insert(actions.begin(), hStep);
    }
    else if (atomType == AND_LINK ||
            atomType == SEQUENTIAL_AND_LINK) {
        for ( Handle hOutgoing : atomspace.get_outgoing(hStep) ) {
            this->getActions(hOutgoing, actions);
        }
    }
    else if (atomType == OR_LINK ) {
        const HandleSeq & outgoings = atomspace.get_outgoing(hStep);
        Handle hRandomSelected = rand_element(outgoings);
        this->getActions(hRandomSelected, actions);
    }
}

bool PsiActionSelectionAgent::getPlan()
{
    // Check the state of latest planning
    HandleSeq tempOutgoingSet =
        { atomspace.add_node(PREDICATE_NODE, "plan_success"),
          atomspace.add_link(LIST_LINK, HandleSeq()) };

    Handle hPlanSuccessEvaluationLink = atomspace.add_link(EVALUATION_LINK,
                                                          tempOutgoingSet);
    if ( atomspace.get_TV(hPlanSuccessEvaluationLink)->getMean() < 0.9 )
        return false;

    // Get the planning result
    Handle hSelectedDemandGoal =
        AtomSpaceUtil::getReference(atomspace,
                                    atomspace.get_handle(CONCEPT_NODE,
                                                        "plan_selected_demand_goal"
                                                       )
                                   );

    this->plan_selected_demand_goal = hSelectedDemandGoal;

    Handle hRuleList =
        AtomSpaceUtil::getReference(atomspace,
                                    atomspace.get_handle(CONCEPT_NODE,
                                                        "plan_rule_list"
                                                       )
                                   );

    this->plan_rule_list = atomspace.get_outgoing(hRuleList);

    Handle hContextList =
        AtomSpaceUtil::getReference(atomspace,
                                    atomspace.get_handle(CONCEPT_NODE,
                                                        "plan_context_list"
                                                       )
                                   );

    this->plan_context_list = atomspace.get_outgoing(hContextList);

    Handle hActionList =
        AtomSpaceUtil::getReference(atomspace,
                                    atomspace.get_handle(CONCEPT_NODE,
                                                        "plan_action_list"
                                                       )
                                   );

    this->plan_action_list = atomspace.get_outgoing(hActionList);
    this->temp_action_list = this->plan_action_list;

    return true;
}

void PsiActionSelectionAgent::printPlan()
{
    if(this->plan_selected_demand_goal == Handle::UNDEFINED)
        return;

    std::cout << std::endl << "Selected Demand Goal [cycle = "
              << this->cycleCount << "]:" << std::endl
              << atomspace.atom_as_string(this->plan_selected_demand_goal)
              << std::endl;

    int i = 1;

    for ( const Handle hAction : this->plan_action_list ) {
        if(hAction == Handle::UNDEFINED)
        {
            ++i;
            continue;
        }
        std::cout << std::endl << "Step No."<< i
                  << std::endl << atomspace.atom_as_string(hAction);

        ++i;
    }

    std::cout << std::endl << std::endl;
}

void PsiActionSelectionAgent::stimulateAtoms()
{
    for (Handle h : this->plan_rule_list)
        this->stimulateAtom(h, 10); // 10 is the same as noiseUnit in ImportanceUpdatingAgent

    for (Handle h : this->plan_context_list)
        this->stimulateAtom(h, 10);

    for (Handle h : this->plan_action_list)
        this->stimulateAtom(h, 10);

    logger().warn("PsiActionSelectionAgent::%s - Stimulate plan related atoms [cycle = %d]",
                   __FUNCTION__, this->cycleCount);
}

void PsiActionSelectionAgent::executeAction(Handle hActionExecutionOutputLink)
{
    Instantiator inst(&atomspace);
    inst.execute(hActionExecutionOutputLink);
}

void PsiActionSelectionAgent::run()
{
    this->cycleCount = _cogserver.getCycleCount();

    logger().debug("PsiActionSelectionAgent::%s - Executing run %d times",
                   __FUNCTION__, this->cycleCount);

    // Initialize the Mind Agent (demandGoalList etc)
    if ( !this->bInitialized )
        this->init();


#if HAVE_GUILE
    // If we've used up the current plan, do a new planning
    if ( this->temp_action_list.empty() && this->current_actions.empty() ) {
        // Initialize scheme evaluator
        SchemeEval evaluator1(&atomspace);
        std::string scheme_expression, scheme_return_value;

        // test: skip for some circles before beginning next planning
        // because there will be some results of the actions taken in
        // last plan are need to changed by other agent due
        static int count = 0;
        if (count++ < 4)
            return;
        count = 0;

        std::cout << "Doing planning ... " << std::endl;

        // @todo this could probably be reformulate the information
        // that do_planning updates are directly output, then call to
        // opencog/execute would return a Handle pointing the plan
        // information
        scheme_expression = "( do_planning )";

        // Run the Procedure that do planning
        scheme_return_value = evaluator1.eval(scheme_expression);

        if ( evaluator1.eval_error() ) {
            logger().error("PsiActionSelectionAgent::%s - Failed to execute '%s'",
                           __FUNCTION__,
                           scheme_expression.c_str());

            //return;
        }

        // Try to get the plan stored in AtomSpace
        if ( !this->getPlan() ) {
            logger().warn("PsiActionSelectionAgent::%s - "
                          "'do_planning' can not find any suitable plan "
                          "for the selected demand goal [cycle = %d]",
                          __FUNCTION__,
                          this->cycleCount);

            if(this->plan_selected_demand_goal == Handle::UNDEFINED)
                return;

            std::cout << "'do_planning' can not find any suitable "
                      << "plan for the selected demand goal:"
                      << atomspace.atom_as_string(this->plan_selected_demand_goal)
                      << ", [cycle = "
                      << this->cycleCount << "]." << std::endl;
            return;
        }

        // XXX: Should the stimulation be handled by the agent?
        //this->stimulateAtoms();

        // std::cout << std::endl
        //           << "Done (do_planning), scheme_return_value(Handle): "
        //           << scheme_return_value;
        // Handle hPlanning = Handle( atol(scheme_return_value.c_str()) );
        // this->getPlan(hPlanning);

        // Print the plan to the screen
        this->printPlan();

        // Update the pet's previously/ currently Demand Goal
        PsiRuleUtil::setCurrentDemandGoal(atomspace,
                                          this->plan_selected_demand_goal);

        if(this->plan_selected_demand_goal != Handle::UNDEFINED)
            logger().debug("PsiActionSelectionAgent::%s - "
                           "do planning for the Demand Goal: %s [cycle = %d]",
                           __FUNCTION__,
                           atomspace.atom_as_string(this->plan_selected_demand_goal).c_str(),
                           this->cycleCount);
    }
#endif // HAVE_GUILE

    // Get next action from current plan
    if ( !this->current_actions.empty() ) {
        this->current_action = this->current_actions.back();
        this->current_actions.pop_back();
    }
    else if ( !this->temp_action_list.empty() ) {
        this->getActions(this->temp_action_list.back(), this->current_actions);
        this->temp_action_list.pop_back();

        this->current_action = this->current_actions.back();
        this->current_actions.pop_back();
    }
    else {
        logger().debug("PsiActionSelectionAgent::%s - "
                       "Failed to get any actions from the planner. "
                       "Try planning next cycle [cycle = %d]",
                       __FUNCTION__, this->cycleCount);
        return;
    }

    // Execute current action
    this->executeAction(this->current_action);
    //this->executeAction(languageTool, procedureInterpreter, procedureRepository,
    //                    this->current_action);
    this->timeStartCurrentAction = time(NULL);

/**
    // TODO: The code snippets below show the idea of how the modulators
    // (or emotions) have impact on cognitive process, more specifically planning
    // here. We should implement the ideas in action_selection.scm later.

    // Figure out a plan for the selected Demand Goal
    int steps = 2000000;   // TODO: Emotional states shall have impact on steps, i.e., resource of cognitive process

    if (this->bPlanByPLN)
        this->planByPLN(server, selectedDemandGoal, this->psiPlanList, steps);
    else
        this->planByNaiveBreadthFirst(server, selectedDemandGoal,
                                      this->psiPlanList, steps);


    // Change the current Demand Goal randomly (controlled by the
    // modulator 'SelectionThreshold')
    float selectionThreshold =
        AtomSpaceUtil::getCurrentModulatorLevel(atomspace,
                                                SELECTION_THRESHOLD_MODULATOR_NAME);
    if ( randGen().randfloat() > selectionThreshold )
    {

        selectedDemandGoal = this->chooseRandomDemandGoal();

        if ( selectedDemandGoal == opencog::Handle::UNDEFINED ) {
            logger().warn("PsiActionSelectionAgent::%s - "
                          "Failed to randomly select a Demand Goal [cycle = %d]",
                          __FUNCTION__,
                          this->cycleCount);
            return;
        }

        Handle hCurrentDemandGoal, hReferenceLink;
        hCurrentDemandGoal =
            PsiRuleUtil::getCurrentDemandGoal(atomspace, hReferenceLink);

        if (selectedDemandGoal != hCurrentDemandGoal) {

            // Update the pet's previously/ currently Demand Goal
            PsiRuleUtil::setCurrentDemandGoal(atomspace, selectedDemandGoal);

            logger().debug("PsiActionSelectionAgent::%s - "
                           "Switch the Demand Goal to: %s [ cycle = %d ].",
                           __FUNCTION__,
                           atomspace.getName(atomspace.get_outgoing(selectedDemandGoal, 0)
                                             ).c_str(),
                           this->cycleCount);

            // Figure out a plan for the selected Demand Goal
            int steps = 2000000; // TODO: Emotional states shall have
                                 // impact on steps, i.e. resource of
                                 // cognitive process

            if (this->bPlanByPLN)
                this->planByPLN(server, selectedDemandGoal,
                                this->psiPlanList, steps);
            else
                this->planByNaiveBreadthFirst(server, selectedDemandGoal,
                                              this->psiPlanList, steps);
        }// if

    }// if
*/
}
