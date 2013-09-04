/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.h
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
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef PSIACTIONSELECTIONAGENT_H
#define PSIACTIONSELECTIONAGENT_H

#include <opencog/server/Agent.h>
#include <opencog/atomspace/AtomSpace.h>

class PsiActionSelectionAgentUTest; 

namespace opencog { namespace oac {

/**
 * @class
 *
 * @brief Action Selection Agent
 *
 * Ideally using PLN, looking for, or inferring cognitive schematics and choosing
 * the one according to their weights (calculating the weight can depend on the
 * Modulator Certainty).
 * 
 * Each Rule here means a cognitive schematic, that is 
 *     Contex & Procedure ==> Goal
 *
 * Rules here has nothing to do with that in PLN, so don't confuse them!
 *
 * An OpenPsi Rule is represented in AtomSpace as below: 
 *
 * Note: AtTimeLink is missing currently
 *       ImplicationLink is used instead of PredictiveImplicationLink, since
 *       PLN doesn't support it while implementation
 *
 *     PredictiveImplicationLink
 *         AndLink
 *             AndLink
 *                 EvaluationLink
 *                     GroundedPredicateNode "precondition_1_name"
 *                     ListLink
 *                         Node:arguments
 *                         ...
 *                 EvaluationLink
 *                     PredicateNode         "precondition_2_name"
 *                     ListLink
 *                         Node:arguments
 *                         ...
 *                 ...
 *                            
 *             ExecutionLink
 *                 GroundedSchemaNode "schema_name"
 *                 ListLink
 *                     Node:arguments
 *                     ...
 *    
 *         EvaluationLink
 *             (SimpleTruthValue indicates how well the demand is satisfied)
 *             (ShortTermInportance indicates the urgency of the demand)
 *             PredicateNode: "goal_name" 
 *             ListLink
 *                 Node:arguments
 *                 ...
 *
 * For each Rule, there's only a Goal, an Action and a bunch of Preconditions. 
 * And all these Preconditions should be grouped in an AndLink.
 * If you want to use OrLink, then just split the Rule into several Rules.
 * For the efficiency and simplicity of the planner (backward chaining),
 * NotLink is forbidden currently.  
*/

class PsiActionSelectionAgent : public opencog::Agent
{
    friend class::PsiActionSelectionAgentUTest;

private:

    unsigned long cycleCount;  // Indicate how many times this mind
                               // agent has been executed

    // Indicate whether the mind agent has been initialized, 
    // then the 'run' method will not initialize it over and over, 
    // unless you call 'forceInitNextCycle' method. 
    bool bInitialized; 

    Procedure::RunningProcedureID currentSchemaId;  // Scheme Id of current running Action (a combo script)

    // Time out for executing Action (combo script) defined by PROCEDURE_EXECUTION_TIMEOUT
    long procedureExecutionTimeout; 

    time_t timeStartCurrentAction; // When the current action was executed 

    std::vector<Handle> psi_demand_goal_list; // Handles to all the demand goals (EvaluationLink)

    // Planning result
    Handle plan_selected_demand_goal; 
    std::vector<Handle> plan_rule_list; 
    std::vector<Handle> plan_context_list; 
    std::vector<Handle> plan_action_list;

    // A copy of plan_action_list, each time pop up and execute one of the action
    std::vector<Handle> temp_action_list;     

    // Each action(or step) in plan_action_list (or temp_action_list) may actually 
    // contains multiple actions. For example in plan_action_list there might
    // be an SequentialAndLink containing goto_obj(food) and eat(food) actions. 
    // However, OAC can not do both actions all at once. So in planner side both
    // actions can be considered as just one action, while in OAC side they are 
    // two actions. 
    //
    // Currently our solution is that firstly we copy both actions to current_actions,
    // then make OAC execute actions in current_actions one by one. When all the
    // actions in current_actions have been done, we ask temp_action_list for more 
    // actions. 
    std::vector<Handle> current_actions; 
    Handle current_action; 

    // Return actions given one step in plan_action_list (or temp_action_list)
    void getActions(AtomSpace & atomSpace, Handle hStep, std::vector<Handle> & actions); 

    // Initialize demandGoalList etc.
    void init();

    // Add the list of demand goals in AtomSpace
    //
    // It firstly reads the names of demands in config file (PSI_DEMANDS) and 
    // creates a ReferenceLink as follows
    //
    // ReferenceLink
    //     ConceptNode "plan_demand_list"
    //     ListLink
    //         EvaluationLink
    //             PredicateNode "xxxDemandGoal"
    //             ListLink
    //         EvaluationLink
    //             PredicateNode "xxxDemandGoal"
    //             ListLink
    //         ...
    //
    void initDemandGoalList(AtomSpace & atomSpace);

    /**
     * Get the plan stored in AtomSpace
     *
     * @return true if get the plan successfully, false otherwise 
     */
    bool getPlan(AtomSpace & atomSpace);

    /**
     * Print the plan to the screen, only used for debugging
     */
    void printPlan(AtomSpace & atomSpace);

    /**
     * Stimulate atoms related to the plan, ImportanceUpdatingAgent will then 
     * use these stimulates to renew corresponding attention values. 
     */
    void stimulateAtoms(); 

    /**
     * Execute the given action
     *
     * @note It also handles SpeechActSchemas and generate 'say' actions 
     *       automatically if the agent has something to say. That is there's 
     *       SentenceNodes in the structure below: 
     *
     * ReferenceLink
     *     UtteranceNode "utterance_sentences"
     *     ListLink
     *         SentenceNode ...
     *         ...
     */
    void executeAction(AtomSpace & atomSpace, 
                       LanguageComprehension & languageTool,
                       Procedure::ProcedureInterpreter & procedureInterpreter, 
                       const Procedure::ProcedureRepository & procedureRepository, 
                       Handle hActionExecutionLink); 

public:

    PsiActionSelectionAgent(CogServer&);
    virtual ~PsiActionSelectionAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiActionSelectionAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    virtual void run();

    // After calling this function, the Agent will invoke its "init" method firstly 
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

} } // namespace opencog::oac

#endif
