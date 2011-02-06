/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-02-04
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

#include <opencog/reasoning/pln/PLN.h>
#include <opencog/reasoning/pln/PLNUtils.h>
#include <opencog/reasoning/pln/rules/Rules.h>
#include <opencog/reasoning/pln/rules/RuleProvider.h>
#include <opencog/reasoning/pln/AtomSpaceWrapper.h>
#include <opencog/reasoning/pln/BackInferenceTreeNode.h>

class PsiActionSelectionAgentUTest; 

namespace OperationalAvatarController
{

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
 *       ImplicationLink is used instead of PredictiveImplicationLink, since PLN doesn't support it while implementation
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
 * For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.  
*/

class PsiActionSelectionAgent : public opencog::Agent
{
    friend class::PsiActionSelectionAgentUTest;

private:

    unsigned long cycleCount;  // Indicate this mind agent has been executed how many times

    // Indicate whether the mind agent has been initialized, 
    // then the 'run' method will not initialize it over and over, 
    // unless you call 'forceInitNextCycle' method. 
    bool bInitialized; 

    // Initialize demandGoalList etc.
    void init(opencog::CogServer * server);

    std::vector<Handle> demandGoalList;  // A list of Demand Goals, also known as Final Goals
    Handle currentDemandGoal;            // Current selected Demand Goal  
    Handle previousDemandGoal;           // Previous selected Demand Goal 

    // Each list is considered as a plan, which contains a bunch of Psi Rules 
    // leading to the currently selected Demand Goal. 
    //
    // TODO: Why there are multiply plans for a single Demand Goal? 
    //
    //       It originated from the fact that PLN may return multiply results (trees) for a single target Atom.
    //
    //       We shall make it more generic later, then the virtual pet can either
    //       jump among different Demand Goals or 
    //       stick to a Demand Goal, but jump among different plans. 
    //
    //       Anyway have a plan B is not so bad. 
    std::vector< std::vector<Handle> > psiRulesLists;

    Handle currentPsiRule;   // Handle to current applied Psi Rule
    Handle previousPsiRule;  // Handle to previous applied Psi Rule

    Procedure::RunningProcedureID currentSchemaId;  // Scheme Id of current running Action (a combo script)

    // Time out for executing Action (combo script) defined by PROCEDURE_EXECUTION_TIMEOUT
    long procedureExecutionTimeout; 

    time_t timeStartCurrentPsiRule; // When the current Psi rule was applied 

    // Boost smart pointer to BITNodeRoot, used by PLN
    // 
    // Examples of using PLN (within "PLNModule.cc"):
    //
    // line 209: Scheme wrapper for PLN, can set Target, but can not return the Tree
    // line 427: CogServer shell for PLN, can not set Target
    // line 463: CogServer shell for PLN, can return the Tree
    //
    Btr<BITNodeRoot> Bstate;

    // Initialize the list of Demand Goals
    void initDemandGoalList(opencog::CogServer * server);
 
    // Return a Demand Goal randomly
    Handle chooseRandomDemandGoal();
 
    // Return the Demand Goal with minimum truth value
    Handle chooseMostCriticalDemandGoal(opencog::CogServer * server); 

    /**
     * Do backward inference given the Goal Handle and maximum steps. 
     *
     * @param  goalHandle  Handle to the selected Goal
     * @param  steps       Maximum steps used by PLN. Modulators may affect this value.
     * @return  The tree that would be used to extract Psi rules which lead to the Goal
     * @note    The returned set of trees may be empty.
    */
    const std::set<VtreeProvider *> & searchBackward(Handle goalHandle, int & steps);

    /**
     * Extract Psi rules from the searching tree returned by searchBackward method
     *
     * @param  server 
     * @param  inferResult   Trees returned by 'searchBackward' method.
     * @param  psiRulesList  Store the extracted Psi Rules  
     * @return  true if success
     *
     * @note  It breadth-first travels the tree returned by PLN from the root and stores the Psi Rule encountering.
     *        This implementation is not very correct in principle, only used for simple demos. 
     *        Maybe we shall incorporate TGP to Action Selection (TODO). 
     */
    bool extractPsiRules(opencog::CogServer * server, 
                         const std::set<VtreeProvider *> & inferResult,
                         std::vector< std::vector<Handle> > & psiRulesList);

    /**
     * Helper function used by extractPsiRules method above. You should never call this method directly. 
     */
    void extractPsiRules(opencog::CogServer * server, pHandle ph, std::vector<Handle> & psiRules, unsigned int level);

    /**
     * Check if the given Handle is a Psi Rule. It is used by extractPsiRules. 
     *
     * @return  true if the given Handle indicating a Psi Rule
     * @note  For the format of Psi Rules, please refer to "./opencog/embodiment/rules_core.scm"
     */
    bool isHandleToPsiRule(opencog::CogServer * server, Handle h);

    /**
     * Print Actions for each Plan, only used for debugging
     */
    void printPlans(opencog::CogServer * server, Handle hDemandGoal,
                    const std::vector< std::vector<Handle> > & psiRulesLists);

    /**
     * Transfer the format of arguments within given ListLink to combo. 
     * 
     * @param  server
     * @parem  hListLink  Handle to ListLink that contains arguments
     * @param  schemaArguments  Return the arguments that would be used while executing the combo procedure
     * @return  The number of arguments got
     */
    int getSchemaArguments(opencog::CogServer * server, Handle hListLink, 
                           std::vector <combo::vertex> & schemaArguments);

    /**
     * Check if the given Precondition is satisfied. It is used by 'pickUpPsiRule' method.
     *
     * @param  server
     * @param  hPrecondition  Handle to the Precondition, i.e. an EvaluationLink
     * @return  true if the given Precondition is satisfied, otherwise returns false
     *
     * @note  If the EvaluationLink(hPrecondition)) contains a PredicateNode, 
     *        then we simply check the truth value of the EvaluationLink. 
     *        If the EvaluationLink holds a GroundedPredicateNode, 
     *        we would run the corresponding combo procedure firstly, and then judge based on the execution result. 
     */
    bool isSatisfied(opencog::CogServer * server, Handle hPrecondition);

    /**
     * Pick up a Psi Rule that all its Preconditions are satisfied. 
     *
     * @param  server
     * @param  psiRules
     * @returns The handle to current selected Rule. Or opencog::Handle::UNDEFINED if Failed. 
     */
    Handle pickUpPsiRule(opencog::CogServer * server, const std::vector<Handle> & psiRules);

    /**
     * Apply the given Psi Rule. 
     *
     * @return  true if success. 
     *
     * @note  Since each Psi Rule associates with a single Action, 
     *        applying a Psi Rule equals executing the corresponding Action.
     *
     *        While this method simply runs the Action, it will not get and analyze the result of the execution, 
     *        because it may take some time to finish the execution. 
     *
     *        'run' method is responsible for dealing with the result of execution during next "cognitive cycle" 
     *        after calling the 'applyPsiRule' method. 
     */
    bool applyPsiRule(opencog::CogServer * server, Handle hPsiRule);


public:

    PsiActionSelectionAgent();
    virtual ~PsiActionSelectionAgent();

    virtual const ClassInfo& classinfo() const {
        return info();
    }

    static const ClassInfo& info() {
        static const ClassInfo _ci("OperationalAvatarController::PsiActionSelectionAgent");
        return _ci;
    }

    // Entry of the Agent, CogServer will invoke this function during its cycle
    void run(opencog::CogServer * server);

    // After calling this function, the Agent will invoke its "init" method firstly 
    // in "run" function during its next cycle
    void forceInitNextCycle() {
        this->bInitialized = false;
    }

}; // class

}  // namespace

#endif
