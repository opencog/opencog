/*
 * @file opencog/embodiment/Control/OperationalAvatarController/PsiActionSelectionAgent.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * @author Zhenhua Cai <czhedu@gmail.com>
 * @date 2011-01-25
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
 * An OpenPsi Rule is represented in AtomSpace as below (AtTimeLink is missing currently): 
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

    unsigned long cycleCount;

    // Initialize demandGoalList etc.
    void init(opencog::CogServer * server);

    // Run updaters (combo scripts)
    void runUpdaters(opencog::CogServer * server);

    // Set updated values to AtomSpace (NumberNodes)
    void setUpdatedValues(opencog::CogServer * server);

    bool bInitialized; 

    AtomSpace * atomSpace;

    const AtomSpace & getAtomSpace() {
        return * atomSpace; 
    }

    // A list of Demand Goals, also known as Final Goals
    std::vector<Handle> demandGoalList;

    // Each list contains a bunch of Psi Rules that would lead to the selected Demand Goal
    std::vector< std::vector<Handle> > psiRulesLists;

    // Handles to current and previous Psi Rule
    Handle currentPsiRule;
    Handle previousPsiRule;

    // Initialize the list of Demand Goals
    void initDemandGoalList(opencog::CogServer * server);
 
    // Return a Demand Goal randomly
    Handle chooseRandomDemandGoal();
 
    // Return the Demand Goal with minimum truth value
    Handle chooseMostCriticalDemandGoal(const AtomSpace & atomSpace); 

    /**
     * PLNModule.cc
     *
     * line 209: Scheme wrapper for PLN, can set Target, but can not return the Tree
     * line 427: CogServer shell for PLN, can not set Target
     * line 463: CogServer shell for PLN, can return the Tree
     *
     */   
    Btr<BITNodeRoot> Bstate;

    // Do backward inference given the Goal Handle and maximum steps. 
    //
    // Return the tree that would be used to extract Psi rules which lead to the Goal.
    // Note the returned set of trees may be empty.
    const std::set<VtreeProvider *> & searchBackward(Handle goalHandle, int & steps);

    // Extract Psi rules from the searching tree returned by searchBackward method
    bool extractPsiRules(const std::set<VtreeProvider *> & inferResult,
                         std::vector< std::vector<Handle> > & psiRulesList);

    // Helper function used by extractPsiRules method above
    void extractPsiRules(pHandle ph, std::vector<Handle> & psiRules, unsigned int level);

    // Return true if the given Handle indicating a Psi Rule
    // For the format of Psi Rules, please refer to "./opencog/embodiment/rules_core.scm"
    bool isHandleToPsiRule(Handle h);

    // Print Actions for each Plan, only used for debugging
    void printPlans(const std::vector< std::vector<Handle> > & psiRulesLists);

    // Get the arguments from the given ListLink, returns the number of arguments got
    int getSchemaArguments(opencog::CogServer * server, Handle hListLink, 
                           std::vector <combo::vertex> & schemaArguments);

    // Returns true if the given Precondition is satisfied, otherwise returns false
    bool isSatisfied(opencog::CogServer * server, Handle hPrecondition);

    // Pick up a Psi Rule, returns the current selected Rule
    Handle pickUpPsiRule(opencog::CogServer * server, const std::vector<Handle> & psiRules);

    // Apply the given Psi Rule
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
