/*
 * OCPlanner.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * Written by Shujing KE
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

#ifndef _OCPANNER_H
#define _OCPANNER_H

#include <vector>
#include <set>
#include <map>
#include <list>
#include <string>
#include <boost/variant.hpp>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include <opencog/util/StringManipulator.h>
#include "PlanningHeaderFiles.h"
#include "Strips.h"



using namespace std;
using namespace opencog::pai;
using namespace boost;

// The Actions described in STRIPS format
// Including the preconditions,effects,parameters of the action

namespace opencog { namespace oac {

class StateNode;

struct UngroundedVariablesInAState
{
    State* state;
    set<string> vars; // all the ungrounded variables in this state
    bool contain_numeric_var;

    // the evalutionlink for this state for using pattern matching
    // default is undefined. only easy state that needs no real time inquery or contains no numeric variables will be generate such a link handle.
    Handle PMLink;
    UngroundedVariablesInAState(State* _state, string firstVar)
    {
        state = _state;
        vars.insert(firstVar);
        contain_numeric_var = opencog::oac::isAVariableNumeric(firstVar);
        PMLink = Handle::UNDEFINED;
    }

    bool operator < (const UngroundedVariablesInAState& other) const
    {
        // the state need inquery is more difficult to ground, so it should gound later
        if ( (! state->need_inquery) && (other.state->need_inquery))
             return true;

        if (( state->need_inquery) && (! other.state->need_inquery))
            return false;

        // the numeric state should be grounded later
        if ( (! contain_numeric_var) && (other.contain_numeric_var))
             return true;

        if (( contain_numeric_var) && (! other.contain_numeric_var))
            return false;

        // if both states need inquery or do not need inquery, the states with less  ungrounded variables should be grounded first
        return (vars.size() < other.vars.size());
    }
};


// a rule node in a Rule Layer planning graph
class RuleNode
{
public:
    Rule* originalRule; // the original rule (usually not grounded)

    int number; // the order of this rule node that has been created during the whole planning process

    ParamGroundedMapInARule currentBindingsFromForwardState; // the current bindings of variables grounded from its forward state

    // the current bindings of variables grounded from selecting variables from Atomspace or others
    // because after grounded from its forward state, tnere is possible some varibales ungrounded, so need to be grounded by selecting values
    ParamGroundedMapInARule currentBindingsViaSelecting;

    ParamGroundedMapInARule currentAllBindings; // currentBindingsFromForwardState + currentBindingsViaSelecting

    set<StateNode*> forwardLinks; // all state nodes in next layer connected to this nodes according to the currently applying rule: the effect state nodes of this rule
    set<StateNode*> backwardLinks; // all the links connect to the state nodes in last layer: the precondition state nodes of this rule

    // cost as soft heuristics inherited from its
    // these carry the heuristics information for the backward steps to select variable values
    // ungrounded, but the variable names should be consistent with its originalRule
    vector<CostHeuristic> costHeuristics;

    // the variables remain ungrounded after groundARuleNodeFromItsForwardState
    // should be in the order of grounding priority: who should be grounded frist, and who should be grounded secondly....
    // the list of State* is all the States this variable appears in, and this State* list should also be in the order of the priority that which state should be satisfied first
    list<UngroundedVariablesInAState> curUngroundedVariables;

//    // to store the history of variables groundings
//    vector<ParamGroundedMapInARule> ParamGroundedHistories;

    // to store the candidates of variables groundings. This is only for the curUngroundedVariables, excluding the varibales in currentBindingsFromForwardState
    vector<ParamGroundedMapInARule> ParamCandidates;

    // How many times of this rule has been tried with different variable bindings in this rule node.
    // This is for make a decision that if this rule has been tried and failed too many times, so that it would has a lower chance to be seleced
    int appliedTimes;

    // some times, we need to mark a rule as not useful anymore after trying it and fail.(e.g. have tried every variable binding and still fail)
    bool still_useful;

    RuleNode(Rule* _originalRule)
    {
        originalRule = _originalRule;
        costHeuristics.clear();
        still_useful = true;
    }

    void AddCostHeuristic(State* cost_cal_state,float cost_coefficient)
    {
        costHeuristics.push_back( CostHeuristic(cost_cal_state,cost_coefficient));
    }

    void AddCostHeuristic(CostHeuristic costHeuristic)
    {
        costHeuristics.push_back(costHeuristic);
    }

};

class StateNode
{
public:

    State* state;
    RuleNode* forwardRuleNode; // the forward rule node connect to this node in next rule layer
    RuleNode* backwardRuleNode; // the backward rule node connect to this node in last rule layer
    State* forwardEffectState; // the corresponding state in the forward rule's effect list
    int depth; // depth = -1 means no rule node need this state node as a precondition

    StateNode(State * _state){state = _state;isAchieved = UNKNOWN;forwardRuleNode = 0; forwardEffectState =0; hasFoundCandidateRules = false;depth = -1; stateNodeSatisfiedMe = 0;}

    // candidate rules to achieve this state, in the order of the priority to try the rule
    // the already be tried and failed rules will be removed from this list
    // the float is the score of this rule, the rules with higher score are put front
    list< pair<float,Rule*> > candidateRules;

    // the rules have been tried on this state node
    list< pair<float,Rule*> > ruleHistory;

    // this function need to be call after its forward rule node assigned, to calculate the depth of this state node
    // the root state node depth is 0, every state node's depth is its forward rule node's forward state node' depth +1
    // it its forward rule node has multiple forward state node, using the deepest one
    void calculateNodeDepth();

    // have tried to find candidateRules
    bool hasFoundCandidateRules;

    bool operator < (const StateNode& other) const
    {
        return ( depth < other.depth);
    }

    ~StateNode()
    {
        delete state;
    }

};


class OCPlanner
{
public:
     OCPlanner();

     // TODO:
     // add a new rule from a implicationLink in the Atomspace
     void addNewRuleByHandle(Handle implicationLinkHandle);

     // add a new rule
     void addNewRule(Rule& newRule);

     // the output plan:vector<PetAction>& plan, is a series of actions.
     // if failed in generating a plan to achieve the goal, return false.
     bool doPlanning(const vector<State*> &goal, vector<PetAction>& plan);

protected:

     // to store all the rules can be used in reasoning
     vector<Rule*> AllRules;

     // map <stateName, all rules have an effect to this state>
     // so that we can quickly find what rules have effect on a specific state during planning
     // map<float,Rule*> is map<probability, rule>
     map<string,map<float,Rule*> > ruleEffectIndexes;

     // add the indexes to ruleEffectIndexes, about which states this rule has effects on
     void addRuleEffectIndex(Rule* r);

     // load All Rules from the Atomspace
     void loadAllRulesFromAtomSpace();

     // for test, load from c++ codes
     void loadTestRulesFromCodes();

//     // to store the intermediate states which may be produced during planning stepps
//     // this vector should be clear every time begin a new plan
//     vector<State*> globalStatesCache;

     // check if a given single goal has been achieved now
     // @ curLayerStates is a state layer in the planning graph
     // @ satisfiedDegree is a return value between [0.0,1.0], which shows how many percentage has this goal been achieved
     // when it's a boolean goal, only can be 0.0 or 1.0
     // @ original_state is the corresponding begin state of this goal state, so that we can compare the current state to both fo the goal and origninal states
     //                  to calculate its satisfiedDegree value.
     // when original_state is not given (defaultly 0), then no satisfiedDegree is going to be calculated
     bool checkIsGoalAchieved(State &oneGoal, float& satisfiedDegree, State *original_state = 0);

     // ground the variables according to its forward state node,
     // by finding the variables in this rule and its forward state node with the same semantic meaning,
     // and put the value of the variables of the froward state to the rule variables.
     bool groundARuleNodeFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode);

     // Because it is possible to have multiple "this->DO_NOTHING_RULE" in the forward rule layers, so we need to find the first non-DO_NOTHING_RULE foward
     // Return the real forwardEffectState by the way
     RuleNode* findFirstRealForwardRuleNode(StateNode *stateNode, State *&forwardEffectState, StateNode* &mostForwardSameStateNode);

     // To ground all the variables  which has not been grounded by "groundARuleNodeFromItsForwardState"
     bool groundARuleNodeBySelectingValues(RuleNode* ruleNode);

     // select the most suitable vaule to ground a variable
     // the value selection criterions are according to these two parts:
     // 1. soft heuristics: the cost heuristics inherited from its parents
     // 2. if it's a recursive rule, borrow some hard constraints from the preconditions of its non-recursive rule which has the same effect with it,
     //    as hard  heuristics.
     // @ variableStr: the ungrounded variable's string representation (StateVariable::ParamValueToString)
     bool selectValueForAVariableToGroundARule(RuleNode* ruleNode, string variableStr);

     // to create the curUngroundedVariables list in a rule node
     // and the list is in the order of grounding priority (which variables should be gounded first, and for each variable which states should be satisfied first)
     void findAllUngroundedVariablesInARuleNode(RuleNode *ruleNode);


     void findCandidateValuesByGA(RuleNode* ruleNode);

     // delete a rule node and recursivly delete all its backward state nodes and rule nodes
     void deleteRuleNode(RuleNode* ruleNode);

     // rebind a state node, replace the old state in this node with the new state generated by new bindings
     void reBindStateNode(StateNode* stateNode, ParamGroundedMapInARule& newBindings);

};



}}

#endif
