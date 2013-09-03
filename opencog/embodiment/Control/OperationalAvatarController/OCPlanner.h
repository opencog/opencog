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
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionPlan.h>
#include <opencog/server/CogServer.h>

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

    // all state nodes connected to this nodes that have been satisfied by this rulenode according to the currently applying rule: the effect state nodes of this rule
    set<StateNode*> forwardLinks;

    // all state nodes connected to this nodes that are negatived by this rule node according to the currently applying rule: the effect state nodes of this rule
    set<StateNode*> negativeForwardLinks;

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

    vector<ParamValue> orginalGroundedParamValues; //to record the orginal state values in the effect before they takes effects, the order is the same in the effect list

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
        appliedTimes = 0;
    }

    void AddCostHeuristic(State* cost_cal_state,float cost_coefficient)
    {
        costHeuristics.push_back( CostHeuristic(cost_cal_state,cost_coefficient));
    }

    void AddCostHeuristic(CostHeuristic costHeuristic)
    {
        costHeuristics.push_back(costHeuristic);
    }

    // after rebinding , add currentBindingsFromForwardState and currentBindingsViaSelecting together to CurrentAllBindings
    void updateCurrentAllBindings();

    static int getDepthOfRuleNode(const RuleNode* r);

    bool operator < (const RuleNode& other) const
    {
        // so , the deeper rule node will be put front
        return ( RuleNode::getDepthOfRuleNode(this) > RuleNode::getDepthOfRuleNode(&other));
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

    StateNode(State * _state){state = _state;forwardRuleNode = 0; forwardEffectState =0; hasFoundCandidateRules = false;depth = -1;}

    // candidate rules to achieve this state, in the order of the priority to try the rule
    // the already be tried and failed rules will be removed from this list
    // the float is the score of this rule, the rules with higher score are put front
    list< pair<float,Rule*> > candidateRules;

    // the rules have been tried on this state node
    list< Rule*> ruleHistory;

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

struct TmpParamCandidate
{
   int satNum;
   ParamGroundedMapInARule aGroupOfParmCandidate;
   TmpParamCandidate(int _satNum, ParamGroundedMapInARule _aGroupOfParmCandidate):satNum(_satNum),aGroupOfParmCandidate(_aGroupOfParmCandidate){}

   bool operator < (const TmpParamCandidate& other) const
   {
       if (satNum < other.satNum)
           return true;
       else
           return false;
   }
};

class OCPlanner
{
public:
     OCPlanner(AtomSpace* _atomspace,string _selfID, string _selfType);

     ~OCPlanner();

     // TODO:
     // add a new rule from a implicationLink in the Atomspace
     void addNewRuleByHandle(Handle implicationLinkHandle);

     // add a new rule
     void addNewRule(Rule& newRule);

     // the output plan:vector<PetAction>& plan, is a series of actions.
     // if failed in generating a plan to achieve the goal, return false.
     ActionPlanID doPlanning(const vector<State*> &goal, const vector<State *> &knownStates, CogServer *server);

     ActionPlanID doPlanningForPsiDemandingGoal(Handle& goalHandle, opencog::CogServer *server);

protected:

     // to store all the rules can be used in reasoning
     vector<Rule*> AllRules;

     AtomSpace* atomSpace;

     ParamValue selfEntityParamValue;

     ActionPlanID planID;

     unsigned long curtimeStamp;

     // All the imaginary atoms put into the Atomspace during planning, which should be removed after planning
     HandleSeq imaginaryHandles;

     // map <stateName, all rules have an effect to this state>
     // so that we can quickly find what rules have effect on a specific state during planning
     // map<float,Rule*> is map<probability, rule>
     map<string,multimap<float,Rule*> > ruleEffectIndexes;

     list<StateNode*> unsatisfiedStateNodes; // all the state nodes that have not been satisfied till current planning step

     // all the state nodes that store the temporary state nodes generated by effect of some rule nodes,
     // all the new elements should be put_front , so the latest update of the same state will already put in front of the older history
     list<StateNode*> temporaryStateNodes;

     // to store all the rule node in current plan. it should be clear everytime begin new planning.
     // every new rule node will be insurt into this list, and it will be removed from the set if it's deleted
     // this list will be sorted after a planning finised according to the order of dependency relations
     list<RuleNode*> allRuleNodeInThisPlan;

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
     bool checkIsGoalAchievedInRealTime(State &oneGoal, float& satisfiedDegree, State *original_state = 0);

     // return how many states in the temporaryStateNodes will be Negatived by this rule
     // bool &negativeGoal return if this rule after grounded will negative this forward goal state
     int checkNegativeStateNumBythisRule(Rule* rule, StateNode* fowardState, bool &negativeGoal);

     bool groundARuleNodeParametersFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode);

     // ground the variables according to its forward state node,
     // by finding the variables in this rule and its forward state node with the same semantic meaning,
     // and put the value of the variables of the froward state to the rule variables.
     bool groundARuleNodeFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode);

     // To ground the non Numeric variables in this rule,  which has not been grounded by "groundARuleNodeFromItsForwardState"
     bool groundARuleNodeBySelectingNonNumericValues(RuleNode* ruleNode);

     // select the most suitable vaule to ground a numeric variable
     // this function should be called after groundARuleNodeBySelectingNonNumericValues
     // the currentbindings will be added new binding pairs if this function find good values for ungrounded variables
     bool selectValueForGroundingNumericState(Rule* rule, ParamGroundedMapInARule& currentbindings, RuleNode *ruleNod = 0);

     // select Best Numeric Value From Candidates by calculating the cost via the cost heuristics of this rule node
     // @ values: the candidate values
     // @ varName: the variable name
     ParamValue selectBestNumericValueFromCandidates(float basic_cost, vector<CostHeuristic>& costHeuristics, ParamGroundedMapInARule& currentbindings, string varName, vector<ParamValue>& values);

     // to create the curUngroundedVariables list in a rule node
     // and the list is in the order of grounding priority (which variables should be gounded first, and for each variable which states should be satisfied first)
     void findAllUngroundedVariablesInARuleNode(RuleNode *ruleNode);

     void findCandidateValuesByGA(RuleNode* ruleNode);

     void recordOrginalParamValuesAfterGroundARule(RuleNode* ruleNode);

     //  return if this same state is found in temporaryStateNodes
     // @ StateNode& *stateNode: the stateNode in temporaryStateNodes which satisfied or dissatisfied this goal
     // @ RuleNode* forwardRuleNode : the state's forward rule node
     // @ ifCheckSameRuleNode: if avoid finding the state node generate by same rule node
     bool findStateInTempStates(State& state, RuleNode *forwardRuleNode, StateNode *&stateNode, bool ifCheckSameRuleNode);

     // @ RuleNode* forwardRuleNode : the goal state's forward rule node
     // @ bool &found: return if this same state is found in temporaryStateNodes
     // @ StateNode& *stateNode: the stateNode in temporaryStateNodes which satisfied or dissatisfied this goal
     // @ ifCheckSameRuleNode: if avoid finding the state node generate by same rule node
     bool checkIfThisGoalIsSatisfiedByTempStates(State& goalState, bool &found, StateNode *&satstateNode,RuleNode *forwardRuleNode,bool ifCheckSameRuleNode);

     // delete a rule node and recursivly delete all its backward state nodes and rule nodes, given the forwardStateNode
     void deleteRuleNodeRecursively(RuleNode* ruleNode, StateNode* forwardStateNode = 0, bool deleteThisforwardStateNode = true);

     // rebind a state node, replace the old state in this node with the new state generated by new bindings
     void reBindStateNode(StateNode* stateNode, ParamGroundedMapInARule& newBindings);

     // execute the current rule action to change the imaginary SpaceMap if any action that involved changing space map
     void executeActionInImaginarySpaceMap(RuleNode* ruleNode,SpaceServer::SpaceMap* iSpaceMap);

     void undoActionInImaginarySpaceMap(RuleNode* ruleNode,SpaceServer::SpaceMap* iSpaceMap);

     void deleteStateNodeInTemporaryList(StateNode* stateNode);

     void deleteRuleNodeInAllRuleNodeList(RuleNode *ruleNode);

     // different rules usually have different variable names to describe a same variable, some times we need to unify these variables names
     // change the variable names in toBeUnifiedRule according to forwardState, return the unified rule
     // the forwardState can be a ungrounded state from other rule
     // if cannot unify it , return 0
     Rule* unifyRuleVariableName(Rule* toBeUnifiedRule, State* forwardState );

};



}}

#endif
