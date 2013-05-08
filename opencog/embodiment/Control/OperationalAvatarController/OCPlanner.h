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
#include <boost/variant.hpp>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionParameter.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include "PlanningHeaderFiles.h"
#include "Strips.h"

using namespace std;
using namespace opencog::pai;
using namespace boost;

// The Actions described in STRIPS format
// Including the preconditions,effects,parameters of the action

namespace opencog { namespace oac {

class RuleLayer;
class StateLayerNode;
class StateLayer;


// a rule node in a Rule Layer planning graph
class RuleLayerNode
{
public:
    Rule* originalRule; // the original rule (usually not grounded)

    ParamGroundedMapInARule currentBindings; // the current bindings of variables

    RuleLayer* ruleLayer; // the layer it belongs to
    set<StateLayerNode*> forwardLinks; // all state nodes in next layer connected to this nodes according to the currently applying rule: the effect state nodes of this rule
    set<StateLayerNode*> backwardLinks; // all the links connect to the state nodes in last layer: the precondition state nodes of this rule

    // cost as soft heuristics inherited from its
    // these carry the heuristics information for the backward steps to select variable values
    // ungrounded, but the variable names should be consistent with its originalRule
    vector<CostHeuristic> costHeuristics;

    RuleLayerNode(Rule* _originalRule)
    {
        originalRule = _originalRule;
        costHeuristics.clear();
    }

};

// the already tried variable bindings history for one rule for achieve one planning state
struct OneRuleHistory
{
    vector<ParamGroundedMapInARule> ParamGroundedHistories;

    // How many times of this rule has been tried with different variable binding.
    // This is for make a decision that if this rule has been tried and failed too many times, so that it would has a lower chance to be seleced
    int appliedTimes;

    // some times, we need to mark a rule as not useful anymore after try it and fail.(e.g. have tried every variable binding and still fail)
    bool still_useful;

    // the first time a new is tried , creat a history struct for it
    OneRuleHistory(ParamGroundedMapInARule firstBindingRecord)
    {
        appliedTimes = 1;
        still_useful = true;
        ParamGroundedHistories.push_back(firstBindingRecord);
    }
};


class StateLayerNode
{
public:
    enum ACHIEVE_STATE
    {
        ACHIEVED,
        NOT_ACHIEVED,
        UNKNOWN
    };

    State* state;
    ACHIEVE_STATE isAchieved;
    StateLayer* stateLayer; // the layer it belongs to
    RuleLayerNode* forwardRuleNode; // the forward rule node connect to this node in next rule layer
    RuleLayerNode* backwardRuleNode; // the backward rule node connect to this node in last rule layer
    StateLayerNode(State * _state){state = _state;isAchieved = UNKNOWN;forwardRuleNode = 0;}

    // history of all the rules with grounded parameters used to be applied in this layer (to generate the backward RuleLayerNode)
    // this is to prevent repeatedly applying the same set of rules with the same gournded parameter values.
    // map to save all the paramGroundedMapInARule for all the rules we applied in one rule layer at one time point
    // Rule* is pointing to one of the rules in the OCPlanner::AllRules
    map<Rule*, OneRuleHistory> ruleHistory;

    void addRuleRecordWithVariableBindingsToHistory(Rule* r,ParamGroundedMapInARule& paramGroundingMap)
    {
        map<Rule*, OneRuleHistory>::iterator it = ruleHistory.find(r);
        // this rule has not been applied on me before, add a new record before
        if (it == ruleHistory.end())
        {
            ruleHistory.insert(std::pair<Rule*, OneRuleHistory >(r, OneRuleHistory(paramGroundingMap)));
        }
        else
        {
            (((OneRuleHistory)(it->second)).ParamGroundedHistories).push_back(paramGroundingMap);
        }

    }

    // check if this rule with this specific variables bindings has been applied to this state node
    bool checkIfBindingsHaveBeenApplied(Rule* r,ParamGroundedMapInARule& paramGroundingMap)
    {
        map<Rule*, OneRuleHistory>::iterator it = ruleHistory.find(r);
        if (it == ruleHistory.end())
            return false;

        vector<ParamGroundedMapInARule> ParamGroundedHistories = (((OneRuleHistory)(it->second)).ParamGroundedHistories);
        vector<ParamGroundedMapInARule>::iterator paramMapit = ParamGroundedHistories.begin();
        for (;paramMapit != ParamGroundedHistories.end(); ++ paramMapit)
        {
            if (paramGroundingMap == (ParamGroundedMapInARule)(*paramMapit) )
                return true;
        }

        return false;

    }

    int getRuleAppliedTime(Rule* r)
    {
        map<Rule*, OneRuleHistory>::iterator it = ruleHistory.find(r);
        if (it == ruleHistory.end())
            return 0;

        return ((OneRuleHistory)(it->second)).appliedTimes;

    }

    bool IsRuleStillUsefule(Rule* r)
    {
        map<Rule*, OneRuleHistory>::iterator it = ruleHistory.find(r);
        if (it == ruleHistory.end())
            return true;

        return ((OneRuleHistory)(it->second)).still_useful;
    }

};


class StateLayer
{
public:
    set<StateLayerNode*> nodes; // all the nodes in this layer currently

    RuleLayer* preRuleLayer;
    RuleLayer* nextRuleLayer;

    StateLayer()
    {
        preRuleLayer = 0;
        nextRuleLayer = 0;
    }

    StateLayer(const vector<State*>& _states)
    {
        vector<State*>::const_iterator it;
        for (it = _states.begin(); it != _states.end(); ++ it)
        {
            StateLayerNode* newStateNode = new StateLayerNode(*it);
            nodes.insert(newStateNode);
            newStateNode->backwardRuleNode = 0;
            newStateNode->forwardRuleNode = 0;
        }

        StateLayer();
    }
};



class RuleLayer
{
public:
    set<RuleLayerNode*> nodes; // all the nodes in this layer currently

    StateLayer* preStateLayer;
    StateLayer* nextStateLayer;
    RuleLayer()
    {
        preStateLayer = 0;
        nextStateLayer = 0;
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

public:
     Rule* DO_NOTHING_RULE;

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

     // to store the orginal state values, avoid inquery in every planning step
     // this vector should be clear every time begin a new plan
     vector<State> originalStatesCache;

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
     bool groundARuleNodeFromItsForwardState(RuleLayerNode* ruleNode, StateLayerNode* forwardStateNode);


     // To ground all the variables  which has not been grounded by "groundARuleNodeFromItsForwardState"
     bool groundARuleNodeBySelectingValues(RuleLayerNode* ruleNode);

     // select the most suitable vaule to ground a variable
     // the value selection criterions are according to these two parts:
     // 1. soft heuristics: the cost heuristics inherited from its parents
     // 2. if it's a recursive rule, borrow some hard constraints from the preconditions of its non-recursive rule which has the same effect with it,
     //    as hard  heuristics.
     // @ variableStr: the ungrounded variable's string representation (StateVariable::ParamValueToString)
     bool selectValueForAVariableToGroundARule(RuleLayerNode* ruleNode, string variableStr);


};



}}

#endif
