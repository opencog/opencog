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
    Rule* curGroundedRule; // the currently applying grounded rule of the the original rule

    RuleLayer* ruleLayer; // the layer it belongs to
    set<StateLayerNode*> forwardLinks; // all nodes in next layer connected to this nodes according to the currently applying rule
    set<StateLayerNode*> backwardLinks; // all the links connect to the nodes in last layer

    RuleLayerNode(Rule* _originalRule)
    {
        originalRule = _originalRule;
    }

};


// map to save grounded values for one rule:
// map<parameter name, grounded value>
// e.g.: <$Entity0,Robot001>
//       <$Vector0,Vector(45,82,29)>
//       <$Entity1,Battery83483>
typedef map<string, StateValue> ParamGroundedMapInARule;

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
    set<RuleLayerNode*> forwardLinks; // all the links connect to the nodes in next layer,and the according rules applied
    set<RuleLayerNode*> backwardLinks; // all the links connect to the nodes in last layer
    StateLayerNode(State * _state){state = _state;isAchieved = UNKNOWN;}

    // history of all the rules with grounded parameters used to be applied in this layer (to generate the backward RuleLayerNode)
    // this is to prevent repeatedly applying the same set of rules with the same gournded parameter values.
    // map to save all the paramGroundedMapInARule for all the rules we applied in one rule layer at one time point
    // Rule* is pointing to one of the rules in the OCPlanner::AllRules
    map<Rule*, OneRuleHistory> ruleHistory;

    int getRuleAppliedTime(Rule* r)
    {
        map<Rule*, OneRuleHistory>::iterator it = ruleHistory.find(r);
        if (it == ruleHistory.end())
            return 0;

        return (OneRuleHistory)(*it).appliedTimes;

    }

    bool IsRuleStillUsefule(Rule* r)
    {
        map<Rule*, OneRuleHistory>::iterator it = ruleHistory.find(r);
        if (it == ruleHistory.end())
            return true;

        return (OneRuleHistory)(*it).still_useful;
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
            newStateNode->backwardLinks.clear();
            newStateNode->forwardLinks.clear();
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


};



}}

#endif
