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

    RuleLayer* actionLayer; // the layer it belongs to
    set<StateLayerNode*> forwardLinks; // all nodes in next layer connected to this nodes according to the currently applying rule
    set<StateLayerNode*> backwardLinks; // all the links connect to the nodes in last layer

    // all the different grounded parameters for this rules which have been tried are put int to this vector, to avoid try them again
    //vector<> histroyParamValues;

    RuleLayerNode(Rule* _originalRule)
    {
        originalRule = _originalRule;
    }

};

class StateLayerNode
{
public:
    State* state;
    bool isAchieved;
    StateLayer* stateLayer; // the layer it belongs to
    set<RuleLayerNode*> forwardLinks; // all the links connect to the nodes in next layer,and the according rules applied
    set<RuleLayerNode*> backwardLinks; // all the links connect to the nodes in last layer
    StateLayerNode(State * _state){state = _state;isAchieved = false;}
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
    set<Rule*> forwardHistory; // to store all the rules has been applied in this layer
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

protected:

     // to store all the rules can be used in reasoning
     vector<Rule*> AllRules;

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
