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

#define ACCESS_DISTANCE "2.1"
#define CLOSED_DISTANCE "0.9"

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
     bool doPlanning(vector<State> &goal, vector<PetAction>& plan);

protected:

     // to store all the rules can be used in reasoning
     vector<Rule*> AllRules;

     // load All Rules from the Atomspace
     void loadAllRulesFromAtomSpace();

     // for test, load from c++ codes
     void loadTestRulesFromCodes();

     // check if a given single goal has been achieved now
     // @ curLayerStates is a state layer in the planning graph
     // @ satisfiedDegree is a return value between [0.0,1.0], which shows how many percentage has this goal been achieved
     // when it's a boolean goal, only can be 0.0 or 1.0
     // @ original_state is the corresponding begin state of this goal state, so that we can compare the current state to both fo the goal and origninal states
     //                  to calculate its satisfiedDegree value.
     // when original_state is not given (defaultly 0), then no satisfiedDegree is going to be calculated
     bool checkIsGoalAchieved(const State &oneGoal, const vector<State> &curLayerStates, float& satisfiedDegree, const State *original_state = 0);

public:
     // define the variables for rules
     // we have 6 kinds of typedef variant<string, Rotation, Vector, Entity, fuzzyInterval, fuzzyIntFloatInterval > StateValue
     /*
     BOOLEAN_CODE,
     INT_CODE,
     FLOAT_CODE,
     STRING_CODE,
     VECTOR_CODE,
     ROTATION_CODE,
     ENTITY_CODE,
     FUZZY_INTERVAL_INT_CODE,
     FUZZY_INTERVAL_FLOAT_CODE,*/

     static const StateValue access_distance;
     static const StateValue SV_TRUE;
     static const StateValue SV_FALSE;

     static const string bool_var[7];
     static const string str_var[7];
     static const string int_var[7];
     static const string float_var[7];
     static const Vector vector_var[7];
     static const Entity entity_var[7];

};

}}

#endif
