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
     bool doPlanning(vector<State> goal, vector<PetAction>& plan);

     // to store the planning time state value
     // like during reasoning, if robotA moves to apple1, then the robotA is closed to apple1, but it is just a virtual state change,
     // not really changed in the world. In the world, the robotA is still not closed to apple1 because it has not really perform moving to apple1
     // but we have to store such virtual changing states for multi-step planning
     static set<State*> virtualStates;

protected:

     // to store all the rules can be used in reasoning
     vector<Rule*> AllRules;

     // load All Rules from the Atomspace
     void loadAllRulesFromAtomSpace();

     // for test, load from c++ codes
     void loadTestRulesFromCodes();

     // check if the given goal has been achieved now
     bool checkIsGoalAchieved(vector<State> goal);

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
