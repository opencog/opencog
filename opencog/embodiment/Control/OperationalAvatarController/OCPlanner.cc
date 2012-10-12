/*
 * OCPlanner.cc
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

#include "OCPlanner.h"
#include <opencog/util/oc_assert.h>
#include "Inquery.h"
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h>
#include <iterator>

using namespace opencog::oac;

const StateValue OCPlanner::access_distance = ACCESS_DISTANCE;
const StateValue OCPlanner::SV_TRUE = "true";
const StateValue OCPlanner::SV_FALSE = "false";

 const string OCPlanner::bool_var[] = { "$bool_var0","$bool_var1", "$bool_var2","$bool_var3", "$bool_var4","$bool_var5","$bool_var6"};
 const  string  OCPlanner::str_var[] = { "$str_var0","$str_var1", "$str_var2","$str_var3", "$str_var4","$str_var5","$str_var6" };
 const  string  OCPlanner::int_var[] = { "$int_var0","$int_var1", "$int_var2", "$int_var3", "$int_var4","$int_var5","$int_var6"};
 const  string  OCPlanner::float_var[] = { "$float_var0","$float_var1","$float_var2","$float_var3","$float_var4","$float_var5","$float_var6"};
 const  Vector OCPlanner::vector_var[] = {
    Vector(999999.00000,999999.00000,999999.00000),
    Vector(999999.01000,999999.01000,999999.01000),
    Vector(999999.02000,999999.02000,999999.02000),
    Vector(999999.03000,999999.03000,999999.03000),
    Vector(999999.04000,999999.04000,999999.04000),
    Vector(999999.05000,999999.05000,999999.05000),
    Vector(999999.06000,999999.06000,999999.06000) };

 const  Entity OCPlanner::entity_var[] = {
   Entity("&entity_var0", "undefined"),
   Entity("&entity_var1", "undefined"),
   Entity("&entity_var2", "undefined"),
   Entity("&entity_var3", "undefined"),
   Entity("&entity_var4", "undefined"),
   Entity("&entity_var5", "undefined"),
   Entity("&entity_var6", "undefined"),
   };

OCPlanner::OCPlanner()
{
    loadAllRulesFromAtomSpace();

    // currently, for experiment, we dirctly add the rules by C++ codes
    loadTestRulesFromCodes();
}

//TODO: need to load from Atomspace
void OCPlanner::loadAllRulesFromAtomSpace()
{

}

// TODO: add a new rule from a implicationLink in the Atomspace
void OCPlanner::addNewRuleByHandle(Handle implicationLinkHandle)
{

}

// TODO: add a new rule
void OCPlanner::addNewRule(Rule& newRule)
{

}

bool OCPlanner::checkIsGoalAchieved(vector<State> goal)
{
    vector<State>::const_iterator it = goal.begin();
    for (;it != goal.end(); it ++)
    {
        State oneGoal = (State)*it;

        // First search this state in the virtualStates
        set<State*>::const_iterator vit = virtualStates.begin();
        for (;vit != virtualStates.end(); vit ++)
        {
            State* vState = (State*)(*vit);


        }
    }
}

bool OCPlanner::doPlanning(vector<State> goal, vector<PetAction> &plan)
{

}

// a bunch of rules for test, load from c++ codes
void OCPlanner::loadTestRulesFromCodes()
{
    //----------------------------Begin Rule: eat food to increase energy-------------------------------------------
    // define variables:
    StateValue var_food = entity_var[0];
    StateValue var_avatar = entity_var[1];
    StateValue var_energy = float_var[0];
    // Add rule: increasing energy by eat an edible object held in hand

    // precondition 1:food exists
    vector<StateValue> existStateOwnerList;
    existStateOwnerList.push_back(var_food);

    State* existState = new State("exist",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,SV_TRUE, existStateOwnerList, true, &opencog::oac::Inquery::inqueryExist);

    // precondition 2: The agent hold an object
    vector<StateValue> holderStateOwnerList;
    holderStateOwnerList.push_back(var_food);

    State* holderState = new State("holder",StateValuleType::ENTITY(),STATE_EQUAL_TO ,var_avatar, holderStateOwnerList);

    // precondition 3: This object is ediable
    vector<StateValue> edibleStateOwnerList;
    edibleStateOwnerList.push_back(var_food);
    State* edibleState = new State("is_edible",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,SV_TRUE, edibleStateOwnerList);

    // action: eat
    PetAction* eatAction = new PetAction(ActionType::EAT());
    eatAction->addParameter(ActionParameter("target",
                                        ActionParamType::ENTITY(),
                                        var_avatar));

    // energy state:
    vector<StateValue> energyStateOwnerList;
    energyStateOwnerList.push_back(var_avatar);
    State* energyState = new State("Energy",StateValuleType::FLOAT(),STATE_EQUAL_TO ,var_energy, energyStateOwnerList, true, Inquery::inqueryEnergy);

    // effect1: energy increases
    Effect* energyIncreaseEffect = new Effect(energyState, OP_ADD, StateValue("0.55"));

    // effect2: the food disappear
    Effect* foodDisappearEffect = new Effect(existState, OP_ASSIGN, "false");

    // effect3: no one holds the food any more
    Effect* nonHolderEffect = new Effect(holderState, OP_ASSIGN, Entity::NON_Entity);

    // rule: increasing energy by eat an edible object held in hand
    Rule* eatRule = new Rule(eatAction,boost::get<Entity>(var_avatar),1);
    eatRule->addPrecondition(existState);
    eatRule->addPrecondition(holderState);
    eatRule->addPrecondition(edibleState);

    eatRule->addEffect(EffectPair(1.0f,energyIncreaseEffect));
    eatRule->addEffect(EffectPair(1.0f,nonHolderEffect));
    eatRule->addEffect(EffectPair(1.0f,foodDisappearEffect));

    this->AllRules.push_back(eatRule);

    //----------------------------End Rule: eat food to increase energy-------------------------------------------

    //----------------------------Begin Rule: pick up an object to hold it if closed enough-------------------------------
    // define variables:
    StateValue varAvatar = entity_var[0];
    StateValue varFood = entity_var[1];
    StateValue var_holder = entity_var[2];


    // precondition 1: The agent and the object is closed enough ( < 2.0)
    vector<StateValue> closedStateOwnerList;
    closedStateOwnerList.push_back(varAvatar);
    closedStateOwnerList.push_back(varFood);
    StateValue svtest = ACCESS_DISTANCE;
    State* closedState = new State("Distance",StateValuleType::FLOAT(),STATE_LESS_THAN ,svtest, closedStateOwnerList, true, &Inquery::inqueryDistance);

    // precondition 2: The object can be picked up
    vector<StateValue> pickupableStateOwnerList;
    pickupableStateOwnerList.push_back(varFood);
    State* pickupableState = new State("is_pickupable",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,"true", pickupableStateOwnerList);

    // todo: precondition 3: The agent doesn't hold other object currently
    // todo: precondition 4: The object is not be held by other agent currently

    // action: pick up
    PetAction* pickupAction = new PetAction(ActionType::GRAB());
    pickupAction->addParameter(ActionParameter("target",
                                        ActionParamType::ENTITY(),
                                        varFood));

    // effect1: The agent hold this object
    // holder state
    vector<StateValue> holderStateOwnerList2;
    holderStateOwnerList2.push_back(varFood);
    State* holderState2 = new State("holder",StateValuleType::ENTITY(),STATE_EQUAL_TO ,var_holder, holderStateOwnerList2);
    Effect* holderEffect = new Effect(holderState2, OP_ASSIGN, varAvatar);

    // rule:  pick up an object if closed enough, to hold it
    Rule* pickupRule = new Rule(pickupAction,boost::get<Entity>(varAvatar),1);
    pickupRule->addPrecondition(pickupableState);
    pickupRule->addPrecondition(closedState);

    pickupRule->addEffect(EffectPair(1.0f,holderEffect));

    this->AllRules.push_back(pickupRule);

    //----------------------------End Rule: to pick up an object if closed enough-------------------------------------------

    //----------------------------Begin Rule: Move_to an object to get closed to it-----------------------------------------
    // define variables:
    StateValue var_obj = entity_var[0];
    StateValue float_dis = float_var[0];

    StateValue var_pos = vector_var[0];
    StateValue var_oldpos = vector_var[1];

    // precondition 1:There exists a path from the agent to object
    vector<StateValue> existPathStateOwnerList;
    existPathStateOwnerList.push_back(var_avatar);
    existPathStateOwnerList.push_back(var_obj);
    State* existPathState = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList, true, &Inquery::inqueryExistPath);

    // action: move to object
    PetAction* moveToObjectAction = new PetAction(ActionType::MOVE_TO_OBJ());
    moveToObjectAction->addParameter(ActionParameter("target",
                                        ActionParamType::ENTITY(),
                                        boost::get<Entity>(var_obj)));

    // effect: get closed to the object
    vector<StateValue> closedStateOwnerList2;
    closedStateOwnerList2.push_back(var_avatar);
    closedStateOwnerList2.push_back(var_obj);
    State* closedState2 = new State("Distance",StateValuleType::BOOLEAN(),STATE_EQUAL_TO , float_dis , closedStateOwnerList2, true, &Inquery::inqueryDistance);
    Effect* getClosedEffect = new Effect(closedState2, OP_ASSIGN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<StateValue> atLocationStateOwnerList2;
    atLocationStateOwnerList2.push_back(var_avatar);
    State* atLocationState2 = new State("AtLocation",StateValuleType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList2, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect2 = new Effect(atLocationState2, OP_ASSIGN_NOT_EQUAL_TO, var_oldpos);

    // rule:   Move_to an object to get closed to it
    Rule* movetoObjRule = new Rule(moveToObjectAction,boost::get<Entity>(var_avatar) ,1);
    movetoObjRule->addPrecondition(existPathState);

    movetoObjRule->addEffect(EffectPair(0.9f,getClosedEffect));
    movetoObjRule->addEffect(EffectPair(0.9f,changedLocationEffect2));

    this->AllRules.push_back(movetoObjRule);
    //----------------------------End Rule: Move_to an object to get closed to it-------------------------------------------

    //----------------------------Begin Rule: walk to a position to get closed to it-----------------------------------------

    // precondition 1:There exists a path from the agent to object
    vector<StateValue> existPathStateOwnerList2;
    existPathStateOwnerList2.push_back(var_avatar);
    existPathStateOwnerList2.push_back(var_pos);
    State* existPathState2 = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList2, true, &Inquery::inqueryExistPath);

    // action: walk to an position
    PetAction* walkAction = new PetAction(ActionType::WALK());
    walkAction->addParameter(ActionParameter("target",
                                            ActionParamType::VECTOR(),
                                            var_pos));

    // effect1: get closed to the position
    vector<StateValue> closedStateOwnerList3;
    closedStateOwnerList3.push_back(var_avatar);
    closedStateOwnerList3.push_back(var_pos);
    State* closedState3 = new State("Distance",StateValuleType::FLOAT(),STATE_EQUAL_TO ,float_dis, closedStateOwnerList3, true, &Inquery::inqueryDistance);
    Effect* getClosedEffect2 = new Effect(closedState3, OP_ASSIGN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<StateValue> atLocationStateOwnerList;
    atLocationStateOwnerList.push_back(var_avatar);
    State* atLocationState = new State("AtLocation",StateValuleType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect = new Effect(atLocationState, OP_ASSIGN, var_pos);

    // rule:   Move_to an object to get closed to it
    Rule* walkRule = new Rule(walkAction,boost::get<Entity>(var_avatar) ,1);
    walkRule->addPrecondition(existPathState2);

    walkRule->addEffect(EffectPair(0.9f,getClosedEffect2));
    walkRule->addEffect(EffectPair(0.9f,changedLocationEffect));

    this->AllRules.push_back(walkRule);
    //----------------------------End Rule: walk to a position to get closed to it-----------------------------------------

    //----------------------------Begin Rule: build a block in a position to make it possible to stand on it-----------------------------------------
    // define variables:
    StateValue var_pos_on = vector_var[1];
    StateValue var_is_standable = bool_var[0];

    // precondition 1: This pos should be empty, if it already has a block in it, you cannot build another block in it
    vector<StateValue> solidStateOwnerList;
    solidStateOwnerList.push_back(var_pos);
    State* solidState = new State("is_solid",StateValuleType::BOOLEAN(),STATE_EQUAL_TO, "false", solidStateOwnerList, true, &Inquery::inqueryIsSolid);

    // precondition 2: The agent should be closed enough to the position to build the block ( < 2.0)
    vector<StateValue> closedStateOwnerList4;
    closedStateOwnerList4.push_back(varAvatar);
    closedStateOwnerList4.push_back(var_pos);
    State* closedState4 = new State("Distance",StateValuleType::FLOAT(),STATE_LESS_THAN ,ACCESS_DISTANCE, closedStateOwnerList4, true, &Inquery::inqueryDistance);

    // precondition 3: The agent should not stand on the position to build the block
    vector<StateValue> atLocationStateOwnerList3;
    atLocationStateOwnerList3.push_back(varAvatar);
    State* atLocationState3 = new State("AtLocation",StateValuleType::VECTOR(),STATE_NOT_EQUAL_TO, var_pos, atLocationStateOwnerList3, true, &Inquery::inqueryAtLocation);

    // precondition 4: The position to bulid a block should be just on the desired position to stand on
    vector<StateValue> IsBelowStateOwnerList;
    IsBelowStateOwnerList.push_back(var_pos);
    IsBelowStateOwnerList.push_back(var_pos_on);
    State* IsBelowState = new State("is_below",StateValuleType::BOOLEAN(),STATE_EQUAL_TO, "true",
                                            IsBelowStateOwnerList, true, &Inquery::inqueryIsBelow);
    vector<StateValue> IsTouchingStateOwnerList;
    IsTouchingStateOwnerList.push_back(var_pos);
    IsTouchingStateOwnerList.push_back(var_pos_on);
    State* IsTouchingState = new State("is_touching",StateValuleType::BOOLEAN(),STATE_EQUAL_TO, "true",
                                            IsTouchingStateOwnerList, true, &Inquery::inqueryIsTouching);

    // action: build an block at an desired position
    PetAction* buildBlockAction = new PetAction(ActionType::BUILD_BLOCK());
    buildBlockAction->addParameter(ActionParameter("position",
                                            ActionParamType::VECTOR(),
                                            var_pos));
    buildBlockAction->addParameter(ActionParameter("blockType",
                                            ActionParamType::STRING(),
                                            "stone"));
    // effect1: the position on it is possible to become standable (the agent can stand on it)
    vector<StateValue> standableStateOwnerList;
    standableStateOwnerList.push_back(var_pos_on);
    State* standableState = new State("is_standable",StateValuleType::BOOLEAN(),STATE_EQUAL_TO , var_is_standable, standableStateOwnerList, true, &Inquery::inqueryIsStandable);
    Effect* becomeStandableEffect2 = new Effect(standableState, OP_ASSIGN, "true");

    // effect2: the position to build this block become solid
    Effect* becomeSolidEffect = new Effect(solidState, OP_ASSIGN, "true");

    // add rule:
    Rule* buildBlockRule = new Rule(buildBlockAction,boost::get<Entity>(varAvatar) ,30);
    buildBlockRule->addPrecondition(solidState);
    buildBlockRule->addPrecondition(closedState4);
    buildBlockRule->addPrecondition(atLocationState3);
    buildBlockRule->addPrecondition(IsBelowState);
    buildBlockRule->addPrecondition(IsTouchingState);

    buildBlockRule->addEffect(EffectPair(0.8f,becomeStandableEffect2));
    buildBlockRule->addEffect(EffectPair(1.0f,becomeSolidEffect));

    this->AllRules.push_back(buildBlockRule);

    //----------------------------End Rule: build a block in a position to make it possible to stand on it-----------------------------------------

    //----------------------------Begin Rule: if a position is standable and adjacent(neighbour) then there is possible existing a path from here to this adjacent postion------------------
    // define variables:
    StateValue var_pos_from = vector_var[0];
    StateValue var_pos_to = vector_var[1];
    StateValue var_exist_path = bool_var[0];

    // precondition 1: this position is standable
    vector<StateValue> standableStateOwnerList2;
    standableStateOwnerList2.push_back(var_pos_to);
    State* standableState2 = new State("is_standable",StateValuleType::BOOLEAN(),STATE_EQUAL_TO , "true", standableStateOwnerList2, true, &Inquery::inqueryIsStandable);

    // precondition 2: this position is adjacent to
    vector<StateValue> adjacentStateOwnerList;
    adjacentStateOwnerList.push_back(var_pos_to);
    adjacentStateOwnerList.push_back(var_pos_from);
    State* adjacentState = new State("is_adjacent",StateValuleType::BOOLEAN(),STATE_EQUAL_TO , "true", adjacentStateOwnerList, true, &Inquery::inqueryIsAdjacent);

    // effect: it's possible to access from var_pos_from to var_pos_to
    vector<StateValue> existPathStateOwnerList3;
     existPathStateOwnerList3.push_back(var_pos_from);
     existPathStateOwnerList3.push_back(var_pos_to);
     State* existPathState3 = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,var_exist_path, existPathStateOwnerList3, true, &Inquery::inqueryExistPath);
    Effect* becomeExistPathEffect = new Effect(existPathState3, OP_ASSIGN, "true");

    // action: do nothing
    PetAction* doNothingAction = new PetAction(ActionType::DO_NOTHING());
    // add rule:
    Rule* accessAdjacentRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0);
    accessAdjacentRule->addPrecondition(standableState2);
    accessAdjacentRule->addPrecondition(adjacentState);

    accessAdjacentRule->addEffect(EffectPair(0.7f,becomeExistPathEffect));

    this->AllRules.push_back(accessAdjacentRule);
    //----------------------------End Rule: if a position is standable and adjacent(neighbour) then there is possible existing a path from here to this adjacent postion-----------------------------

    //----------------------------Begin Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---------------------
    // define variables:
    StateValue var_pos_1 = vector_var[0];
    StateValue var_pos_2 = vector_var[1];
    StateValue var_pos_3 = vector_var[2];

    // precondition 1:There exists a path from the pos1 to pos2
    vector<StateValue> existPathStateOwnerList4;
    existPathStateOwnerList4.push_back(var_pos_1);
    existPathStateOwnerList4.push_back(var_pos_2);
    State* existPathState4 = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList4, true, &Inquery::inqueryExistPath);

    // precondition 2:There exists a path from the pos2 to pos3
    vector<StateValue> existPathStateOwnerList5;
    existPathStateOwnerList5.push_back(var_pos_2);
    existPathStateOwnerList5.push_back(var_pos_3);
    State* existPathState5 = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList5, true, &Inquery::inqueryExistPath);

    // effect: it's possible to access from var_pos_from to var_pos_to
    vector<StateValue> existPathStateOwnerList6;
     existPathStateOwnerList6.push_back(var_pos_1);
     existPathStateOwnerList6.push_back(var_pos_2);
     State* existPathState6 = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,var_exist_path, existPathStateOwnerList6, true, &Inquery::inqueryExistPath);
    Effect* becomeExistPathEffect2 = new Effect(existPathState6, OP_ASSIGN, "true");

    // add rule:
    Rule* pathTransmitRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0);
    pathTransmitRule->addPrecondition(existPathState4);
    pathTransmitRule->addPrecondition(existPathState5);

    pathTransmitRule->addEffect(EffectPair(1.0f,becomeExistPathEffect2));

    this->AllRules.push_back(pathTransmitRule);

    //----------------------------End Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---------------------
}


