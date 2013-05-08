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
#include <list>
#include <opencog/spacetime/SpaceServer.h>

using namespace opencog::oac;


OCPlanner::OCPlanner()
{

    loadAllRulesFromAtomSpace();

    // currently, for experiment, we dirctly add the rules by C++ codes
    loadTestRulesFromCodes();

    // preprocess the rule:
    vector<Rule*>::iterator it;
    for (it = AllRules.begin(); it != AllRules.end(); ++ it)
    {
        Rule* r = (Rule*)(*it);

        // preprocess the rule
        // adding indexes about the ungrounded parameters in it and and check if it's a recursive rule
        r->preProcessRule();

        // adding indexes about which rules have effects to which stateName
        addRuleEffectIndex(r);

    }
}

void OCPlanner::addRuleEffectIndex(Rule* r)
{
    vector<EffectPair>::iterator effectIt;
    for(effectIt = r->effectList.begin(); effectIt != r->effectList.end(); ++effectIt)
    {
        Effect* e = effectIt->second;

        State* s = e->state;

        map<string,map<float,Rule*> >::iterator it;
        it = ruleEffectIndexes.find(s->name());

        if (it == ruleEffectIndexes.end())
        {
            map<float,Rule*> rules;
            rules.insert(std::pair<float,Rule*>(effectIt->first,r));
            ruleEffectIndexes.insert(std::pair<string , map<float,Rule*> >(s->name(),rules));
        }
        else
        {
            // the map can make sure the rules are put in the list in the order of their probabilities from large to small

            ((map<float,Rule*>)(it->second)).insert(std::pair<float,Rule*>(effectIt->first,r));

        }

    }
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

// basically, we only care about the satisfied degree of the numberic state
bool OCPlanner::checkIsGoalAchieved(State& oneGoal, float& satisfiedDegree,  State* original_state)
{
    // if this goal doesn't really require an exact value, just return fully achieved
    if (oneGoal.stateVariable->getValue() == UNDEFINED_VALUE)
    {
        satisfiedDegree = 1.0f;
        return 1.0f;
    }

    // First search this state in the originalStatesCache
    vector<State>::const_iterator vit = originalStatesCache.begin();
    for (;vit != originalStatesCache.end(); vit ++)
    {
        State vState = (State)(*vit);
        if (vState.isSameState(oneGoal))
            return vState.isSatisfied(oneGoal,satisfiedDegree,original_state);

    }

    // has not been found in the originalStatesCache,
    // then we should inquery this state from the run time environment
    if (oneGoal.need_inquery)
    {
        // call its inquery funciton
        InqueryFun f = oneGoal.inqueryFun;
        StateValue inqueryValue = f(oneGoal.stateOwnerList);
        OC_ASSERT(!(inqueryValue == UNDEFINED_VALUE),
                  "OCPlanner::checkIsGoalAchieved: the inqueried value for state: %s is invalid.\n",
                  oneGoal.name().c_str());

        return oneGoal.isSatisfiedMe(inqueryValue,satisfiedDegree,original_state);

    }
    else // it doesn't need real time calculation, then we search for its latest evaluation link value in the atomspace
    {
        // TODO
        StateValue value = Inquery::getStateValueFromAtomspace(oneGoal);
        OC_ASSERT(!(value == UNDEFINED_VALUE),
                  "OCPlanner::checkIsGoalAchieved: the inqueried value for state: %s is invalid.\n",
                  oneGoal.name().c_str());

        // put this state into the cache, so we don't need to search for it next time
        State curState(oneGoal.name(),oneGoal.getStateValuleType(),oneGoal.stateType,value,oneGoal.stateOwnerList);

        return curState.isSatisfied(oneGoal, satisfiedDegree,original_state);

    }

}


bool OCPlanner::doPlanning(const vector<State*>& goal, vector<PetAction> &plan)
{
    originalStatesCache.clear();

    // clone a spaceMap for image all the the steps happen in the spaceMap, like building a block in some postion.
    // Cuz it only happens in imagination, not really happen, we should not really change in the real spaceMap
    SpaceServer::SpaceMap* clonedMap = spaceServer().cloneTheLatestSpaceMap();

    // Set this cloned spaceMap for Inquery
    Inquery::setSpaceMap(clonedMap);

    // we use the basic idea of the graph planner for plan searching:
    // alternated state layers with action layers
    // But we use backward depth-first chaining, instead of forward breadth-frist reasoning
    // Because our embodiment game world is not a simple finite boolean-state world, we cannot use a full forward breadth-frist which will be too slowly

    // Firstly, we construct the goal state layer
    StateLayer goalLayer(goal);

    StateLayer* curStateLayer = &goalLayer;

    // All the ActionLayers & All the StateLayers
    set<RuleLayer*> allRuleLayers;
    set<StateLayer*> allStateLayers;

    // planning process: All the rules should be grounded during planning.

    set<StateLayerNode*>::iterator stateLayerIter;
    while(true)
    {

        // first for loop, find a unsatisfied state
        // todo: there should some function to decide the order that which state should be achieved frist

        bool goalsAllAchieved = true;
        float satisfiedDegree;
        bool alreadyDealOneState = false;

        StateLayerNode* selectedStateNode;
        Rule* selectedRule;

        for (stateLayerIter = curStateLayer->nodes.begin(); stateLayerIter != curStateLayer->nodes.end();++stateLayerIter)
        {
            StateLayerNode* curStateNode = (StateLayerNode*)(*stateLayerIter);

            // some state has been checked in last circle, so we only need to check the states remain unknown
            if (curStateNode->isAchieved == StateLayerNode::ACHIEVED)
                continue;


            if ((curStateNode->isAchieved == StateLayerNode::UNKNOWN))
            {
                if (checkIsGoalAchieved(*(curStateNode->state), satisfiedDegree))
                {
                    curStateNode->isAchieved = StateLayerNode::ACHIEVED;
                    continue;
                }
                else
                {
                    curStateNode->isAchieved = StateLayerNode::NOT_ACHIEVED;
                    goalsAllAchieved = false;
                }

            }

            // if this state has been achieved , then it has been continue above.
            // only a state has not been achieved will come to here
            OC_ASSERT(curStateNode->isAchieved == StateLayerNode::NOT_ACHIEVED, "OCPLanner::doPlanning: The state " + curStateNode->state->name() + "is not not-achieved!/n");

            // In every layer, we only deal with one not_achieved state
            if (alreadyDealOneState)
                continue;

            alreadyDealOneState = true;

            map<string,map<float,Rule*> >::iterator it;
            it = ruleEffectIndexes.find(curStateNode->state->name());

            // Select a rule to apply

            map<float,Rule*> rules = (map<float,Rule*>)(it->second);

            if ( rules.size() == 1)
            {
                // if there is one rule to achieve this goal, just select it
                selectedRule = (((map<float,Rule*>)(it->second)).begin())->second;

                // check in the rule using history for achieving this state, if found this rule bas been marked as not useful, then break

            }
            else
            {
                // if there are multiple rules,choose the most suitable one

                // For non-numberic goals:
                // 1. This rule has not been applied in this layer for this goal before,
                //    or it has been applied but has not been used up yet (it only has been tried some variables for this rule, still can try other variables)
                // 2. todo: with the highest fitness for the current heuristics, currently we don't consider heuristics for non-numberic goals


                // Generate a score for each rules based on above criterions:
                // score = less used time(30%) + probability (35%) + lowest cost (35%)
                // recursive rules have higher priority, so the score of a recursive rule will plus 0.5

                float highestScore = 0.0;
                map<float,Rule*> ::iterator ruleIt;
                bool allRulesUnuseful = true;
                for (ruleIt = rules.begin(); ruleIt != rules.end(); ruleIt ++)
                {
                    Rule* r = ruleIt->second;
                    if (! curStateNode->IsRuleStillUsefule(r))
                        continue;

                    allRulesUnuseful = false;

                    // Because grounding every rule is time consuming, but some cost of rule requires the calculation of grounded variables.
                    // So here we just use the basic cost of every rule as the cost value.
                    float curRuleScore = 0.3f * (1.0f/(curStateNode->getRuleAppliedTime(r) +1)) + 0.35f* ruleIt->first + 0.35*(1.0f - r->getBasicCost());
                    if (r->IsRecursiveRule)
                        curRuleScore += 0.5f;

                    if (curRuleScore > highestScore)
                    {
                        selectedRule = r;
                        highestScore = curRuleScore;
                    }
                }

                // if find all the possible rules are already useless for current state (all rules have been tried but failed)
                // it suggests that the current state is impossible to achieve, so that we need to go back to last step
                if (allRulesUnuseful)
                {
                    // we have to delete the current state layer and the forward rule layer


                    curStateLayer = curStateLayer->preRuleLayer->preStateLayer;
                }

                selectedStateNode = curStateNode;



            }

            break;

        }

        if (goalsAllAchieved)
            return true;

        // Till now have select the an unsatisfied state and the rule to applied to try to do one step backward chaining to satisfy it
        // Creat a new rule layer backward trying to achieve the
        // and also create a new state Layer which are the preconditions of this new rule layer
        RuleLayer* newRuleLayer = new RuleLayer();
        StateLayer* newStateLayer = new StateLayer();

        allRuleLayers.insert(newRuleLayer);
        allStateLayers.insert(newStateLayer);

        newRuleLayer->nextStateLayer = curStateLayer;
        newRuleLayer->preStateLayer = newStateLayer;

        newStateLayer->nextRuleLayer = newRuleLayer;
        curStateLayer->preRuleLayer = newRuleLayer;

        // in every rule layer, there is always only one rule is applied
        // so as, in every state layer, there is always only one non-satisfied state being deal with every time

        // create a new RuleLayerNode to apply this selected rule
        RuleLayerNode* ruleNode = new RuleLayerNode(selectedRule);
        newRuleLayer->nodes.insert(ruleNode);

        ruleNode->forwardLinks.insert(selectedStateNode);
        selectedStateNode->backwardRuleNode = ruleNode;

        // When apply a rule, we need to select proper variables to ground it.
        // A rule should be grounded during its rule layer.

        // To ground a rule, first, we get all the variable values from the current state node to ground it.
        groundARuleNodeFromItsForwardState(ruleNode, selectedStateNode);

        // And then is possible to still have some variables cannot be grounded by just copy from the forward state
        // So we need to select suitable variables to ground them.
        groundARuleNodeBySelectingValues(ruleNode);

        // this rule has not been applied in this state yet, add this rule to the history of this state node
        //if (curStateNode->getRuleAppliedTime(selectedRule) == 0)
        //    curStateNode->addRuleRecordWithVariableBindingsToHistory();

        // Second for loop, for the rest states,create a do nothing rule node, to bring this state to the new state layer
        // except the states have been affected by the rule applied in the first for loop
        for (stateLayerIter = curStateLayer->nodes.begin(); stateLayerIter != curStateLayer->nodes.end();++stateLayerIter)
        {

            StateLayerNode* curStateNode = (StateLayerNode*)(*stateLayerIter);
            // check if this state has already been changed by a rule in this step during deal with other state
            if (curStateNode->backwardRuleNode == 0)
                continue;

            // create a do nothing rule node, to bring this state to the new state layer
            RuleLayerNode* donothingRuleLayerNode = new RuleLayerNode(DO_NOTHING_RULE);
            newRuleLayer->nodes.insert(donothingRuleLayerNode);
            donothingRuleLayerNode->ruleLayer = newRuleLayer;

            curStateNode->backwardRuleNode = donothingRuleLayerNode;

            State* cloneState = (curStateNode->state)->clone();
            StateLayerNode* cloneStateNode = new StateLayerNode(cloneState);
            cloneStateNode->stateLayer = newStateLayer;
            cloneStateNode->isAchieved = curStateNode->isAchieved;
            cloneStateNode->forwardRuleNode = donothingRuleLayerNode;

            donothingRuleLayerNode->backwardLinks.insert(cloneStateNode);
            donothingRuleLayerNode->forwardLinks.insert(curStateNode);

        }

        curStateLayer = newStateLayer;

    }

    // Reset the spaceMap for inquery back to the real spaceMap
    Inquery::reSetSpaceMap();

    return false;
}

bool OCPlanner::groundARuleNodeFromItsForwardState(RuleLayerNode* ruleNode, StateLayerNode* forwardStateNode)
{
    // first, find the state in the effect list of this rule which the same to this forward state
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    // Todo: Maybe need to check if all the non-variables/consts are the same to find the exact state rather than just check the state name
    for(effectIt = ruleNode->originalRule->effectList.begin(); effectIt != ruleNode->originalRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;

        s = e->state;
        if (s->name() ==  forwardStateNode->state->name())
            break;
    }

    if (effectIt == ruleNode->originalRule->effectList.end())
        return false;

    // check if all the stateOwner parameters grounded
    vector<StateValue>::iterator f_ownerIt = forwardStateNode->state->stateOwnerList.begin(); // state owner list in forward state
    vector<StateValue>::iterator r_ownerIt = s->stateOwnerList.begin(); // state owner list in rule effect state

    for ( ; r_ownerIt != s->stateOwnerList.end(); ++ f_ownerIt, ++r_ownerIt)
    {
        if (Rule::isParamValueUnGrounded(*r_ownerIt))
        {
            string variableName = StateVariable::ParamValueToString((StateValue)(*r_ownerIt));
            map<string, StateValue>::iterator paraIt = ruleNode->currentBindings.find(variableName);
            if (paraIt == ruleNode->currentBindings.end())
                ruleNode->currentBindings.insert(std::pair<string, StateValue>(variableName,*f_ownerIt));
        }
    }

    // If there is no cost_cal_state in this rule, we need to borrow from the forward state node's foward rule node
    if (ruleNode->originalRule->CostHeuristics.size() == 0)
    {
        if (forwardStateNode->forwardRuleNode)
            return true;

        if (forwardStateNode->forwardRuleNode->originalRule->CostHeuristics.size() != 0)
        {


        }
        else if (forwardStateNode->forwardRuleNode->costHeuristics.size() != 0)
        {

        }


    }

    return true;

}

bool OCPlanner::groundARuleNodeBySelectingValues(RuleLayerNode *ruleNode)
{

}

bool OCPlanner::selectValueForAVariableToGroundARule(RuleLayerNode* ruleNode, string variableStr)
{

}

// hard constraints as heuristics for recursive rule, borrowed from the non-recursive rule has the same effect with it.
// only applied for recursive rules.

// a bunch of rules for test, load from c++ codes
void OCPlanner::loadTestRulesFromCodes()
{
    // define a special action: do nothing
    PetAction* doNothingAction = new PetAction(ActionType::DO_NOTHING());

    //----------------------------Begin Rule: if energy value is high enough, the energy goal is achieved-------------------------------------------
    StateValue var_avatar = entity_var[1];
    StateValue var_achieve_energy_goal = bool_var[1];

    // precondition 1:
    vector<StateValue> energyStateOwnerList0;
    energyStateOwnerList0.push_back(var_avatar);
    State* energyState0 = new State("Energy",StateValuleType::FLOAT(),STATE_GREATER_THAN ,StateValue("0.8"), energyStateOwnerList0, true, Inquery::inqueryEnergy);
    // effect1: energy increases
    State* energyGoalState = new State("EnergyGoal",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,var_achieve_energy_goal, energyStateOwnerList0);

    Effect* energyGoalAchievedEffect = new Effect(energyGoalState, OP_ASSIGN, SV_TRUE);

    Rule* highEnergyAchieveEnergyGoalRule = new Rule(doNothingAction,boost::get<Entity>(var_avatar),0.0f);
    highEnergyAchieveEnergyGoalRule->addPrecondition(energyState0);

    highEnergyAchieveEnergyGoalRule->addEffect(EffectPair(1.0f,energyGoalAchievedEffect));

    this->AllRules.push_back(highEnergyAchieveEnergyGoalRule);

    //----------------------------End Rule: increase energy is to achieve energygoal-------------------------------------------

    //----------------------------Begin Rule: eat food to increase energy-------------------------------------------
    // define variables:
    StateValue var_food = entity_var[0];
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
    Rule* eatRule = new Rule(eatAction,boost::get<Entity>(var_avatar),0.2f);
    eatRule->addPrecondition(existState);
    eatRule->addPrecondition(edibleState);
    eatRule->addPrecondition(holderState);

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


    // precondition 1: The agent and the object is closed enough ( e.g. < 2.0)
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
    Rule* pickupRule = new Rule(pickupAction,boost::get<Entity>(varAvatar),0.1f);
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
    Effect* getClosedEffect = new Effect(closedState2, OP_ASSIGN_LESS_THAN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<StateValue> atLocationStateOwnerList2;
    atLocationStateOwnerList2.push_back(var_avatar);
    State* atLocationState2 = new State("AtLocation",StateValuleType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList2, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect2 = new Effect(atLocationState2, OP_ASSIGN_NOT_EQUAL_TO, var_oldpos);

    // rule:   Move_to an object to get closed to it
    Rule* movetoObjRule = new Rule(moveToObjectAction,boost::get<Entity>(var_avatar) ,0.01f);
    movetoObjRule->addPrecondition(existPathState);

    movetoObjRule->addEffect(EffectPair(0.9f,getClosedEffect));
    movetoObjRule->addEffect(EffectPair(0.9f,changedLocationEffect2));

    movetoObjRule->addCostHeuristic(CostHeuristic(closedState2, 0.01f));

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
    Effect* getClosedEffect2 = new Effect(closedState3, OP_ASSIGN_LESS_THAN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<StateValue> atLocationStateOwnerList;
    atLocationStateOwnerList.push_back(var_avatar);
    State* atLocationState = new State("AtLocation",StateValuleType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect = new Effect(atLocationState, OP_ASSIGN, var_pos);

    // rule:   Move_to an object to get closed to it
    Rule* walkRule = new Rule(walkAction,boost::get<Entity>(var_avatar) ,0.01f);
    walkRule->addPrecondition(existPathState2);

    walkRule->addEffect(EffectPair(0.9f,getClosedEffect2));
    walkRule->addEffect(EffectPair(0.9f,changedLocationEffect));

    walkRule->addCostHeuristic(CostHeuristic(closedState3, 0.01f));

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
    Rule* buildBlockRule = new Rule(buildBlockAction,boost::get<Entity>(varAvatar) ,0.5f);
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


    // add rule:
    Rule* accessAdjacentRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f);
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
    Rule* pathTransmitRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f);
    pathTransmitRule->addPrecondition(existPathState4);
    pathTransmitRule->addPrecondition(existPathState5);

    pathTransmitRule->addEffect(EffectPair(1.0f,becomeExistPathEffect2));

    this->AllRules.push_back(pathTransmitRule);

    //----------------------------End Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---------------------

    //Define the DO_NOTHING_RULE , this rule is a class member, not need to add it to the AllRules
    this->DO_NOTHING_RULE = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f);

}




