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
#include <opencog/util/macros.h>
#include "Inquery.h"
#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h>
#include <iterator>
#include <list>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/WorldWrapper/PAIWorldWrapper.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

#include <stdlib.h>
#include <map>
#include <math.h>
#include <cstdio>
#include <sstream>
#include "OAC.h"

using namespace opencog::oac;
using namespace std;

RuleNode OCPlanner::goalRuleNode = RuleNode();

string RuleNode::getDepthOfRuleNode()
{
     // check the depth of the effect state nodes of this rule, get the deepest state node.
    if (this == &(OCPlanner::goalRuleNode))
        return "goalDepth";

    StateNode* deepestStateNode = getLastForwardStateNode();

    return deepestStateNode->depth;
}

StateNode* RuleNode::getLastForwardStateNode() const
{
    // get the forwardStateNode with the deepest depth
    vector<StateNode*>::const_iterator it = forwardLinks.begin();
    StateNode* lastedStateNode = 0;

    for (; it != forwardLinks.end(); ++ it)
    {
        if (lastedStateNode == 0)
            lastedStateNode = (StateNode*)(*it);
        if ( lastedStateNode < ((StateNode*)(*it)))
        {
            lastedStateNode = (StateNode*)(*it);
        }
    }

    return lastedStateNode;
}

StateNode* RuleNode::getDeepestBackwardStateNode() const
{
    // get the backward StateNode with the deepest depth
    if (backwardLinks.size() == 0)
        return 0;

    vector<StateNode*>::const_iterator it = backwardLinks.begin();
    StateNode* lastedStateNode = 0;

    for (; it != backwardLinks.end(); ++ it)
    {
        if (lastedStateNode == 0)
            lastedStateNode = (StateNode*)(*it);

        if ( lastedStateNode < ((StateNode*)(*it)))
        {
            lastedStateNode = (StateNode*)(*it);
        }
    }
    return lastedStateNode;
}

void RuleNode::updateCurrentAllBindings()
{
    if (currentBindingsFromForwardState.size() > 0)
        currentAllBindings = currentBindingsFromForwardState;

    if (currentBindingsViaSelecting.size() > 0)
        currentAllBindings.insert(currentBindingsViaSelecting.begin(),currentBindingsViaSelecting.end());
}

bool findInStateNodeList(list<StateNode*> &stateNodeList, StateNode* state)
{
    list<StateNode*>::iterator it;
    for (it = stateNodeList.begin(); it != stateNodeList.end(); ++ it)
    {
        if (state == (*it))
            return true;
    }

    return false;
}


// this should be called after its forward node is assigned or changed
void StateNode::calculateNodesDepth()
{
    if (depth == "Deepest")
        return; // this the orginal state

    if (! forwardRuleNode)
    {
        if ( sizeof(depth) == 1)
        {
            return; // it's the goal state nodes
        }
        else if (backwardRuleNode != 0)
        {
            // the state node of its backwardRuleNode's forward state node , which with the deepest depth
            StateNode* deepestNode0 = backwardRuleNode->getLastForwardStateNode();
            StateNode* deepestNode = deepestNode0->forwardRuleNode->getDeepestBackwardStateNode();
            // To be improved: currently the index is only 0 ~ 9, need to support A~Z , a~z
            string deepestStr = deepestNode->depth;
            int index = atoi((deepestStr.substr(deepestStr.size() - 1,1)).c_str());
            index ++;
            if (deepestStr.size() == 1)
                depth = opencog::toString(index);
            else
                depth = deepestStr.substr(0,deepestStr.size() - 1) + opencog::toString(index);

        }
        else
        {
            cout << "Debug Error: StateNode::calculateNodesDepth: this state node is created out of nothing, it doesn't have any backward or forward rule node!"<<std::endl;
            return;
        }
    }
    else
    {

        string fowardRuleNodeDepth = forwardRuleNode->getDepthOfRuleNode();

        // find the index of this StateNode in its forward rule node
        int index = 0;
        vector<StateNode*>::iterator it = forwardRuleNode->backwardLinks.begin();

        for (; it != forwardRuleNode->backwardLinks.end(); ++ it, ++ index)
        {

            if (this == (StateNode*)(*it))
                break;
        }

        if (it == forwardRuleNode->backwardLinks.end())
        {
            cout<< "Debug Error: the this state node cannot be found in its forward rule node!"<<std::endl;
            return;
        }

        // To be improved: currently the index is only 0 ~ 9, need to support A~Z , a~z
        depth = fowardRuleNodeDepth + opencog::toString(index);

        if ((backwardRuleNode == 0) || (backwardRuleNode->backwardLinks.size() == 0))
            return;

        // this state node has backward nodes, calculate all of their depth recursively
        vector<StateNode*>::iterator bit = backwardRuleNode->backwardLinks.begin();

        for (; bit != backwardRuleNode->backwardLinks.end(); ++ bit)
        {
            ((StateNode*)(*bit))->calculateNodesDepth();
        }

    }


}

StateNode* RuleNode::getMostClosedBackwardStateNode() const
{
    // get the BackwardStateNode with the least depth
    vector<StateNode*>::const_iterator it = backwardLinks.begin();
    StateNode* lastedStateNode = 0;

    for (; it != backwardLinks.end(); ++ it)
    {
        if (lastedStateNode == 0)
            lastedStateNode = (StateNode*)(*it);

        if ( ((StateNode*)(*it)) <  lastedStateNode)
        {
            lastedStateNode = (StateNode*)(*it);
        }
    }
}

SpaceServer::SpaceMap* OCPlanner::getLatestSpaceMapFromBackwardStateNodes(RuleNode* ruleNode)
{
    // first , get the BackwardStateNodes
    if (ruleNode->backwardLinks.size() == 0)
        return &(spaceServer().getLatestMap());

    StateNode* lastedStateNode = ruleNode->getMostClosedBackwardStateNode();

    // get the curMap of the backward rule of this state node
    if (lastedStateNode->backwardRuleNode != 0)
        return lastedStateNode->backwardRuleNode->curMap;
    else
        return &(spaceServer().getLatestMap());
}

// @ bool &found: return if this same state is found in temporaryStateNodes
// @ StateNode& *stateNode: the stateNode in temporaryStateNodes which satisfied or dissatisfied this goal
// @ ifCheckSameRuleNode: if avoid finding the state node generate by same rule node
bool OCPlanner::checkIfThisGoalIsSatisfiedByTempStates(State& goalState, bool &found, StateNode *&satstateNode,RuleNode *forwardRuleNode,
                                                       bool ifCheckSameRuleNode, StateNode* curStateNode)
{
    //   if curStateNode is 0, it means it has no backward links yet, so only check in startStateNodes.
    //   if curStateNode is not 0, check temporaryStateNodes first , if cannot find in temporaryStateNodes, check in startStateNodes.
    satstateNode = 0;
    if ( (curStateNode != 0) && (curStateNode->backwardRuleNode != 0))
    {
        // find this state in temporaryStateNodes
        if (findStateInTempStates(goalState,forwardRuleNode,satstateNode,ifCheckSameRuleNode))
        {
            //  check if this state has beed satisfied by the previous state nodes
            float satisfiedDegree;
            found = true;

            State* vState = satstateNode->state;
            if (vState->isSatisfied(goalState,satisfiedDegree))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            // cannot find this state in temporaryStateNodes
            found = false;
        }
    }

    // cannot find in temporaryStateNodes , try to find in startStateNodes
    if (findStateInStartStateNodes(goalState,satstateNode))
    {
        //  check if this state has beed satisfied by the previous state nodes
        float satisfiedDegree;
        found = true;

        State* vState = satstateNode->state;
        if (vState->isSatisfied(goalState,satisfiedDegree))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // cannot find this state in temporaryStateNodes
        found = false;
        return false;
    }

}

//  return if this same state is found in temporaryStateNodes
// @ StateNode& *stateNode: the stateNode in temporaryStateNodes which satisfied or dissatisfied this goal
bool OCPlanner::findStateInTempStates(State& state, RuleNode *forwardRuleNode,StateNode* &stateNode,bool ifCheckSameRuleNode)
{
    //  check if this state has beed satisfied by the previous state nodes
    list<StateNode*>::const_iterator vit = temporaryStateNodes.begin();

    // most closed state node, describing the same state of the input state
    stateNode = 0;
    StateNode* mostClosedNode;

    if (forwardRuleNode)
        mostClosedNode = forwardRuleNode->getMostClosedBackwardStateNode();

    for (;vit != temporaryStateNodes.end(); vit ++)
    {
        // only check the states more backward (deeper) then the mostClosedNode
        // cuz the forward state won't affect the current state
        if (mostClosedNode)
        {
            if (((StateNode*)(*vit)) < mostClosedNode)
                continue;
        }

        // toBeImproved: there is some messy logic order problem in backward reasoning,
        // sometimes need to add the effect states of a rulenode into temporaryStateNodes, before check the precondition states of this rule node
        // so we if the input checked state node and the found state node are generated by same rule node (precondition and effect of this rule), skip it.
        if (ifCheckSameRuleNode && (((StateNode*)(*vit))->backwardRuleNode == forwardRuleNode))
            return false;

        State* vState = ((StateNode*)(*vit))->state;
        if (vState->isSameState(state))
        {
            if (stateNode == 0)
            {
                stateNode = (StateNode*)(*vit);
            }
            else if (((StateNode*)(*vit)) < stateNode) // get the most closed state node backward from the forwardRuleNode
            {
                stateNode = ((StateNode*)(*vit));
            }

        }

    }

    // cannot find this state in temporaryStateNodes
    if (stateNode != 0)
        return true;
    else
        return false;

}

bool OCPlanner::findStateInStartStateNodes(State& state, StateNode* &stateNode)
{
    //  check if this state has beed satisfied by the previous state nodes
    list<StateNode*>::const_iterator vit = startStateNodes.begin();

    // most closed state node, describing the same state of the input state
    stateNode = 0;

    for (;vit != startStateNodes.end(); vit ++)
    {
        State* vState = ((StateNode*)(*vit))->state;
        if (vState->isSameState(state))
        {
            stateNode = ((StateNode*)(*vit));
            return true;
        }
    }

    return false;

}

OCPlanner::OCPlanner(AtomSpace *_atomspace, string _selfID, string _selfType)
{

    atomSpace = _atomspace;

    selfEntityParamValue = Entity(_selfID,_selfType);

    std::cout << "Debug: OCPlanner init: selfID = " << _selfID << ", slef type = " << _selfType << std::endl;

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

    // debug: print all the rule indexes:
    cout<< "Debug: all rule indexes:" << std::endl;
    map<string,multimap<float,Rule*> >::iterator itr;
    for (itr = ruleEffectIndexes.begin(); itr != ruleEffectIndexes.end(); ++ itr)
    {
        cout<< itr->first <<":" << std::endl;
        multimap<float,Rule*>::iterator iter;

        for(iter =  ((multimap<float,Rule*>&)(itr->second)).begin(); iter !=  ((multimap<float,Rule*>&)(itr->second)).end(); ++ iter)
        {
            cout<< (((Rule*)(iter->second))->action)->getName() << std::endl;

        }
    }

}

OCPlanner::~OCPlanner()
{
    // todo: delete everything

}

void OCPlanner::addRuleEffectIndex(Rule* r)
{
    vector<EffectPair>::iterator effectIt;
    for(effectIt = r->effectList.begin(); effectIt != r->effectList.end(); ++effectIt)
    {
        Effect* e = effectIt->second;

        State* s = e->state;

        map<string,multimap<float,Rule*> >::iterator it;
        it = ruleEffectIndexes.find(s->name());

        cout << "Debug: addRuleEffectIndex: State name:" << s->name() << std::endl;

        if (it == ruleEffectIndexes.end())
        {
            multimap<float,Rule*> rules;
            rules.insert(std::pair<float,Rule*>(effectIt->first,r));
            ruleEffectIndexes.insert(std::pair<string , multimap<float,Rule*> >(s->name(),rules));
        }
        else
        {
            // the map can make sure the rules are put in the list in the order of their probabilities from large to small
            // map<string,map<float,Rule*> >

            multimap<float,Rule*>& indexMap = ((multimap<float,Rule*>&)(it->second));

            indexMap.insert(std::pair<float,Rule*>(effectIt->first,r));

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
bool OCPlanner::checkIsGoalAchievedInRealTime(State& oneGoal, float& satisfiedDegree,  State* original_state)
{
    // if this goal doesn't really require an exact value, just return fully achieved
    if (oneGoal.stateVariable->getValue() == UNDEFINED_VALUE)
    {
        satisfiedDegree = 1.0f;
        return 1.0f;
    }

//    // First search this state in the temporaryStateNodes list
//    vector<StateNode*>::const_iterator vit = temporaryStateNodes.begin();
//    for (;vit != temporaryStateNodes.end(); vit ++)
//    {
//        // there are possible mutiple same state nodes describe this same state in temporaryStateNodes,
//        // but the latest one is put in the front, so we can just check the first one we find
//        State* vState = ((StateNode*)(*vit))->state;
//        if (vState->isSameState(oneGoal))
//            return vState->isSatisfied(oneGoal,satisfiedDegree,original_state);

//    }

    // then we should inquery this state from the run time environment
    if (oneGoal.need_inquery)
    {
        // call its inquery funciton
        InqueryStateFun f = oneGoal.inqueryStateFun;
        ParamValue inqueryValue = f(oneGoal.stateOwnerList);
        OC_ASSERT(!(inqueryValue == UNDEFINED_VALUE),
                  "OCPlanner::checkIsGoalAchievedInRealTime: the inqueried value for state: %s is invalid.\n",
                  oneGoal.name().c_str());

        return oneGoal.isSatisfiedMe(inqueryValue,satisfiedDegree,original_state);

    }
    else // it doesn't need real time calculation, then we search for its latest evaluation link value in the atomspace
    {
        // TODO
        ParamValue value = Inquery::getParamValueFromAtomspace(oneGoal);
        OC_ASSERT(!(value == UNDEFINED_VALUE),
                  "OCPlanner::checkIsGoalAchievedInRealTime: the inqueried value for state: %s is invalid.\n",
                  oneGoal.name().c_str());

        // put this state into the cache, so we don't need to search for it next time
        State curState(oneGoal.name(),oneGoal.getActionParamType(),oneGoal.stateType,value,oneGoal.stateOwnerList);

        return curState.isSatisfied(oneGoal, satisfiedDegree,original_state);

    }

}

ActionPlanID OCPlanner::doPlanningForPsiDemandingGoal(Handle& goalHandle,opencog::CogServer * server)
{
    // need to translate the psi demanding goal Handle into vector<State*>
    vector<State*> goal,knownStates;

    // the goal handle is like: [EvaluationLink <EnergyDemandGoal> 0.999857 0.00124844]
    Handle goal_predicate = this->atomSpace->getOutgoing(goalHandle,0);
    OC_ASSERT( (goal_predicate != Handle::UNDEFINED), "OCPlanner::doPlanningForPsiDemandingGoal: The current goal is Handle::UNDEFINED!");

    string goalname = this->atomSpace->getName(goal_predicate);
    std::cout<< "OCPLANNER_DEBUG:Goalgame: "<< goalname.c_str() << std::endl;

    State* goalState = new State(goalname,ActionParamType::BOOLEAN(),STATE_EQUAL_TO,SV_TRUE);
    goalState->addOwner(this->selfEntityParamValue);

    State* knownState = new State(goalname,ActionParamType::BOOLEAN(),STATE_EQUAL_TO,SV_FALSE);
    knownState->addOwner(this->selfEntityParamValue);

    goal.push_back(goalState);

    knownStates.push_back(knownState);

    return doPlanning(goal,knownStates,server);

}

ActionPlanID OCPlanner::doPlanning(const vector<State*>& goal,const vector<State*>& knownStates,opencog::CogServer * server)
{

    // Get OAC
    OAC* oac = dynamic_cast<OAC*>(server);
    OC_ASSERT(oac, "OCPlanner::doPlanning: Did not get an OAC server!");

    int ruleNodeCount = 0;

    curtimeStamp = oac->getPAI().getLatestSimWorldTimestamp();

    // clone a spaceMap for image all the the steps happen in the spaceMap, like building a block in some postion.
    // Cuz it only happens in imagination, not really happen, we should not really change in the real spaceMap

    imaginarySpaceMaps.clear();
    startStateNodes.clear();
    allRuleNodeInThisPlan.clear();
    unsatisfiedStateNodes.clear();
    temporaryStateNodes.clear();
    imaginaryHandles.clear(); // TODO: Create imaginary atoms
    satisfiedGoalStateNodes.clear();
    OCPlanner::goalRuleNode.backwardLinks.clear();


    // we use the basic idea of the graph planner for plan searching:
    // alternated state layers with action layers
    // But we use backward depth-first chaining, instead of forward breadth-frist reasoning
    // Because our embodiment game world is not a simple finite boolean-state world, we cannot use a full forward breadth-frist which will be too slowly

    // Firstlyt, we construct the knownStates as the original temporaryStateNodes from input :
    vector<State*>::const_iterator itknown;
    for (itknown = knownStates.begin(); itknown != knownStates.end(); ++ itknown)
    {
        StateNode* newStateNode = new StateNode(*itknown);

        newStateNode->backwardRuleNode = 0;
        newStateNode->forwardRuleNode = 0;
        // A node that is Deepest is the orginal starting state, it's the deepest in our backward planning network, the most far away from the goal state
        newStateNode->depth = "Deepest";

        startStateNodes.push_front(newStateNode);

    }

    // And then, we construct the original unsatisfiedStateNodes and temporaryStateNodes from the input goals

    // satisfiedGoalStateNodes is to store the already satisfied goal states before beginning planning, in case some planning steps  will change them into unsatisfied
    // Everytime when it's changed into unsatisfied, it should be put into the unsatisfiedStateNodes list and removed from satisfiedGoalStateNodes list

    vector<State*>::const_iterator it;
    int goalNum = 0;
    for (it = goal.begin(); it != goal.end(); ++ it, ++ goalNum)
    {
        float satisfiedDegree;
        StateNode* newStateNode = new StateNode(*it);

        newStateNode->backwardRuleNode = 0;
        newStateNode->forwardRuleNode = &(OCPlanner::goalRuleNode);
        newStateNode->depth = opencog::toString(goalNum);

        OCPlanner::goalRuleNode.backwardLinks.push_back(newStateNode);

        bool found;
        StateNode* knownStateNode;

        if (checkIfThisGoalIsSatisfiedByTempStates(*(newStateNode->state), found, knownStateNode, 0,false))
        {
            satisfiedGoalStateNodes.push_back(newStateNode);
            temporaryStateNodes.push_front(newStateNode);
        }

        if (found)
        {
            unsatisfiedStateNodes.push_back(newStateNode);
        }
        else if (checkIsGoalAchievedInRealTime(*(newStateNode->state), satisfiedDegree))
        {
            satisfiedGoalStateNodes.push_back(newStateNode);
            temporaryStateNodes.push_front(newStateNode);
        }
        else
        {
            unsatisfiedStateNodes.push_back(newStateNode);
        }

    }


    if (unsatisfiedStateNodes.size() == 0)
    {
        std::cout << "The goal is already satisfied! There is no need to do planning!" << std::endl;
        return "";
    }

    int tryStepNum = 0;

    while(unsatisfiedStateNodes.size() != 0)
    {
        Rule* selectedRule = 0;

        curtimeStamp ++;

        tryStepNum ++;
        if (tryStepNum > 999)
        {
            std::cout << "Planning failed! Has tried more than 999 steps of planning, cannot find a plan!" << std::endl;
            return "";
        }
/*
        // decide which state should be chosed to achieved first
        list<StateNode*>::iterator stateNodeIter;
        for (stateNodeIter = unsatisfiedStateNodes.begin(); stateNodeIter != unsatisfiedStateNodes.end();++stateNodeIter)
        {
            ((StateNode*)(*stateNodeIter))->calculateNodeDepth();
        }
*/
        // unsatisfiedStateNodes.sort();

        // the state node with deeper depth will be solved first
        StateNode* curStateNode = (StateNode*)(unsatisfiedStateNodes.back());

        // out put selected subgoal debug info:
        cout<< std::endl << "Debug planning step " << tryStepNum <<": Selected subgoal :";
        outputStateInfo(curStateNode->state,true);
        cout<<std::endl;

        SpaceServer::SpaceMap* curImaginaryMap;
        if (curStateNode->backwardRuleNode == 0)
            curImaginaryMap = &(spaceServer().getLatestMap());
        else
            curImaginaryMap = curStateNode->backwardRuleNode->curMap;

        // Set this cloned spaceMap for Inquery
        Inquery::setSpaceMap(curImaginaryMap);

        // if the deepest state node has a depth of -1, it means all the required states have been satisfied
        if (curStateNode->depth == "-1")
            break;

        // if we have not tried to achieve this state node before, find all the candidate rules first
        if (! curStateNode->hasFoundCandidateRules)
        {
            map<string,multimap<float,Rule*> >::iterator it;
            it = ruleEffectIndexes.find(curStateNode->state->name());
            curStateNode->hasFoundCandidateRules = true;

            // if there is not any rule related to this goal, continue to next loop;
            // in fact in next loop,it will go into the else for processing the situation when curStateNode->hasFoundCandidateRules is true;
            if (it == ruleEffectIndexes.end())
                continue;

            // Select a rule to apply

            multimap<float,Rule*>& rules = (multimap<float,Rule*>&)(it->second);

            if ( rules.size() == 1)
            {
                // if there is one rule related to this goal,
                // check if it's negative or positive for this goal:
                Rule* r = (((multimap<float,Rule*>)(it->second)).begin())->second;
                bool isNegativeGoal, isDiffStateOwnerType, preconImpossible;
                int negativeNum,satisfiedPreconNum;
                checkRuleFitnessRoughly(r,curStateNode,satisfiedPreconNum,negativeNum,isNegativeGoal,isDiffStateOwnerType,preconImpossible,true);

                if (isNegativeGoal || isDiffStateOwnerType) // if this rule will negative this goal, we should not choose to apply it.
                    continue;

                // this rule is positive for this goal, apply it.
                selectedRule = r;
                curStateNode->ruleHistory.push_back(selectedRule);
            }
            else
            {
                // if there are multiple rules,choose the most suitable one

                // For non-numberic goals:
                // 1. This rule has not been applied in this layer for this state before,
                //    or it has been applied but has not been used up yet (it only has been tried some variables for this rule, still can try other variables)
                // 2. todo: with the highest fitness for the current heuristics, currently we don't consider heuristics for non-numberic goals

                // Generate a score for each rules based on above criterions:
                // score = probability (50%) + lowest cost (50%)
                // recursive rules have higher priority, so the score of a recursive rule will plus 0.5
                // will also check the fitness of this rule , see checkRuleFitnessRoughly

                multimap<float,Rule*> ::iterator ruleIt;

                for (ruleIt = rules.begin(); ruleIt != rules.end(); ruleIt ++)
                {
                    Rule* r = ruleIt->second;

                    // Because grounding every rule fully is time consuming, but some cost of rule requires the calculation of grounded variables.
                    // So here we just use the basic cost of every rule as the cost value.
                    float curRuleScore = 0.5f* ruleIt->first + 0.5*(1.0f - r->getBasicCost());
                    if (r->IsRecursiveRule)
                        curRuleScore += 0.5f;

                    // the rules with higher score are put in front
                    list< pair<float,Rule*> >::iterator canIt;
                    canIt = curStateNode->candidateRules.begin();

                    // ground it only by this current state node,  to check if its effect will negative this current selected goal state
                    // and also check if other effects negative some of the other temporaryStateNodes
                    bool isNegativeGoal, isDiffStateOwnerType, preconImpossible;
                    int negativeNum,satisfiedPreconNum;
                    checkRuleFitnessRoughly(r,curStateNode,satisfiedPreconNum,negativeNum,isNegativeGoal,isDiffStateOwnerType,preconImpossible);

                    //  its effect will negative this current selected goal state, or it has any unsatisfied precondition which is impossible to achieve,
                    //  then it should not add it into candidate rules
                    if (isNegativeGoal || preconImpossible || isDiffStateOwnerType)
                        continue;

                    curRuleScore -= negativeNum;
                    curRuleScore += satisfiedPreconNum;

                    while(true)
                    {
                        if (canIt == curStateNode->candidateRules.end())
                        {
                            curStateNode->candidateRules.push_back( pair<float, Rule*>(curRuleScore,r));
                            break;
                        }

                        if (curRuleScore > canIt->first)
                        {
                            curStateNode->candidateRules.insert(canIt,pair<float, Rule*>(curRuleScore,r));
                            break;
                        }

                        canIt ++;
                    }

                }

                // if cannot find a proper rule for this goal, continue to next loop;
                // in fact in next loop,it will go into the else for processing the situation when curStateNode->hasFoundCandidateRules is true;
                if (curStateNode->candidateRules.size() == 0)
                    continue;

                selectedRule = (curStateNode->candidateRules.begin())->second;
                curStateNode->ruleHistory.push_back((curStateNode->candidateRules.begin())->second);
                curStateNode->candidateRules.pop_front();

            }


        }
        else //  we have  tried to achieve this state node before,which suggests we have found all the candidate rules
        {

            // check if there is any rule left not been tried in the candidate rules
            if (curStateNode->candidateRules.size() != 0)
            {
                selectedRule = (curStateNode->candidateRules.front()).second;
                curStateNode->ruleHistory.push_back((curStateNode->candidateRules.front()).second);
                curStateNode->candidateRules.erase(curStateNode->candidateRules.begin());
            }
            else
            {
                // we have tried all the candidate rules, still cannot achieve this state, which means this state is impossible to be achieved here
                // so go back to the its foward rule which produce this state to check if we can apply another bindings to the same rule or we shoud try another rule

                RuleNode* forwardRuleNode = curStateNode->forwardRuleNode;

                if (forwardRuleNode == 0)
                {
                    // oh, this state node is already the goal state node, and it's impossible to achieve, return planning fail
                    return "";
                }

                if (forwardRuleNode->ParamCandidates.size() == 0)
                {
                    // we have tried all the Candidate bindings in previous steps,
                    // so it means this rule doesn't work, we have to go back to its forward state node

                    // Remove this rule from the candidate rules of all its foward state nodes
                    // and move all its foward state nodes from temporaryStateNodes to unsatisfiedStateNodes
                    vector<StateNode*>::iterator forwardStateIt;
                    for (forwardRuleNode->forwardLinks.begin(); forwardStateIt != forwardRuleNode->forwardLinks.end(); ++ forwardStateIt)
                    {
                        if (((StateNode*)(*forwardStateIt))->candidateRules.size() > 0)
                        {
                            list< pair<float,Rule*> >::iterator candiIt;
                            for (candiIt = ((StateNode*)(*forwardStateIt))->candidateRules.begin(); candiIt != ((StateNode*)(*forwardStateIt))->candidateRules.end(); ++ candiIt)
                            {
                                if (candiIt->second == forwardRuleNode->originalRule)
                                {
                                    ((StateNode*)(*forwardStateIt))->candidateRules.erase(candiIt);
                                    break;
                                }

                            }

                        }

                        deleteStateNodeInTemporaryList((StateNode*)(*forwardStateIt));

                        // only when there is any rule node need this state node as a prediction, put it to the unsatisfed list
                        if ((StateNode*)(*forwardStateIt)->forwardRuleNode != 0)
                            unsatisfiedStateNodes.push_back(*forwardStateIt);
                    }

                    deleteRuleNodeRecursively(forwardRuleNode);

                }
                else
                {
                    // There are still Candidate bindings for this rule node to try.

                    // check which states of the preconditions of this forwardRuleNode have been sovled , which still remand unsloved.
                    vector<StateNode*>::iterator preconItor;
                    map<State*,StateNode*> solvedStateNodes; // all the state nodes in forwardRuleNode's preditions that have been solved by previous planning steps
                    for (preconItor = forwardRuleNode->backwardLinks.begin(); preconItor != forwardRuleNode->backwardLinks.end(); ++ preconItor)
                    {
                        // skip the current state node
                        if ((*preconItor) == curStateNode)
                            continue;

                        // when this state node has already been tried a backward rule to solve it, or this state node is not in the unsatisfiedStateNodes
                        // it means it's solved
                        if (((*preconItor)->backwardRuleNode != 0) || (! findInStateNodeList(unsatisfiedStateNodes,*preconItor)))
                        {
                            // so put it in the solvedStateNodes map
                            StateNode* sn = (StateNode*)(*preconItor);
                            solvedStateNodes.insert(pair<State*,StateNode*>(sn->forwardEffectState , sn ) );
                        }
                    }

                    // If all the effect states of this  forwardRuleNode remand unsolved,
                    // which suggests try another random group of candidate bindings will not affect the planning step that has been conducted
                    // so just try any other bindings for this rule node
                    if (solvedStateNodes.size() == 0)
                    {
                       // just try a new binding to ground the effect states
                        forwardRuleNode->currentBindingsViaSelecting =  forwardRuleNode->ParamCandidates.front();
                        forwardRuleNode->ParamCandidates.erase(forwardRuleNode->ParamCandidates.begin());

                        forwardRuleNode->updateCurrentAllBindings();
                        recordOrginalParamValuesAfterGroundARule(forwardRuleNode);

                        // rebind all the precondition state nodes in the forward rule
                        for (preconItor = forwardRuleNode->backwardLinks.begin(); preconItor != forwardRuleNode->backwardLinks.end(); ++ preconItor)
                        {
                            reBindStateNode((*preconItor),forwardRuleNode->currentAllBindings);
                        }

                        continue; // continue with the big while loop to check the next subgoal in unsatisfiedStateNodes
                    }
                    else // some of the predition states of this forwardRuleNode has been solved, so we cannot simply replace the current bindings with another random bindings
                    {
                        // try to find a group of bindings from the candidates, that won't affect the solved states
                        // if cannot find any , try to find a group of candidate bindings will affect as few as possible solved states
                        // to record which solved states will be affected if use this candidate binding
                        // a vector in the order of vector<ParamGroundedMapInARule> ParamCandidates
                        // each element - vector<StateNode*>  is the list of all the state nodes that a ParamGroundedMapInARule in vector<ParamGroundedMapInARule> ParamCandidates
                        vector< vector<StateNode*> >  willBeAffecteds;
                        map<State*,StateNode*>::iterator solvedItor;
                        vector<ParamGroundedMapInARule>::iterator candidateIt;
                        vector<ParamGroundedMapInARule>::iterator bestBindingIt = forwardRuleNode->ParamCandidates.end(); // the best Binding with the least affect
                        int affectLeastStateNum = 999999; // the least number of state nodes the candidate bindings will affect
                        int index = -1; // The index of the candidate binding in ParamCandidates which causes the least affect on the solved state nodes
                        int i = 0;
                        for (candidateIt = forwardRuleNode->ParamCandidates.begin(); candidateIt != forwardRuleNode->ParamCandidates.end(); ++ candidateIt, ++i)
                        {
                            vector<StateNode*> oneAffectRecord;
                            list<UngroundedVariablesInAState>::iterator ungroundVarIt;
                            for (ungroundVarIt = forwardRuleNode->curUngroundedVariables.begin(); ungroundVarIt != forwardRuleNode->curUngroundedVariables.end(); ++ ungroundVarIt)
                            {
                                UngroundedVariablesInAState& ungroundVarInAState = (UngroundedVariablesInAState&)(*ungroundVarIt);

                                // Try to find this state in the solvedStateNodes.
                                solvedItor = solvedStateNodes.find(ungroundVarInAState.state);
                                if (solvedItor == solvedStateNodes.end())
                                    continue;

                                // Try to find the ungrounded variable names in this candidate binding
                                ParamGroundedMapInARule& bindings = (ParamGroundedMapInARule&)(*candidateIt);
                                ParamGroundedMapInARule::iterator bindIt;
                                for (bindIt = bindings.begin(); bindIt !=  bindings.end(); ++ bindIt)
                                {
                                    string& varName = (string&)(bindIt->first);

                                    // try to find this find this variable name() in the ungrounded variable records
                                    if (ungroundVarInAState.vars.find(varName) != ungroundVarInAState.vars.end())
                                    {
                                        oneAffectRecord.push_back((StateNode*)(solvedItor->second));
                                    }

                                }

                            }

                            willBeAffecteds.push_back(oneAffectRecord);

                            if (oneAffectRecord.size() < (std::size_t)affectLeastStateNum)
                            {
                                index = i;
                                affectLeastStateNum = oneAffectRecord.size();
                                bestBindingIt = candidateIt;
                            }

                        }

                        OC_ASSERT( (index != -1),
                                  "OCPlanner::doPlanning: Failed to find a group of variable bindings from the candidates with least affect to the solved state nodes!\n");

                        // apply the new bindings of least affect to the solved states
                        forwardRuleNode->currentBindingsViaSelecting =  *bestBindingIt;
                        forwardRuleNode->ParamCandidates.erase(bestBindingIt);
                        forwardRuleNode->updateCurrentAllBindings();
                        recordOrginalParamValuesAfterGroundARule(forwardRuleNode);

                        // have to rebind the affected state nodes and delete the affected effect states' branches
                        vector<StateNode*>::iterator affectedStateNodeIt;
                        for ( affectedStateNodeIt =  (willBeAffecteds[index]).begin(); affectedStateNodeIt !=  (willBeAffecteds[index]).end(); ++ affectedStateNodeIt)
                        {
                            StateNode* affectedStateNode = (StateNode* )(*affectedStateNodeIt);

                            deleteStateNodeInTemporaryList(affectedStateNode);

                            reBindStateNode(affectedStateNode,forwardRuleNode->currentAllBindings);

                            // now affectedStateNode is a new node after being rebinded
                            // only when there is other rule node used it as a predition , we need to put it into the unsatisfiedStateNodes list
                            if (affectedStateNode->forwardRuleNode != 0)
                                unsatisfiedStateNodes.push_back(affectedStateNode);

                            // the affectedStateNode has been rebinded, should not delete it as well
                            deleteRuleNodeRecursively(affectedStateNode->backwardRuleNode,affectedStateNode, false);

                        }

                        continue; // continue with the big while loop to check the next subgoal in unsatisfiedStateNodes

                    }
                }

            }
        }

        // Till now have selected one unsatisfied state and the rule to applied to try to do one step backward chaining to satisfy it
        // out put selected rule debug info:
        cout<<"Debug planning step " << tryStepNum <<": Selected rule :"<< selectedRule->ruleName << std::endl;

        // create a new RuleNode to apply this selected rule
        RuleNode* ruleNode = new RuleNode(selectedRule);
        ruleNode->number = ruleNodeCount ++;
        ruleNode->forwardLinks.push_back(curStateNode);
        curStateNode->backwardRuleNode = ruleNode; // the depth of curStateNode has been calculated when it created as a precondition of its forward rule node

        allRuleNodeInThisPlan.push_front(ruleNode);

        // When apply a rule, we need to select proper variables to ground it.

        // To ground a rule, first, we get all the variable values from the current state node to ground it.
        // ToBeImproved:  need to groundARuleNodeFromItsForwardState when choose which rule to apply, to avoid the rules that will cause a lot of effect on the already solved states
        groundARuleNodeFromItsForwardState(ruleNode, curStateNode);

        // And then is possible to still have some variables cannot be grounded by just copy from the forward state
        // So we need to select suitable variables to ground them.
        groundARuleNodeBySelectingNonNumericValues(ruleNode);

        if (ruleNode->ParamCandidates.size() > 0)
        {
            ruleNode->currentBindingsViaSelecting = ruleNode->ParamCandidates.front();
            ruleNode->ParamCandidates.erase(ruleNode->ParamCandidates.begin());
        }

        ruleNode->updateCurrentAllBindings();

        // if the actor of this rule has not been grounded till now, it indicates it doesn't matter who is to be the actor
        // so we just simply ground it as the self agent.
        if (Rule::isParamValueUnGrounded(ruleNode->originalRule->actor))
        {
            // look for the value of this variable in the parameter map
            string varName = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            ParamGroundedMapInARule::iterator paramMapIt = ruleNode->currentAllBindings.find(varName);
            if (paramMapIt == ruleNode->currentAllBindings.end())
            {
                ruleNode->currentAllBindings.insert(std::pair<string, ParamValue>(varName,selfEntityParamValue));
            }
        }

        // ToBeImproved: currently it can only solve the numeric state with only one ungrounded Numeric variable
        if (! selectValueForGroundingNumericState(ruleNode->originalRule,ruleNode->currentAllBindings,ruleNode))
        {
            cout << "SelectValueForGroundingNumericState failded!"<< std::endl;
            // todo
        }

        // out put all the bindings:
        cout<<"Debug planning step " << tryStepNum <<": All Variable bindings for rule :"<< ruleNode->originalRule->ruleName << std::endl;
        ParamGroundedMapInARule::iterator curparamit = ruleNode->currentAllBindings.begin();
        for (; curparamit != ruleNode->currentAllBindings.end(); ++ curparamit)
        {
            cout << curparamit->first << "= " << ActionParameter::ParamValueToString(curparamit->second) << std::endl;
        }

        recordOrginalParamValuesAfterGroundARule(ruleNode);

        //  find if there are other previous states besides curStateNode will be affected by this rule
        vector<EffectPair>::iterator effectItor;
        vector<ParamValue>::iterator oldValItor = ruleNode->orginalGroundedParamValues.begin();

        // out put all the effects debug info:
        cout<<"Debug planning step " << tryStepNum <<": All effects for rule :"<< ruleNode->originalRule->ruleName << std::endl;
        int effectNum = 1;
        for (effectItor = ruleNode->originalRule->effectList.begin(); effectItor != ruleNode->originalRule->effectList.end(); ++ effectItor, ++ oldValItor, ++effectNum)
        {
            Effect* e = (Effect*)(((EffectPair)(*effectItor)).second);

            State* effState =  Rule::groundAStateByRuleParamMap(e->state, ruleNode->currentAllBindings,true,false,(*oldValItor));
            OC_ASSERT( ( effState != 0),
                      "OCPlanner::doPlanning: effect state: %s is not able to be grounded.\n",
                       e->state->name().c_str());

            bool isEffectExecuted = Effect::executeEffectOp(effState,e,ruleNode->currentAllBindings);

            OC_ASSERT( isEffectExecuted,
                      "OCPlanner::doPlanning: in Effect::executeEffectOp of state: %s, the effect value is ungrounded.\n",
                       e->state->name().c_str());

            cout<<"Effect  " << effectNum <<": ";
            outputStateInfo(effState, false);
            cout << std::endl;
            cout << "           [value changed]:  "
                 << STATE_TYPE_NAME[e->state->stateType] << " " << ActionParameter::ParamValueToString(*oldValItor)
                 << " -> " << STATE_TYPE_NAME[effState->stateType]<< " "<< effState->stateVariable->stringRepresentation()
                 << std::endl;

            // skip the current state node
            if (curStateNode->state->isSameState(*effState) )
            {
                delete effState;
                continue;
            }

            // create a new state node for it
            StateNode* newStateNode = new StateNode(effState);
            newStateNode->backwardRuleNode = ruleNode;
            newStateNode->forwardRuleNode = 0;
            newStateNode->forwardEffectState = 0;
            temporaryStateNodes.push_front(newStateNode);

            list<StateNode*>::iterator unsait;

            // check if there are any unsatisfied nodes in previous steps will be satisfied by this effect
            for (unsait = unsatisfiedStateNodes.begin(); unsait != unsatisfiedStateNodes.end(); )
            {
                if (effState->isSameState( *((StateNode*)(*unsait))->state ))
                {
                    // check if this effect can satisfy this unsatisfiedState
                    float satDegree;
                    StateNode* unsatStateNode = (StateNode*)(*unsait);
                    if (effState->isSatisfied(*(unsatStateNode->state ),satDegree))
                    {
                        ruleNode->forwardLinks.push_back(unsatStateNode);
                        unsatisfiedStateNodes.erase(unsait);

                        unsatStateNode->backwardRuleNode = ruleNode;

                    }
                    else
                        ++ unsait;
                }
                else
                    ++ unsait;

            }


            list<StateNode*>::iterator sait;

            for (sait = temporaryStateNodes.begin(); sait != temporaryStateNodes.end(); )
            {
                // only check the state nodes without backward rule node,
                // because we are doing backward chaining, the state node which has backward rule node will be satisfied later
                if (((StateNode*)(*sait))->backwardRuleNode == 0)
                {
                     ++ sait;
                    continue;
                }

                if (effState->isSameState( *((StateNode*)(*sait))->state ))
                {
                    // check if this effect unsatisfy this state
                    float satDegree;
                    StateNode* satStateNode = (StateNode*)(*sait);
                    if (! effState->isSatisfied(*(satStateNode->state ),satDegree))
                    {
                        ruleNode->negativeForwardLinks.insert(satStateNode);

                        // if there is any other rule node in any forward step used it as a precondition, which suggests it's an intermediate goal that need to be satisfied,
                        // put it into the unsatisfied node list
                        if (satStateNode->forwardRuleNode != 0)
                        {
                            unsatisfiedStateNodes.push_back(satStateNode);
                        }

                        // remove it from the already satisfied node list
                        temporaryStateNodes.erase(sait);

                    }
                    else
                        ++ sait;
                }
                else
                    ++ sait;

            }

            // need to check in the goal state node list, if this effect change the already satisfied states in the goal list
            vector<StateNode*>::iterator goIt;
            for (goIt = satisfiedGoalStateNodes.begin(); goIt != satisfiedGoalStateNodes.end();)
            {
                StateNode* goalNode = (StateNode*)(*goIt);
                if (effState->isSameState( *(goalNode->state) ))
                {
                    float satDegree;

                    if (! effState->isSatisfied(*(goalNode->state) ,satDegree))
                    {
                        // This effect dissatisfied this goal, put this goal into the unsatisfied state node list and remove it from goal list
                        unsatisfiedStateNodes.push_back(goalNode);
                        satisfiedGoalStateNodes.erase(goIt);
                    }
                    else
                         ++ goIt;
                }
                else
                     ++ goIt;
            }

            newStateNode->calculateNodesDepth();

        }

        // move the current state node from unsatisfied to satisfied list
        temporaryStateNodes.push_front(curStateNode);
        unsatisfiedStateNodes.pop_back();

        // ground all the precondition state and create new nodes for each and add the forwardEffectStates
        // and then check each precondition if it has already been satisfied, put it the unsatisfied one to the
        vector<State*>::iterator itpre;
        float satisfiedDegree;
        // out put all the effects debug info:
        cout<<"Debug planning step " << tryStepNum <<": All preconditions for rule :"<< ruleNode->originalRule->ruleName << std::endl;
        int preConNum = 1;
        itpre = ruleNode->originalRule->preconditionList.end();
        itpre --;
        for (; ; -- itpre, ++preConNum)
        {
            State* ps = *itpre;
            State* groundPs = Rule::groundAStateByRuleParamMap(ps, ruleNode->currentAllBindings);

            // create a new state node for this grounded precondition state
            StateNode* newStateNode = new StateNode(groundPs);
            newStateNode->forwardRuleNode = ruleNode;
            newStateNode->backwardRuleNode = 0;
            newStateNode->forwardEffectState = ps;
            ruleNode->backwardLinks.push_back(newStateNode);
            newStateNode->calculateNodesDepth();

            // first check if this state has beed satisfied by the previous state nodes
            bool found = false;
            bool isSat = false;
            StateNode* satStateNode;

            bool satByTemp = checkIfThisGoalIsSatisfiedByTempStates(*groundPs, found, satStateNode,ruleNode,true);

            // if it's found in the temporaryStateNodes
            if (found)
            {
                if (satByTemp)
                {

                    // the current state satisfied this precondition,
                    // connect this precondition to the backward rule node of the state which satisfied this precondition

                    RuleNode* backwardRuleNode = satStateNode->backwardRuleNode;
                    if (backwardRuleNode)
                    {
                        satStateNode->backwardRuleNode->forwardLinks.push_back(newStateNode);
                        newStateNode->backwardRuleNode =  satStateNode->backwardRuleNode;
                    }

                    isSat = true;

                    // ToBeImproved: Need to check if there is any dependency loop,
                    // if any precondiction in the backward branch depends on any states in the forward branch of current rule node,
                    // it means two branches depend on each other. How to deal with this case?

                    // not need to add it in the temporaryStateNodes list, because it has beed satisfied by the previous steps

                }
                else
                {
                    if (newStateNode->backwardRuleNode)
                        newStateNode->backwardRuleNode->negativeForwardLinks.insert(newStateNode);

                    isSat = false;

                }
            }
            else
            {
                // cannot find this state in the temporaryStateNodes list, need to check it in real time
                // check real time
                if (! checkIsGoalAchievedInRealTime(*groundPs,satisfiedDegree))
                {
                    isSat = false;
                }
                else
                {
                    isSat = true;
                }

            }

            cout<<"Precondition  " << preConNum <<": ";
            outputStateInfo(groundPs, true);
            if(isSat)
            {
                temporaryStateNodes.push_front(newStateNode);
                cout << " is satisfied :)" << std::endl;
            }
            else
            {
                // add it to unsatisfied list
                unsatisfiedStateNodes.push_back(newStateNode);
                cout << " is unsatisfied :(" << std::endl;
            }

            if (itpre == ruleNode->originalRule->preconditionList.begin())
                break;
        }

        // execute the current rule action to change the imaginary SpaceMap if any action that involved changing space map
        // return the changed map
        SpaceServer::SpaceMap* newCurMap = executeActionInImaginarySpaceMap(ruleNode,curImaginaryMap);
        ruleNode->curMap = newCurMap;

    }

    // finished planning!

    // generate the action series according to the planning network we have constructed in this planning process
    planID = oac->getPAI().createActionPlan();

    std::cout<<std::endl<<"OCPlanner::Planning success! Plan ID = "<< planID <<std::endl;

    // sort the list of rule node
    allRuleNodeInThisPlan.sort();

    // and then encapsule the action for each step and send to PAI
    list<RuleNode*>::iterator planRuleNodeIt, lastStepIt;
    int stepNum = 1;
    for (planRuleNodeIt = allRuleNodeInThisPlan.begin(); planRuleNodeIt != allRuleNodeInThisPlan.end(); ++ planRuleNodeIt)
    {
        RuleNode* r = (RuleNode*)(*planRuleNodeIt);

        SpaceServer::SpaceMap* backwardStepMap ;

        if (stepNum == 1)
            backwardStepMap = &(spaceServer().getLatestMap());
        else
        {
            lastStepIt = planRuleNodeIt;
            lastStepIt --;
            backwardStepMap = ((RuleNode*)(*lastStepIt))->curMap;
        }

        // generate the action series according to the planning network we have constructed in this planning process
        PetAction* originalAction = r->originalRule->action;
        if (originalAction->getType().getCode() == DO_NOTHING_CODE)
            continue;

        // ground the parameter according to the current bindings
        PetAction action(originalAction->getType());
        list<ActionParameter>::const_iterator paraIt;
        const list<ActionParameter>& params = originalAction->getParameters();

        std::cout<<std::endl<<"Step No."<< stepNum << ": "<< originalAction->getName();

        // for navigation actions, need to call path finder to create every step of it
        if ((originalAction->getType().getCode() == WALK_CODE) || (originalAction->getType().getCode() == MOVE_TO_OBJ_CODE) )
        {
            spatial::BlockVector startPos,targetPos;

            ActionParameter oparam = (ActionParameter)params.front();

            ParamValue value;

            if  (Rule::isParamValueUnGrounded(oparam.getValue()))
            {
                ParamGroundedMapInARule::iterator bindIt = r->currentAllBindings.find(ActionParameter::ParamValueToString(oparam.getValue()));
                OC_ASSERT( (bindIt != r->currentAllBindings.end()),
                          "OCPlanner::sendPlan: in action %s, parameter: %s is ungrounded.\n",
                           originalAction->getType().getName().c_str() ,oparam.getName().c_str());
                value = bindIt->second;

            }
            else
                value =  oparam.getValue();

            if ((originalAction->getType().getCode() == WALK_CODE))
            {
                Vector v1 = boost::get<Vector>(value);
                targetPos = SpaceServer::SpaceMapPoint(v1.x,v1.y,v1.z);

                std::cout<< ActionParameter::ParamValueToString(v1)<<std::endl;
            }
            else
            {
                Entity entity1 = boost::get<Entity>(value);
                targetPos = backwardStepMap->getObjectLocation(entity1.id);

                std::cout<< entity1.stringRepresentation()<<std::endl;
            }

            // Get the right location of it at current step, from the orginalGroundedParamValues, it's the starting position before it moves

            // For any moving action, it's the second element in the effect list, as well as in the  orginalGroundedParamValues
            Vector v1 = boost::get<Vector>( r->orginalGroundedParamValues[1]);
            startPos = SpaceServer::SpaceMapPoint(v1.x,v1.y,v1.z);

            opencog::world::PAIWorldWrapper::createNavigationPlanAction(oac->getPAI(),*backwardStepMap,startPos,targetPos,planID);

        }
        else
        {
            for (paraIt = params.begin(); paraIt != params.end(); ++ paraIt)
            {
                ActionParameter oparam = (ActionParameter)(*paraIt);
                if  (Rule::isParamValueUnGrounded(oparam.getValue()))
                {
                    ParamGroundedMapInARule::iterator bindIt = r->currentAllBindings.find(ActionParameter::ParamValueToString(oparam.getValue()));
                    OC_ASSERT( (bindIt != r->currentAllBindings.end()),
                              "OCPlanner::sendPlan: in action %s, parameter: %s is ungrounded.\n",
                               originalAction->getType().getName().c_str() ,oparam.getName().c_str());

                    action.addParameter(ActionParameter(oparam.getName(),
                                                        oparam.getType(),
                                                        bindIt->second) );
                    std::cout<< ActionParameter::ParamValueToString(bindIt->second);
                }
                else
                {
                    action.addParameter(ActionParameter(oparam.getName(),
                                                        oparam.getType(),
                                                        oparam.getValue() ));
                    std::cout<< oparam.stringRepresentation();
                }

            }

            oac->getPAI().addAction(planID, action);

        }
        stepNum ++;
    }

    std::cout<<std::endl;

    // Reset the spaceMap for inquery back to the real spaceMap
    Inquery::reSetSpaceMap();

    // delete all imaginary space map
    vector<SpaceServer::SpaceMap*>::iterator iMapIt = imaginarySpaceMaps.begin();
    for (;iMapIt != imaginarySpaceMaps.end(); ++ iMapIt)
    {
        SpaceServer::SpaceMap* iMap = (SpaceServer::SpaceMap*)(*iMapIt);

        if (iMap)
            delete iMap;
    }

    // todo: remove all the imaginary atoms in imaginaryHandles

    return planID;
}

int OCPlanner::checkPreconditionFitness(RuleNode* ruleNode, bool &preconImpossible)
{
    int satisfiedPreconNum = 0;

    // check how many preconditions will be satisfied
    vector<State*>::iterator itpre;
    for (itpre = ruleNode->originalRule->preconditionList.begin(); itpre != ruleNode->originalRule->preconditionList.end(); ++ itpre)
    {
        State* ps = *itpre;
        State* groundPs = Rule::groundAStateByRuleParamMap(ps, ruleNode->currentBindingsFromForwardState);
        if (! groundPs)
            continue;

        // first check if this state has beed satisfied by the previous state nodes
        bool found = false;

        StateNode* satStateNode;

        bool satByTemp = checkIfThisGoalIsSatisfiedByTempStates(*groundPs, found, satStateNode,ruleNode,true);

        // if it's found in the temporaryStateNodes
        if (found)
        {
            if (satByTemp)
                ++ satisfiedPreconNum;
            else
            {
                // check if there is any rule related to achieve this unsatisfied precondition
                if (ruleEffectIndexes.find(groundPs->name()) == ruleEffectIndexes.end())
                {
                    preconImpossible = true;
                    return -999;
                }
            }

            delete groundPs;
            continue;

        }
        else
        {
            // cannot find this state in the temporaryStateNodes list, need to check it in real time
            // check real time
            float satisfiedDegree;
            if ( checkIsGoalAchievedInRealTime(*groundPs,satisfiedDegree))
                 ++ satisfiedPreconNum;
            else
            {
                // check if there is any rule related to achieve this unsatisfied precondition
                if (ruleEffectIndexes.find(groundPs->name()) == ruleEffectIndexes.end())
                {
                    preconImpossible = true;
                    return -999;
                }
            }

            delete groundPs;
            continue;

        }

    }

    return satisfiedPreconNum;
}

void OCPlanner::checkRuleFitnessRoughly(Rule* rule, StateNode* fowardState, int &satisfiedPreconNum, int &negateveStateNum, bool &negativeGoal,
                                        bool &isDiffStateOwnerType, bool &preconImpossible, bool onlyCheckIfNegativeGoal)
{

    RuleNode* tmpRuleNode = new RuleNode(rule);

    groundARuleNodeParametersFromItsForwardState(tmpRuleNode,fowardState);

    // if the actor of this rule has not been grounded till now, it indicates it doesn't matter who is to be the actor
    // so we just simply ground it as the self agent.
    if (Rule::isParamValueUnGrounded(rule->actor))
    {
        // look for the value of this variable in the parameter map
        string varName = ActionParameter::ParamValueToString(rule->actor);
        ParamGroundedMapInARule::iterator paramMapIt = tmpRuleNode->currentBindingsFromForwardState.find(varName);
        if (paramMapIt == tmpRuleNode->currentAllBindings.end())
        {
            tmpRuleNode->currentBindingsFromForwardState.insert(std::pair<string, ParamValue>(varName,selfEntityParamValue));
        }
    }

    negateveStateNum = 0;
    satisfiedPreconNum = 0;
    negativeGoal = false;
    isDiffStateOwnerType = false;
    preconImpossible = false;

    // check all the effects:
    vector<EffectPair>::iterator effectItor;

    for (effectItor = rule->effectList.begin(); effectItor != rule->effectList.end(); ++ effectItor)
    {
        Effect* e = (Effect*)(((EffectPair)(*effectItor)).second);

        State* effState =  Rule::groundAStateByRuleParamMap(e->state, tmpRuleNode->currentBindingsFromForwardState, false,false);

        if (! effState)
            continue;

        if (! Effect::executeEffectOp(effState,e,tmpRuleNode->currentBindingsFromForwardState))
        {
            delete effState;
            continue;
        }

        if (effState->isSameState(*(fowardState->state)))
        {
            if (e->ifCheckStateOwnerType )
            {
                if (! e->state->isStateOwnerTypeTheSameWithMe( *(fowardState->state)) )
                {
                    isDiffStateOwnerType = true;
                    return;
                }
            }

            float satDegree;
            if (! effState->isSatisfied(*(fowardState->state),satDegree))
            {
                negativeGoal = true;
                delete effState;
                return;
            }
        }

        list<StateNode*>::iterator sait;

        for (sait = temporaryStateNodes.begin(); sait != temporaryStateNodes.end(); ++ sait)
        {
            // skip the current state node
            if (fowardState == (StateNode*)(*sait))
                continue;

            // if a state node has not any forward rule, it means it is not used by any rule node yet, so it doesn't matter if it's affected or not
            if ((StateNode*)(*sait)->forwardRuleNode == 0)
               continue;
 /*
            // only check the state nodes without backward rule node,
            // because we are doing backward chaining, the state node which has backward rule node will be satisfied later
            if (((StateNode*)(*sait))->backwardRuleNode == 0)
            {
                continue;
            }
*/
            if (effState->isSameState( *((StateNode*)(*sait))->state ))
            {
                // check if this effect unsatisfy this state
                float satDegree;
                StateNode* satStateNode = (StateNode*)(*sait);
                if (! effState->isSatisfied(*(satStateNode->state ),satDegree))
                {
                    negateveStateNum ++;
                }
                else
                    continue;
            }

        }

        delete effState;
    }

    if (onlyCheckIfNegativeGoal)
    {
        delete tmpRuleNode;
        return;
    }

    // check how many preconditions will be satisfied
    satisfiedPreconNum = checkPreconditionFitness(tmpRuleNode,preconImpossible);

    delete tmpRuleNode;

}

// ToBeImproved: this function is very ugly...the right way is to add a callback function for each action to auto execute
SpaceServer::SpaceMap* OCPlanner::executeActionInImaginarySpaceMap(RuleNode* ruleNode, SpaceServer::SpaceMap *iSpaceMap)
{
    static int imaginaryBlockNum = 1;

    // currently we just cheat to enable the following actions
    // ToBeImproved: the right way is to add a callback function for each action to auto execute

    SpaceServer::SpaceMap* newClonedMap = iSpaceMap;

    switch (ruleNode->originalRule->action->getType().getCode())
    {
        case pai::EAT_CODE:
        {
            // get the handle of the food to eat
            newClonedMap = iSpaceMap->clone();
            string foodVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Entity foodEntity =  boost::get<Entity>((ruleNode->currentAllBindings)[foodVarName]);
            Handle foodH = AtomSpaceUtil::getEntityHandle(*atomSpace,foodEntity.id);
            newClonedMap->removeNoneBlockEntity(foodH);
            break;
        }
        case pai::MOVE_TO_OBJ_CODE:
        {
            // get the actor entity handle
            newClonedMap = iSpaceMap->clone();
            string actorVarName0 = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            Entity agent0 =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName0]);
            Handle agentH0 = AtomSpaceUtil::getAgentHandle(*atomSpace,agent0.id);

            // get the object want to move to
            string targetVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Entity target =  boost::get<Entity>((ruleNode->currentAllBindings)[targetVarName]);
            Handle targetH = AtomSpaceUtil::getAgentHandle(*atomSpace,target.id);
            // get new location it moves tol
            spatial::BlockVector targetLocation = newClonedMap->getObjectLocation(targetH);
            newClonedMap->updateNoneBLockEntityLocation(agentH0,targetLocation,curtimeStamp);
            break;
        }
        case pai::WALK_CODE:
        {
            // get the actor entity handle
            newClonedMap = iSpaceMap->clone();
            string actorVarName = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            Entity agent =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName]);
            Handle agentH = AtomSpaceUtil::getAgentHandle(*atomSpace,agent.id);

            // get the new location it moves to
            string newPosVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Vector movedToVec = boost::get<Vector>((ruleNode->currentAllBindings)[newPosVarName]);
            spatial::BlockVector newPos(movedToVec.x,movedToVec.y,movedToVec.z);
            newClonedMap->updateNoneBLockEntityLocation(agentH,newPos,curtimeStamp);

            break;
        }
        case pai::BUILD_BLOCK_CODE:
        {
            newClonedMap = iSpaceMap->clone();
            // get the  location of block
            string blockBuildPposVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Vector buildVec = boost::get<Vector>((ruleNode->currentAllBindings)[blockBuildPposVarName]);
            spatial::BlockVector buildPos(buildVec.x,buildVec.y,buildVec.z);
            Handle iBlockH = AtomSpaceUtil::addNode(*atomSpace, IMAGINARY_STRUCTURE_NODE,"Imaginary_Block_" + imaginaryBlockNum++);
            newClonedMap->addSolidUnitBlock(buildPos,iBlockH);
            break;
        }
        default:
            // TODO: TNick: is this the right way of handling other codes?
            break;
    }

    if (newClonedMap != iSpaceMap)
        imaginarySpaceMaps.push_back(newClonedMap);

    return newClonedMap;

}

void OCPlanner::undoActionInImaginarySpaceMap(RuleNode* ruleNode,SpaceServer::SpaceMap* iSpaceMap)
{
    // currently we just cheat to enable the following actions
    // ToBeImproved: this function is very ugly...the right way is to add a callback function for each action to auto execute

    switch (ruleNode->originalRule->action->getType().getCode())
    {
        case pai::EAT_CODE:
        {
            // get the handle of the food to eat
            string foodVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Entity foodEntity =  boost::get<Entity>((ruleNode->currentAllBindings)[foodVarName]);
            Handle foodH = AtomSpaceUtil::getEntityHandle(*atomSpace,foodEntity.id);
            // get the location last time it appeared
            iSpaceMap->addNoneBlockEntity(foodH,iSpaceMap->getLastAppearedLocation(foodH),1,1,1,0.0,foodEntity.id,foodEntity.type,false,curtimeStamp);

            break;
        }

        case pai::MOVE_TO_OBJ_CODE:
        {
            // get the actor entity handle
            string actorVarName0 = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            Entity agent0 =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName0]);
            Handle agentH0 = AtomSpaceUtil::getAgentHandle(*atomSpace,agent0.id);

            // get old location from the record before execute this effect
            Vector movedToVec0 = boost::get<Vector>((ruleNode->orginalGroundedParamValues)[1]);
            spatial::BlockVector targetLocation0(movedToVec0.x, movedToVec0.y, movedToVec0.z);
            iSpaceMap->updateNoneBLockEntityLocation(agentH0,targetLocation0,curtimeStamp);

            break;
        }

        case pai::WALK_CODE:
        {
            // get the actor entity handle
            string actorVarName = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            Entity agent =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName]);
            Handle agentH = AtomSpaceUtil::getAgentHandle(*atomSpace,agent.id);

            // get old location from the record before execute this effect
            Vector movedToVec = boost::get<Vector>((ruleNode->orginalGroundedParamValues)[1]);
            spatial::BlockVector targetLocation(movedToVec.x, movedToVec.y, movedToVec.z);
            iSpaceMap->updateNoneBLockEntityLocation(agentH,targetLocation,curtimeStamp);

            break;
        }

        case pai::BUILD_BLOCK_CODE:
        {
            // get the  location the new block is to be build at
            string blockBuildPposVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Vector buildVec = boost::get<Vector>((ruleNode->currentAllBindings)[blockBuildPposVarName]);
            spatial::BlockVector buildPos(buildVec.x,buildVec.y,buildVec.z);

            iSpaceMap->removeSolidUnitBlock(iSpaceMap->getUnitBlockHandleFromPosition(buildPos));
            break;
        }
        default:
            // TODO: TNick: is this the right way of handling other codes?
            break;
    }
}

void OCPlanner::reBindStateNode(StateNode* stateNode, ParamGroundedMapInARule& newBindings)
{
    State* newGroundedState = Rule::groundAStateByRuleParamMap(stateNode->state, newBindings);
    delete stateNode->state;
    stateNode->state = newGroundedState;
}

// delete a rule node and recursivly delete all its backward state nodes and rule nodes
void OCPlanner::deleteRuleNodeRecursively(RuleNode* ruleNode, StateNode* forwardStateNode, bool deleteThisforwardStateNode)
{

    // For the other forward state node of this rule node which is not the given forwardStateNode,
    // does not belong to the main branch of this given forwardStateNode
    // so don't need to delete such state nodes, only need to remove them from the temporaryStateNodes ,
    // and put them into the unsatisfied list if there is any other rule node used them as preconditions
    vector<StateNode*>::iterator forwardStateIt;
    for (forwardStateIt = ruleNode->forwardLinks.begin(); forwardStateIt != ruleNode->forwardLinks.end(); ++ forwardStateIt)
    {
        StateNode* curStateNode = (StateNode*)(*forwardStateIt);
        if (curStateNode == forwardStateNode)
            continue;

        deleteStateNodeInTemporaryList(curStateNode);

        if (curStateNode->forwardRuleNode != 0)
            unsatisfiedStateNodes.push_back(curStateNode);
    }

    if (forwardStateNode && deleteThisforwardStateNode)
        delete forwardStateNode;

    // check all its backward state nodes
    // delete them recursively
    vector<StateNode*>::iterator backwardStateIt;
    for (backwardStateIt = ruleNode->backwardLinks.begin(); backwardStateIt != ruleNode->backwardLinks.end(); ++ backwardStateIt)
    {
        StateNode* curStateNode = (StateNode*)(*backwardStateIt);

        deleteStateNodeInTemporaryList(curStateNode);

         if (curStateNode->backwardRuleNode != 0)
             deleteRuleNodeRecursively(curStateNode->backwardRuleNode, curStateNode);
         else
             delete curStateNode;

    }

    deleteRuleNodeInAllRuleNodeList(ruleNode);
    delete ruleNode;

}

void OCPlanner::deleteStateNodeInTemporaryList(StateNode* stateNode)
{
    list<StateNode*>::iterator it;
    for (it = temporaryStateNodes.begin(); it != temporaryStateNodes.end(); ++it)
    {
        if ((*it) == stateNode)
        {
            temporaryStateNodes.erase(it);
            break;
        }
    }
}

void OCPlanner::deleteRuleNodeInAllRuleNodeList(RuleNode *ruleNode)
{
    list<RuleNode*>::iterator it;
    for (it = allRuleNodeInThisPlan.begin(); it != allRuleNodeInThisPlan.end(); ++it)
    {
        if ((*it) == ruleNode)
        {
            allRuleNodeInThisPlan.erase(it);
            break;
        }
    }
}

// the forwardState can be a ungrounded state from other rule
// if cannot unify it , return 0
Rule* OCPlanner::unifyRuleVariableName(Rule* toBeUnifiedRule, State* forwardState )
{
    // first, find the state in the effect list of this rule which the same to this forward state
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    // Todo:  need to check if all the non-variables/consts are the same to find the exact state rather than just check the state name()
    for (effectIt = toBeUnifiedRule->effectList.begin(); effectIt != toBeUnifiedRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;

        s = e->state;
        if (s->name() ==  forwardState->name())
            break;
    }

    if (effectIt == toBeUnifiedRule->effectList.end())
        return 0;

    // check if all the stateOwner parameters grounded
    vector<ParamValue>::iterator f_ownerIt = forwardState->stateOwnerList.begin(); // state owner list in forward state
    vector<ParamValue>::iterator r_ownerIt = s->stateOwnerList.begin(); // state owner list in rule effect state

    map<string, ParamValue> currentBindingsFromForwardState;

    for ( ; r_ownerIt != s->stateOwnerList.end(); ++ f_ownerIt, ++r_ownerIt)
    {
        if (Rule::isParamValueUnGrounded(*r_ownerIt))
        {
            string variableName = ActionParameter::ParamValueToString((ParamValue)(*r_ownerIt));
            map<string, ParamValue>::iterator paraIt = currentBindingsFromForwardState.find(variableName);
            if (paraIt == currentBindingsFromForwardState.end())
            {
                // debug:
                string Forwardvalue =  ActionParameter::ParamValueToString((ParamValue)(*f_ownerIt));
                currentBindingsFromForwardState.insert(std::pair<string, ParamValue>(variableName,*f_ownerIt));
            }
        }
    }

    // unify actor
    ParamValue actor;
    if (Rule::isParamValueUnGrounded(toBeUnifiedRule->actor))
    {
        string variableName = ActionParameter::ParamValueToString(toBeUnifiedRule->actor);
        map<string, ParamValue>::iterator paraIt = currentBindingsFromForwardState.find(variableName);
        if (paraIt == currentBindingsFromForwardState.end())
            actor = toBeUnifiedRule->actor;
        else
            actor = paraIt->second;
    }

    Rule* unifiedRule = new Rule(toBeUnifiedRule->action,actor,toBeUnifiedRule->basic_cost);

    // unify preconditionList
    vector<State*>::iterator itpre = toBeUnifiedRule->preconditionList.begin();
    for (; itpre != toBeUnifiedRule->preconditionList.end(); ++ itpre)
    {
        State* unifiedState = Rule::groundAStateByRuleParamMap(*itpre,currentBindingsFromForwardState);
        if (unifiedState == 0)
            return 0;
        unifiedRule->addPrecondition(unifiedState);
    }

    // unify effect
    for (effectIt = toBeUnifiedRule->effectList.begin(); effectIt != toBeUnifiedRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;
        s = e->state;
        State* unifiedState = Rule::groundAStateByRuleParamMap(s,currentBindingsFromForwardState,false);
        if (unifiedState == 0)
            return 0;

        // unify effect operator value
        ParamValue opValue;
        if (Rule::isParamValueUnGrounded(e->opParamValue))
        {
            string variableName = ActionParameter::ParamValueToString(toBeUnifiedRule->actor);
            map<string, ParamValue>::iterator paraIt = currentBindingsFromForwardState.find(variableName);
            if (paraIt == currentBindingsFromForwardState.end())
                opValue = e->opParamValue;
            else
                opValue = paraIt->second;
        }

        Effect* unifiedEffect = new Effect(unifiedState, e->effectOp, opValue);

        unifiedRule->addEffect(EffectPair(effectIt->first,unifiedEffect));

    }

    // unify BestNumericVariableInqueryFun
    map<string,BestNumericVariableInqueryStruct>::iterator beIt = toBeUnifiedRule->bestNumericVariableinqueryStateFuns.begin();
    for (; beIt != toBeUnifiedRule->bestNumericVariableinqueryStateFuns.end(); ++ beIt)
    {
        BestNumericVariableInqueryStruct& bs = beIt->second;
        State* unifiedState = Rule::groundAStateByRuleParamMap(bs.goalState,currentBindingsFromForwardState,false);
        if (unifiedState == 0)
            return 0;

        BestNumericVariableInqueryStruct unifiedBS;
        unifiedBS.bestNumericVariableInqueryFun = bs.bestNumericVariableInqueryFun;
        unifiedBS.goalState = unifiedState;

        string varName;
        map<string, ParamValue>::iterator paraIt = currentBindingsFromForwardState.find(beIt->first);
        if (paraIt == currentBindingsFromForwardState.end())
            varName = beIt->first;
        else
            varName = ActionParameter::ParamValueToString(paraIt->second);

        unifiedRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(varName, unifiedBS));
    }

    // unify heuristics
    vector<CostHeuristic>::iterator costIt;
    for(costIt = toBeUnifiedRule->CostHeuristics.begin(); costIt != toBeUnifiedRule->CostHeuristics.end(); ++costIt)
    {
        State* cost_state = ((CostHeuristic)(*costIt)).cost_cal_state;
        State* unifiedState = Rule::groundAStateByRuleParamMap(cost_state,currentBindingsFromForwardState);
        if (unifiedState == 0)
            return 0;

        unifiedRule->addCostHeuristic(CostHeuristic(unifiedState,((CostHeuristic)(*costIt)).cost_coefficient));
    }

    unifiedRule->IsRecursiveRule = toBeUnifiedRule->IsRecursiveRule;

    return unifiedRule;

}

bool OCPlanner::groundARuleNodeParametersFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode)
{
    // first, find the state in the effect list of this rule which the same to this forward state
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    std::cout << "Debug: Begin grounding variables for rule: "<< ruleNode->originalRule->ruleName
              << " from its forward state " << forwardStateNode->state->name().c_str() << std::endl;

    // Todo:  need to check if all the non-variables/consts are the same to find the exact state rather than just check the state name()
    for (effectIt = ruleNode->originalRule->effectList.begin(); effectIt != ruleNode->originalRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;

        s = e->state;
        if (s->name() ==  forwardStateNode->state->name())
        {
            break;
        }
    }

    if (effectIt == ruleNode->originalRule->effectList.end())
        return false;

    // ground stateOwner parameters
    vector<ParamValue>::iterator f_ownerIt = forwardStateNode->state->stateOwnerList.begin(); // state owner list in forward state
    vector<ParamValue>::iterator r_ownerIt = s->stateOwnerList.begin(); // state owner list in rule effect state

    for ( ; r_ownerIt != s->stateOwnerList.end(); ++ f_ownerIt, ++r_ownerIt)
    {
        if (Rule::isParamValueUnGrounded(*r_ownerIt))
        {
            string variableName = ActionParameter::ParamValueToString((ParamValue)(*r_ownerIt));
            map<string, ParamValue>::iterator paraIt = ruleNode->currentBindingsFromForwardState.find(variableName);
            if (paraIt == ruleNode->currentBindingsFromForwardState.end())
            {
                std::cout << "Debug: Add currentBindingsFromForwardState pair: variableName = "<< variableName
                          << ", value = "<< ActionParameter::ParamValueToString(*f_ownerIt) << std::endl;
                ruleNode->currentBindingsFromForwardState.insert(std::pair<string, ParamValue>(variableName,*f_ownerIt));
            }
        }
    }

    // don't need to ground the old state value itself
    // just ground the effect op value
    if (Rule::isParamValueUnGrounded(e->opParamValue))
    {
        string variableName = ActionParameter::ParamValueToString(e->opParamValue);
        map<string, ParamValue>::iterator paraIt = ruleNode->currentBindingsFromForwardState.find(variableName);
        if (paraIt == ruleNode->currentBindingsFromForwardState.end())
        {
            std::cout << "Debug: Add currentBindingsFromForwardState pair: variableName = "<< variableName
                      << ", value = "<< ActionParameter::ParamValueToString(forwardStateNode->state->getParamValue()) << std::endl;
            ruleNode->currentBindingsFromForwardState.insert(std::pair<string, ParamValue>(variableName,forwardStateNode->state->getParamValue()));
        }
    }

    std::cout << "Debug: End grounding variables for rule: "<< ruleNode->originalRule->ruleName
              << " from its forward state " << forwardStateNode->state->name().c_str() << std::endl;

    return true;
}

bool OCPlanner::groundARuleNodeFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode)
{

    // First ground all the parameters
    if (! groundARuleNodeParametersFromItsForwardState (ruleNode,forwardStateNode))
        return false;

    // The cost calculation states should in the same context of the main parts of the rule, so it should not contain other new ungrounded variables
    // so we don't need to check the cost calculation states.
    if (ruleNode->originalRule->CostHeuristics.size() != 0)
        return true;

    // If there is no CostHeuristics in this rule, we need to borrow from the forward state node's foward rule node
    // In fact, it only make sense for recursive rules do this kind of borrowing.
    // Because the all the variables in foward rule CostHeuristics should have already been grounded, it doesn't make sense for a non-recursive rule to copy it.
    if (! ruleNode->originalRule->IsRecursiveRule)
        return true;

    // Now begin to do the "borrowing" for Recursive Rules

    // If the foward rule node doesn't have CostHeuristics, then we just leave the CostHeuristics empty.
    // It usually makes no sense to borrow from the foward rule node of the forward rule node, because the context is changing.

    // If this rule is the first rule in current planning, it doesn't have a forward rule to borrow from

    RuleNode* forwardRuleNode = forwardStateNode->forwardRuleNode;

    if (! forwardRuleNode)
        return true;

    // If the foward rule node's originalRule has CostHeuristics, borrow them.
    // Else if the forward rule node has CostHeuristics, borrow them.
    // Note: here need to distinguish the CostHeuristics of the originalRule, and the CostHeuristics of a rule Node
    // The CostHeuristics of the originalRule is the Cost Heuristics pre-defined by the orginal rule, which will not be changed during different planning processes.
    // The CostHeuristics of a rule Node is when there is no pre-defined CostHeuristics in the originalRule,
    //                                      it borrows from its forward rule node and put in the rule node, which usually change during different planning processes.
    if (forwardRuleNode->originalRule->CostHeuristics.size() != 0)
    {
        // because a recursive rule has the same state in effect and preconditions
        // we can copy the cost_state from forward Rule to every precondition of this recursive rule , and then add up them as the total cost heuristics of this rule node
        // e.g.: the forward rule is Move(x,y), precondition is ExistAPath(x,y), costheuristics is Distance(x,y)
        //       current recursive rule is if ExistAPath(x,m) & ExistAPath(m,y) then ExistAPath(x,y), has not costheuristics
        //       so that we can borrow from the "Move" rule, the number of this recursive rule's preconditions is 2, so the coefficient for each is 1/2 = 0.5
        //       so the total cost of this recursive rule = 0.5*Distance(x,m) + 0.5*(m,y)

        float coefficient = 1.0f / (ruleNode->originalRule->preconditionList.size());

        vector<State*>::iterator itpre;
        for (itpre = ruleNode->originalRule->preconditionList.begin(); itpre != ruleNode->originalRule->preconditionList.end(); ++ itpre)
        {

            vector<CostHeuristic>::iterator costIt;
            for(costIt = forwardRuleNode->originalRule->CostHeuristics.begin(); costIt != forwardRuleNode->originalRule->CostHeuristics.end(); ++costIt)
            {
                State* forward_cost_state = ((CostHeuristic)(*costIt)).cost_cal_state;
                State* cost_state = new State(forward_cost_state->name(),forward_cost_state->stateVariable->getType(),forward_cost_state->stateType,
                                              forward_cost_state->stateVariable->getValue(),forward_cost_state->need_inquery,forward_cost_state->inqueryStateFun);
                vector<ParamValue>::iterator ownerIt;
                for (ownerIt = forward_cost_state->stateOwnerList.begin(); ownerIt != forward_cost_state->stateOwnerList.end(); ++ ownerIt)
                {
                    // if this state_owner is a const, not a variable, just copy it
                    if (! Rule::isParamValueUnGrounded(*ownerIt))
                        cost_state->addOwner(*ownerIt);
                    else
                    {
                        // this state_owner is a variable, we need to change its variable name() into the corresponding variable name() in current rule
                        // e.g.: the forward rule is: MoveTo(pos1,pos2), cost is :Distance(pos1,pos2).
                        //       But the current rule variables are different: If ExistAPath(x,m) & ExistAPath(m,y), then ExistAPath(x,y)
                        //       so we need to make the cost in current rule like: Distance(x,m)+ Distance(m,y), using the variables x,m,y, rather than pos1, pos2

                        vector<ParamValue>::iterator f_rule_ownerIt = forwardStateNode->forwardEffectState->stateOwnerList.begin();
                        vector<ParamValue>::iterator cur_ownerIt = ((State*)(*itpre))->stateOwnerList.begin();
                        // This two state should be the same state just with possible different variable names
                        // So the state owners in the same order of bot stateOwnerLists should suggest the same usage
                        for ( ; f_rule_ownerIt != forwardStateNode->forwardEffectState->stateOwnerList.end(); ++ f_rule_ownerIt, ++ cur_ownerIt)
                        {
                            // need to find the state owner of this cost heuristic in the forward effect state
                            if ((*f_rule_ownerIt) == (*ownerIt))
                            {
                                // assign the state owner (variable name()) in the same position of current rule precondition to the new cost_state we creat
                                cost_state->addOwner(*cur_ownerIt);

                                break;
                            }
                        }

                        // If cannot find this state owner of this cost heuristic in the forward effect state, it means this owner doesn't affect the calculation of the cost in backward rule
                        // So in this case, we just bind this state owner in the backward cost_cal_state as the binded value in the forward rule node
                        if (f_rule_ownerIt == forwardStateNode->forwardEffectState->stateOwnerList.end())
                        {
                            // find the grounded value of this variable
                            ParamGroundedMapInARule::iterator bindIt = forwardRuleNode->currentBindingsFromForwardState.find(ActionParameter::ParamValueToString((ParamValue)(*ownerIt)));
                            OC_ASSERT(!(bindIt == forwardRuleNode->currentBindingsFromForwardState.end()),
                                      "OCPlanner::groundARuleNodeFromItsForwardState: Cannot find the binding of this variable:\n",
                                      ActionParameter::ParamValueToString((ParamValue)(*ownerIt)).c_str());

                            cost_state->addOwner(bindIt->second);

                        }
                    }

                }

                ruleNode->AddCostHeuristic(cost_state, ((CostHeuristic)(*costIt)).cost_coefficient * coefficient);

            }

        }

    }
    else if (forwardRuleNode->costHeuristics.size() != 0)
    {
        // The forward rule node has borrowed cost heuristics, it means the forward rule is also a recursive rule
        // TODO: If the forward rule is different from the current rule, not need to borrow it,BUT ususally they should be the same rule

        // So forward rule is the same rule with this current one, then just copy the CostHeuristics.
        // Need not to split the variables and do the adding up

        // Because they are the same rule, so they have the same variables names, so we even don't need to do the variable consistency processing
        vector<CostHeuristic>::iterator costIt;
        for(costIt = forwardRuleNode->costHeuristics.begin(); costIt != forwardRuleNode->costHeuristics.end(); ++costIt)
        {
            ruleNode->AddCostHeuristic((CostHeuristic)(*costIt));
        }

    }

    return true;

}


void OCPlanner::findAllUngroundedVariablesInARuleNode(RuleNode *ruleNode)
{
    if (ruleNode->curUngroundedVariables.size() != 0)
        return; // we have find All the Ungounded Variables for this rule before, we don't need to find them again

    map<string , vector<paramIndex> >::iterator paraIt = ruleNode->originalRule->paraIndexMap.begin();
    ParamGroundedMapInARule::iterator bindIt ;
    for( ; paraIt != ruleNode->originalRule->paraIndexMap.end(); ++ paraIt)
    {
        bindIt = ruleNode->currentBindingsFromForwardState.find(paraIt->first);

        // try to find this variable name() in the currentBindings
        // if cannot find it, it means this variable remains ungrounded, add it into the curUngroundedVariables
        if (bindIt == ruleNode->currentBindingsFromForwardState.end())
        {
            bool is_numeric_var = opencog::oac::isAVariableNumeric(paraIt->first);

            vector<paramIndex>::iterator indexIt = (paraIt->second).begin();
            for (; indexIt != (paraIt->second).end(); ++ indexIt)
            {
                if (((paramIndex)(*indexIt)).first == 0)
                    continue;

                list<UngroundedVariablesInAState>::iterator uvIt= ruleNode->curUngroundedVariables.begin();
                for (; uvIt != ruleNode->curUngroundedVariables.end(); ++ uvIt)
                {
                    if (((UngroundedVariablesInAState&)(*uvIt)).state == ((paramIndex)(*indexIt)).first)
                        break;
                }

                if (uvIt != ruleNode->curUngroundedVariables.end())
                {
                    ((UngroundedVariablesInAState&)(*uvIt)).vars.insert(paraIt->first);

                    if (is_numeric_var)
                        ((UngroundedVariablesInAState&)(*uvIt)).contain_numeric_var = true;

                }
                else
                {
                    ruleNode->curUngroundedVariables.push_back(UngroundedVariablesInAState(((paramIndex)(*indexIt)).first,paraIt->first));

                    std::cout<< "Debug: find ungrounded variable : " << paraIt->first << std::endl;

                }

            }

        }
    }

    // please see the operator < overloading function of UngroundedVariablesInAState for the sort rules
    // the state need inquery is more difficult to ground, so it should gound later
    // the numeric state should be grounded later
    // the state with less ungrounded variables will be put in front of this list
    ruleNode->curUngroundedVariables.sort();
}


void generateNextCombinationGroup(bool* &indexes, int n_max)
{
    int trueCount = -1;
    int i = 0;
    for (; i < n_max - 1; ++ i)
    {
        if (indexes[i]   )
        {
            ++ trueCount;

            if (! indexes[i+1])
                break;
        }
    }

    indexes[i] = false;
    indexes[i+1] = true;

    for (int j = 0; j < trueCount; ++ j)
        indexes[j] = true;

    for (int j = trueCount; j < i; ++ j)
        indexes[j] = false;

}

bool isLastNElementsAllTrue(bool* array, int size, int n)
{
    for (int i = size - 1; i >= size - n; i --)
    {
        if (! array[i])
            return false;
    }

    return true;
}


// this function should be called after groundARuleNodeFromItsForwardState.
// this function only ground non-numeric states
bool OCPlanner::groundARuleNodeBySelectingNonNumericValues(RuleNode *ruleNode)
{
    // First find all the ungrounded variables
    findAllUngroundedVariablesInARuleNode(ruleNode);

    // If there are multiple variables need to be grounded, the curUngroundedVariables list has already decided the order for grounding

    // First, get all the non-need-real-time-inquery states as the conditions for mattern matching
    // if cannot find any solution, then remove the last condition. Repeat this, till only have one condition left

    // Because the curUngroundedVariables list has already been in the order easiness of grounding,
    // so we just need to find out the first need-inquery one,
    // and only select variables by pattern matching for the states before

    // find the last state in the list ParamCandidates, which needs no real-time inquery state , and not numeric

    cout<<"Debug: groundARuleNodeBySelectingNonNumericValues() is finding candidate groups..." << std::endl;
    int number_easy_state = 0;
    list<UngroundedVariablesInAState>::iterator uvIt= ruleNode->curUngroundedVariables.begin();
    for (; uvIt != ruleNode->curUngroundedVariables.end(); ++ uvIt)
    {

        if (((((UngroundedVariablesInAState&)(*uvIt)).state)->need_inquery )
                || ((UngroundedVariablesInAState&)(*uvIt)).contain_numeric_var)
            break;
        else
        {
            // generate all the evaluationlink will be used to do pattern matching for each easy state
            ((UngroundedVariablesInAState&)(*uvIt)).PMLink = Inquery::generatePMLinkFromAState((((UngroundedVariablesInAState&)(*uvIt)).state), ruleNode);
            number_easy_state ++;
        }
    }

    // ToBeImproved: Is it possible that some non-need-real-time-inquery states contains some variables that need to be grounded by other need_real-time-inquery states?

    // find all the canditates meet as many as possible preconditions, and put all the candidates in ParamCandidates
    // first, find from the easy state via selecting values from the Atomspace

    int n_max = number_easy_state;
    bool* indexes = new bool[n_max];

    // ToBeImproved: in fact we don't need to generate all the possible groups of candidates at this moment,
    //               we can just allow it generateNextCandidates every time one group failed in a planning step
    // we generate the candidates by trying full combination calculation
    for (int n_gram = n_max; n_gram >= 1; -- n_gram)
    {
        // Use the binary method to generate all combinations:

        // generate the first combination
        for (int i = 0; i < n_gram; ++ i)
            indexes[i] = true;

        for (int i = n_gram; i <n_max; ++ i)
            indexes[i] = false;

        // the int in the pair is how many other real-time-inquery states this group of candicates meets
        list< TmpParamCandidate > tmpcandidates;

        while (true)
        {
            // the state indexes vector is the indexes of states in the curUngroundedVariables of this rule node
            // that will be used as conditions of patttern matching query in this function
            vector<int> indexesVector;

            for (int x = 0; x < n_max; ++ x)
            {
                if (indexes[x])
                    indexesVector.push_back(x);
            }
            vector<string> varNames;
            HandleSeq candidateListHandles = Inquery::findCandidatesByPatternMatching(ruleNode,indexesVector,varNames);
            int candidateGroupNum = 1;

            if (candidateListHandles.size() != 0)
            {

                foreach (Handle listH, candidateListHandles)
                {
                    HandleSeq candidateHandlesInOneGroup = atomSpace->getOutgoing(listH);
                    ParamGroundedMapInARule oneGroupCandidate;
                    int index = 0;

                    cout<<"CandidateGroup "<< candidateGroupNum ++ << std::endl;

                    foreach (Handle h, candidateHandlesInOneGroup)
                    {
                        ParamValue v = Inquery::getParamValueFromHandle(varNames[index],h);
                        oneGroupCandidate.insert(std::pair<string, ParamValue>(varNames[index],v));

                        cout<<varNames[index]<< "= " << ActionParameter::ParamValueToString(v) << std::endl;

                        // Check if this group of candidates is able to ground some other need_real_time_inquery states
                        // and how many of these need_real_time_inquery state can be satisifed by this group of candidates
                        int numOfSat = 0;
                        list<UngroundedVariablesInAState>::iterator nrtiIt= uvIt;
                        for (; nrtiIt != ruleNode->curUngroundedVariables.end(); ++ nrtiIt)
                        {
                            if (((UngroundedVariablesInAState&)(*nrtiIt)).contain_numeric_var)
                                break;

                            if (! ((UngroundedVariablesInAState&)(*nrtiIt)).state->need_inquery)
                                break;

                            ParamGroundedMapInARule tryBindings = ruleNode->currentBindingsFromForwardState;
                            tryBindings.insert(oneGroupCandidate.begin(),oneGroupCandidate.end());
                            State* groundedState = Rule::groundAStateByRuleParamMap(((UngroundedVariablesInAState&)(*nrtiIt)).state, tryBindings);
                            if (groundedState != 0)
                            {
                                bool found;
                                StateNode* satStateNode;
                                if (checkIfThisGoalIsSatisfiedByTempStates(*groundedState,found,satStateNode,0, false))
                                    numOfSat ++;
                                else if (found)
                                    continue; // found in temp state list, but is not satisfied
                                else
                                {
                                    // cannot find it in temp state list, check it in real time
                                    float satDegree;
                                    if (checkIsGoalAchievedInRealTime(*groundedState,satDegree ))
                                        numOfSat ++;
                                    else
                                        continue;
                                }

                            }

                            delete groundedState;

                        }

                        tmpcandidates.push_back(TmpParamCandidate(numOfSat, oneGroupCandidate));

                        index ++;
                    }

                }

            }

            if (isLastNElementsAllTrue(indexes, n_max, n_gram))
                break;

            generateNextCombinationGroup(indexes, n_max);

        }

        // Finished n-gram candidates generation
        // sort the groups of candidates for this n-gram, in the order of meeting as many as possible other need_real_time_inquery states
        // pls reference to above function:
        // bool operator < (const pair<int,ParamGroundedMapInARule>& pair1, const pair<int,ParamGroundedMapInARule>& pair2)
        tmpcandidates.sort();

        // now, it's in the right order, copy it to candidates
        list< TmpParamCandidate >::iterator tmpIt;
        for(tmpIt = tmpcandidates.begin(); tmpIt != tmpcandidates.end(); ++ tmpIt)
            ruleNode->ParamCandidates.push_back(((TmpParamCandidate&)(*tmpIt)).aGroupOfParmCandidate);

    }

    delete indexes;

    cout<<"Debug: groundARuleNodeBySelectingNonNumericValues() found candidate group number totally: "<<ruleNode->ParamCandidates.size()<<std::endl;


    // Till now,  all the easy states have been dealed with, now we need to deal with numberic states if any
    // we won't ground the numeric states here, because it's too time-consuming,
    // we won't give all the possible combinations of numeric values and non-numeric values for candidates

    return true;
}

// this should be called only after the currentAllBindings has been chosen
// ToBeImproved: currently it can only solve that kind of rules with only one ungrounded Numeric variable
bool OCPlanner::selectValueForGroundingNumericState(Rule* rule, ParamGroundedMapInARule& currentbindings, RuleNode* ruleNode)
{
    // todo: need to implement the general way of numeric selection without pre-defined or learnt heuristic functions

    // if this rule variable has pre-defined or learnt heuristic functions to inquery a best value which is most closed to this numeric goal
    // in fact all such variable with pre-defined inquery functions, can also be grounded by general way, but general way is slower,
    // so pre-defined inquery functions can make it more efficient, which may be a bit cheating, not AGI enough, but still make sense

    // only deal with the numeric states, assuming that all the other non-numeric variables should have been grounded
    // ToBeImproved: currently we are only able to deal with the numeric rule that only has one numeric ungrounded
    map<string,BestNumericVariableInqueryStruct>::iterator beIt = rule->bestNumericVariableinqueryStateFuns.begin();
    for (; beIt != rule->bestNumericVariableinqueryStateFuns.end(); ++ beIt)
    {
        string varName = (string)(beIt->first);
        // find in the currentAllBindings to find an ungrounded variable
        if (currentbindings.find(beIt->first) != currentbindings.end())
            continue;

        break;
    }

    if (beIt == rule->bestNumericVariableinqueryStateFuns.end())
        return true;

    // this variable has not been grouned , call its BestNumericVariableInquery to ground it
    // first, ground the state required by bestNumericVariableinqueryStateFun
    BestNumericVariableInqueryStruct& bs = (BestNumericVariableInqueryStruct&)(beIt->second);
    State* groundedState = Rule::groundAStateByRuleParamMap( bs.goalState,currentbindings,false,false,UNDEFINED_VALUE,false);
    if (groundedState == 0)
    {
        // todo: Currently we cannot solve such problem that this state cannot be grouded by previous grouding steps
        logger().error("OCPlanner::selectValueForGroundingNumericState :the goal state needed in BestNumericVariableInquery cannot be grouned in previous grouding steps. State name() is:,"
                       +  bs.goalState->name() );
        return false;
    }

    vector<ParamValue> values = bs.bestNumericVariableInqueryFun(groundedState->stateOwnerList);

    delete groundedState;

    if (values.size() == 0)
    {
        // cannot find a good value by this pre-defined inquery functions

        // if this is not a recursive rule, currently our framework cannot give out any soluction, just return failed
        if (! rule->IsRecursiveRule)
            return false;

        // for recursive rule, we borrow the preconditions from the unrecursive rule which has the same effect of this rule
        // ToBeImproved: now recursive rule only has one effect

        if (ruleNode == 0)
            return false;

        // find the first unrecursive rule
        // you can also find it by ruleEffectIndexes
        // ToBeImproved: currently we only borrow from the first unrecursive rule found
        Rule* unrecursiveRule = 0;

        map<string,multimap<float,Rule*> >::iterator it;
        it = ruleEffectIndexes.find(bs.goalState->name());

        multimap<float,Rule*>& rules = (multimap<float,Rule*>&)(it->second);

        multimap<float,Rule*> ::iterator ruleIt;

        for (ruleIt = rules.begin(); ruleIt != rules.end(); ruleIt ++)
        {
            Rule* r = ruleIt->second;
            if (! r->IsRecursiveRule)
            {
                unrecursiveRule = r;
                break;
            }
        }

        if (unrecursiveRule == 0)
            return false; // cannot find a unrecursiveRule to borrow from

        // ToBeImproved: currently only apply the borrowed rule in the first precondition of recursiveRule
        // ToBeImproved: need to find out if the numeric variable this unrecursive rule try to ground is applied in the first or second  precondition of this recursiveRule
        Rule* borrowedRule =  unifyRuleVariableName(unrecursiveRule, rule->preconditionList[0]);
        if (borrowedRule == 0)
            return false; // cannot unify the borrowed rule

        // because we have unified the rules, so the borrowed rule can use the same grounding map with
        if (! selectValueForGroundingNumericState(borrowedRule,currentbindings,ruleNode))
             return false; // cannot find a proper value from the borrowed rule

        //  check if this variable has been grounded by selectValueForGroundingNumericState from the borrowed rule
        if (currentbindings.find(beIt->first) != currentbindings.end())
            return true;
        else
            return false;

        // todo: for general way:
        // if there are more than two preconditions in this rule, borrow the one without any other backward rule to satisfy it, as hard restrics for selecting values.
        // because others preconditions can be achieved by other rules.
        // so that we can select values at least satisfied one condition.

    }

    //  select the best one in this vector<ParamValue> that get the highest score according to the heuristics cost calculations
    ParamValue bestValue;

    if (rule->CostHeuristics.size() != 0)
    {
        bestValue = selectBestNumericValueFromCandidates(rule,rule->basic_cost, rule->CostHeuristics,currentbindings, beIt->first,values);
    }
    else if (ruleNode->costHeuristics.size()!= 0)
    {
        bestValue = selectBestNumericValueFromCandidates(rule,0.0f, ruleNode->costHeuristics,currentbindings, beIt->first,values);
    }
    else
    {
        cout<< "Debug: OCPlanner::selectValueForGroundingNumericState: this rule doesn't have any costHeuristics for calculation! Randomly select one for it." <<std::endl;
        bestValue = values.front();
    }

    if (bestValue == UNDEFINED_VALUE)
    {
        logger().error("OCPlanner::selectValueForGroundingNumericState: failed to find the best value for grounding numeric state!" );
        return false;
    }

    currentbindings.insert(std::pair<string, ParamValue>(beIt->first,bestValue));
    return true;


}



ParamValue OCPlanner::selectBestNumericValueFromCandidates(Rule* rule, float basic_cost, vector<CostHeuristic>& costHeuristics, ParamGroundedMapInARule& currentbindings, string varName, vector<ParamValue>& values)
{

    // check how many preconditions will be satisfied
    RuleNode* tmpRuleNode = new RuleNode(rule);
    tmpRuleNode->currentAllBindings = currentbindings;

    vector<ParamValue>::iterator vit;
    float bestScore = -999999.9;
    ParamValue bestValue = UNDEFINED_VALUE;

    for (vit = values.begin(); vit != values.end(); ++ vit)
    {
        tmpRuleNode->currentAllBindings.insert(std::pair<string, ParamValue>(varName,*vit));
        currentbindings.insert(std::pair<string, ParamValue>(varName,*vit));

        float cost = Rule::getCost(basic_cost, costHeuristics, currentbindings);
        if (cost < -0.00001f)
        {
            logger().error("OCPlanner::selectBestNumericValueFromCandidates: this rule has not been grounded fully!" );
            return UNDEFINED_VALUE;
        }

        bool preconImpossible;
        int satisfiedPreconNum = checkPreconditionFitness(tmpRuleNode,preconImpossible);
        float score = satisfiedPreconNum * 10.0f - cost;
        if (preconImpossible)
            score -= 99999.9f;

        currentbindings.erase(varName);
        tmpRuleNode->currentAllBindings.erase(varName);

        if (score > bestScore)
        {
            bestScore = score;
            bestValue = *vit;
        }
    }

    if (bestScore < -999999.89)
    {
        logger().error("OCPlanner::selectBestNumericValueFromCandidates failed! Cannot find a best value!" );
        return UNDEFINED_VALUE;
    }

    delete tmpRuleNode;
    return bestValue;
}

// this function should be called after completely finished grounding a rule.but it is before the effect taking place.
void OCPlanner::recordOrginalParamValuesAfterGroundARule(RuleNode* ruleNode)
{
    vector<EffectPair>::iterator effectIt;
    Effect* e;

    ruleNode->orginalGroundedParamValues.clear();

    for (effectIt = ruleNode->originalRule->effectList.begin(); effectIt != ruleNode->originalRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;
        State* s = Rule::groundAStateByRuleParamMap(e->state, ruleNode->currentAllBindings,false,false);
        OC_ASSERT( (s != 0),
                  "OCPlanner::recordOrginalParamValuesAfterGroundARule: cannot ground state: %s!\n",
                    s->name().c_str());

        StateNode* stateNode;
        if (findStateInTempStates(*s, 0, stateNode, false))
        {
            ruleNode->orginalGroundedParamValues.push_back(stateNode->state->getParamValue());
        }
        else if (findStateInStartStateNodes(*s,stateNode))
        {
            ruleNode->orginalGroundedParamValues.push_back(stateNode->state->getParamValue());
        }
        else
        {
            ruleNode->orginalGroundedParamValues.push_back(s->getParamValue());
        }

        delete s;

    }
}

void OCPlanner::outputStateInfo(State* s,bool outPutStateValue)
{
    cout<< "State: "<< s->name() << " ( " ;
    vector<ParamValue>::iterator ownerIt;
    bool isFirstOne = true;
    for (ownerIt = s->stateOwnerList.begin(); ownerIt != s->stateOwnerList.end(); ++ ownerIt)
    {
        if (! isFirstOne)
            cout<<", ";
        cout<< ActionParameter::ParamValueToString( (ParamValue&)(*ownerIt) );
        isFirstOne = false;
    }
    cout<< " )";

    if (outPutStateValue)
    {
        cout << " " << STATE_TYPE_NAME[s->stateType] << " " << s->stateVariable->stringRepresentation();
    }

}

// a bunch of rules for test, load from c++ codes
void OCPlanner::loadTestRulesFromCodes()
{
    // define a special action: do nothing
    PetAction* doNothingAction = new PetAction(ActionType::DO_NOTHING());

    //----------------------------Begin Rule: if energy value is high enough, the energy goal is achieved-------------------------------------------
//    ParamValue var_avatar = entity_var[1];
//    ParamValue var_achieve_energy_goal = bool_var[1];

//    // precondition 1:
//    vector<ParamValue> energyStateOwnerList0;
//    energyStateOwnerList0.push_back(var_avatar);
//    State* energyState0 = new State("Energy",ActionParamType::FLOAT(),STATE_GREATER_THAN ,ParamValue("0.8"), energyStateOwnerList0);
//    // effect1: energy increases
//    State* energyGoalState = new State("EnergyGoal",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,var_achieve_energy_goal, energyStateOwnerList0);

//    Effect* energyGoalAchievedEffect = new Effect(energyGoalState, OP_ASSIGN, SV_TRUE);

//    Rule* highEnergyAchieveEnergyGoalRule = new Rule(doNothingAction,boost::get<Entity>(var_avatar),0.0f);
//    highEnergyAchieveEnergyGoalRule->addPrecondition(energyState0);

//    highEnergyAchieveEnergyGoalRule->addEffect(EffectPair(1.0f,energyGoalAchievedEffect));

//    this->AllRules.push_back(highEnergyAchieveEnergyGoalRule);

    //----------------------------End Rule: increase energy is to achieve energygoal-------------------------------------------

    //----------------------------Begin Rule: eat food to achieve energy demanding goal -------------------------------------------
    // define variables:
    ParamValue var_food = entity_var[0];
    ParamValue var_achieve_energy_goal = bool_var[1];
    ParamValue var_avatar =  entity_var[1];

    std::cout<<"holder: "<< ActionParameter::ParamValueToString(var_avatar).c_str()<< std::endl;

    // Add rule: increasing energy by eat an edible object held in hand

    // precondition 1:food exists
    vector<ParamValue> existStateOwnerList;
    existStateOwnerList.push_back(var_food);

    string statename = "exist";
    State* existState = new State(statename,ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,SV_TRUE, existStateOwnerList, true, &opencog::oac::Inquery::inqueryExist);

    // precondition 2: The agent hold an object
    vector<ParamValue> holderStateOwnerList;
    holderStateOwnerList.push_back(var_food);


    State* holderState = new State("holder",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_avatar, holderStateOwnerList);

    // precondition 3: This object is ediable
    vector<ParamValue> edibleStateOwnerList;
    edibleStateOwnerList.push_back(var_food);
    State* edibleState = new State("is_edible",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,SV_TRUE, edibleStateOwnerList);

    // action: eat
    PetAction* eatAction = new PetAction(ActionType::EAT());
    eatAction->addParameter(ActionParameter("target",
                                        ActionParamType::ENTITY(),
                                        var_food));

    // energy state:
    vector<ParamValue> energyStateOwnerList;
    energyStateOwnerList.push_back(var_avatar);
    State* energyGoalState = new State("EnergyDemandGoal",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,var_achieve_energy_goal, energyStateOwnerList);


    // effect1: energy Goal Achieved
    Effect* energyGoalAchievedEffect = new Effect(energyGoalState, OP_ASSIGN, SV_TRUE);

    // effect2: the food disappear
    Effect* foodDisappearEffect = new Effect(existState, OP_ASSIGN, SV_FALSE);

    // effect3: no one holds the food any more
    Effect* nonHolderEffect = new Effect(holderState, OP_ASSIGN, Entity::NON_Entity);

    // rule: increasing energy by eat an edible object held in hand
    Rule* eatRule = new Rule(eatAction,boost::get<Entity>(var_avatar),0.2f);
    eatRule->ruleName = "eatFoodtoAchieveEnergyDemand";
    eatRule->addPrecondition(existState);
    eatRule->addPrecondition(edibleState);
    eatRule->addPrecondition(holderState);

    eatRule->addEffect(EffectPair(1.0f,energyGoalAchievedEffect));
    eatRule->addEffect(EffectPair(1.0f,nonHolderEffect));
    eatRule->addEffect(EffectPair(1.0f,foodDisappearEffect));

    this->AllRules.push_back(eatRule);

    //----------------------------End Rule: eat food to increase energy-------------------------------------------

    //----------------------------Begin Rule: pick up an object to hold it if closed enough-------------------------------
    // define variables:
    ParamValue varAvatar = entity_var[0];
    ParamValue varFood = entity_var[1];
    ParamValue var_holder = entity_var[2];

    // precondition 1: The agent and the object is closed enough ( e.g. < 2.0)
    vector<ParamValue> closedStateOwnerList;
    closedStateOwnerList.push_back(varAvatar);
    closedStateOwnerList.push_back(varFood);
    ParamValue svtest = ACCESS_DISTANCE;
    State* closedState = new State("Distance",ActionParamType::FLOAT(),STATE_LESS_THAN ,svtest, closedStateOwnerList, true, &Inquery::inqueryDistance);

    // precondition 2: The object can be picked up
    vector<ParamValue> pickupableStateOwnerList;
    pickupableStateOwnerList.push_back(varFood);
    State* pickupableState = new State("is_pickupable",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,"true", pickupableStateOwnerList);

    // todo: precondition 3: The agent doesn't hold other object currently
    // todo: precondition 4: The object is not be held by other agent currently

    // action: pick up
    PetAction* pickupAction = new PetAction(ActionType::GRAB());
    pickupAction->addParameter(ActionParameter("target",
                                        ActionParamType::ENTITY(),
                                        varFood));

    // effect1: The agent hold this object
    // holder state
    vector<ParamValue> holderStateOwnerList2;
    holderStateOwnerList2.push_back(varFood);
    State* holderState2 = new State("holder",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_holder, holderStateOwnerList2);
    Effect* holderEffect = new Effect(holderState2, OP_ASSIGN, varAvatar);

    // rule:  pick up an object if closed enough, to hold it
    Rule* pickupRule = new Rule(pickupAction,boost::get<Entity>(varAvatar),0.1f);
    pickupRule->ruleName = "pickupObjecttoHoldIt";
    pickupRule->addPrecondition(pickupableState);
    pickupRule->addPrecondition(closedState);

    pickupRule->addEffect(EffectPair(1.0f,holderEffect));

    this->AllRules.push_back(pickupRule);

    //----------------------------End Rule: to pick up an object if closed enough-------------------------------------------

    //----------------------------Begin Rule: Move_to an object to get closed to it-----------------------------------------
    // define variables:
    ParamValue var_obj = entity_var[0];
    ParamValue float_dis = float_var[0];
    ParamValue var_oldpos = vector_var[1];

    // precondition 1:There exists a path from the agent to object
    vector<ParamValue> existPathStateOwnerList;
    existPathStateOwnerList.push_back(var_avatar);
    existPathStateOwnerList.push_back(var_obj);
    State* existPathState = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList, true, &Inquery::inqueryExistPath);

    // action: move to object
    PetAction* moveToObjectAction = new PetAction(ActionType::MOVE_TO_OBJ());
    moveToObjectAction->addParameter(ActionParameter("target",
                                        ActionParamType::ENTITY(),
                                        boost::get<Entity>(var_obj)));

    // effect: get closed to the object
    vector<ParamValue> closedStateOwnerList2;
    closedStateOwnerList2.push_back(var_avatar);
    closedStateOwnerList2.push_back(var_obj);
    State* closedState2 = new State("Distance",ActionParamType::FLOAT(),STATE_EQUAL_TO , float_dis , closedStateOwnerList2, true, &Inquery::inqueryDistance);
    Effect* getClosedEffect = new Effect(closedState2, OP_ASSIGN_LESS_THAN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<ParamValue> atLocationStateOwnerList2;
    atLocationStateOwnerList2.push_back(var_avatar);
    State* atLocationState2 = new State("AtLocation",ActionParamType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList2, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect2 = new Effect(atLocationState2, OP_ASSIGN_NOT_EQUAL_TO, var_oldpos);

    // rule:   Move_to an object to get closed to it
    Rule* movetoObjRule = new Rule(moveToObjectAction,boost::get<Entity>(var_avatar) ,0.01f);
    movetoObjRule->ruleName = "movetoObjectTogetClosedToIt";
    movetoObjRule->addPrecondition(existPathState);

    movetoObjRule->addEffect(EffectPair(0.9f,getClosedEffect));
    movetoObjRule->addEffect(EffectPair(0.9f,changedLocationEffect2));

    movetoObjRule->addCostHeuristic(CostHeuristic(closedState2, 0.01f));

    this->AllRules.push_back(movetoObjRule);
    //----------------------------End Rule: Move_to an object to get closed to it-------------------------------------------

    //----------------------------Begin Rule: walk to a position to get closed to it  and stand on it-----------------------------------------
    // define variables:
    ParamValue var_pos = vector_var[0];

    // precondition 1:There exists a path from the agent to object
    vector<ParamValue> existPathStateOwnerList2;
    existPathStateOwnerList2.push_back(var_avatar);
    existPathStateOwnerList2.push_back(var_pos);
    State* existPathState2 = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList2, true, &Inquery::inqueryExistPath);

    // action: walk to an position
    PetAction* walkAction = new PetAction(ActionType::WALK());
    walkAction->addParameter(ActionParameter("target",
                                            ActionParamType::VECTOR(),
                                            var_pos));

    // effect1: get closed to the position
    vector<ParamValue> closedStateOwnerList3;
    closedStateOwnerList3.push_back(var_avatar);
    closedStateOwnerList3.push_back(var_pos);
    State* closedState3 = new State("Distance",ActionParamType::FLOAT(),STATE_EQUAL_TO ,float_dis, closedStateOwnerList3, true, &Inquery::inqueryDistance);
    Effect* getClosedEffect2 = new Effect(closedState3, OP_ASSIGN_LESS_THAN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<ParamValue> atLocationStateOwnerList;
    atLocationStateOwnerList.push_back(var_avatar);
    State* atLocationState = new State("AtLocation",ActionParamType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect = new Effect(atLocationState, OP_ASSIGN, var_pos);

    // rule:   Move_to an object to get closed to it
    Rule* walkRule = new Rule(walkAction,boost::get<Entity>(var_avatar) ,0.01f);
    walkRule->ruleName = "waldToPositionToGetClosedToItAndStandOnIt";
    walkRule->addPrecondition(existPathState2);

    walkRule->addEffect(EffectPair(0.9f,getClosedEffect2));
    walkRule->addEffect(EffectPair(0.9f,changedLocationEffect));

    walkRule->addCostHeuristic(CostHeuristic(closedState3, 0.01f));

    this->AllRules.push_back(walkRule);
    //----------------------------End Rule: walk to a position to get closed to it and stand on it-----------------------------------------

    //----------------------------Begin Rule: walk closed a position to get closed to it, but not to stand on it-----------------------------------------

    // precondition 1:There exists a path from the agent to a position closed to the target postion
    ParamValue nearby_pos = vector_var[0];
    ParamValue target_pos = vector_var[2];

    vector<ParamValue> existPathStateOwnerList7;
    existPathStateOwnerList7.push_back(var_avatar);
    existPathStateOwnerList7.push_back(nearby_pos);
    State* existPathState7 = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList7, true, &Inquery::inqueryExistPath);

    // precondition 2: nearby_pos is adjacent to target_pos
    vector<ParamValue> adjacentStateOwnerList0;
    adjacentStateOwnerList0.push_back(target_pos);
    adjacentStateOwnerList0.push_back(nearby_pos);
    State* adjacentState0 = new State("is_adjacent",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", adjacentStateOwnerList0, true, &Inquery::inqueryIsAdjacent);

    // action: walk to the nearby_pos
    PetAction* walkAction0 = new PetAction(ActionType::WALK());
    walkAction0->addParameter(ActionParameter("target",
                                            ActionParamType::VECTOR(),
                                            nearby_pos));

    // effect1: get closed to the position
    vector<ParamValue> closedStateOwnerList7;
    closedStateOwnerList7.push_back(var_avatar);
    closedStateOwnerList7.push_back(target_pos);
    State* closedState7 = new State("Distance",ActionParamType::FLOAT(),STATE_EQUAL_TO ,float_dis, closedStateOwnerList7, true, &Inquery::inqueryDistance);
    Effect* getClosedEffect7 = new Effect(closedState7, OP_ASSIGN_LESS_THAN, CLOSED_DISTANCE);

    // effect2: position changed
    vector<ParamValue> atLocationStateOwnerList7;
    atLocationStateOwnerList7.push_back(var_avatar);
    State* atLocationState7 = new State("AtLocation",ActionParamType::VECTOR(),STATE_EQUAL_TO, var_oldpos, atLocationStateOwnerList7, true, &Inquery::inqueryAtLocation);
    Effect* changedLocationEffect7 = new Effect(atLocationState7, OP_ASSIGN, nearby_pos);

    // rule:  walk to a position to get closed to it, but not stand on it
    Rule* walkclosedRule = new Rule(walkAction,boost::get<Entity>(var_avatar) ,0.01f);
    walkclosedRule->ruleName = "waldToPositionToGetClosedToItButNotStandOnIt";
    walkclosedRule->addPrecondition(existPathState7);
    walkclosedRule->addPrecondition(adjacentState0);

    walkclosedRule->addEffect(EffectPair(0.9f,getClosedEffect7));
    walkclosedRule->addEffect(EffectPair(0.9f,changedLocationEffect7));

    vector<ParamValue> closedStateOwnerList8;
    closedStateOwnerList8.push_back(var_avatar);
    closedStateOwnerList8.push_back(nearby_pos);
    State* closedState8 = new State("Distance",ActionParamType::FLOAT(),STATE_EQUAL_TO ,float_dis, closedStateOwnerList8, true, &Inquery::inqueryDistance);

    walkclosedRule->addCostHeuristic(CostHeuristic(closedState8, 0.01f));

    BestNumericVariableInqueryStruct bs2;
    bs2.bestNumericVariableInqueryFun = &Inquery::inqueryAdjacentPosition;
    bs2.goalState = adjacentState0;
    walkclosedRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(ActionParameter::ParamValueToString(nearby_pos), bs2));

    this->AllRules.push_back(walkclosedRule);
    //----------------------------End Rule: walk to a position to get closed to it, but not stand on it-----------------------------------------

    //----------------------------Begin Rule: build a block in a position to make it possible to stand on it-----------------------------------------
    // define variables:
    ParamValue var_pos_on = vector_var[1];
    ParamValue var_is_standable = bool_var[0];

    // precondition 1: This pos should be empty, if it already has a block in it, you cannot build another block in it
    vector<ParamValue> solidStateOwnerList;
    solidStateOwnerList.push_back(var_pos);
    State* solidState = new State("is_solid",ActionParamType::BOOLEAN(),STATE_EQUAL_TO, "false", solidStateOwnerList, true, &Inquery::inqueryIsSolid);

    // precondition 2: The pos on it should be empty, if it has a block in it, you cannot stand in it
    vector<ParamValue> solidStateOwnerList2;
    solidStateOwnerList2.push_back(var_pos_on);
    State* solidState2 = new State("is_solid",ActionParamType::BOOLEAN(),STATE_EQUAL_TO, "false", solidStateOwnerList2, true, &Inquery::inqueryIsSolid);

    // precondition 3: The agent should be closed enough to the position to build the block ( < 1.5)
    vector<ParamValue> closedStateOwnerList4;
    closedStateOwnerList4.push_back(varAvatar);
    closedStateOwnerList4.push_back(var_pos);
    State* closedState4 = new State("Distance",ActionParamType::FLOAT(),STATE_LESS_THAN ,ACCESS_DISTANCE, closedStateOwnerList4, true, &Inquery::inqueryDistance);

    // precondition 4: The agent should not stand on the position to build the block
    vector<ParamValue> atLocationStateOwnerList3;
    atLocationStateOwnerList3.push_back(varAvatar);
    State* atLocationState3 = new State("AtLocation",ActionParamType::VECTOR(),STATE_NOT_EQUAL_TO, var_pos, atLocationStateOwnerList3, true, &Inquery::inqueryAtLocation);

    // precondition 5: The position to bulid a block should be just on the desired position to stand on
    vector<ParamValue> IsBelowStateOwnerList;
    IsBelowStateOwnerList.push_back(var_pos);
    IsBelowStateOwnerList.push_back(var_pos_on);
    State* IsBelowState = new State("is_below",ActionParamType::BOOLEAN(),STATE_EQUAL_TO, "true",
                                            IsBelowStateOwnerList, true, &Inquery::inqueryIsBelow);
    vector<ParamValue> IsTouchingStateOwnerList;
    IsTouchingStateOwnerList.push_back(var_pos);
    IsTouchingStateOwnerList.push_back(var_pos_on);
    State* IsTouchingState = new State("is_touching",ActionParamType::BOOLEAN(),STATE_EQUAL_TO, "true",
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
    vector<ParamValue> standableStateOwnerList;
    standableStateOwnerList.push_back(var_pos_on);
    State* standableState = new State("is_standable",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , var_is_standable, standableStateOwnerList, true, &Inquery::inqueryIsStandable);
    Effect* becomeStandableEffect2 = new Effect(standableState, OP_ASSIGN, "true");

    // effect2: the position to build this block become solid
    Effect* becomeSolidEffect = new Effect(solidState, OP_ASSIGN, "true");

    // add rule:
    Rule* buildBlockRule = new Rule(buildBlockAction,boost::get<Entity>(varAvatar) ,0.5f);
    buildBlockRule->ruleName = "buildABlockToEnableThisPositionStandable";
    buildBlockRule->addPrecondition(solidState);
    buildBlockRule->addPrecondition(solidState2);
    buildBlockRule->addPrecondition(closedState4);
    buildBlockRule->addPrecondition(atLocationState3);
    buildBlockRule->addPrecondition(IsBelowState);
    buildBlockRule->addPrecondition(IsTouchingState);

    buildBlockRule->addEffect(EffectPair(0.8f,becomeStandableEffect2));
    buildBlockRule->addEffect(EffectPair(1.0f,becomeSolidEffect));

    BestNumericVariableInqueryStruct bs3;
    bs3.bestNumericVariableInqueryFun = &Inquery::inqueryUnderPosition; // the function to get the position just under the var_pos_on, so is to get var_pos given var_pos_on is grounded.
    bs3.goalState = solidState2;
    buildBlockRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(ActionParameter::ParamValueToString(var_pos), bs3));

    this->AllRules.push_back(buildBlockRule);

    //----------------------------End Rule: build a block in a position to make it possible to stand on it-----------------------------------------



    //----------------------------Begin Rule: if a position is standable and adjacent(neighbour) then there is possible existing a path from here to this adjacent postion------------------
    // define variables:
    ParamValue var_pos_from = vector_var[0];
    ParamValue var_pos_to = vector_var[1];
    ParamValue var_exist_path = bool_var[0];

    // precondition 1: the end position is standable
    vector<ParamValue> standableStateOwnerList2;
    standableStateOwnerList2.push_back(var_pos_to);
    State* standableState2 = new State("is_standable",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", standableStateOwnerList2, true, &Inquery::inqueryIsStandable);

    // precondition 2: the start position is standable
    vector<ParamValue> standableStateOwnerList3;
    standableStateOwnerList3.push_back(var_pos_from);
    State* standableState3 = new State("is_standable",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", standableStateOwnerList3, true, &Inquery::inqueryIsStandable);

    // precondition 3: the two positions are adjacent
    vector<ParamValue> adjacentStateOwnerList;
    adjacentStateOwnerList.push_back(var_pos_to);
    adjacentStateOwnerList.push_back(var_pos_from);
    State* adjacentState = new State("is_adjacent",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", adjacentStateOwnerList, true, &Inquery::inqueryIsAdjacent);

    // effect: it's possible to access from var_pos_from to var_pos_to
    vector<ParamValue> existPathStateOwnerList3;
     existPathStateOwnerList3.push_back(var_pos_from);
     existPathStateOwnerList3.push_back(var_pos_to);
     State* existPathState3 = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,var_exist_path, existPathStateOwnerList3, true, &Inquery::inqueryExistPath);
    Effect* becomeExistPathEffect = new Effect(existPathState3, OP_ASSIGN, "true",false);

    // add rule:
    Rule* accessAdjacentRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f);
    accessAdjacentRule->ruleName = "adjacentPositionsExistingPath";
    accessAdjacentRule->addPrecondition(standableState2);
    accessAdjacentRule->addPrecondition(standableState3);
    accessAdjacentRule->addPrecondition(adjacentState);

    accessAdjacentRule->addEffect(EffectPair(0.7f,becomeExistPathEffect));

    BestNumericVariableInqueryStruct bs0;
    bs0.bestNumericVariableInqueryFun = &Inquery::inqueryAdjacentPosition;
    bs0.goalState = existPathState3;
    accessAdjacentRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(ActionParameter::ParamValueToString(var_pos_to), bs0));


    this->AllRules.push_back(accessAdjacentRule);
    //----------------------------End Rule: if a position is standable and adjacent(neighbour) then there is possible existing a path from here to this adjacent postion-----------------------------

    //----------------------------Begin Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---------------------
    // define variables:
    ParamValue var_pos_1 = vector_var[0];
    ParamValue var_pos_2 = vector_var[1];
    ParamValue var_pos_3 = vector_var[2];

    // precondition 1:There exists a path from the pos1 to pos2
    vector<ParamValue> existPathStateOwnerList4;
    existPathStateOwnerList4.push_back(var_pos_1);
    existPathStateOwnerList4.push_back(var_pos_2);
    State* existPathState4 = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList4, true, &Inquery::inqueryExistPath);

    // precondition 2:There exists a path from the pos2 to pos3
    vector<ParamValue> existPathStateOwnerList5;
    existPathStateOwnerList5.push_back(var_pos_2);
    existPathStateOwnerList5.push_back(var_pos_3);
    State* existPathState5 = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,"true", existPathStateOwnerList5, true, &Inquery::inqueryExistPath);

    // effect: it's possible to access from var_pos_from to var_pos_to
    vector<ParamValue> existPathStateOwnerList6;
    existPathStateOwnerList6.push_back(var_pos_1);
    existPathStateOwnerList6.push_back(var_pos_3);
    State* existPathState6 = new State("existPath",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,var_exist_path, existPathStateOwnerList6, true, &Inquery::inqueryExistPath);
    Effect* becomeExistPathEffect2 = new Effect(existPathState6, OP_ASSIGN, "true", false);

    // add rule:
    Rule* pathTransmitRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f);
    pathTransmitRule->ruleName = "IfExistpathAtoBandBtoCthenExistpathAtoC";
    pathTransmitRule->addPrecondition(existPathState4);
    pathTransmitRule->addPrecondition(existPathState5);

    BestNumericVariableInqueryStruct bs;
    bs.bestNumericVariableInqueryFun = &Inquery::inqueryNearestAccessiblePosition;
    bs.goalState = existPathState6;
    pathTransmitRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(ActionParameter::ParamValueToString(var_pos_2), bs));

    pathTransmitRule->addEffect(EffectPair(1.0f,becomeExistPathEffect2));

    this->AllRules.push_back(pathTransmitRule);

    //----------------------------End Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---

}



