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
#include <boost/bind.hpp>
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
        else if ( (*lastedStateNode) < (*((StateNode*)(*it))))
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
        else if ( (*lastedStateNode) < (*((StateNode*)(*it))))
        {
            lastedStateNode = (StateNode*)(*it);
        }
    }
    return lastedStateNode;
}

bool OCPlanner::isActionChangeSPaceMap(PetAction* action)
{
    ActionTypeCode type = action->getType().getCode();

    if ( (type== pai::EAT_CODE) || (type == pai::MOVE_TO_OBJ_CODE)
         || (type == pai::WALK_CODE) || (type== pai::BUILD_BLOCK_CODE) )
        return true;
    else
        return false;
}

bool compareRuleNodeDepth (const RuleNode* one, const RuleNode* other)
{
    // so , the deeper rule node will be put front
    StateNode* m = one->getLastForwardStateNode();
    StateNode* o = other->getLastForwardStateNode() ;
//    cout << "Debug: compare rule node depth: me = "  << m->depth << " other = " << o->depth << " ; ";
    bool d = (*o) < (*m) ;
//    cout << "me is put before other :" << d << std::endl;
    return (d);
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

StateNode* RuleNode::getMostClosedBackwardStateNode() const
{
    // get the BackwardStateNode with the least depth
    vector<StateNode*>::const_iterator it = backwardLinks.begin();
    StateNode* lastedStateNode = 0;

    for (; it != backwardLinks.end(); ++ it)
    {
        if (lastedStateNode == 0)
            lastedStateNode = (StateNode*)(*it);
        else if ( (*((StateNode*)(*it))) <  (*lastedStateNode))
        {
            lastedStateNode = (StateNode*)(*it);
        }
    }

    return lastedStateNode;
}

// this should be called after its forward node is assigned or changed
void StateNode::calculateNodesDepth()
{
    if (depth == "Deepest")
        return; // this the orginal state

    if (! forwardRuleNode)
    {
        if ( depth.size() == 1)
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

SpaceServer::SpaceMap* OCPlanner::getClosestBackwardSpaceMap(StateNode* stateNode)
{
    // sort all the rule nodes in current planning network, in the order of from left to right (from starting to goal / from backward to forward)
    sort(allRuleNodeInThisPlan.begin(), allRuleNodeInThisPlan.end(),compareRuleNodeDepth );
    RuleNode* clostestRuleNode = 0;

    if (stateNode->backwardRuleNode)
    {
         clostestRuleNode = stateNode->backwardRuleNode;
    }
    else
    {
        // get the most closest rule node to the input stateNode in its backward side,
        // because only the back ward node will affect the input state node

        vector<RuleNode*>::const_iterator it = allRuleNodeInThisPlan.begin();

        for (; it != allRuleNodeInThisPlan.end(); ++ it)
        {
            RuleNode* rnode = (RuleNode*)(*it);

            StateNode* snode = rnode->getLastForwardStateNode();

            if (snode == stateNode)
                continue;

            if ((*snode) < (*stateNode))
                break; // the node with a lower depth is more closed to the goal, so it won't affect the input state node

            clostestRuleNode = rnode;
        }
    }

    if (clostestRuleNode == 0)
        return curMap;

    SpaceServer::SpaceMap *iSpaceMap = curMap->clone();

    // execute the all the actions in current planning network to change the imaginary space map in the order in allRuleNodeInThisPlan,
    // till the clostestRuleNode
    vector<RuleNode*>::const_iterator it = allRuleNodeInThisPlan.begin();

    for (; it != allRuleNodeInThisPlan.end(); ++ it)
    {
        RuleNode* rnode = (RuleNode*)(*it);

        executeActionInImaginarySpaceMap(rnode,iSpaceMap);

        if (rnode == clostestRuleNode)
            break;
    }

    return iSpaceMap;

}

// @ bool &found: return if this same state is found in temporaryStateNodes
// @ StateNode& *stateNode: the stateNode in temporaryStateNodes which satisfied or dissatisfied this goal
// @ ifCheckSameRuleNode: if avoid finding the state node generate by same rule node
bool OCPlanner::checkIfThisGoalIsSatisfiedByTempStates(State& goalState, bool &found, StateNode *&satstateNode,RuleNode *forwardRuleNode,
                                                       bool ifCheckSameRuleNode, StateNode* curSNode)
{
    //   if curSNode is 0, it means it has no backward links yet, so only check in startStateNodes.
    //   if curSNode is not 0, check temporaryStateNodes first , if cannot find in temporaryStateNodes, check in startStateNodes.
    satstateNode = 0;
    if ( (curSNode != 0) && (curSNode->backwardRuleNode != 0))
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
    StateNode* mostClosedNode = 0;

    if (forwardRuleNode)
        mostClosedNode = forwardRuleNode->getMostClosedBackwardStateNode();

    for (;vit != temporaryStateNodes.end(); vit ++)
    {
        // only check the states more backward (deeper) then the mostClosedNode
        // cuz the forward state won't affect the current state
        if (mostClosedNode)
        {
            if ((*((StateNode*)(*vit))) < (*mostClosedNode))
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
            // when the state require a more precise operator type like STATE_EQUAL_TO, but the states in the temporaryStateNodes has a much less unprecise type
            // return false, but here need to be improved: need to figure out other combinations
            if ( (state.stateType == STATE_EQUAL_TO) && (vState->stateType != STATE_NOT_EQUAL_TO) )
                return false;

            if (stateNode == 0)
            {
                stateNode = (StateNode*)(*vit);
            }
            else if ((*((StateNode*)(*vit))) < (*stateNode)) // get the most closed state node backward from the forwardRuleNode
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
        return true;
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
        ParamValue inqueryValue;
        if (f == 0)
            inqueryValue = UNDEFINED_VALUE;
        else
            inqueryValue = f(oneGoal.stateOwnerList);

        if (inqueryValue == UNDEFINED_VALUE)
        {
            // there is not information in Atomspace about this state. We don't know the state, so just return false
            satisfiedDegree = 0.0f;
            return false;
        }

        return oneGoal.isSatisfiedMe(inqueryValue,satisfiedDegree,original_state);

    }
    else // it doesn't need real time calculation, then we search for its latest evaluation link value in the atomspace
    {
        // TODO
        ParamValue value = Inquery::getParamValueFromAtomspace(oneGoal);
        if (value == UNDEFINED_VALUE)
        {
            // there is not information in Atomspace about this state. We don't know the state, so just return false
            satisfiedDegree = 0.0f;
            return false;
        }


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

    curMap = &(spaceServer().getLatestMap());

    startStateNodes.clear();
    allRuleNodeInThisPlan.clear();
    unsatisfiedStateNodes.clear();
    temporaryStateNodes.clear();
    imaginaryHandles.clear(); // TODO: Create imaginary atoms
    satisfiedGoalStateNodes.clear();
    OCPlanner::goalRuleNode.backwardLinks.clear();
    curStateNode = 0;

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

    tryStepNum = 0;

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
        curStateNode = (StateNode*)(unsatisfiedStateNodes.back());

        // out put selected subgoal debug info:
        cout<< std::endl << "Debug planning step " << tryStepNum <<": Selected subgoal :";
        outputStateInfo(curStateNode->state,true);
        cout<<std::endl;

        curImaginaryMap = getClosestBackwardSpaceMap(curStateNode);

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
                bool isNegativeGoal, isDiffStateOwnerType, preconImpossible, willAddCirle;
                int negativeNum,satisfiedPreconNum;
                checkRuleFitnessRoughly(r,curStateNode,satisfiedPreconNum,negativeNum,isNegativeGoal,isDiffStateOwnerType,preconImpossible,willAddCirle, true);

                if (isNegativeGoal || isDiffStateOwnerType || willAddCirle) // if this rule will negative this goal, we should not choose to apply it.
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
                    bool isNegativeGoal, isDiffStateOwnerType, preconImpossible,willAddCirle;
                    int negativeNum,satisfiedPreconNum;
                    checkRuleFitnessRoughly(r,curStateNode,satisfiedPreconNum,negativeNum,isNegativeGoal,isDiffStateOwnerType,preconImpossible,willAddCirle);

                    //  its effect will negative this current selected goal state, or it has any unsatisfied precondition which is impossible to achieve,
                    //  then it should not add it into candidate rules
                    if (isNegativeGoal || preconImpossible || isDiffStateOwnerType)
                        continue;

                    // todo: check rule direct help this goal or not
                    bool directHelp;

                    // if this rule will negative this goal, we should not choose to apply it.
                    r->isRulePossibleToHelpToAchieveGoal(curStateNode->state,directHelp);

                    if (directHelp)
                        curRuleScore += 2.0f;

                    curRuleScore -= (r->paraIndexMap.size()) * 0.3f;

                    curRuleScore -= negativeNum*1.2f;
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

        allRuleNodeInThisPlan.insert(allRuleNodeInThisPlan.begin(),ruleNode);

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

            State* effState =  Rule::groundAStateByRuleParamMap(e->state, ruleNode->currentAllBindings,false,false,(*oldValItor));
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
            newStateNode->calculateNodesDepth();

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


            list<StateNode*>::iterator sait,delIt;

            for (sait = temporaryStateNodes.begin(); sait != temporaryStateNodes.end(); )
            {
                // only check the state nodes without backward rule node,
                // because we are doing backward chaining, the state node which has backward rule node will be satisfied later
                if (((StateNode*)(*sait))->backwardRuleNode != 0)
                {
                     ++ sait;
                    continue;
                }

                // if this effect state node has a lower depth than this state node, it means the effect will happen after this state node
                // so this state node will not be affected by this effect.
                if ( (*newStateNode) < (*(StateNode*)(*sait)))
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
                        delIt = sait;
                        ++ sait;
                        temporaryStateNodes.erase(delIt);

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
        int unsatisfiedPreconNum = 0;

        list<StateNode*> unsatisfiedPreconList;
        unsatisfiedPreconList.clear();

        itpre = ruleNode->originalRule->preconditionList.begin();
        for (; itpre != ruleNode->originalRule->preconditionList.end();  ++itpre, ++preConNum)
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

            cout<<"Precondition  " << preConNum <<": depth: " << newStateNode->depth << " ";
            outputStateInfo(groundPs, true);
            if(isSat)
            {
                temporaryStateNodes.push_front(newStateNode);
                cout << " is satisfied :)" << std::endl;
            }
            else
            {
                // add it to unsatisfied list
                getHardnessScoreOfPrecon(newStateNode);
                unsatisfiedPreconList.push_back(newStateNode);

                cout << " is unsatisfied :(" << std::endl;
                unsatisfiedPreconNum ++;
            }
        }

        // when the preconditions is not order dependent , need to check the easiness of each unsatisfied preconditions
        if ( (! ruleNode->originalRule->precondOrderDependent)  && (unsatisfiedPreconNum > 1) )
        {
            // if the order of preconds doesn't matter ,we sort the preconds from hard to easy
            unsatisfiedPreconList.sort(PreconHarder());
        }

        // add the unsatisfiedPreconList to the end of unsatisfiedStateNodes
        unsatisfiedStateNodes.insert(unsatisfiedStateNodes.end(), unsatisfiedPreconList.begin(), unsatisfiedPreconList.end());

//        // there is a tricky logic here: if it's an recursive rule, switch its two precons in the unsatisfiedStateNodes list if it  has two
//        // , so that the precon which is more closer to current state will be put in the end of  the unsatisfiedStateNodes list, so that it will be dealt with next loop
//        // because for a resursiv rule, it's usually more easy to do a forward reasoning than backward
//        if (ruleNode->originalRule->IsRecursiveRule && (unsatisfiedPreconNum == 2) )
//        {
//            StateNode* precon1 = unsatisfiedStateNodes.back();
//            unsatisfiedStateNodes.pop_back();
//            list<StateNode*>::iterator uitpre = unsatisfiedStateNodes.end();
//            uitpre --;
//            unsatisfiedStateNodes.insert(uitpre,precon1);
//        }

        if (curImaginaryMap != curMap)
            delete curImaginaryMap;

    }

    // finished planning!

    // generate the action series according to the planning network we have constructed in this planning process
    planID = oac->getPAI().createActionPlan();

    std::cout<<std::endl<<"OCPlanner::Planning success! Plan ID = "<< planID <<std::endl;

    // sort the list of rule node
    sort(allRuleNodeInThisPlan.begin(), allRuleNodeInThisPlan.end(),compareRuleNodeDepth );

    // and then encapsule the action for each step and send to PAI
    vector<RuleNode*>::iterator planRuleNodeIt, lastStepIt;
    int stepNum = 1;
    SpaceServer::SpaceMap* backwardStepMap = curMap->clone();

    for (planRuleNodeIt = allRuleNodeInThisPlan.begin(); planRuleNodeIt != allRuleNodeInThisPlan.end(); ++ planRuleNodeIt)
    {
        RuleNode* r = (RuleNode*)(*planRuleNodeIt);

        outputRuleNodeStep(r,false);

        if (stepNum != 1)
        {
            lastStepIt = planRuleNodeIt;
            lastStepIt --;
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
                          "OCPlanner::doPlanning: in action %s, parameter: %s is ungrounded.\n",
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
            // start pos is the actor's pos, get actor first
            Entity* actor = 0;
            if  (Rule::isParamValueUnGrounded(r->originalRule->actor))
            {
                ParamGroundedMapInARule::iterator bindIt = r->currentAllBindings.find(ActionParameter::ParamValueToString(r->originalRule->actor));
                OC_ASSERT( (bindIt != r->currentAllBindings.end()),
                          "OCPlanner::doPlanning: in action %s, parameter: the actor is ungrounded.\n",
                           originalAction->getType().getName().c_str());
                actor = boost::get<Entity>(&(bindIt->second));

            }
            else
                actor = boost::get<Entity>(&(r->originalRule->actor));

            OC_ASSERT( (actor != 0),
                      "OCPlanner::doPlanning: in action %s, parameter:the actor is invalid.\n",
                       originalAction->getType().getName().c_str());

            startPos = backwardStepMap->getObjectLocation(actor->id);

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

        executeActionInImaginarySpaceMap(r,backwardStepMap);
        stepNum ++;
    }

    delete backwardStepMap;

    std::cout<<std::endl;

    // Reset the spaceMap for inquery back to the real spaceMap
    Inquery::reSetSpaceMap();

    // todo: remove all the imaginary atoms in imaginaryHandles

    return planID;
}

int OCPlanner::getHardnessScoreOfPrecon(StateNode* stateNode)
{
    map<string,multimap<float,Rule*> >::iterator it = ruleEffectIndexes.find(stateNode->state->name());

    if ( it == ruleEffectIndexes.end())
        return 999999;

    multimap<float,Rule*>& rules = (multimap<float,Rule*>&)(it->second);
    multimap<float,Rule*> ::iterator ruleIt;
    bool ruleToHelp = false;
    bool directHelp = false;
    int numOfUngroundedVars = 1000; // the more variables this rule has, the more difficult to use it

    // at least one of the related rules should not be negative this subgoal
    for (ruleIt = rules.begin(); ruleIt != rules.end(); ruleIt ++)
    {
        Rule* r = ruleIt->second;

        bool _directHelp;
        // if this rule will negative this goal, we should not choose to apply it.
        if (r->isRulePossibleToHelpToAchieveGoal(stateNode->state ,_directHelp))
        {
            ruleToHelp = true;
            if (_directHelp)
                directHelp =true;

            int varNum = r->paraIndexMap.size();
            if (varNum < numOfUngroundedVars)
                numOfUngroundedVars = varNum;
        }

    }

    if (! ruleToHelp)
        return 999999;

    int hardnessScore = 1000;

    if (directHelp)
        hardnessScore = 0;

    hardnessScore += numOfUngroundedVars * 10;

    if (stateNode->state->isNumbericState())
        hardnessScore += 2000;

    return hardnessScore;
}


int OCPlanner::checkPreconditionFitness(RuleNode* ruleNode, StateNode* fowardState, bool &preconImpossible, bool &willCauseCirleNetWork,  bool &hasDirectHelpRule, Rule* orginalRule)
{
    int satisfiedPreconNum = 0;
    willCauseCirleNetWork = false;
    hasDirectHelpRule = false;

    // check how many preconditions will be satisfied
    vector<State*>::iterator itpre;
    vector<State*>& precondList = ruleNode->originalRule->preconditionList;

    if (orginalRule)
        precondList = orginalRule->preconditionList;

    for (itpre = precondList.begin(); itpre != precondList.end(); ++ itpre)
    {
        State* ps = *itpre;
        State* groundPs = Rule::groundAStateByRuleParamMap(ps, ruleNode->currentAllBindings,false);
        if (! groundPs)
            continue;

        // first check if this state has beed satisfied by the previous state nodes
        bool found = false;

        StateNode* satStateNode;

        bool satByTemp = checkIfThisGoalIsSatisfiedByTempStates(*groundPs, found, satStateNode,ruleNode,true);
        bool satisfied = false;

        // if it's found in the temporaryStateNodes
        if (found)
        {
            if (satByTemp)
            {
                ++ satisfiedPreconNum;
                satisfied = true;
            }

        }
        else
        {
            // cannot find this state in the temporaryStateNodes list, need to check it in real time
            // check real time
            float satisfiedDegree;
            if ( checkIsGoalAchievedInRealTime(*groundPs,satisfiedDegree))
            {
                ++ satisfiedPreconNum;
                satisfied = true;
            }

        }

        if (! satisfied)
        {
            // check if there is any rule related to achieve this unsatisfied precondition
            map<string,multimap<float,Rule*> >::iterator it = ruleEffectIndexes.find(groundPs->name());
            if ( it == ruleEffectIndexes.end())
            {
                delete groundPs;
                preconImpossible = true;
                return -999;
            }
            else
            {
                multimap<float,Rule*>& rules = (multimap<float,Rule*>&)(it->second);
                multimap<float,Rule*> ::iterator ruleIt;
                bool ruleImpossibleToHelp = true;

                // at least one of the related rules should not be negative this subgoal
                for (ruleIt = rules.begin(); ruleIt != rules.end(); ruleIt ++)
                {
                    Rule* r = ruleIt->second;
                    bool directHelp;
                    // if this rule will negative this goal, we should not choose to apply it.
                    if (r->isRulePossibleToHelpToAchieveGoal(groundPs,directHelp))
                    {
                        ruleImpossibleToHelp = false;

                        if (directHelp)
                            hasDirectHelpRule = true;

                    }
                }

                if (ruleImpossibleToHelp)
                {
                    delete groundPs;
                    preconImpossible = true;
                    return -999;
                }
            }

            // check if this precond will add a cirle to the planning network
            // if this precond is unsatified and exactly the same with one of its previous / forward state node,
            // which is expectly to be satisfied partly by this effect directly or undirectly.
            list<StateNode*>::iterator sait;
            for (sait = temporaryStateNodes.begin(); sait != temporaryStateNodes.end(); ++ sait)
            {
                StateNode* tempStateNode = (StateNode*)(*sait);

                // only check the StateNode which is more backward than the input fowardState
                // if this effect state node has a lower depth than this state node, it means the effect will happen after this state node
                // so this state node will not be affected by this effect.

                if ((*fowardState) < (*tempStateNode))
                    continue;

                if (tempStateNode->isTheSameDepthLevelWithMe(*fowardState))
                    continue;

                if (groundPs->isSameState( *(tempStateNode->state) ))
                {
                    if (groundPs->stateVariable->getValue() == tempStateNode->state->stateVariable->getValue())
                    {
                        delete groundPs;
                        willCauseCirleNetWork = true;
                        return -999;
                    }
                }

                // todo: recursively check if the future backward branch to achieve this subgoal contains any precondition impossibilities

            }

        }

        delete groundPs;
    }


    return satisfiedPreconNum;
}

int OCPlanner::checkSpaceMapEffectFitness(RuleNode* ruleNode,StateNode* fowardState)
{
    if (! isActionChangeSPaceMap(ruleNode->originalRule->action))
        return 0;

    int negativeNum = 0;
    SpaceServer::SpaceMap*  clonedCurImaginaryMap = curImaginaryMap->clone();
    executeActionInImaginarySpaceMap(ruleNode,clonedCurImaginaryMap);
    Inquery::setSpaceMap(clonedCurImaginaryMap);

    list<StateNode*>::iterator sait;

    for (sait = temporaryStateNodes.begin(); sait != temporaryStateNodes.end(); ++ sait)
    {
        StateNode* tempStateNode = (StateNode*)(*sait);

        // if a state node has not any forward rule, it means it is not used by any rule node yet, so it doesn't matter if it's affected or not
        if (tempStateNode->forwardRuleNode == 0)
           continue;

        // only check the state nodes without backward rule node,
        // because we are doing backward chaining, the state node which has backward rule node will be satisfied later
        if (tempStateNode->backwardRuleNode != 0)
            continue;

        // only check the state nodes need real time inquery in the space map
        if (! tempStateNode->state->need_inquery)
            continue;

        // only check the StateNode which is more backward than the input fowardState
        // if this effect state node has a lower depth than this state node, it means the effect will happen after this state node
        // so this state node will not be affected by this effect.
        if ((*fowardState) < (*tempStateNode))
        {
            // need to check if it's the same level with fowardState,
            if ((fowardState->forwardRuleNode != tempStateNode->forwardRuleNode))
                continue;
        }


        float sd;

        // check if this state has been negative
        if (! checkIsGoalAchievedInRealTime(*(tempStateNode->state), sd))
            negativeNum ++;

    }

    // set back the space map for inquery
    Inquery::setSpaceMap(curImaginaryMap);
    delete clonedCurImaginaryMap;

    return negativeNum;

}

int OCPlanner::checkEffectFitness(RuleNode* ruleNode, StateNode* fowardState, bool &isDiffStateOwnerType, bool &negativeGoal)
{
    int negateveStateNum = 0;
    negativeGoal = false;
    isDiffStateOwnerType = false;

    // check all the effects:
    vector<EffectPair>::iterator effectItor;

    for (effectItor = ruleNode->originalRule->effectList.begin(); effectItor != ruleNode->originalRule->effectList.end(); ++ effectItor)
    {
        Effect* e = (Effect*)(((EffectPair)(*effectItor)).second);

        State* effState =  Rule::groundAStateByRuleParamMap(e->state, ruleNode->currentAllBindings, false,false);

        if (! effState)
            continue;

        if (! Effect::executeEffectOp(effState,e,ruleNode->currentAllBindings))
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
                    return 1000;
                }
            }

            float satDegree;
            if (! effState->isSatisfied(*(fowardState->state),satDegree))
            {
                negativeGoal = true;
                delete effState;
                return 1000;
            }
        }

        // check how many already satisifed state nodes will be negated by this effect
        list<StateNode*>::iterator sait;
        for (sait = temporaryStateNodes.begin(); sait != temporaryStateNodes.end(); ++ sait)
        {
            StateNode* tempStateNode = (StateNode*)(*sait);

            // skip the current state node
            if (fowardState == tempStateNode)
                continue;

            // if a state node has not any forward rule, it means it is not used by any rule node yet, so it doesn't matter if it's affected or not
            if (tempStateNode->forwardRuleNode == 0)
               continue;

            // only check the state nodes without backward rule node,
            // because we are doing backward chaining, the state node which has backward rule node will be satisfied later
            if (tempStateNode->backwardRuleNode != 0)
                continue;

            // only check the StateNode which is more backward than the input fowardState
            // if this effect state node has a lower depth than this state node, it means the effect will happen after this state node
            // so this state node will not be affected by this effect.
            if ((*fowardState) < (*tempStateNode))
            {
                // need to check if it's the same level with fowardState,
                if ((fowardState->forwardRuleNode != tempStateNode->forwardRuleNode))
                    continue;
            }

            if (tempStateNode->state->isSameState(*effState))
            {
                // check if this effect unsatisfy this state
                float satDegree;
                if (! effState->isSatisfied(*(tempStateNode->state ),satDegree))
                {
                    negateveStateNum ++;
                }
                else
                    continue;
            }

        }

        delete effState;
    }

    return negateveStateNum;

}

void OCPlanner::checkRuleFitnessRoughly(Rule* rule, StateNode* fowardState, int &satisfiedPreconNum, int &negateveStateNum, bool &negativeGoal,
                                        bool &isDiffStateOwnerType, bool &preconImpossible, bool &willAddCirle, bool onlyCheckIfNegativeGoal)
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

    tmpRuleNode->updateCurrentAllBindings();

    negateveStateNum = 0;
    satisfiedPreconNum = 0;
    negativeGoal = false;
    isDiffStateOwnerType = false;
    preconImpossible = false;
    willAddCirle = false;


    // check all the effects:
    negateveStateNum = checkEffectFitness(tmpRuleNode,fowardState,isDiffStateOwnerType,negativeGoal);

    if (onlyCheckIfNegativeGoal)
    {
        delete tmpRuleNode;
        return;
    }

    bool hasDirectHelpRule;

    // check how many preconditions will be satisfied
    satisfiedPreconNum = checkPreconditionFitness(tmpRuleNode,fowardState,preconImpossible,willAddCirle, hasDirectHelpRule);

    delete tmpRuleNode;

}

// ToBeImproved: this function is very ugly...the right way is to add a callback function for each action to auto execute
void OCPlanner::executeActionInImaginarySpaceMap(RuleNode* ruleNode, SpaceServer::SpaceMap *iSpaceMap)
{
    static int imaginaryBlockNum = 1;

    // currently we just cheat to enable the following actions
    // ToBeImproved: the right way is to add a callback function for each action to auto execute

    switch (ruleNode->originalRule->action->getType().getCode())
    {
        case pai::EAT_CODE:
        {
            // get the handle of the food to eat
            string foodVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Entity foodEntity =  boost::get<Entity>((ruleNode->currentAllBindings)[foodVarName]);
            Handle foodH = AtomSpaceUtil::getEntityHandle(*atomSpace,foodEntity.id);
            iSpaceMap->removeNoneBlockEntity(foodH);
            break;
        }
        case pai::MOVE_TO_OBJ_CODE:
        {
            // get the actor entity handle
            string actorVarName0 = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            Entity agent0 =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName0]);
            Handle agentH0 = AtomSpaceUtil::getAgentHandle(*atomSpace,agent0.id);

            // get the object want to move to
            string targetVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Entity target =  boost::get<Entity>((ruleNode->currentAllBindings)[targetVarName]);
            Handle targetH = AtomSpaceUtil::getAgentHandle(*atomSpace,target.id);
            // get new location it moves tol
            spatial::BlockVector targetLocation = iSpaceMap->getObjectLocation(targetH);
            iSpaceMap->updateNoneBLockEntityLocation(agentH0,targetLocation,curtimeStamp,true);
            break;
        }
        case pai::WALK_CODE:
        {
            // get the actor entity handle
            string actorVarName = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
            Entity agent =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName]);
            Handle agentH = AtomSpaceUtil::getAgentHandle(*atomSpace,agent.id);

            // get the new location it moves to
            string newPosVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Vector movedToVec = boost::get<Vector>((ruleNode->currentAllBindings)[newPosVarName]);
            spatial::BlockVector newPos(movedToVec.x,movedToVec.y,movedToVec.z);
            iSpaceMap->updateNoneBLockEntityLocation(agentH,newPos,curtimeStamp,true);

            break;
        }
        case pai::BUILD_BLOCK_CODE:
        {
            // get the  location of block
            string blockBuildPposVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
            Vector buildVec = boost::get<Vector>((ruleNode->currentAllBindings)[blockBuildPposVarName]);
            spatial::BlockVector buildPos(buildVec.x,buildVec.y,buildVec.z);
            Handle iBlockH = AtomSpaceUtil::addNode(*atomSpace, IMAGINARY_STRUCTURE_NODE,"Imaginary_Block_" + imaginaryBlockNum++);
            iSpaceMap->addSolidUnitBlock(buildPos,iBlockH);
            break;
        }
        default:
            // TODO: TNick: is this the right way of handling other codes?
            break;
    }

}

//void OCPlanner::undoActionInImaginarySpaceMap(RuleNode* ruleNode,SpaceServer::SpaceMap* iSpaceMap)
//{
//    // currently we just cheat to enable the following actions
//    // ToBeImproved: this function is very ugly...the right way is to add a callback function for each action to auto execute

//    switch (ruleNode->originalRule->action->getType().getCode())
//    {
//        case pai::EAT_CODE:
//        {
//            // get the handle of the food to eat
//            string foodVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
//            Entity foodEntity =  boost::get<Entity>((ruleNode->currentAllBindings)[foodVarName]);
//            Handle foodH = AtomSpaceUtil::getEntityHandle(*atomSpace,foodEntity.id);
//            // get the location last time it appeared
//            iSpaceMap->addNoneBlockEntity(foodH,iSpaceMap->getLastAppearedLocation(foodH),1,1,1,0.0,foodEntity.id,foodEntity.type,false,curtimeStamp);

//            break;
//        }

//        case pai::MOVE_TO_OBJ_CODE:
//        {
//            // get the actor entity handle
//            string actorVarName0 = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
//            Entity agent0 =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName0]);
//            Handle agentH0 = AtomSpaceUtil::getAgentHandle(*atomSpace,agent0.id);

//            // get old location from the record before execute this effect
//            Vector movedToVec0 = boost::get<Vector>((ruleNode->orginalGroundedParamValues)[1]);
//            spatial::BlockVector targetLocation0(movedToVec0.x, movedToVec0.y, movedToVec0.z);
//            iSpaceMap->updateNoneBLockEntityLocation(agentH0,targetLocation0,curtimeStamp,true);

//            break;
//        }

//        case pai::WALK_CODE:
//        {
//            // get the actor entity handle
//            string actorVarName = ActionParameter::ParamValueToString(ruleNode->originalRule->actor);
//            Entity agent =  boost::get<Entity>((ruleNode->currentAllBindings)[actorVarName]);
//            Handle agentH = AtomSpaceUtil::getAgentHandle(*atomSpace,agent.id);

//            // get old location from the record before execute this effect
//            Vector movedToVec = boost::get<Vector>((ruleNode->orginalGroundedParamValues)[1]);
//            spatial::BlockVector targetLocation(movedToVec.x, movedToVec.y, movedToVec.z);
//            iSpaceMap->updateNoneBLockEntityLocation(agentH,targetLocation,curtimeStamp,true);

//            break;
//        }

//        case pai::BUILD_BLOCK_CODE:
//        {
//            // get the  location the new block is to be build at
//            string blockBuildPposVarName = (ruleNode->originalRule->action->getParameters().front()).stringRepresentation();
//            Vector buildVec = boost::get<Vector>((ruleNode->currentAllBindings)[blockBuildPposVarName]);
//            spatial::BlockVector buildPos(buildVec.x,buildVec.y,buildVec.z);

//            iSpaceMap->removeSolidUnitBlock(iSpaceMap->getUnitBlockHandleFromPosition(buildPos));
//            break;
//        }
//        default:
//            // TODO: TNick: is this the right way of handling other codes?
//            break;
//    }
//}

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
        StateNode* curSNode = (StateNode*)(*forwardStateIt);
        if (curSNode == forwardStateNode)
            continue;

        deleteStateNodeInTemporaryList(curSNode);

        if (curSNode->forwardRuleNode != 0)
            unsatisfiedStateNodes.push_back(curSNode);
    }

    if (forwardStateNode && deleteThisforwardStateNode)
        delete forwardStateNode;

    // check all its backward state nodes
    // delete them recursively
    vector<StateNode*>::iterator backwardStateIt;
    for (backwardStateIt = ruleNode->backwardLinks.begin(); backwardStateIt != ruleNode->backwardLinks.end(); ++ backwardStateIt)
    {
        StateNode* curSNode = (StateNode*)(*backwardStateIt);

        deleteStateNodeInTemporaryList(curSNode);

         if (curSNode->backwardRuleNode != 0)
             deleteRuleNodeRecursively(curSNode->backwardRuleNode, curSNode);
         else
             delete curSNode;

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
    vector<RuleNode*>::iterator it;
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

//    std::cout << "Debug: Begin grounding variables for rule: "<< ruleNode->originalRule->ruleName
//              << " from its forward state " << forwardStateNode->state->name().c_str() << std::endl;

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
//                std::cout << "Debug: Add currentBindingsFromForwardState pair: variableName = "<< variableName
//                          << ", value = "<< ActionParameter::ParamValueToString(*f_ownerIt) << std::endl;
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
//            std::cout << "Debug: Add currentBindingsFromForwardState pair: variableName = "<< variableName
//                      << ", value = "<< ActionParameter::ParamValueToString(forwardStateNode->state->getParamValue()) << std::endl;
            ruleNode->currentBindingsFromForwardState.insert(std::pair<string, ParamValue>(variableName,forwardStateNode->state->getParamValue()));
        }
    }

//    std::cout << "Debug: End grounding variables for rule: "<< ruleNode->originalRule->ruleName
//              << " from its forward state " << forwardStateNode->state->name().c_str() << std::endl;

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

float OCPlanner::checkNonNumericValueFitness(RuleNode *ruleNode, StateNode* fowardState, ParamGroundedMapInARule& oneGroupOfbindings,bool &impossible)
{

    float fitnessScore = 0;

   ruleNode->currentAllBindings = ruleNode->currentBindingsFromForwardState;

   ruleNode->currentAllBindings.insert(oneGroupOfbindings.begin(),oneGroupOfbindings.end());

    int negateveStateNum = 0;
    int satisfiedPreconNum = 0;
    bool negativeGoal = false;
    bool isDiffStateOwnerType = false;
    bool preconImpossible = false;
    bool willAddCirle = false;
    bool hasDirectHelpRule = false;

    impossible = false;

    // check all the effects:
    negateveStateNum = checkEffectFitness(ruleNode,fowardState,isDiffStateOwnerType,negativeGoal);

    // check how many preconditions will be satisfied
    satisfiedPreconNum = checkPreconditionFitness(ruleNode,fowardState,preconImpossible,willAddCirle, hasDirectHelpRule);

    // ruleNode resetbinding
    ruleNode->currentAllBindings = ruleNode->currentBindingsFromForwardState;

    fitnessScore = fitnessScore - negateveStateNum*100.0f + satisfiedPreconNum*100.0f;

    if (isDiffStateOwnerType || negativeGoal || preconImpossible || willAddCirle)
    {
        impossible = true;
        fitnessScore -= 999999;
    }

    if (hasDirectHelpRule)
        fitnessScore += 500.0f;

    return fitnessScore;

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

                    string outputVarStr;

                    foreach (Handle h, candidateHandlesInOneGroup)
                    {
                        ParamValue v = Inquery::getParamValueFromHandle(varNames[index],h);
                        oneGroupCandidate.insert(std::pair<string, ParamValue>(varNames[index],v));
                        outputVarStr += varNames[index] + "= " + ActionParameter::ParamValueToString(v) + "\n";
                        index ++;
                    }

                    bool impossile;
                    float fitnessScore = checkNonNumericValueFitness(ruleNode, curStateNode, oneGroupCandidate, impossile);

//                    cout << "Fitness score = " << fitnessScore ;
//                    if (impossile)
//                        cout << " , impossible bindings, would not be put into candidate list. " ;



                    if (! impossile)
                    {
                        cout<< "CandidateGroup "<< candidateGroupNum ++ << std::endl;
                        cout<< outputVarStr;
                        cout << "Fitness score = " << fitnessScore << std::endl;

                        tmpcandidates.push_back(TmpParamCandidate(fitnessScore, oneGroupCandidate));
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

        // test
        if (tmpcandidates.size() > 0)
            break;

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

        // ToBeImproved: currently only apply the borrowed rule in the second precondition of recursiveRule,
        // because the second precondition is more closed to the orginal state than the goal which should be more easier or executed first.
        // ToBeImproved: need to find out if the numeric variable this unrecursive rule try to ground is applied in the first or second  precondition of this recursiveRule
        Rule* borrowedRule =  unifyRuleVariableName(unrecursiveRule, rule->preconditionList[1]);
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

    if (values.size() == 1)
        bestValue = values.front();
    else
    {
        if (rule->CostHeuristics.size() != 0)
        {
            bestValue = selectBestNumericValueFromCandidates(rule,rule->basic_cost, rule->CostHeuristics,currentbindings, beIt->first,values);
        }
        else if (ruleNode->costHeuristics.size()!= 0)
        {
            bestValue = selectBestNumericValueFromCandidates(rule,0.0f, ruleNode->costHeuristics,currentbindings, beIt->first,values,ruleNode->originalRule,false);
        }
        else
        {
            cout<< "Debug: OCPlanner::selectValueForGroundingNumericState: this rule doesn't have any costHeuristics for calculation! Randomly select one for it." <<std::endl;
            bestValue = values.front();
        }
    }

    if (bestValue == UNDEFINED_VALUE)
    {
        logger().error("OCPlanner::selectValueForGroundingNumericState: failed to find the best value for grounding numeric state!" );
        return false;
    }

    currentbindings.insert(std::pair<string, ParamValue>(beIt->first,bestValue));
    return true;


}


ParamValue OCPlanner::selectBestNumericValueFromCandidates(Rule* rule, float basic_cost, vector<CostHeuristic>& costHeuristics, ParamGroundedMapInARule& currentbindings,
                                                           string varName, vector<ParamValue>& values, Rule *orginalRule, bool checkPrecons)
{
    // check how many preconditions will be satisfied
    RuleNode* tmpRuleNode = new RuleNode(rule);
    tmpRuleNode->currentAllBindings = currentbindings;

    vector<ParamValue>::iterator vit;
    float bestScore = -9999999.9;
    ParamValue bestValue = UNDEFINED_VALUE;

    for (vit = values.begin(); vit != values.end(); ++ vit)
    {
        // calculate the cost
        currentbindings.insert(std::pair<string, ParamValue>(varName,*vit));
        float cost = Rule::getCost(basic_cost, costHeuristics, currentbindings);
        if (cost < -0.00001f)
        {
            logger().error("OCPlanner::selectBestNumericValueFromCandidates: this rule has not been grounded fully!" );
            return UNDEFINED_VALUE;
        }

        float score = 0.0f-cost*100;
        currentbindings.erase(varName);

        // check effect
        bool isDiffStateOwnerType,  negativeGoal, willAddCirle;
        tmpRuleNode->currentAllBindings.insert(std::pair<string, ParamValue>(varName,*vit));

        int negativeNum = checkEffectFitness(tmpRuleNode, curStateNode, isDiffStateOwnerType,  negativeGoal);
        if (isActionChangeSPaceMap(rule->action))
        {
            negativeNum += checkSpaceMapEffectFitness(tmpRuleNode, curStateNode);
        }

        score -= (negativeNum * 12.0f);

        // check how many preconditions will be satisfied

        bool preconImpossible;
        bool hasDirectHelpRule;
        int satisfiedPreconNum = checkPreconditionFitness(tmpRuleNode,curStateNode,preconImpossible,willAddCirle,hasDirectHelpRule, orginalRule);

        if (preconImpossible)
            score -= 99999.9f;

        if (willAddCirle)
            score -= 99999.9f;

        if (checkPrecons)
        {
            score += satisfiedPreconNum * 10.0f;

            if(hasDirectHelpRule)
                score += 50.0f;
        }


        tmpRuleNode->currentAllBindings.erase(varName);

        if (score > bestScore)
        {
            bestScore = score;
            bestValue = *vit;
        }
    }

    if (bestScore < -9999999.89)
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

void OCPlanner::outputRuleNodeStep(RuleNode* ruleNode, bool outputForwardStateNodes)
{
    cout << std::endl << "RuleNode:" << ruleNode->originalRule->ruleName << " Action:" << ruleNode->originalRule->action->getName() <<  std::endl;

    if (outputForwardStateNodes)
    {
        cout << " All forward state nodes:" << std::endl;
    }

    // get the forwardStateNode with the deepest depth
    vector<StateNode*>::const_iterator it = ruleNode->forwardLinks.begin();
    StateNode* lastedStateNode = 0;

    for (; it != ruleNode->forwardLinks.end(); ++ it)
    {
        if (outputForwardStateNodes)
        {
            cout << std::endl;
            outputStateInfo(((StateNode*)(*it))->state, true);
            cout << " Depth = " << ((StateNode*)(*it))->depth;
            cout << std::endl;
        }
        if (lastedStateNode == 0)
            lastedStateNode = (StateNode*)(*it);
        if ( (*lastedStateNode) < (*((StateNode*)(*it))))
        {
            lastedStateNode = (StateNode*)(*it);
        }
    }

    cout << " Rule node dept = " << lastedStateNode->depth << std::endl;
}

// a bunch of rules for test, load from c++ codes
void OCPlanner::loadTestRulesFromCodes()
{
    // define a special action: do nothing
    PetAction* doNothingAction = new PetAction(ActionType::DO_NOTHING());


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
    bs2.bestNumericVariableInqueryFun = &Inquery::inqueryAdjacentAccessPosition;
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
    bs0.bestNumericVariableInqueryFun = &Inquery::inqueryAdjacentAccessPosition;
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
    Rule* pathTransmitRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f,true);
    pathTransmitRule->ruleName = "IfExistpathAtoBandBtoCthenExistpathAtoC";
    // note: the order of adding pos_2 -> pos_3 before adding pos_1 -> pos_2 ,
    // becuase when execute, the agent will move from pos_1 -> pos_2 before pos_2 -> pos_3
    // so pos_2 -> pos_3 is more closed to the target , so it is to put in front
    pathTransmitRule->addPrecondition(existPathState5);
    pathTransmitRule->addPrecondition(existPathState4);

    BestNumericVariableInqueryStruct bs;
    bs.bestNumericVariableInqueryFun = &Inquery::inqueryBestAccessiblePosition;
    bs.goalState = existPathState6;
    pathTransmitRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(ActionParameter::ParamValueToString(var_pos_2), bs));

    pathTransmitRule->addEffect(EffectPair(1.0f,becomeExistPathEffect2));

    this->AllRules.push_back(pathTransmitRule);

    //----------------------------End Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---




    //----------------------------Begin Einstein puzzle rules---------------------------------------------------------------------
    //----------------------------Begin RUle: if house1 is left to house2,then house1 is next to house2---------------------------
    // define variables:
    ParamValue var_house_1 = entity_var[0];
    ParamValue var_house_2 = entity_var[1];
    ParamValue var_house_next = entity_var[2];
    ParamValue var_house_next_2 = entity_var[3];

    // precondition 1: house1 is left to house2
    vector<ParamValue> leftToStateOwnerList1;
    leftToStateOwnerList1.push_back(var_house_1);
    State* leftToState1 = new State("leftOf",ActionParamType::ENTITY(),STATE_EQUAL_TO , var_house_2, leftToStateOwnerList1);

    // effect1: house1 is next to house2
    vector<ParamValue> nextToStateOwnerList1;
    nextToStateOwnerList1.push_back(var_house_1);
    State* nextToState1 = new State("nextTo",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_next, nextToStateOwnerList1);
    Effect* nextToEffect1 = new Effect(nextToState1, OP_ASSIGN, var_house_2,true);

    // effect2: house2 is next to house1
    vector<ParamValue> nextToStateOwnerList2;
    nextToStateOwnerList2.push_back(var_house_2);
    State* nextToState2 = new State("nextTo",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_next_2, nextToStateOwnerList2);
    Effect* nextToEffect2 = new Effect(nextToState2, OP_ASSIGN, var_house_1,true);

    // add rule:
    Rule* leftNextToRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    leftNextToRule->ruleName = "houseLeftTo->NextTo";
    leftNextToRule->addPrecondition(leftToState1);

    leftNextToRule->addEffect(EffectPair(1.0f,nextToEffect1));
    leftNextToRule->addEffect(EffectPair(1.0f,nextToEffect2));

    this->AllRules.push_back(leftNextToRule);
    //----------------------------End RUle: if house1 is left to house2,then house1 is next to house2---------------------------

    //----------------------------Begin RUle: if house1 is right to house2,then house1 is next to house2---------------------------
    // precondition 1: house1 is left to house2
    vector<ParamValue> rightToStateOwnerList1;
    rightToStateOwnerList1.push_back(var_house_1);
    State* rightToState1 = new State("rightOf",ActionParamType::ENTITY(),STATE_EQUAL_TO , var_house_2, rightToStateOwnerList1);

    // add rule:
    Rule* rightNextToRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    rightNextToRule->ruleName = "houseRightTo->NextTo";
    rightNextToRule->addPrecondition(rightToState1);

    rightNextToRule->addEffect(EffectPair(1.0f,nextToEffect1));
    rightNextToRule->addEffect(EffectPair(1.0f,nextToEffect2));

    this->AllRules.push_back(rightNextToRule);
    //----------------------------End RUle: if house1 is right to house2,then house1 is next to house2---------------------------

    //----------------------------Begin RUle: if house1 is next to house2,then house2 is next to house1---------------------------
    // precondition 1: house1 is left to house2
    vector<ParamValue> nextToStateOwnerList12;
    nextToStateOwnerList12.push_back(var_house_1);
    State* nextToState12 = new State("nextTo",ActionParamType::ENTITY(),STATE_EQUAL_TO , var_house_2, nextToStateOwnerList12);

    // add rule:
    Rule* nextToRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    nextToRule->ruleName = "nextToRule";
    nextToRule->addPrecondition(nextToState12);

    nextToRule->addEffect(EffectPair(1.0f,nextToEffect2));

    this->AllRules.push_back(nextToRule);
    //----------------------------End RUle: if house1 is right to house2,then house1 is next to house2---------------------------

    //----------------------------Begin Rule: The Englishman lives in the red house.----------------------------------------------
    // define variables:
    ParamValue var_man_x = entity_var[0];
    ParamValue var_red_house = entity_var[1];

    // precondition 1: var_man_x is people
    vector<ParamValue> peopleStateOwnerList1;
    peopleStateOwnerList1.push_back(var_man_x);
    State* peopleState1 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", peopleStateOwnerList1);

    // precondition 2: var_man_x's nation is British
    vector<ParamValue> nationStateOwnerList1;
    nationStateOwnerList1.push_back(var_man_x);
    State* isBritishState1 = new State("nation",ActionParamType::STRING(),STATE_EQUAL_TO , "British", nationStateOwnerList1, true, 0);

    // precondition 3: var_red_house's color is red
    vector<ParamValue> colorStateOwnerList1;
    colorStateOwnerList1.push_back(var_red_house);
    State* isRedState1 = new State("color",ActionParamType::STRING(),STATE_EQUAL_TO , "red", colorStateOwnerList1, true, 0);

    // effect1: var_man_x lives in var_red_house
    vector<ParamValue> liveInRedStateOwnerList1;
    liveInRedStateOwnerList1.push_back(var_man_x);
    State* liveInRedState1 = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_next, liveInRedStateOwnerList1);
    Effect* liveInRedEffect1 = new Effect(liveInRedState1, OP_ASSIGN, var_red_house,true);

    // add rule:
    Rule* BritishLiveInRedRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    BritishLiveInRedRule->ruleName = "BritishLiveInRedRule";
    BritishLiveInRedRule->addPrecondition(peopleState1);
    BritishLiveInRedRule->addPrecondition(isBritishState1);
    BritishLiveInRedRule->addPrecondition(isRedState1);

    BritishLiveInRedRule->addEffect(EffectPair(1.0f,liveInRedEffect1));

    this->AllRules.push_back(BritishLiveInRedRule);
    //----------------------------End Rule: The Englishman lives in the red house.----------------------------------------------

    //----------------------------Begin Rule: The Swedish man keeps dogs as pets..----------------------------------------------
    // define variables:
    ParamValue var_dog_pet = str_var[0];

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x's nation is Swedish
    vector<ParamValue> nationStateOwnerList2;
    nationStateOwnerList2.push_back(var_man_x);
    State* isSwedishState1 = new State("nation",ActionParamType::STRING(),STATE_EQUAL_TO , "Swedish", nationStateOwnerList2, true, 0);

    // effect1: var_man_x keeps dogs
    vector<ParamValue> keepDogsStateOwnerList1;
    keepDogsStateOwnerList1.push_back(var_man_x);
    State* keepDogsState1 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,var_dog_pet, keepDogsStateOwnerList1, true,0);
    Effect* keepDogsEffect1 = new Effect(keepDogsState1, OP_ASSIGN, "dogs",true);

    // add rule:
    Rule* SwedishKeepsDogsRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    SwedishKeepsDogsRule->ruleName = "SwedishKeepsDogsRule";
    SwedishKeepsDogsRule->addPrecondition(peopleState1);
    SwedishKeepsDogsRule->addPrecondition(isSwedishState1);

    SwedishKeepsDogsRule->addEffect(EffectPair(1.0f,keepDogsEffect1));

    this->AllRules.push_back(SwedishKeepsDogsRule);
    //----------------------------End Rule: The Swedish man keeps dogs as pets..----------------------------------------------

    //----------------------------Begin Rule: The Danish man drinks tea------------------------------------------------------
    // define variables:
    ParamValue var_tea = str_var[0];

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x's nation is Danish
    vector<ParamValue> nationStateOwnerList3;
    nationStateOwnerList3.push_back(var_man_x);
    State* isDanishState1 = new State("nation",ActionParamType::STRING(),STATE_EQUAL_TO , "Swedish", nationStateOwnerList3, true, 0);

    // effect1: var_man_x drinks tea
    vector<ParamValue> drinkTeaStateOwnerList1;
    drinkTeaStateOwnerList1.push_back(var_man_x);
    State* drinkTeaState1 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,var_tea, drinkTeaStateOwnerList1, true, 0);
    Effect* drinkTeaEffect1 = new Effect(drinkTeaState1, OP_ASSIGN, "tea",true);

    // add rule:
    Rule* DanishDrinksTeaRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    DanishDrinksTeaRule->ruleName = "DanishDrinksTeaRule";
    DanishDrinksTeaRule->addPrecondition(peopleState1);
    DanishDrinksTeaRule->addPrecondition(isDanishState1);

    DanishDrinksTeaRule->addEffect(EffectPair(1.0f,drinkTeaEffect1));

    this->AllRules.push_back(DanishDrinksTeaRule);
    //----------------------------End Rule:  The Danish man drinks tea--------------------------------------------------

    //----------------------------Begin Rule: The Green house is next to, and on the left of the White house--------------------------------------------------
    // define variables:
    ParamValue var_green_house = entity_var[0];
    ParamValue var_white_house = entity_var[1];

    // precondition 1: var_green_house's color is green
    vector<ParamValue> greenStateOwnerList;
    greenStateOwnerList.push_back(var_green_house);
    State* isGreenState1 = new State("color",ActionParamType::STRING(),STATE_EQUAL_TO , "green", greenStateOwnerList, true, 0);

    // precondition 2: var_white_house's color is white
    vector<ParamValue> whiteStateOwnerList;
    whiteStateOwnerList.push_back(var_white_house);
    State* isWhiteState1 = new State("color",ActionParamType::STRING(),STATE_EQUAL_TO , "white", whiteStateOwnerList, true, 0);

    // effect1: var_green_house is left of  var_white_house
    vector<ParamValue> leftOfStateOwnerList1;
    leftOfStateOwnerList1.push_back(var_green_house);
    State* leftOfState1 = new State("leftOf",ActionParamType::ENTITY(),STATE_EQUAL_TO , var_white_house, leftOfStateOwnerList1);
    Effect* leftOfEffect1 = new Effect(leftOfState1, OP_ASSIGN, var_white_house,true);

    // add rule:
    Rule* GreenLeftOfWhiteRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    GreenLeftOfWhiteRule->ruleName = "GreenLeftOfWhiteRule";
    GreenLeftOfWhiteRule->addPrecondition(isGreenState1);
    GreenLeftOfWhiteRule->addPrecondition(isWhiteState1);

    GreenLeftOfWhiteRule->addEffect(EffectPair(1.0f,leftOfEffect1));

    this->AllRules.push_back(GreenLeftOfWhiteRule);
    //----------------------------End Rule: The Green house is next to, and on the left of the White house----------------------------------------------

    //----------------------------Begin Rule: The person lives in the Green house drinks coffee.------------------------------------------------------
    // define variables:
    ParamValue var_coffee = str_var[0];

    // precondition 1: var_man_x is people

    // precondition 2: var_green_house's color is green

    // precondition 3: var_man_x lives in green house
    vector<ParamValue> liveInGreenStateOwnerList1;
    liveInGreenStateOwnerList1.push_back(var_man_x);
    State* liveInGreenState1 = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_green_house, liveInGreenStateOwnerList1);

    // effect1: var_man_x drinks coffee
    vector<ParamValue> drinkCoffeeStateOwnerList1;
    drinkCoffeeStateOwnerList1.push_back(var_man_x);
    State* drinkCoffeeState1 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,var_coffee, drinkCoffeeStateOwnerList1, true, 0);
    Effect* drinkCoffeeEffect1 = new Effect(drinkCoffeeState1, OP_ASSIGN, "coffee",true);

    // add rule:
    Rule* greenHouseManDrinksCoffeeRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    greenHouseManDrinksCoffeeRule->ruleName = "greenHouseManDrinksCoffeeRule";
    greenHouseManDrinksCoffeeRule->addPrecondition(peopleState1);
    greenHouseManDrinksCoffeeRule->addPrecondition(isGreenState1);
    greenHouseManDrinksCoffeeRule->addPrecondition(liveInGreenState1);

    greenHouseManDrinksCoffeeRule->addEffect(EffectPair(1.0f,drinkCoffeeEffect1));

    this->AllRules.push_back(greenHouseManDrinksCoffeeRule);
    //----------------------------End Rule:  The person lives in the Green house drinks coffee--------------------------------------------------


    //----------------------------Begin Rule: The person who smokes Pall Mall rears birds.----------------------------------------------
    // define variables:
    ParamValue var_birds_pet = str_var[0];

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x smokes pall Mall
    vector<ParamValue> smokePallMallStateOwnerList2;
    smokePallMallStateOwnerList2.push_back(var_man_x);
    State* smokePallMallState1 = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO , "pallMall", smokePallMallStateOwnerList2, true,0);

    // effect1: var_man_x keeps birds
    vector<ParamValue> keepBirdsStateOwnerList1;
    keepBirdsStateOwnerList1.push_back(var_man_x);
    State* keepBirdsState1 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,var_birds_pet, keepBirdsStateOwnerList1, true,0);
    Effect* keepBirdsEffect1 = new Effect(keepBirdsState1, OP_ASSIGN, "birds",true);

    // add rule:
    Rule* PallMallSmokerKeepsBirdsRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    PallMallSmokerKeepsBirdsRule->ruleName = "PallMallSmokerKeepsBirdsRule";
    PallMallSmokerKeepsBirdsRule->addPrecondition(peopleState1);
    PallMallSmokerKeepsBirdsRule->addPrecondition(smokePallMallState1);

    PallMallSmokerKeepsBirdsRule->addEffect(EffectPair(1.0f,keepBirdsEffect1));

    this->AllRules.push_back(PallMallSmokerKeepsBirdsRule);
    //----------------------------End Rule: The person who smokes Pall Mall rears birds..----------------------------------------------


    //----------------------------Begin Rule: The person who lives in the Yellow house smokes Dunhill..----------------------------------------------
    // define variables:
    ParamValue var_cigarette_brand = str_var[0];
    ParamValue var_yellow_house = entity_var[1];

    // precondition 1: var_man_x is people


    // precondition 2: var_yellow_house's color is yellow
    vector<ParamValue> yellowStateOwnerList;
    yellowStateOwnerList.push_back(var_yellow_house);
    State* isYellowState1 = new State("color",ActionParamType::STRING(),STATE_EQUAL_TO , "yellow", yellowStateOwnerList, true, 0);

    // precondition 3: var_man_x lives in yellow house
    vector<ParamValue> liveInYellowStateOwnerList1;
    liveInYellowStateOwnerList1.push_back(var_man_x);
    State* liveInYellowState1 = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_yellow_house, liveInYellowStateOwnerList1);

    // effect1: var_man_x smoke Dunhill
    vector<ParamValue> smokeDunhillStateOwnerList;
    smokeDunhillStateOwnerList.push_back(var_man_x);
    State* smokeDunhillState1 = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO ,var_cigarette_brand, smokeDunhillStateOwnerList, true,0);
    Effect* smokeDunhillEffect1 = new Effect(smokeDunhillState1, OP_ASSIGN, "dunhill",true);

    // add rule:
    Rule* yellowHouseManSmokesDunhillRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    yellowHouseManSmokesDunhillRule->ruleName = "yellowHouseManSmokesDunhillRule";
    yellowHouseManSmokesDunhillRule->addPrecondition(peopleState1);
    yellowHouseManSmokesDunhillRule->addPrecondition(isYellowState1);
    yellowHouseManSmokesDunhillRule->addPrecondition(liveInYellowState1);

    yellowHouseManSmokesDunhillRule->addEffect(EffectPair(1.0f,smokeDunhillEffect1));

    this->AllRules.push_back(yellowHouseManSmokesDunhillRule);
    //----------------------------End Rule: The person who lives in the Yellow house smokes Dunhill.----------------------------------------------

    //----------------------------Begin Rule: The man living in the center house drinks milk.------------------------------------------------------
    // define variables:
    ParamValue var_milk = str_var[0];

    ParamValue centerHouse = Entity("thirdHouse","object");

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x lives in center house
    vector<ParamValue> liveInCenterHouseStateOwnerList;
    liveInCenterHouseStateOwnerList.push_back(var_man_x);
    State* liveInCenterHouseState = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,centerHouse, liveInCenterHouseStateOwnerList);

    // effect1: var_man_x drinks milk
    vector<ParamValue> drinkMilkStateOwnerList;
    drinkMilkStateOwnerList.push_back(var_man_x);
    State* drinkMilkState = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,var_milk, drinkMilkStateOwnerList, true, 0);
    Effect* drinkMilkEffect = new Effect(drinkMilkState, OP_ASSIGN, "milk",true);

    // add rule:
    Rule* centerHouseManDrinksMilkRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    centerHouseManDrinksMilkRule->ruleName = "centerHouseManDrinksMilkRule";
    centerHouseManDrinksMilkRule->addPrecondition(peopleState1);
    centerHouseManDrinksMilkRule->addPrecondition(liveInCenterHouseState);

    centerHouseManDrinksMilkRule->addEffect(EffectPair(1.0f,drinkMilkEffect));

    this->AllRules.push_back(centerHouseManDrinksMilkRule);
    //----------------------------End Rule:  The man living in the center house drinks milk-------------------------------------------------

    //----------------------------Begin Rule: The Norwegian lives in the first house------------------------------------------------------
    // define variables:
    ParamValue firstHouse = Entity("firstHouse","object");
    ParamValue var_house = entity_var[1];

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x's nation is Norwegian
    vector<ParamValue> NorwegianNationStateOwnerList;
    NorwegianNationStateOwnerList.push_back(var_man_x);
    State* NorwegianNationState = new State("nation",ActionParamType::STRING(),STATE_EQUAL_TO , "Norwegian", NorwegianNationStateOwnerList, true, 0);

    // effect1: var_man_x  lives in the first house
    vector<ParamValue> livesInFirstHouseStateOwnerList;
    livesInFirstHouseStateOwnerList.push_back(var_man_x);
    State* livesInFirstHouseState = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house, livesInFirstHouseStateOwnerList);
    Effect* livesInFirstHouseEffect = new Effect(livesInFirstHouseState, OP_ASSIGN, firstHouse,true);

    // add rule:
    Rule* NorwegianLivesInFirstHouseRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    NorwegianLivesInFirstHouseRule->ruleName = "NorwegianLivesInFirstHouseRule";
    NorwegianLivesInFirstHouseRule->addPrecondition(peopleState1);
    NorwegianLivesInFirstHouseRule->addPrecondition(NorwegianNationState);

    NorwegianLivesInFirstHouseRule->addEffect(EffectPair(1.0f,livesInFirstHouseEffect));

    this->AllRules.push_back(NorwegianLivesInFirstHouseRule);
    //----------------------------End Rule:  The Norwegian lives in the first house--------------------------------------------------

    //----------------------------Begin Rule: The man who smokes Blends lives next to the one who keeps cats------------------------------------------------------
    // define variables:
    ParamValue var_man_y = entity_var[1];
    ParamValue var_house_x = entity_var[2];
    ParamValue var_house_y = entity_var[3];

    // precondition 1: var_man_x is people

    // precondition : var_house_x and var_house_y are house
    vector<ParamValue> isVarXHouseStateOwnerList;
    isVarXHouseStateOwnerList.push_back(var_house_x);
    State* isVarXHouseState = new State("is_house",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isVarXHouseStateOwnerList);

    vector<ParamValue> isVarYHouseStateOwnerList;
    isVarYHouseStateOwnerList.push_back(var_house_y);
    State* isVarYHouseState = new State("is_house",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isVarYHouseStateOwnerList);

    // precondition 2: var_man_x  smokes Blends
    vector<ParamValue> smokeBlendsStateOwnerList;
    smokeBlendsStateOwnerList.push_back(var_man_x);
    State* smokeBlendsState = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO , "blend", smokeBlendsStateOwnerList, true,0);

    // precondition 3: var_man_x lives in var_house_x
    vector<ParamValue> liveInXHouseStateOwnerList;
    liveInXHouseStateOwnerList.push_back(var_man_x);
    State* liveInXHouseState = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_x, liveInXHouseStateOwnerList);

    // precondition 4: var_man_y is people
    vector<ParamValue> peopleStateOwnerList2;
    peopleStateOwnerList2.push_back(var_man_x);
    State* peopleState2 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", peopleStateOwnerList2);

    // precondition 5: var_man_y keeps cats
    vector<ParamValue> keepCatsStateOwnerList;
    keepCatsStateOwnerList.push_back(var_man_y);
    State* keepCatsState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"cats", keepCatsStateOwnerList, true,0);

    // precondition 6: var_man_y lives in var_house_y
    vector<ParamValue> liveInYHouseStateOwnerList;
    liveInYHouseStateOwnerList.push_back(var_man_y);
    State* liveInYHouseState = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_y, liveInYHouseStateOwnerList);

    // effect1: var_house_x is  next to var_house_y
    vector<ParamValue> xnextToyStateOwnerList;
    xnextToyStateOwnerList.push_back(var_house_x);
    State* xnextToyState = new State("nextTo",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_next, xnextToyStateOwnerList);
    Effect* xnextToyEffect = new Effect(xnextToyState, OP_ASSIGN, var_house_y,true);

    // add rule:
    Rule* blendSmokerNextToCatKeeperRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    blendSmokerNextToCatKeeperRule->ruleName = "blendSmokerNextToCatKeeperRule";
    blendSmokerNextToCatKeeperRule->addPrecondition(peopleState1);
    blendSmokerNextToCatKeeperRule->addPrecondition(isVarXHouseState);
    blendSmokerNextToCatKeeperRule->addPrecondition(isVarYHouseState);
    blendSmokerNextToCatKeeperRule->addPrecondition(smokeBlendsState);
    blendSmokerNextToCatKeeperRule->addPrecondition(liveInXHouseState);
    blendSmokerNextToCatKeeperRule->addPrecondition(peopleState2);
    blendSmokerNextToCatKeeperRule->addPrecondition(keepCatsState);
    blendSmokerNextToCatKeeperRule->addPrecondition(liveInYHouseState);

    blendSmokerNextToCatKeeperRule->addEffect(EffectPair(1.0f,xnextToyEffect));

    this->AllRules.push_back(blendSmokerNextToCatKeeperRule);
    //----------------------------End Rule: The man who smokes Blends lives next to the one who keeps cats--------------------------------------------------

    //----------------------------Begin Rule: The man who keeps horse lives next to the man who smokes Dunhill------------------------------------------------------
    // define variables:
    // precondition 1: var_man_x is people

    // precondition 2: var_man_x  smokes Dunhill

    // precondition 3: var_man_x lives in var_house_x

    // precondition 4: var_man_y is people

    // precondition 5: var_man_y keeps horse
    vector<ParamValue> keepHorseStateOwnerList;
    keepHorseStateOwnerList.push_back(var_man_y);
    State* keepHorseState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"horse", keepHorseStateOwnerList, true,0);

    // precondition 6: var_man_y lives in var_house_y

    // effect1: var_house_x is  next to var_house_y

    // add rule:
    Rule* DunhillSmokerNextToHorseKeeperRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    DunhillSmokerNextToHorseKeeperRule->ruleName = "DunhillSmokerNextToHorseKeeperRule";
    DunhillSmokerNextToHorseKeeperRule->addPrecondition(peopleState1);
    DunhillSmokerNextToHorseKeeperRule->addPrecondition(smokeDunhillState1);
    DunhillSmokerNextToHorseKeeperRule->addPrecondition(liveInXHouseState);
    DunhillSmokerNextToHorseKeeperRule->addPrecondition(peopleState2);
    DunhillSmokerNextToHorseKeeperRule->addPrecondition(keepHorseState);
    DunhillSmokerNextToHorseKeeperRule->addPrecondition(liveInYHouseState);

    blendSmokerNextToCatKeeperRule->addEffect(EffectPair(1.0f,xnextToyEffect));

    this->AllRules.push_back(DunhillSmokerNextToHorseKeeperRule);
    //----------------------------End Rule: The man who keeps horses lives next to the man who smokes Dunhill--------------------------------------------------



    //----------------------------Begin Rule: The man who smokes Blue Master drinks beer.----------------------------------------------
    // define variables:
    ParamValue var_beer = str_var[0];

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x smokes Blue Master
    vector<ParamValue> smokeBluemasterStateOwnerList;
    smokeBluemasterStateOwnerList.push_back(var_man_x);
    State* smokeBluemasterState = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO , "bluemaster", smokeBluemasterStateOwnerList, true,0);

    // effect1: var_man_x drinks beer
    vector<ParamValue> drinkBeerStateOwnerList;
    drinkBeerStateOwnerList.push_back(var_man_x);
    State* drinkBeerState = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,var_beer, drinkBeerStateOwnerList, true, 0);
    Effect* drinkBeerEffect = new Effect(drinkBeerState, OP_ASSIGN, "beer",true);

    // add rule:
    Rule* BluemasterSmokerDrinksBeerRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    BluemasterSmokerDrinksBeerRule->ruleName = "BluemasterSmokerDrinksBeerRule";
    BluemasterSmokerDrinksBeerRule->addPrecondition(peopleState1);
    BluemasterSmokerDrinksBeerRule->addPrecondition(smokeBluemasterState);

    BluemasterSmokerDrinksBeerRule->addEffect(EffectPair(1.0f,drinkBeerEffect));

    this->AllRules.push_back(BluemasterSmokerDrinksBeerRule);
    //----------------------------End Rule: The man who smokes Blue Master drinks beer.----------------------------------------------

    //----------------------------Begin Rule: The German smokes Prince.---------------------------------------------------l---
    // define variables:

    // precondition 1: var_man_x is people

    // precondition 2: var_man_x's nation is German
    vector<ParamValue> GermannationStateOwnerList;
    GermannationStateOwnerList.push_back(var_man_x);
    State* GermannationState = new State("nation",ActionParamType::STRING(),STATE_EQUAL_TO , "German", GermannationStateOwnerList, true, 0);

    // effect1: var_man_x smokes Prince
    vector<ParamValue> smokePrinceStateOwnerList;
    smokePrinceStateOwnerList.push_back(var_man_x);
    State* smokePrinceState = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO ,var_cigarette_brand, smokePrinceStateOwnerList, true,0);
    Effect* smokePrinceEffect = new Effect(smokePrinceState, OP_ASSIGN, "prince",true);

    // add rule:
    Rule* GermanSmokesPrincesRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    GermanSmokesPrincesRule->ruleName = "GermanSmokesPrincesRule";
    GermanSmokesPrincesRule->addPrecondition(peopleState1);
    GermanSmokesPrincesRule->addPrecondition(GermannationState);

    GermanSmokesPrincesRule->addEffect(EffectPair(1.0f,smokePrinceEffect));

    this->AllRules.push_back(GermanSmokesPrincesRule);
    //----------------------------End Rule:  The German smokes Prince.--------------------------------------------------

    //----------------------------Begin Rule: The Norwegian lives next to the blue house.-----------------------------------------------------
    // define variables:
    ParamValue var_house_Norwegian = entity_var[0];
    ParamValue var_blue_house = entity_var[1];
    ParamValue var_color = str_var[0];


    // precondition 1: var_man_x is people

    // precondition 2: var_man_x's nation is Norwegian

    // precondition 3: var_man_x lives in var_house_Norwegian
    vector<ParamValue> livesInNorwegianHouseStateOwnerList;
    livesInNorwegianHouseStateOwnerList.push_back(var_man_x);
    State* livesInNorwegianHouseState = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,var_house_Norwegian, livesInNorwegianHouseStateOwnerList);

    // precondition 4: var_house_Norwegian is next to var_blue_house
    vector<ParamValue> NorwegianNextToStateOwnerList;
    NorwegianNextToStateOwnerList.push_back(var_house_Norwegian);
    State* NorwegianNextToState = new State("nextTo",ActionParamType::ENTITY(),STATE_EQUAL_TO , var_blue_house, NorwegianNextToStateOwnerList);

    // effect1 : var_blue_house's color is blue
    vector<ParamValue> blueStateOwnerList;
    blueStateOwnerList.push_back(var_blue_house);
    State* blueState = new State("color",ActionParamType::STRING(),STATE_EQUAL_TO , var_color, blueStateOwnerList, true, 0);
    Effect* blueStateEffect = new Effect(blueState, OP_ASSIGN, "blue",true);

    // add rule:
    Rule* NorwegianLivesNextToBlueHouseRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    NorwegianLivesNextToBlueHouseRule->ruleName = "NorwegianLivesNextToBlueHouseRule";
    NorwegianLivesNextToBlueHouseRule->addPrecondition(peopleState1);
    NorwegianLivesNextToBlueHouseRule->addPrecondition(NorwegianNationState);
    NorwegianLivesNextToBlueHouseRule->addPrecondition(livesInNorwegianHouseState);
    NorwegianLivesNextToBlueHouseRule->addPrecondition(NorwegianNextToState);

    NorwegianLivesNextToBlueHouseRule->addEffect(EffectPair(1.0f,blueStateEffect));

    this->AllRules.push_back(NorwegianLivesNextToBlueHouseRule);
    //----------------------------End Rule:  The Norwegian lives next to the blue house.--------------------------------------------------

    //----------------------------Begin Rule: The Blends smoker lives next to the one who drinks water.------------------------------------------------------
    // define variables:
    // precondition 1: var_man_x is people

    // precondition 2: var_man_x  smokes Blends

    // precondition 3: var_man_x lives in var_house_x

    // precondition 4: var_man_y is people

    // precondition 5: var_man_y drinks water
    vector<ParamValue> drinkWaterStateOwnerList;
    drinkWaterStateOwnerList.push_back(var_man_y);
    State* drinkWaterState = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"water", drinkWaterStateOwnerList, true, 0);

    // precondition 6: var_man_y lives in var_house_y

    // effect1: var_house_x is  next to var_house_y

    // add rule:
    Rule* BlendSmokerNextToWaterDrinkRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    BlendSmokerNextToWaterDrinkRule->ruleName = "BlendSmokerNextToWaterDrinkRule";
    BlendSmokerNextToWaterDrinkRule->addPrecondition(peopleState1);
    BlendSmokerNextToWaterDrinkRule->addPrecondition(smokeBlendsState);
    BlendSmokerNextToWaterDrinkRule->addPrecondition(liveInXHouseState);
    BlendSmokerNextToWaterDrinkRule->addPrecondition(peopleState2);
    BlendSmokerNextToWaterDrinkRule->addPrecondition(drinkWaterState);
    BlendSmokerNextToWaterDrinkRule->addPrecondition(liveInYHouseState);

    blendSmokerNextToCatKeeperRule->addEffect(EffectPair(1.0f,xnextToyEffect));

    this->AllRules.push_back(BlendSmokerNextToWaterDrinkRule);
    //----------------------------End Rule: The Blends smoker lives next to the one who drinks water.--------------------------------------------------



    //----------------------------Begin Rule: closed to person who keep fish to achieve Integrity demanding goal -------------------------------------------
    // define variables:
    ParamValue var_fish_keeper = entity_var[0];
    ParamValue var_achieve_integrity_goal = bool_var[0];

    // precondition 1: var_fish_keeper is people
    vector<ParamValue> peopleStateOwnerList;
    peopleStateOwnerList.push_back(var_fish_keeper);
    State* fishKeeperPeopleState = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", peopleStateOwnerList);

    // precondition 2: var_fish_keeper keeps fish
    vector<ParamValue> keepFishStateOwnerList;
    keepFishStateOwnerList.push_back(var_fish_keeper);
    State* keepFishState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"fish", keepFishStateOwnerList, true,0);

    // precondition 3: closed to var_fish_keeper
    vector<ParamValue> closedToFishKeeperStateOwnerList;
    closedToFishKeeperStateOwnerList.push_back(var_avatar);
    closedToFishKeeperStateOwnerList.push_back(var_fish_keeper);
    State* closedToFishKeeperState = new State("Distance",ActionParamType::FLOAT(),STATE_LESS_THAN ,ACCESS_DISTANCE, closedToFishKeeperStateOwnerList, true, &Inquery::inqueryDistance);

    // energy state:
    vector<ParamValue> IntegrityStateOwnerList;
    IntegrityStateOwnerList.push_back(var_avatar);
    State* IntegrityGoalState = new State("IntegrityDemandGoal",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,var_achieve_integrity_goal, IntegrityStateOwnerList);

    // effect1: Integrity Goal Achieved
    Effect* IntegrityGoalAchievedEffect = new Effect(IntegrityGoalState, OP_ASSIGN, SV_TRUE);

    Rule* IntegrityRule = new Rule(doNothingAction,boost::get<Entity>(var_avatar),0.2f);
    IntegrityRule->ruleName = "ClosedToFishKeeperIIntegrityRule";
    IntegrityRule->addPrecondition(fishKeeperPeopleState);
    IntegrityRule->addPrecondition(keepFishState);
    IntegrityRule->addPrecondition(closedToFishKeeperState);

    IntegrityRule->addEffect(EffectPair(1.0f,IntegrityGoalAchievedEffect));

    this->AllRules.push_back(IntegrityRule);

    //----------------------------End Rule:closed to person who keep fish to achieve Integrity demanding goal -------------------------------------------

    //----------------------------Begin Rule: if other 4 people do not live in house_n, then man_1 live in house_n--------------------------------------
    // define variables:
    ParamValue man_1 = entity_var[0];
    ParamValue man_2 = entity_var[1];
    ParamValue man_3 = entity_var[2];
    ParamValue man_4 = entity_var[3];
    ParamValue man_5 = entity_var[4];
    ParamValue house_n = entity_var[5];
    ParamValue house_man_1 = entity_var[6];

    // precondition 0: house_n is house
    vector<ParamValue> isHouseStateOwnerList1;
    isHouseStateOwnerList1.push_back(house_n);
    State* isHouseState1 = new State("is_house",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isHouseStateOwnerList1);

    // precondition 1 -5 : man_1-5 are people
    vector<ParamValue> ispeopleStateOwnerList1;
    ispeopleStateOwnerList1.push_back(man_1);
    State* ispeopleState1 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList1);

    vector<ParamValue> ispeopleStateOwnerList2;
    ispeopleStateOwnerList2.push_back(man_2);
    State* ispeopleState2 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList2);

    vector<ParamValue> ispeopleStateOwnerList3;
    ispeopleStateOwnerList3.push_back(man_3);
    State* ispeopleState3 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList3);

    vector<ParamValue> ispeopleStateOwnerList4;
    ispeopleStateOwnerList4.push_back(man_4);
    State* ispeopleState4 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList4);

    vector<ParamValue> ispeopleStateOwnerList5;
    ispeopleStateOwnerList5.push_back(man_5);
    State* ispeopleState5 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList5);

    // additional preconditions: man_2, man_3, man_4, man_5 are not the same to man_1
    vector<ParamValue> isNotSameOwnerList1;
    isNotSameOwnerList1.push_back(man_1);
    isNotSameOwnerList1.push_back(man_2);
    State* isNotSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameOwnerList1,true, &Inquery::inqueryIsSame);

    vector<ParamValue> isNotSameOwnerList2;
    isNotSameOwnerList2.push_back(man_1);
    isNotSameOwnerList2.push_back(man_3);
    State* isNotSameState2 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameOwnerList2,true, &Inquery::inqueryIsSame);

    vector<ParamValue> isNotSameOwnerList3;
    isNotSameOwnerList3.push_back(man_1);
    isNotSameOwnerList3.push_back(man_4);
    State* isNotSameState3 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameOwnerList3,true, &Inquery::inqueryIsSame);

    vector<ParamValue> isNotSameOwnerList4;
    isNotSameOwnerList4.push_back(man_1);
    isNotSameOwnerList4.push_back(man_5);
    State* isNotSameState4 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameOwnerList4,true, &Inquery::inqueryIsSame);

    // precondition 6-9 : other people do not live in house_1
    vector<ParamValue> notlivesInStateOwnerList1;
    notlivesInStateOwnerList1.push_back(man_2);
    State* notlivesInState1 = new State("liveIn",ActionParamType::ENTITY(),STATE_NOT_EQUAL_TO ,house_n, notlivesInStateOwnerList1);

    vector<ParamValue> notlivesInStateOwnerList2;
    notlivesInStateOwnerList2.push_back(man_3);
    State* notlivesInState2 = new State("liveIn",ActionParamType::ENTITY(),STATE_NOT_EQUAL_TO ,house_n, notlivesInStateOwnerList2);

    vector<ParamValue> notlivesInStateOwnerList3;
    notlivesInStateOwnerList3.push_back(man_4);
    State* notlivesInState3 = new State("liveIn",ActionParamType::ENTITY(),STATE_NOT_EQUAL_TO ,house_n, notlivesInStateOwnerList3);

    vector<ParamValue> notlivesInStateOwnerList4;
    notlivesInStateOwnerList4.push_back(man_5);
    State* notlivesInState4 = new State("liveIn",ActionParamType::ENTITY(),STATE_NOT_EQUAL_TO ,house_n, notlivesInStateOwnerList4);


    // effect1: man_1 lives in house_n
    vector<ParamValue> livesInHouseNStateOwnerList;
    livesInHouseNStateOwnerList.push_back(man_1);
    State* livesInHouseNState = new State("liveIn",ActionParamType::ENTITY(),STATE_EQUAL_TO ,house_man_1, livesInHouseNStateOwnerList);
    Effect* livesInHouseNEffect = new Effect(livesInHouseNState, OP_ASSIGN, house_n,true);

    // add rule:
    Rule* notLiveInOtherPeoplesHouseRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    notLiveInOtherPeoplesHouseRule->ruleName = "notLiveInOtherPeoplesHouseRule";
    notLiveInOtherPeoplesHouseRule->addPrecondition(isHouseState1);
    notLiveInOtherPeoplesHouseRule->addPrecondition(ispeopleState1);
    notLiveInOtherPeoplesHouseRule->addPrecondition(ispeopleState2);
    notLiveInOtherPeoplesHouseRule->addPrecondition(ispeopleState3);
    notLiveInOtherPeoplesHouseRule->addPrecondition(ispeopleState4);
    notLiveInOtherPeoplesHouseRule->addPrecondition(ispeopleState5);

    notLiveInOtherPeoplesHouseRule->addPrecondition(isNotSameState1);
    notLiveInOtherPeoplesHouseRule->addPrecondition(isNotSameState2);
    notLiveInOtherPeoplesHouseRule->addPrecondition(isNotSameState3);
    notLiveInOtherPeoplesHouseRule->addPrecondition(isNotSameState4);

    notLiveInOtherPeoplesHouseRule->addPrecondition(notlivesInState1);
    notLiveInOtherPeoplesHouseRule->addPrecondition(notlivesInState2);
    notLiveInOtherPeoplesHouseRule->addPrecondition(notlivesInState3);
    notLiveInOtherPeoplesHouseRule->addPrecondition(notlivesInState4);

    notLiveInOtherPeoplesHouseRule->addEffect(EffectPair(1.0f,livesInHouseNEffect));

    this->AllRules.push_back(notLiveInOtherPeoplesHouseRule);
    //----------------------------End Rule: if other 4 people do not live in house_1, then man_1 live in house_1-------------------------------------------------

    //----------------------------Begin Rule: if other 4 people do not smoke brand_x, then man_1 smokes it--------------------------------------
    // define variables:
    ParamValue brand_x = str_var[1];

    // precondition 0: brand_x is cigaretteBrand
    vector<ParamValue> isBrandStateOwnerList1;
    isBrandStateOwnerList1.push_back(brand_x);
    State* isBrandState1 = new State("is_cigaretteBrand",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isBrandStateOwnerList1);

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not smoke brand_x
    vector<ParamValue> notSmokeXStateOwnerList1;
    notSmokeXStateOwnerList1.push_back(man_2);
    State* notSmokeXState1 = new State("smoke",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,brand_x, notSmokeXStateOwnerList1, true,0);

    vector<ParamValue> notSmokeXStateOwnerList2;
    notSmokeXStateOwnerList2.push_back(man_3);
    State* notSmokeXState2 = new State("smoke",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,brand_x, notSmokeXStateOwnerList2, true,0);

    vector<ParamValue> notSmokeXStateOwnerList3;
    notSmokeXStateOwnerList3.push_back(man_4);
    State* notSmokeXState3 = new State("smoke",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,brand_x, notSmokeXStateOwnerList3, true,0);

    vector<ParamValue> notSmokeXStateOwnerList4;
    notSmokeXStateOwnerList4.push_back(man_5);
    State* notSmokeXState4 = new State("smoke",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,brand_x, notSmokeXStateOwnerList4, true,0);

    // effect1: man_1 smokes brand_x
    vector<ParamValue> smokeXStateOwnerList;
    smokeXStateOwnerList.push_back(man_1);
    State* smokeXState = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO ,var_cigarette_brand, smokeXStateOwnerList, true,0);
    Effect* smokeXEffect = new Effect(smokeXState, OP_ASSIGN, brand_x,true);

    // add rule:
    Rule* notSmokeOtherPeoplesBrandRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    notSmokeOtherPeoplesBrandRule->ruleName = "notSmokeOtherPeoplesBrandRule";
    notSmokeOtherPeoplesBrandRule->addPrecondition(isBrandState1);
    notSmokeOtherPeoplesBrandRule->addPrecondition(ispeopleState1);
    notSmokeOtherPeoplesBrandRule->addPrecondition(ispeopleState2);
    notSmokeOtherPeoplesBrandRule->addPrecondition(ispeopleState3);
    notSmokeOtherPeoplesBrandRule->addPrecondition(ispeopleState4);
    notSmokeOtherPeoplesBrandRule->addPrecondition(ispeopleState5);

    notSmokeOtherPeoplesBrandRule->addPrecondition(isNotSameState1);
    notSmokeOtherPeoplesBrandRule->addPrecondition(isNotSameState2);
    notSmokeOtherPeoplesBrandRule->addPrecondition(isNotSameState3);
    notSmokeOtherPeoplesBrandRule->addPrecondition(isNotSameState4);

    notSmokeOtherPeoplesBrandRule->addPrecondition(notSmokeXState1);
    notSmokeOtherPeoplesBrandRule->addPrecondition(notSmokeXState2);
    notSmokeOtherPeoplesBrandRule->addPrecondition(notSmokeXState3);
    notSmokeOtherPeoplesBrandRule->addPrecondition(notSmokeXState4);

    notSmokeOtherPeoplesBrandRule->addEffect(EffectPair(1.0f,smokeXEffect));

    this->AllRules.push_back(notSmokeOtherPeoplesBrandRule);
    //----------------------------End Rule:  if other 4 people do not smoke brand_x, then man_1 smokes it-------------------------------------------------

//    //----------------------------Begin Rule: if other 4 people do not keep pet_x, then man_1 keeps it--------------------------------------
//    // define variables:
//    ParamValue man_1_pet = str_var[0];
//    ParamValue pet_x = str_var[1];

//    // precondition 0: pet_x is pet
//    vector<ParamValue> isPetStateOwnerList1;
//    isPetStateOwnerList1.push_back(pet_x);
//    State* isPetState1 = new State("is_pet",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isPetStateOwnerList1);

//    // precondition 1 -5 : man_1-5 are people

//    // precondition 6-9 : other people do not keep pet_x
//    vector<ParamValue> notkeepXStateOwnerList1;
//    notkeepXStateOwnerList1.push_back(man_2);
//    State* notkeepXState1 = new State("keep_pet",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,pet_x, notkeepXStateOwnerList1, true,0);

//    vector<ParamValue> notkeepXStateOwnerList2;
//    notkeepXStateOwnerList2.push_back(man_3);
//    State* notkeepXState2 = new State("keep_pet",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,pet_x, notkeepXStateOwnerList2, true,0);

//    vector<ParamValue> notkeepXStateOwnerList3;
//    notkeepXStateOwnerList3.push_back(man_4);
//    State* notkeepXState3 = new State("keep_pet",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,pet_x, notkeepXStateOwnerList3, true,0);

//    vector<ParamValue> notkeepXStateOwnerList4;
//    notkeepXStateOwnerList4.push_back(man_5);
//    State* notkeepXState4 = new State("keep_pet",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,pet_x, notkeepXStateOwnerList4, true,0);

//    // effect1: man_1 keeps pet_x
//    vector<ParamValue> keepXStateOwnerList;
//    keepXStateOwnerList.push_back(man_1);
//    State* keepXState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepXStateOwnerList, true,0);
//    Effect* keepXEffect = new Effect(keepXState, OP_ASSIGN, pet_x,true);

//    // add rule:
//    Rule* notKeepOtherPeoplesPetRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
//    notKeepOtherPeoplesPetRule->ruleName = "notKeepOtherPeoplesPetRule";
//    notKeepOtherPeoplesPetRule->addPrecondition(isPetState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState2);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState3);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState4);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState5);

//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSameState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSameState2);
//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSameState3);
//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSameState4);

//    notKeepOtherPeoplesPetRule->addPrecondition(notkeepXState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(notkeepXState2);
//    notKeepOtherPeoplesPetRule->addPrecondition(notkeepXState3);
//    notKeepOtherPeoplesPetRule->addPrecondition(notkeepXState4);

//    notKeepOtherPeoplesPetRule->addEffect(EffectPair(1.0f,keepXEffect));

//    this->AllRules.push_back(notKeepOtherPeoplesPetRule);
//    //----------------------------End Rule:  if other 4 people do not keep pet_x, then man_1 keeps it-------------------------------------------------

    //----------------------------Begin Rule: if other 4 people's nation is not nation_x, then man_1's nation is nation_x--------------------------------------
    // define variables:
    ParamValue nation_man_1 = str_var[0];
    ParamValue nation_x = str_var[1];

    // precondition 0: nation_x is nation
    vector<ParamValue> isNationStateOwnerList1;
    isNationStateOwnerList1.push_back(nation_x);
    State* isNationState1 = new State("is_nation",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isNationStateOwnerList1);

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not drink drink_x
    vector<ParamValue> notNaxtionXStateOwnerList1;
    notNaxtionXStateOwnerList1.push_back(man_2);
    State* notNaxtionXState1 = new State("nation",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,nation_x, notNaxtionXStateOwnerList1, true, 0);

    vector<ParamValue> notNaxtionXStateOwnerList2;
    notNaxtionXStateOwnerList2.push_back(man_3);
    State* notNaxtionXState2 = new State("nation",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,nation_x, notNaxtionXStateOwnerList2, true, 0);

    vector<ParamValue> notNaxtionXStateOwnerList3;
    notNaxtionXStateOwnerList3.push_back(man_4);
    State* notNaxtionXState3 = new State("nation",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,nation_x, notNaxtionXStateOwnerList3, true, 0);

    vector<ParamValue> notNaxtionXStateOwnerList4;
    notNaxtionXStateOwnerList4.push_back(man_5);
    State* notNaxtionXState4 = new State("nation",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,nation_x, notNaxtionXStateOwnerList4, true, 0);

    // effect1: man_1's nation is nation_x
    vector<ParamValue> isNationXStateOwnerList;
    isNationXStateOwnerList.push_back(man_1);
    State* isNationXState = new State("nation",ActionParamType::STRING(),STATE_EQUAL_TO ,nation_man_1, isNationXStateOwnerList, true, 0);
    Effect* isNationXEffect = new Effect(isNationXState, OP_ASSIGN, nation_x,true);


    // add rule:
    Rule* notTakeOtherPeoplesNationRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    notTakeOtherPeoplesNationRule->ruleName = "notTakeOtherPeoplesNationRule";
    notTakeOtherPeoplesNationRule->addPrecondition(isNationState1);
    notTakeOtherPeoplesNationRule->addPrecondition(ispeopleState1);
    notTakeOtherPeoplesNationRule->addPrecondition(ispeopleState2);
    notTakeOtherPeoplesNationRule->addPrecondition(ispeopleState3);
    notTakeOtherPeoplesNationRule->addPrecondition(ispeopleState4);
    notTakeOtherPeoplesNationRule->addPrecondition(ispeopleState5);

    notTakeOtherPeoplesNationRule->addPrecondition(isNotSameState1);
    notTakeOtherPeoplesNationRule->addPrecondition(isNotSameState2);
    notTakeOtherPeoplesNationRule->addPrecondition(isNotSameState3);
    notTakeOtherPeoplesNationRule->addPrecondition(isNotSameState4);

    notTakeOtherPeoplesNationRule->addPrecondition(notNaxtionXState1);
    notTakeOtherPeoplesNationRule->addPrecondition(notNaxtionXState2);
    notTakeOtherPeoplesNationRule->addPrecondition(notNaxtionXState3);
    notTakeOtherPeoplesNationRule->addPrecondition(notNaxtionXState4);

    notTakeOtherPeoplesNationRule->addEffect(EffectPair(1.0f,isNationXEffect));

    this->AllRules.push_back(notTakeOtherPeoplesNationRule);
    //----------------------------End Rule: if other 4 people's nation is not nation_x, then man_1's nation is nation_x--------------------------------------------------

//    //----------------------------Begin Rule: if other 4 people do not drink drink_x, then man_1 drinks it--------------------------------------
//    // define variables:
//    ParamValue drink_man_1 = str_var[0];
//    ParamValue drink_x = str_var[1];

//    // precondition 0: drink_x is drink
//    vector<ParamValue> isDrinkStateOwnerList1;
//    isDrinkStateOwnerList1.push_back(drink_x);
//    State* isDrinkState1 = new State("is_drink",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isDrinkStateOwnerList1);

//    // precondition 1 -5 : man_1-5 are people

//    // precondition 6-9 : other people do not drink drink_x
//    vector<ParamValue> notdrinkXStateOwnerList1;
//    notdrinkXStateOwnerList1.push_back(man_2);
//    State* notdrinkXState1 = new State("drink",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,drink_x, notdrinkXStateOwnerList1, true, 0);

//    vector<ParamValue> notdrinkXStateOwnerList2;
//    notdrinkXStateOwnerList2.push_back(man_3);
//    State* notdrinkXState2 = new State("drink",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,drink_x, notdrinkXStateOwnerList2, true, 0);

//    vector<ParamValue> notdrinkXStateOwnerList3;
//    notdrinkXStateOwnerList3.push_back(man_4);
//    State* notdrinkXState3 = new State("drink",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,drink_x, notdrinkXStateOwnerList3, true, 0);

//    vector<ParamValue> notdrinkXStateOwnerList4;
//    notdrinkXStateOwnerList4.push_back(man_5);
//    State* notdrinkXState4 = new State("drink",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,drink_x, notdrinkXStateOwnerList4, true, 0);

//    // effect1: man_1 drinks drink_x
//    vector<ParamValue> drinkXStateOwnerList;
//    drinkXStateOwnerList.push_back(man_1);
//    State* drinkXState = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,drink_man_1, drinkXStateOwnerList, true, 0);
//    Effect* drinkXEffect = new Effect(drinkXState, OP_ASSIGN, drink_x,true);


//    // add rule:
//    Rule* notDrinkOtherPeoplesDrinkRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
//    notDrinkOtherPeoplesDrinkRule->ruleName = "notDrinkOtherPeoplesDrinkRule";
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isDrinkState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState2);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState3);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState4);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState5);

//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameState2);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameState3);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameState4);

//    notDrinkOtherPeoplesDrinkRule->addPrecondition(notdrinkXState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(notdrinkXState2);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(notdrinkXState3);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(notdrinkXState4);

//    notDrinkOtherPeoplesDrinkRule->addEffect(EffectPair(1.0f,drinkXEffect));

//    this->AllRules.push_back(notDrinkOtherPeoplesDrinkRule);
//    //----------------------------End Rule:  if other 4 people do not drink drink_x, then man_1 drinks it--------------------------------------------------

    //----------------------------Begin Rule: if var_man_x lives in var_house_x, and var_house_x is not the same to var_house_y, then var_man_x doesn't live in var_house_y-------
    // precondition 0: var_house_x and house_y are pets

    // precondition 1: var_man_x lives in var_house_x

    // precondition 2:  var_house_x is not the same to var_house_y
    vector<ParamValue> houseXYisNotSameOwnerList1;
    houseXYisNotSameOwnerList1.push_back(var_house_x);
    houseXYisNotSameOwnerList1.push_back(var_house_y);
    State* houseXYisNotSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", houseXYisNotSameOwnerList1,true, &Inquery::inqueryIsSame);

    // effect 1: var_man_x doesn't live in var_house_y
    Effect* notLiveInHouseYStateEffect = new Effect(liveInXHouseState, OP_ASSIGN_NOT_EQUAL_TO, var_house_y,true);

    Rule* ifLivesinHouseXThenNotLivesinYRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifLivesinHouseXThenNotLivesinYRule->ruleName = "ifLivesinHouseXThenNotLivesinYRule";
    ifLivesinHouseXThenNotLivesinYRule->addPrecondition(peopleState1);
    ifLivesinHouseXThenNotLivesinYRule->addPrecondition(isVarXHouseState);
    ifLivesinHouseXThenNotLivesinYRule->addPrecondition(isVarYHouseState);
    ifLivesinHouseXThenNotLivesinYRule->addPrecondition(liveInXHouseState);
    ifLivesinHouseXThenNotLivesinYRule->addPrecondition(houseXYisNotSameState1);

    ifLivesinHouseXThenNotLivesinYRule->addEffect(EffectPair(1.0f,notLiveInHouseYStateEffect));

    this->AllRules.push_back(ifLivesinHouseXThenNotLivesinYRule);

    //----------------------------End Rule: if man_1 lives in house_x, and house_x is not the same to house_y, then man_1 doesn't live in house_y-------

    //----------------------------Begin Rule: if man_1 keeps pet_x, and pet_x is not the same to pet_y, then man_1 doesn't keep pet_y----------------------
    // define variables:
    ParamValue man_1_pet = str_var[0];
    ParamValue pet_x = str_var[1];
    ParamValue pet_y = str_var[2];

    // precondition 0: pet_x pet_y is pet
    vector<ParamValue> isPetStateOwnerList1;
    isPetStateOwnerList1.push_back(pet_x);
    State* isPetState1 = new State("is_pet",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isPetStateOwnerList1);

    vector<ParamValue> isPetyStateOwnerList1;
    isPetyStateOwnerList1.push_back(pet_y);
    State* isPetyState1 = new State("is_pet",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isPetyStateOwnerList1);

    // precondition 1: man_1 keepsl pet_x
    vector<ParamValue> keepXStateOwnerList2;
    keepXStateOwnerList2.push_back(man_1);
    State* keepXState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,pet_x, keepXStateOwnerList2, true,0);

    // precondition 2: pet_x is not the same to pet_y
    vector<ParamValue> petXYisNotSameOwnerList1;
    petXYisNotSameOwnerList1.push_back(pet_x);
    petXYisNotSameOwnerList1.push_back(pet_y);
    State* petXYisNotSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", petXYisNotSameOwnerList1,true, &Inquery::inqueryIsSame);

    // effect 1: man_1 doesn't keep pet_y
    Effect* notkeepYStateEffect = new Effect(keepXState2, OP_ASSIGN_NOT_EQUAL_TO, pet_y,true);

    // add rule:
    Rule* ifKeepsPetXThenNotKeepYRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifKeepsPetXThenNotKeepYRule->ruleName = "ifKeepsPetXThenNotKeepYRule";
    ifKeepsPetXThenNotKeepYRule->addPrecondition(ispeopleState1);
    ifKeepsPetXThenNotKeepYRule->addPrecondition(isPetState1);
    ifKeepsPetXThenNotKeepYRule->addPrecondition(isPetyState1);
    ifKeepsPetXThenNotKeepYRule->addPrecondition(keepXState2);
    ifKeepsPetXThenNotKeepYRule->addPrecondition(petXYisNotSameState1);

    ifKeepsPetXThenNotKeepYRule->addEffect(EffectPair(1.0f,notkeepYStateEffect));

    this->AllRules.push_back(ifKeepsPetXThenNotKeepYRule);
    //----------------------------End Rule: if man_1 keeps pet_x, and pet_x is not the same to pet_y, then man_1 doesn't keep pet_y----------------------

    //----------------------------Begin Rule:  if man_1 smokes brand_x, and brand_x is not the same to brand_y, then man_1 doesn't smokes brand_y----
    // define variables:
    ParamValue brand_y = str_var[2];

    // precondition 0: peopleState1
    // precondition 1: brand_x and brand_y is cigaretteBrand
    vector<ParamValue> isBrandStateOwnerList2;
    isBrandStateOwnerList2.push_back(brand_y);
    State* isBrandState2 = new State("is_cigaretteBrand",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isBrandStateOwnerList2);

    // precondition 2: man_1 smokes brand_x
    vector<ParamValue> smokeXStateOwnerList2;
    smokeXStateOwnerList2.push_back(man_1);
    State* smokeXState2 = new State("smoke",ActionParamType::STRING(),STATE_EQUAL_TO ,brand_x, smokeXStateOwnerList2, true,0);

    // precondition 3:  brand_x is not the same to brand_y
    vector<ParamValue> brandXYisNotSameOwnerList1;
    brandXYisNotSameOwnerList1.push_back(brand_x);
    brandXYisNotSameOwnerList1.push_back(brand_y);
    State* brandXYisNotSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", brandXYisNotSameOwnerList1,true, &Inquery::inqueryIsSame);

    // effect 1: man_1 doesn't smokes brand_y
    Effect* notSmokeYStateEffect = new Effect(smokeXState2, OP_ASSIGN_NOT_EQUAL_TO, brand_y,true);

    // add rule:
    Rule* ifSmokesBrandXThenNotSmokesYRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifSmokesBrandXThenNotSmokesYRule->ruleName = "ifSmokesBrandXThenNotSmokesYRule";
    ifSmokesBrandXThenNotSmokesYRule->addPrecondition(ispeopleState1);
    ifSmokesBrandXThenNotSmokesYRule->addPrecondition(isBrandState1);
    ifSmokesBrandXThenNotSmokesYRule->addPrecondition(isBrandState2);
    ifSmokesBrandXThenNotSmokesYRule->addPrecondition(smokeXState2);
    ifSmokesBrandXThenNotSmokesYRule->addPrecondition(brandXYisNotSameState1);

    ifSmokesBrandXThenNotSmokesYRule->addEffect(EffectPair(1.0f,notSmokeYStateEffect));

    this->AllRules.push_back(ifSmokesBrandXThenNotSmokesYRule);
    //----------------------------End Rule:  if man_1 smokes brand_x, and brand_x is not the same to brand_y, then man_1 doesn't smokes brand_y----

    //----------------------------Begin Rule:  if man_1 drinks drink_x, and brand_x is not the same to brand_y, then man_1 doesn't smokes brand_y----
    ParamValue drink_x = str_var[1];
    ParamValue drink_y = str_var[2];

    // precondition 0: peopleState1

    // precondition 1: drink_x drink_y are drink
    vector<ParamValue> isDrinkStateOwnerList1;
    isDrinkStateOwnerList1.push_back(drink_x);
    State* isDrinkState1 = new State("is_drink",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isDrinkStateOwnerList1);

    vector<ParamValue> isDrinkStateOwnerList2;
    isDrinkStateOwnerList2.push_back(drink_y);
    State* isDrinkState2 = new State("is_drink",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isDrinkStateOwnerList2);

    // precondition 2: man_1 drinks drink_x
    vector<ParamValue> drinkXStateOwnerList1;
    drinkXStateOwnerList1.push_back(man_1);
    State* drinkXState1 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,drink_x, drinkXStateOwnerList1, true, 0);

    // precondition 3: brand_x is not the same to brand_y
    vector<ParamValue> drinkXYisNotSameOwnerList1;
    drinkXYisNotSameOwnerList1.push_back(drink_x);
    drinkXYisNotSameOwnerList1.push_back(drink_y);
    State* drinkXYisNotSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", drinkXYisNotSameOwnerList1,true, &Inquery::inqueryIsSame);

    // effect1: man_1 drinks drink_x
    Effect* notdrinkYEffect = new Effect(drinkXState1, OP_ASSIGN_NOT_EQUAL_TO, drink_y,true);

    // add rule:
    Rule* ifDrinkXThenNotDrinkYRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifDrinkXThenNotDrinkYRule->ruleName = "ifDrinkXThenNotDrinkYRule";
    ifDrinkXThenNotDrinkYRule->addPrecondition(ispeopleState1);
    ifDrinkXThenNotDrinkYRule->addPrecondition(isDrinkState1);
    ifDrinkXThenNotDrinkYRule->addPrecondition(isDrinkState2);
    ifDrinkXThenNotDrinkYRule->addPrecondition(drinkXState1);
    ifDrinkXThenNotDrinkYRule->addPrecondition(drinkXYisNotSameState1);

    ifDrinkXThenNotDrinkYRule->addEffect(EffectPair(1.0f,notdrinkYEffect));

    this->AllRules.push_back(ifSmokesBrandXThenNotSmokesYRule);
    //----------------------------ENd Rule:  if man_1 drinks drink_x, and brand_x is not the same to brand_y, then man_1 doesn't smokes brand_y----

    //----------------------------Begin Rule: if man_1 keeps pet_x, man_1 and man_2 are not the same, so man_2 doesn't keep pet_x----------------------
    // define variables:
    // precondition 0: man_1 and man_2 are people, pet_x and pet_y are pet
    // precondition 1: man_1 keepsl pet_x

    // precondition 2: man_1 is not the same to man_2
    vector<ParamValue> man12isNotSameOwnerList1;
    man12isNotSameOwnerList1.push_back(man_1);
    man12isNotSameOwnerList1.push_back(man_2);
    State* man12isNotSameSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", man12isNotSameOwnerList1,true, &Inquery::inqueryIsSame);

    // effect 1: man_2 doesn't keep pet_x
    vector<ParamValue> man2notkeepPetXStateOwnerList;
    man2notkeepPetXStateOwnerList.push_back(man_2);
    State* man2notkeepPetXState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,pet_y, man2notkeepPetXStateOwnerList, true,0);
    Effect* man2notkeepPetXStateEffect = new Effect(man2notkeepPetXState, OP_ASSIGN_NOT_EQUAL_TO, pet_x,true);

    // add rule:
    Rule* ifMan1KeepsPetXThenMan2NotKeepXRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifMan1KeepsPetXThenMan2NotKeepXRule->ruleName = "ifMan1KeepsPetXThenMan2NotKeepXRule";
    ifMan1KeepsPetXThenMan2NotKeepXRule->addPrecondition(ispeopleState1);
    ifMan1KeepsPetXThenMan2NotKeepXRule->addPrecondition(ispeopleState2);
    ifMan1KeepsPetXThenMan2NotKeepXRule->addPrecondition(isPetState1);
    ifMan1KeepsPetXThenMan2NotKeepXRule->addPrecondition(isPetyState1);
    ifMan1KeepsPetXThenMan2NotKeepXRule->addPrecondition(keepXState2);
    ifMan1KeepsPetXThenMan2NotKeepXRule->addPrecondition(man12isNotSameSameState1);

    ifMan1KeepsPetXThenMan2NotKeepXRule->addEffect(EffectPair(1.0f,man2notkeepPetXStateEffect));

    this->AllRules.push_back(ifMan1KeepsPetXThenMan2NotKeepXRule);
    //----------------------------End Rule: if man_1 keeps pet_x, and pet_x is not the same to pet_y, then man_1 doesn't keep pet_y----------------------

    //----------------------------Begin Rule: if other 4 people keep dogs,horse,fish,birds, then man_1 keeps cats--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not keep pet_x
    vector<ParamValue> keepDogsStateOwnerList2;
    keepDogsStateOwnerList2.push_back(man_2);
    State* keepDogsState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"dogs", keepDogsStateOwnerList2, true,0);

    vector<ParamValue> keepHorseStateOwnerList2;
    keepHorseStateOwnerList2.push_back(man_3);
    State* keepHorseState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"horse", keepHorseStateOwnerList2, true,0);

    vector<ParamValue> keepFishStateOwnerList2;
    keepFishStateOwnerList2.push_back(man_4);
    State* keepFishState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"fish", keepFishStateOwnerList2, true,0);

    vector<ParamValue> keepBirdsStateOwnerList2;
    keepBirdsStateOwnerList2.push_back(man_5);
    State* keepBirdsState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"birds", keepBirdsStateOwnerList2, true,0);

    // effect1: man_1 keeps cats
    vector<ParamValue> keepCatsStateOwnerList2;
    keepCatsStateOwnerList2.push_back(man_1);
    State* keepCatsState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepCatsStateOwnerList2, true,0);
    Effect* keepCatsEffect2 = new Effect(keepCatsState2, OP_ASSIGN, "cats",true);

    // add rule:
    Rule* ifOthersKeepOtherPetsThenMan1KeepCatsRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->ruleName = "ifOthersKeepOtherPetsThenMan1KeepCatsRule";
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(ispeopleState1);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(ispeopleState2);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(ispeopleState3);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(ispeopleState4);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(ispeopleState5);

    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(isNotSameState1);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(isNotSameState2);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(isNotSameState3);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(isNotSameState4);

    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(keepDogsState2);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(keepHorseState2);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(keepFishState2);
    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addPrecondition(keepBirdsState2);

    ifOthersKeepOtherPetsThenMan1KeepCatsRule->addEffect(EffectPair(1.0f,keepCatsEffect2));

    this->AllRules.push_back(ifOthersKeepOtherPetsThenMan1KeepCatsRule);
    //----------------------------End Rule: if other 4 people keep dogs,horse,fish,birds, then man_1 keeps cats--------------------------------------

    //----------------------------Begin Rule: if other 4 people keep cats,horse,fish,birds, then man_1 keeps dogs--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not keep pet_x
    vector<ParamValue> keepCatsStateOwnerList3;
    keepCatsStateOwnerList3.push_back(man_2);
    State* keepCatsState3 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"cats", keepCatsStateOwnerList3, true,0);

    // effect1: man_1 keeps dogs
    vector<ParamValue> keepDogsStateOwnerList3;
    keepDogsStateOwnerList3.push_back(man_1);
    State* keepDogsState3 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepDogsStateOwnerList3, true,0);
    Effect* keepDogsEffect3 = new Effect(keepDogsState3, OP_ASSIGN, "cats",true);

    // add rule:
    Rule* ifOthersKeepOtherPetsThenMan1KeepDogsRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->ruleName = "ifOthersKeepOtherPetsThenMan1KeepDogsRule";
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(ispeopleState1);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(ispeopleState2);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(ispeopleState3);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(ispeopleState4);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(ispeopleState5);

    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(isNotSameState1);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(isNotSameState2);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(isNotSameState3);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(isNotSameState4);

    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(keepCatsState3);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(keepHorseState2);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(keepFishState2);
    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addPrecondition(keepBirdsState2);

    ifOthersKeepOtherPetsThenMan1KeepDogsRule->addEffect(EffectPair(1.0f,keepDogsEffect3));

    this->AllRules.push_back(ifOthersKeepOtherPetsThenMan1KeepDogsRule);
    //----------------------------End Rule: if other 4 people keep dogs,horse,fish,birds, then man_1 keeps cats--------------------------------------

    //----------------------------Begin Rule: if other 4 people keep cats,horse,dogs,birds, then man_1 keeps fish--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not keep fish
    vector<ParamValue> keepCatsStateOwnerList4;
    keepCatsStateOwnerList4.push_back(man_4);
    State* keepCatsState4 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"cats", keepCatsStateOwnerList4, true,0);

    // effect1: man_1 keeps fish
    vector<ParamValue> keepFishStateOwnerList3;
    keepFishStateOwnerList3.push_back(man_1);
    State* keepFishState3 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepFishStateOwnerList3, true,0);
    Effect* keepFishEffect3 = new Effect(keepFishState3, OP_ASSIGN, "fish",true);

    // add rule:
    Rule* ifOthersKeepOtherPetsThenMan1KeepFishRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->ruleName = "ifOthersKeepOtherPetsThenMan1KeepFishRule";
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(ispeopleState1);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(ispeopleState2);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(ispeopleState3);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(ispeopleState4);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(ispeopleState5);

    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(isNotSameState1);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(isNotSameState2);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(isNotSameState3);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(isNotSameState4);

    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(keepDogsState2);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(keepHorseState2);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(keepCatsState4);
    ifOthersKeepOtherPetsThenMan1KeepFishRule->addPrecondition(keepBirdsState2);

    ifOthersKeepOtherPetsThenMan1KeepFishRule->addEffect(EffectPair(1.0f,keepFishEffect3));

    this->AllRules.push_back(ifOthersKeepOtherPetsThenMan1KeepFishRule);
    //----------------------------End Rule: if other 4 people keep cats,horse,dogs,birds, then man_1 keeps fish--------------------------------------

    //----------------------------Begin Rule: if other 4 people keep dogs,cats,fish,birds, then man_1 keeps horse--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not keep horse
    vector<ParamValue> keepCatsStateOwnerList5;
    keepCatsStateOwnerList5.push_back(man_3);
    State* keepCatsState5 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"cats", keepCatsStateOwnerList5, true,0);


    // effect1: man_1 keeps horse
    vector<ParamValue> keepHorseStateOwnerList3;
    keepHorseStateOwnerList3.push_back(man_1);
    State* keepHorseState3 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepHorseStateOwnerList3, true,0);
    Effect* keepHorseEffect3 = new Effect(keepHorseState3, OP_ASSIGN, "horse",true);

    // add rule:
    Rule* ifOthersKeepOtherPetsThenMan1KeepHorseRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->ruleName = "ifOthersKeepOtherPetsThenMan1KeepHorseRule";
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(ispeopleState1);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(ispeopleState2);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(ispeopleState3);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(ispeopleState4);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(ispeopleState5);

    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(isNotSameState1);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(isNotSameState2);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(isNotSameState3);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(isNotSameState4);

    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(keepDogsState2);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(keepCatsState5);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(keepFishState2);
    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addPrecondition(keepBirdsState2);

    ifOthersKeepOtherPetsThenMan1KeepHorseRule->addEffect(EffectPair(1.0f,keepHorseEffect3));

    this->AllRules.push_back(ifOthersKeepOtherPetsThenMan1KeepHorseRule);
    //----------------------------End Rule: if other 4 people keep dogs,cats,fish,birds, then man_1 keeps horse--------------------------------------

    //----------------------------Begin Rule: if other 4 people keep dogs,horse,fish,cats, then man_1 keeps birds--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other people do not keep birds
    vector<ParamValue> keepCatsStateOwnerList6;
    keepCatsStateOwnerList6.push_back(man_5);
    State* keepCatsState6 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"cats", keepCatsStateOwnerList6, true,0);

    // effect1: man_1 keeps birds
    vector<ParamValue> keepBirdsStateOwnerList4;
    keepBirdsStateOwnerList4.push_back(man_1);
    State* keepBirdsState4 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepBirdsStateOwnerList4, true,0);
    Effect* keepBirdsEffect4 = new Effect(keepBirdsState4, OP_ASSIGN, "birds",true);

    // add rule:
    Rule* ifOthersKeepOtherPetsThenMan1KeepBirdsRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->ruleName = "ifOthersKeepOtherPetsThenMan1KeepBirdsRule";
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(ispeopleState1);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(ispeopleState2);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(ispeopleState3);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(ispeopleState4);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(ispeopleState5);

    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(isNotSameState1);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(isNotSameState2);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(isNotSameState3);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(isNotSameState4);

    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(keepDogsState2);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(keepHorseState2);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(keepFishState2);
    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addPrecondition(keepCatsState6);

    ifOthersKeepOtherPetsThenMan1KeepBirdsRule->addEffect(EffectPair(1.0f,keepBirdsEffect4));

    this->AllRules.push_back(ifOthersKeepOtherPetsThenMan1KeepBirdsRule);
    //----------------------------End Rule: if other 4 people keep dogs,horse,fish,cats, then man_1 keeps birds--------------------------------------


    //----------------------------Begin Rule: if other 4 people drink other drinks, then man_1 drinks tea--------------------------------------
    // define variables
    ParamValue drink_man_1 = str_var[0];

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other peopledrink other drinks
    vector<ParamValue> drinkCoffeeStateOwnerList2;
    drinkCoffeeStateOwnerList2.push_back(man_2);
    State* drinkCoffeeState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"coffee", drinkCoffeeStateOwnerList2, true, 0);

    vector<ParamValue> drinkMilkStateOwnerList2;
    drinkMilkStateOwnerList2.push_back(man_3);
    State* drinkMilkState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"milk", drinkMilkStateOwnerList2, true, 0);

    vector<ParamValue> drinkBeerStateOwnerList2;
    drinkBeerStateOwnerList2.push_back(man_4);
    State* drinkBeerState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"beer", drinkBeerStateOwnerList2, true, 0);

    vector<ParamValue> drinkWaterStateOwnerList2;
    drinkWaterStateOwnerList2.push_back(man_5);
    State* drinkWaterState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"water", drinkWaterStateOwnerList2, true, 0);

    // effect1: man_1 drinks tea
    vector<ParamValue> drinkTeaStateOwnerList2;
    drinkTeaStateOwnerList2.push_back(man_1);
    State* drinkTeaState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO , drink_man_1, drinkTeaStateOwnerList2, true, 0);
    Effect* drinkTeaEffect2 = new Effect(drinkTeaState2, OP_ASSIGN, "tea",true);

    // add rule:
    Rule* ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->ruleName = "ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule";
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(ispeopleState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(ispeopleState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(ispeopleState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(ispeopleState4);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(ispeopleState5);

    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(isNotSameState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(isNotSameState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(isNotSameState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(isNotSameState4);

    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(drinkCoffeeState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(drinkMilkState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(drinkBeerState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addPrecondition(drinkWaterState2);

    ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule->addEffect(EffectPair(1.0f,drinkTeaEffect2));

    this->AllRules.push_back(ifOthersDrinkOtherDrinksThenMan1DrinkTeaRule);
    //----------------------------End Rule:  if other 4 people drink other drinks, then man_1 drinks tea--------------------------------------------------

    //----------------------------Begin Rule: if other 4 people drink other drinks, then man_1 drinks coffee--------------------------------------
    // define variables:
    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other peopledrink other drinks
    vector<ParamValue> drinkTeaStateOwnerList3;
    drinkTeaStateOwnerList3.push_back(man_2);
    State* drinkTeaState3 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"tea", drinkTeaStateOwnerList3, true, 0);


    // effect1: man_1 drinks tea
    vector<ParamValue> drinkCoffeeStateOwnerList3;
    drinkCoffeeStateOwnerList3.push_back(man_1);
    State* drinkCoffeeState3 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO , drink_man_1, drinkCoffeeStateOwnerList3, true, 0);
    Effect* drinkCoffeeEffect3 = new Effect(drinkCoffeeState3, OP_ASSIGN, "coffee",true);


    // add rule:
    Rule* ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->ruleName = "ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule";
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(ispeopleState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(ispeopleState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(ispeopleState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(ispeopleState4);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(ispeopleState5);

    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(isNotSameState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(isNotSameState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(isNotSameState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(isNotSameState4);

    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(drinkTeaState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(drinkMilkState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(drinkBeerState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addPrecondition(drinkWaterState2);

    ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule->addEffect(EffectPair(1.0f,drinkCoffeeEffect3));

    this->AllRules.push_back(ifOthersDrinkOtherDrinksThenMan1DrinkCoffeeRule);
    //----------------------------End Rule: if other 4 people drink other drinks, then man_1 drinks coffee--------------------------------------------------

    //----------------------------Begin Rule: if other 4 people drink other drinks, then man_1 drinks milk--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other peopledrink other drinks

    vector<ParamValue> drinkTeaStateOwnerList4;
    drinkTeaStateOwnerList4.push_back(man_3);
    State* drinkTeaState4 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"tea", drinkTeaStateOwnerList4, true, 0);

    // effect1: man_1 drinks tea
    vector<ParamValue> drinkMilkStateOwnerList3;
    drinkMilkStateOwnerList3.push_back(man_1);
    State* drinkMilkState3 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO , drink_man_1, drinkMilkStateOwnerList3, true, 0);
    Effect* drinkMilkEffect3 = new Effect(drinkMilkState3, OP_ASSIGN, "milk",true);

    // add rule:
    Rule* ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->ruleName = "ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule";
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(ispeopleState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(ispeopleState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(ispeopleState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(ispeopleState4);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(ispeopleState5);

    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(isNotSameState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(isNotSameState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(isNotSameState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(isNotSameState4);

    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(drinkCoffeeState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(drinkTeaState4);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(drinkBeerState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addPrecondition(drinkWaterState2);

    ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule->addEffect(EffectPair(1.0f,drinkMilkEffect3));

    this->AllRules.push_back(ifOthersDrinkOtherDrinksThenMan1DrinkMilkRule);
    //----------------------------End Rule:  if other 4 people drink other drinks, then man_1 drinks milk--------------------------------------------------

    //----------------------------Begin Rule: if other 4 people drink other drinks, then man_1 drinks beer--------------------------------------
    // define variables:

    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other peopledrink other drinks
    vector<ParamValue> drinkTeaStateOwnerList5;
    drinkTeaStateOwnerList5.push_back(man_4);
    State* drinkTeaState5 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"tea", drinkTeaStateOwnerList5, true, 0);

    // effect1: man_1 drinks beer
    vector<ParamValue> drinkBeerStateOwnerList3;
    drinkBeerStateOwnerList3.push_back(man_1);
    State* drinkBeerState3 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO , drink_man_1, drinkBeerStateOwnerList3, true, 0);
    Effect* drinkBeerEffect3 = new Effect(drinkBeerState3, OP_ASSIGN, "beer",true);


    // add rule:
    Rule* ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->ruleName = "ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule";
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(ispeopleState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(ispeopleState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(ispeopleState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(ispeopleState4);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(ispeopleState5);

    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(isNotSameState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(isNotSameState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(isNotSameState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(isNotSameState4);

    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(drinkCoffeeState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(drinkMilkState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(drinkTeaState5);
    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addPrecondition(drinkWaterState2);

    ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule->addEffect(EffectPair(1.0f,drinkBeerEffect3));

    this->AllRules.push_back(ifOthersDrinkOtherDrinksThenMan1DrinkBeerRule);
    //----------------------------End Rule:  if other 4 people drink other drinks, then man_1 drinks beer--------------------------------------------------

    //----------------------------Begin Rule: if other 4 people drink other drinks, then man_1 drinks water--------------------------------------
    // define variables:
    // precondition 1 -5 : man_1-5 are people

    // precondition 6-9 : other peopledrink other drinks
    vector<ParamValue> drinkTeaStateOwnerList6;
    drinkTeaStateOwnerList6.push_back(man_5);
    State* drinkTeaState6 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"tea", drinkTeaStateOwnerList6, true, 0);

    // effect1: man_1 drinks tea
    vector<ParamValue> drinkWaterStateOwnerList3;
    drinkWaterStateOwnerList3.push_back(man_1);
    State* drinkWaterState3 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO , drink_man_1, drinkWaterStateOwnerList3, true, 0);
    Effect* drinkWaterEffect3 = new Effect(drinkWaterState3, OP_ASSIGN, "water",true);

    // add rule:
    Rule* ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->ruleName = "ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule";
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(ispeopleState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(ispeopleState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(ispeopleState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(ispeopleState4);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(ispeopleState5);

    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(isNotSameState1);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(isNotSameState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(isNotSameState3);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(isNotSameState4);

    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(drinkCoffeeState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(drinkMilkState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(drinkBeerState2);
    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addPrecondition(drinkTeaState6);

    ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule->addEffect(EffectPair(1.0f,drinkWaterEffect3));

    this->AllRules.push_back(ifOthersDrinkOtherDrinksThenMan1DrinkWaterRule);
    //----------------------------End Rule:  if other 4 people drink other drinks, then man_1 drinks water--------------------------------------------------



}



