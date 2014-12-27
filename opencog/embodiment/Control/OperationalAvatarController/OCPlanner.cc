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
#include <opencog/embodiment/Control/PerceptionActionInterface/AvatarAction.h>
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
#include <sys/time.h>
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

bool OCPlanner::isActionChangeSPaceMap(AvatarAction* action)
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
    found = false;
    if ( (curSNode != 0) && (curSNode->backwardRuleNode != 0))
    {
        // find this state in temporaryStateNodes
        if (findStateInTempStates(goalState,forwardRuleNode,satstateNode,ifCheckSameRuleNode))
        {
            //  check if this state has beed satisfied by the previous state nodes
            float satisfiedDegree;
            bool unknown;
            found = true;

            State* vState = satstateNode->state;
            if (vState->isSatisfied(goalState,satisfiedDegree,unknown))
            {
                return true;
            }
            else if (unknown)
            {
                found = false;
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
        bool unknown;

        State* vState = satstateNode->state;
        if (vState->isSatisfied(goalState,satisfiedDegree, unknown))
        {
            return true;
        }
        else if (unknown)
        {
            found = false;
            return false;
        }
    }
    else
    {
        // cannot find this state in temporaryStateNodes
        found = false;
        return false;
    }

    return false;

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
bool OCPlanner::checkIsGoalAchievedInRealTime(State& oneGoal, float& satisfiedDegree, bool &isUnknownValue, bool &unknown, State* original_state)
{
    unknown = false;

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
            isUnknownValue = true;
            satisfiedDegree = 0.0f;
            return false;
        }

        // the real time inquery interface only return STATE_EQUAL_TO type
        State curState(oneGoal.name(),oneGoal.getActionParamType(),STATE_EQUAL_TO,inqueryValue,oneGoal.stateOwnerList);
        curState.permanent = oneGoal.permanent;

        return curState.isSatisfied(oneGoal, satisfiedDegree,unknown, original_state);
    }
    else // it doesn't need real time calculation, then we search for its latest evaluation link value in the atomspace
    {

        bool is_true;
        ParamValue value = Inquery::getParamValueFromAtomspace(oneGoal,is_true);
        if (value == UNDEFINED_VALUE)
        {
            // there is not information in Atomspace about this state. We don't know the state, so just return false
            isUnknownValue = true;
            satisfiedDegree = 0.0f;
            return false;
        }

        State curState(oneGoal.name(),oneGoal.getActionParamType(),oneGoal.stateType,value,oneGoal.stateOwnerList);
        curState.permanent = oneGoal.permanent;

        if ((!is_true) && (oneGoal.stateType != STATE_NOT_EQUAL_TO))
            curState.stateType = STATE_NOT_EQUAL_TO;

        return curState.isSatisfied(oneGoal, satisfiedDegree,unknown, original_state);

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

    // test :
    loadFacts(knownStates);

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
    satisfiedGoalStateNodes.clear();
    OCPlanner::goalRuleNode.backwardLinks.clear();
    curStateNode = 0;

    removedHypotheticalLinkCount = 0;

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

        if (newStateNode->state->permanent)
            addHypotheticalLinkForStateNode(newStateNode);

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
        bool isUnknownValue, known;

        if (checkIfThisGoalIsSatisfiedByTempStates(*(newStateNode->state), found, knownStateNode, 0,false))
        {
            satisfiedGoalStateNodes.push_back(newStateNode);
            temporaryStateNodes.push_front(newStateNode);
        }

        if (found)
        {
            unsatisfiedStateNodes.push_back(newStateNode);
        }
        else if (checkIsGoalAchievedInRealTime(*(newStateNode->state), satisfiedDegree, isUnknownValue,known))
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

    timeval t1;
    gettimeofday(&t1, NULL);

    long startTime = t1.tv_sec*1000 + t1.tv_usec/1000;

    while(unsatisfiedStateNodes.size() != 0)
    {
        Rule* selectedRule = 0;
        RuleNode* selectedRuleNode = 0;

        curtimeStamp ++;

        tryStepNum ++;
        if (tryStepNum > 200)
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
                Rule* r = (((multimap<float,Rule*>)(it->second)).begin())->second;

                // check if this rule has any NoCoexistenceRule already been used in previous steps, if yes, should not use this rule
                if (checkHasNoCoexistenceRuleInPlanningNetWork(r))
                    continue;


                // check if it's negative or positive for this goal:
                bool isNegativeGoal, isDiffStateOwnerType, preconImpossible, willAddCirle, contradictoryOtherGoal, needRollback;
                int negativeNum,satisfiedPreconNum;
                checkRuleFitnessRoughly(r,curStateNode,satisfiedPreconNum,negativeNum,isNegativeGoal,isDiffStateOwnerType,preconImpossible,willAddCirle,contradictoryOtherGoal,needRollback);

                if (isNegativeGoal || isDiffStateOwnerType || willAddCirle || contradictoryOtherGoal || preconImpossible)
                    continue;

                // this rule is positive for this goal, apply it.
                selectedRule = r;
                curStateNode->ruleHistory.push_back(selectedRule);
                curStateNode->candidateRules.push_back(pair<float, Rule*>(0.0f,selectedRule));
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

                    // check if this rule has any NoCoexistenceRule already been used in previous steps, if yes, should not use this rule
                    if (checkHasNoCoexistenceRuleInPlanningNetWork(r))
                        continue;

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
                    bool isNegativeGoal, isDiffStateOwnerType, preconImpossible,willAddCirle,contradictoryOtherGoal, needRollback;
                    int negativeNum,satisfiedPreconNum;
                    checkRuleFitnessRoughly(r,curStateNode,satisfiedPreconNum,negativeNum,isNegativeGoal,isDiffStateOwnerType,preconImpossible,willAddCirle,contradictoryOtherGoal,needRollback);

                    if (needRollback)
                    {
                        std::cout << "Rule: " << r->ruleName <<" implys that this subgoal could never be achieved. " << std::endl;
                        curStateNode->candidateRules.clear();
                        break;
                    }

                    //  its effect will negative this current selected goal state, or it has any unsatisfied precondition which is impossible to achieve,
                    //  then it should not add it into candidate rules
                    if (isNegativeGoal || preconImpossible || isDiffStateOwnerType || contradictoryOtherGoal || willAddCirle)
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

            }

        }
        else //  we have  tried to achieve this state node before,which suggests we have found all the candidate rules
        {

            // check if there is any rule left not been tried in the candidate rules
            if (curStateNode->candidateRules.size() != 0)
            {
                selectedRule = (curStateNode->candidateRules.front()).second;
                curStateNode->ruleHistory.push_back((curStateNode->candidateRules.front()).second);
            }
            else
            {
                cout<< std::endl << "Debug planning step " << tryStepNum <<" : current subgoal has not any candidate rules left to try. Roll back! " << std::endl;
                // we have tried all the candidate rules, still cannot achieve this state, which means this state is impossible to be achieved here
                // so go back to the its forward rule which produce this state to check if we can apply another bindings to the same rule or we shoud try another rule

                RuleNode* forwardRuleNode = curStateNode->forwardRuleNode;

                if (forwardRuleNode == (&OCPlanner::goalRuleNode))
                {
                    // oh, this state node is already the goal state node, and it's impossible to achieve, return planning fail
                    return "";
                }

                if (forwardRuleNode->ParamCandidates.size() == 0)
                {
                    cout<< std::endl << "Debug planning step " << tryStepNum <<" : its forward RuleNode: " << forwardRuleNode->originalRule->ruleName
                        << " has tried all the candidate bindings. Roll back again! " << std::endl;

                    if (forwardRuleNode->originalRule->isReversibleRule && forwardRuleNode->originalRule->action->getType().getCode() == DO_NOTHING_CODE)
                    {
                        cout<<"This fail also imply that its forward state :";
                        outputStateInfo(forwardRuleNode->forAchieveThisSubgoal->state, true);
                        cout<< " is impossile to be achieved by other rules. So clean up other candidate rules if any. "<< std::endl;
                        forwardRuleNode->forAchieveThisSubgoal->candidateRules.clear();
                    }

                    // so it means this rule doesn't work, we have to go back to its forward state node
                    deleteRuleNodeRecursively(forwardRuleNode);

                    continue; // continue to next big loop for next unsatisfied subgoal

                }
                else
                {
                    // There are still Candidate bindings for this rule node to try.
                    // just try a new binding to ground the effect states
                    selectedRuleNode = forwardRuleNode;
                    forwardRuleNode->currentBindingsViaSelecting =  forwardRuleNode->ParamCandidates.front();
                    forwardRuleNode->ParamCandidates.erase(forwardRuleNode->ParamCandidates.begin());

                    forwardRuleNode->updateCurrentAllBindings();

                    curStateNode = selectedRuleNode->forAchieveThisSubgoal;

                    cout<< std::endl << "Debug planning step " << tryStepNum <<" : reselected subgoal: ";
                    outputStateInfo(curStateNode->state, true);
                    cout<< std::endl;

                    // clear up the previous context of this rule node without delete this rule node
                    deleteRuleNodeRecursively(selectedRuleNode,curStateNode,false,false);

                    selectedRuleNode->backwardLinks.clear();
                    selectedRuleNode->forwardLinks.clear();

                    selectedRuleNode->forwardLinks.push_back(curStateNode);
                    curStateNode->backwardRuleNode = selectedRuleNode;


//                    // check all its backward state nodes
//                    // delete them recursively
//                    vector<StateNode*>::iterator backwardStateIt;
//                    for (backwardStateIt = forwardRuleNode->backwardLinks.begin(); backwardStateIt != forwardRuleNode->backwardLinks.end(); ++ backwardStateIt)
//                    {
//                        StateNode* curSNode = (StateNode*)(*backwardStateIt);

//                        deleteStateNodeInTemporaryList(curSNode);

//                        if (curSNode->hypotheticalLink != Handle::UNDEFINED)
//                        {
//                           atomSpace->removeAtom(curSNode->hypotheticalLink);
//                           curSNode->hypotheticalLink = Handle::UNDEFINED;
//                        }

//                         if (curSNode->backwardRuleNode != 0)
//                             deleteRuleNodeRecursively(curSNode->backwardRuleNode, curSNode);
//                         else
//                             delete curSNode;

//                    }



//                    // check which states of the preconditions of this forwardRuleNode have been sovled , which still remand unsloved.
//                    vector<StateNode*>::iterator preconItor;
//                    map<State*,StateNode*> solvedStateNodes; // all the state nodes in forwardRuleNode's preditions that have been solved by previous planning steps
//                    for (preconItor = forwardRuleNode->backwardLinks.begin(); preconItor != forwardRuleNode->backwardLinks.end(); ++ preconItor)
//                    {
//                        // skip the current state node
//                        if ((*preconItor) == curStateNode)
//                            continue;

//                        // when this state node has already been tried a backward rule to solve it, or this state node is not in the unsatisfiedStateNodes
//                        // it means it's solved
//                        if (((*preconItor)->backwardRuleNode != 0) || (! findInStateNodeList(unsatisfiedStateNodes,*preconItor)))
//                        {
//                            // so put it in the solvedStateNodes map
//                            StateNode* sn = (StateNode*)(*preconItor);
//                            solvedStateNodes.insert(pair<State*,StateNode*>(sn->forwardEffectState , sn ) );
//                        }
//                    }

//                    // If all the effect states of this  forwardRuleNode remand unsolved,
//                    // which suggests try another random group of candidate bindings will not affect the planning step that has been conducted
//                    // so just try any other bindings for this rule node
//                    if (solvedStateNodes.size() == 0)
//                    {
//                       // just try a new binding to ground the effect states
//                        forwardRuleNode->currentBindingsViaSelecting =  forwardRuleNode->ParamCandidates.front();
//                        forwardRuleNode->ParamCandidates.erase(forwardRuleNode->ParamCandidates.begin());

//                        forwardRuleNode->updateCurrentAllBindings();
//                        recordOrginalParamValuesAfterGroundARule(forwardRuleNode);

//                        // rebind all the precondition state nodes in the forward rule
//                        for (preconItor = forwardRuleNode->backwardLinks.begin(); preconItor != forwardRuleNode->backwardLinks.end(); ++ preconItor)
//                        {
//                            reBindStateNode((*preconItor),forwardRuleNode->currentAllBindings);
//                        }

//                        continue; // continue with the big while loop to check the next subgoal in unsatisfiedStateNodes
//                    }
//                    else // some of the predition states of this forwardRuleNode has been solved, so we cannot simply replace the current bindings with another random bindings
//                    {


//                        // try to find a group of bindings from the candidates, that won't affect the solved states
//                        // if cannot find any , try to find a group of candidate bindings will affect as few as possible solved states
//                        // to record which solved states will be affected if use this candidate binding
//                        // a vector in the order of vector<ParamGroundedMapInARule> ParamCandidates
//                        // each element - vector<StateNode*>  is the list of all the state nodes that a ParamGroundedMapInARule in vector<ParamGroundedMapInARule> ParamCandidates
//                        vector< vector<StateNode*> >  willBeAffecteds;
//                        map<State*,StateNode*>::iterator solvedItor;
//                        vector<ParamGroundedMapInARule>::iterator candidateIt;
//                        vector<ParamGroundedMapInARule>::iterator bestBindingIt = forwardRuleNode->ParamCandidates.end(); // the best Binding with the least affect
//                        int affectLeastStateNum = 999999; // the least number of state nodes the candidate bindings will affect
//                        int index = -1; // The index of the candidate binding in ParamCandidates which causes the least affect on the solved state nodes
//                        int i = 0;
//                        for (candidateIt = forwardRuleNode->ParamCandidates.begin(); candidateIt != forwardRuleNode->ParamCandidates.end(); ++ candidateIt, ++i)
//                        {
//                            vector<StateNode*> oneAffectRecord;
//                            list<UngroundedVariablesInAState>::iterator ungroundVarIt;
//                            for (ungroundVarIt = forwardRuleNode->curUngroundedVariables.begin(); ungroundVarIt != forwardRuleNode->curUngroundedVariables.end(); ++ ungroundVarIt)
//                            {
//                                UngroundedVariablesInAState& ungroundVarInAState = (UngroundedVariablesInAState&)(*ungroundVarIt);

//                                // Try to find this state in the solvedStateNodes.
//                                solvedItor = solvedStateNodes.find(ungroundVarInAState.state);
//                                if (solvedItor == solvedStateNodes.end())
//                                    continue;

//                                // Try to find the ungrounded variable names in this candidate binding
//                                ParamGroundedMapInARule& bindings = (ParamGroundedMapInARule&)(*candidateIt);
//                                ParamGroundedMapInARule::iterator bindIt;
//                                for (bindIt = bindings.begin(); bindIt !=  bindings.end(); ++ bindIt)
//                                {
//                                    string& varName = (string&)(bindIt->first);

//                                    // try to find this find this variable name() in the ungrounded variable records
//                                    if (ungroundVarInAState.vars.find(varName) != ungroundVarInAState.vars.end())
//                                    {
//                                        oneAffectRecord.push_back((StateNode*)(solvedItor->second));
//                                    }

//                                }

//                            }

//                            willBeAffecteds.push_back(oneAffectRecord);

//                            if (oneAffectRecord.size() < (std::size_t)affectLeastStateNum)
//                            {
//                                index = i;
//                                affectLeastStateNum = oneAffectRecord.size();
//                                bestBindingIt = candidateIt;
//                            }

//                        }

//                        OC_ASSERT( (index != -1),
//                                  "OCPlanner::doPlanning: Failed to find a group of variable bindings from the candidates with least affect to the solved state nodes!\n");

//                        // apply the new bindings of least affect to the solved states
//                        forwardRuleNode->currentBindingsViaSelecting =  *bestBindingIt;
//                        forwardRuleNode->ParamCandidates.erase(bestBindingIt);
//                        forwardRuleNode->updateCurrentAllBindings();
//                        recordOrginalParamValuesAfterGroundARule(forwardRuleNode);

//                        // have to rebind the affected state nodes and delete the affected effect states' branches
//                        vector<StateNode*>::iterator affectedStateNodeIt;
//                        for ( affectedStateNodeIt =  (willBeAffecteds[index]).begin(); affectedStateNodeIt !=  (willBeAffecteds[index]).end(); ++ affectedStateNodeIt)
//                        {
//                            StateNode* affectedStateNode = (StateNode* )(*affectedStateNodeIt);

//                            deleteStateNodeInTemporaryList(affectedStateNode);

//                            reBindStateNode(affectedStateNode,forwardRuleNode->currentAllBindings);

//                            // now affectedStateNode is a new node after being rebinded
//                            // only when there is other rule node used it as a predition , we need to put it into the unsatisfiedStateNodes list
//                            if (affectedStateNode->forwardRuleNode != 0)
//                                unsatisfiedStateNodes.push_back(affectedStateNode);

//                            // the affectedStateNode has been rebinded, should not delete it as well
//                            deleteRuleNodeRecursively(affectedStateNode->backwardRuleNode,affectedStateNode, false);

//                        }

//                        continue; // continue with the big while loop to check the next subgoal in unsatisfiedStateNodes

//                    }
                }

            }
        }

        RuleNode* ruleNode;

        // Till now have selected one unsatisfied state and the rule to applied to try to do one step backward chaining to satisfy it
        // out put selected rule debug info:
        if (selectedRuleNode == 0)
        {
            cout<<"Debug planning step " << tryStepNum <<": Selected rule :"<< selectedRule->ruleName << std::endl;

            curStateNode->candidateRules.pop_front();

            // create a new RuleNode to apply this selected rule
            ruleNode = new RuleNode(selectedRule, curStateNode);
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
            if (! groundARuleNodeBySelectingNonNumericValues(ruleNode))
            {
                // fail to ground all non numeric variables, roll back to previous step
                cout<< "GroundARuleNodeBySelectingNonNumericValues failed! This rule doesn't work!"<< std::endl;

                if ((curStateNode->forwardRuleNode != (&OCPlanner::goalRuleNode)) && curStateNode->forwardRuleNode->originalRule->isReversibleRule && curStateNode->forwardRuleNode->originalRule->action->getType().getCode() == DO_NOTHING_CODE)
                {
                    cout<<"This fail also imply that its forward state :";
                    outputStateInfo(curStateNode->forwardRuleNode->forAchieveThisSubgoal->state, true);
                    cout<< " is impossile to be achieved by other rules. So clean up other candidate rules if any. "<< std::endl;
                    curStateNode->forwardRuleNode->forAchieveThisSubgoal->candidateRules.clear();
                }

                cleanUpContextBeforeRollBackToPreviousStep();
                continue;

            }

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
                cout << "SelectValueForGroundingNumericState failed! This rule doesn't work!"<< std::endl;
                if ((curStateNode->forwardRuleNode != (&OCPlanner::goalRuleNode)) && curStateNode->forwardRuleNode->originalRule->isReversibleRule && curStateNode->forwardRuleNode->originalRule->action->getType().getCode() == DO_NOTHING_CODE)
                {
                    cout<<"This fail also imply that its forward state :";
                    outputStateInfo(curStateNode->forwardRuleNode->forAchieveThisSubgoal->state, true);
                    cout<< " is impossile to be achieved by other rules. So clean up other candidate rules if any. "<< std::endl;
                    curStateNode->forwardRuleNode->forAchieveThisSubgoal->candidateRules.clear();
                }
                cleanUpContextBeforeRollBackToPreviousStep();
                continue;
            }
        }
        else
        {
            ruleNode = selectedRuleNode;
            allRuleNodeInThisPlan.insert(allRuleNodeInThisPlan.begin(),ruleNode);
            cout<<"Debug planning step " << tryStepNum <<": Selected rule again:"<< selectedRuleNode->originalRule->ruleName <<" : Trying another group of bindings" << std::endl;
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

            if (newStateNode->state->permanent)
                addHypotheticalLinkForStateNode(newStateNode);

            list<StateNode*>::iterator unsait;

            // check if there are any unsatisfied nodes in previous steps will be satisfied by this effect
            for (unsait = unsatisfiedStateNodes.begin(); unsait != unsatisfiedStateNodes.end(); )
            {
                if (effState->isSameState( *((StateNode*)(*unsait))->state ))
                {
                    // check if this effect can satisfy this unsatisfiedState
                    float satDegree;
                    bool unknown;
                    StateNode* unsatStateNode = (StateNode*)(*unsait);
                    if (effState->isSatisfied(*(unsatStateNode->state ),satDegree, unknown))
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
                    bool unknown;
                    StateNode* satStateNode = (StateNode*)(*sait);
                    if (! effState->isSatisfied(*(satStateNode->state ),satDegree, unknown))
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
                    bool unknown;

                    if (! effState->isSatisfied(*(goalNode->state) ,satDegree, unknown))
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
                bool isUnknownValue, unknown;
                // cannot find this state in the temporaryStateNodes list, need to check it in real time
                // check real time
                if (! checkIsGoalAchievedInRealTime(*groundPs,satisfiedDegree,isUnknownValue,unknown))
                {
                    isSat = false;

                    if (groundPs->permanent)
                        addHypotheticalLinkForStateNode(newStateNode);
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

    timeval t2;
    gettimeofday(&t2, NULL);

    long endTime = t2.tv_sec*1000 + t2.tv_usec/1000;

    // generate the action series according to the planning network we have constructed in this planning process
    planID = "";

    std::cout<<std::endl<<"OCPlanner::Planning success! Total steps = "<< tryStepNum <<", Cost time = "<< endTime - startTime << "ms" << std::endl;

    // sort the list of rule node
    sort(allRuleNodeInThisPlan.begin(), allRuleNodeInThisPlan.end(),compareRuleNodeDepth );

    // and then encapsule the action for each step and send to PAI
    vector<RuleNode*>::iterator planRuleNodeIt, lastStepIt;
    int stepNum = 1;
    SpaceServer::SpaceMap* backwardStepMap = curMap->clone();
    bool notRealActionRequired = true;

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
        AvatarAction* originalAction = r->originalRule->action;
        if (originalAction->getType().getCode() == DO_NOTHING_CODE)
            continue;

        notRealActionRequired = false;

        if (planID == "")
        {
            planID = oac->getPAI().createActionPlan();
        }

        // ground the parameter according to the current bindings
        AvatarAction action(originalAction->getType());
        list<ActionParameter>::const_iterator paraIt;
        const list<ActionParameter>& params = originalAction->getParameters();

        std::cout<<std::endl<<"Step No."<< stepNum << ": "<< originalAction->getName();

        // for navigation actions, need to call path finder to create every step of it
        if ((originalAction->getType().getCode() == WALK_CODE) || (originalAction->getType().getCode() == MOVE_TO_OBJ_CODE) )
        {
            spatial::BlockVector startPos,targetPos;

            bool includesLastStep;

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
                includesLastStep = true;

                std::cout<< ActionParameter::ParamValueToString(v1)<<std::endl;
            }
            else
            {
                Entity entity1 = boost::get<Entity>(value);
                targetPos = backwardStepMap->getObjectLocation(entity1.id);
                includesLastStep = false;

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

            opencog::world::PAIWorldWrapper::createNavigationPlanAction(oac->getPAI(),*backwardStepMap,startPos,targetPos,planID,includesLastStep);

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

    cleanUpEverythingAfterPlanning();

    if (notRealActionRequired)
    {
        std::cout << "It's able to achieve the goal without doing any action! Not need a plan!" << std::endl;
    }

    return planID;
}

void OCPlanner::cleanUpEverythingAfterPlanning()
{
    // delete all rule nodes
    vector<RuleNode*>::iterator planRuleNodeIt;
    for (planRuleNodeIt = allRuleNodeInThisPlan.begin(); planRuleNodeIt != allRuleNodeInThisPlan.end(); ++ planRuleNodeIt)
    {
         RuleNode* rn = *planRuleNodeIt;
         if (rn)
             delete rn;
    }

    // delete all the state nodes, remove their hypotheticalLinks from Atomspace at the same time if any

    vector<StateNode*>::iterator sit;
    for (sit = satisfiedGoalStateNodes.begin(); sit != satisfiedGoalStateNodes.end(); ++ sit)
    {
        StateNode* sn = *sit;
        if (sn)
        {
            if (sn->hypotheticalLink != Handle::UNDEFINED)
            {
//                cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                    << atomSpace->atomAsString(sn->hypotheticalLink).c_str() << std::endl;
                atomSpace->removeAtom(sn->hypotheticalLink);
            }

            delete sn;
        }
    }

    list<StateNode*>::iterator unsit;
    for (unsit = unsatisfiedStateNodes.begin(); unsit != unsatisfiedStateNodes.end(); ++ unsit)
    {
        StateNode* sn = *unsit;
        if (sn)
        {
            if (sn->hypotheticalLink != Handle::UNDEFINED)
            {
//                cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                    << atomSpace->atomAsString(sn->hypotheticalLink).c_str() << std::endl;
                atomSpace->removeAtom(sn->hypotheticalLink);
            }

            delete sn;
        }
    }

    list<StateNode*>::iterator tsit;
    // remove duplicate elements if any
    set<StateNode*>::iterator setSit;
    set<StateNode*> setTempStateNodes;
    for (tsit = temporaryStateNodes.begin(); tsit != temporaryStateNodes.end(); ++ tsit)
    {
        if (setTempStateNodes.find(*tsit) == setTempStateNodes.end())
            setTempStateNodes.insert(*tsit);
        else
        {
            cout <<std::endl << "Warning: Found duplicate elements in temporaryStateNodes:";
            outputStateInfo(((StateNode*)(*tsit))->state,true);
            cout << std::endl;
        }
    }

    for (setSit = setTempStateNodes.begin(); setSit != setTempStateNodes.end(); ++ setSit)
    {
        StateNode* sn = *setSit;
        if (sn)
        {
            if (sn->hypotheticalLink != Handle::UNDEFINED)
            {
//                cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                    << atomSpace->atomAsString(sn->hypotheticalLink).c_str() << std::endl;
                atomSpace->removeAtom(sn->hypotheticalLink);
            }

            delete sn;
        }
    }

    list<StateNode*>::iterator stsit;
    for (stsit = startStateNodes.begin(); stsit != startStateNodes.end(); ++ stsit)
    {
        StateNode* sn = *stsit;
        if (sn)
        {
            if (sn->hypotheticalLink != Handle::UNDEFINED)
            {
//                cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                    << atomSpace->atomAsString(sn->hypotheticalLink).c_str() << std::endl;
                atomSpace->removeAtom(sn->hypotheticalLink);
            }

            delete sn;
        }
    }
}

void OCPlanner::cleanUpContextBeforeRollBackToPreviousStep()
{
    allRuleNodeInThisPlan.erase(allRuleNodeInThisPlan.begin());
    delete(curStateNode->backwardRuleNode);
    curStateNode->backwardRuleNode = 0;
}

int OCPlanner::getHardnessScoreOfPrecon(StateNode* stateNode)
{
    map<string,multimap<float,Rule*> >::iterator it = ruleEffectIndexes.find(stateNode->state->name());

    if ( it == ruleEffectIndexes.end())
    {
        stateNode->hardnessScore = 999999;
        return 999999;
    }

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
    {
        stateNode->hardnessScore = 999999;
        return 999999;
    }

    int hardnessScore = 1000;

    if (directHelp)
        hardnessScore = 0;

    hardnessScore += numOfUngroundedVars * 10;

    if (stateNode->state->isNumbericState())
        hardnessScore += 2000;

    stateNode->hardnessScore = hardnessScore;
    return hardnessScore;
}


int OCPlanner::checkPreconditionFitness(RuleNode* ruleNode, StateNode* fowardState, bool &preconImpossible, bool &willCauseCirleNetWork,
                                        bool &hasDirectHelpRule, bool &contradictoryOtherGoal, bool& isRecursivePrecon0Sat, bool& isRecursivePrecon1Sat, Rule* orginalRule)
{
    int satisfiedPreconNum = 0;
    preconImpossible = false;
    willCauseCirleNetWork = false;
    hasDirectHelpRule = false;
    contradictoryOtherGoal = false;
    isRecursivePrecon0Sat = true;
    isRecursivePrecon1Sat = true;

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
            else if (groundPs->permanent)
            {
                // this goal has been found in TempStates but not satisfied, and this goal is permanent (change not be changed), which means this goal will never be satified
                delete groundPs;
                preconImpossible = true;
                return -999;
            }

        }
        else
        {
            // cannot find this state in the temporaryStateNodes list, need to check it in real time
            // check real time
            float satisfiedDegree;
            bool isUnknownValue, unknown;
            if ( checkIsGoalAchievedInRealTime(*groundPs,satisfiedDegree,isUnknownValue, unknown))
            {
                ++ satisfiedPreconNum;
                satisfied = true;
            }

        }

        if (! satisfied)
        {
            if (ruleNode->originalRule->IsRecursiveRule)
            {
                if (itpre == precondList.begin())
                    isRecursivePrecon0Sat = false;
                else
                    isRecursivePrecon1Sat = false;
            }

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
                    if ((! groundPs->permanent) )
                    {
                        // a little hack here, only check willCauseCirleNetWork for the States that is not permanent
                        if (groundPs->stateVariable->getValue() == tempStateNode->state->stateVariable->getValue())
                        {
                            delete groundPs;
                            willCauseCirleNetWork = true;
                            return -999;
                        }
                    }
                    else
                    {
                        // check if permanent temporaryStateNodes negative this subgoal , then subgoal is impossible to be achieved
                        float sd;
                        bool unknown;
                        if (! tempStateNode->state->isSatisfied(*groundPs, sd, unknown))
                        {
                            if (! unknown)
                            {
                                delete groundPs;
                                preconImpossible = true;
                                return -999;
                            }
                        }

                    }

                }

                // todo: recursively check if the future backward branch to achieve this subgoal contains any precondition impossibilities

            }

            // check if unsatisifed subgoals will be contradictory by this effect
            for (sait = unsatisfiedStateNodes.begin(); sait != unsatisfiedStateNodes.end(); ++ sait)
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

    //            // only check the StateNode which is more backward than the input fowardState
    //            // if this effect state node has a lower depth than this state node, it means the effect will happen after this state node
    //            // so this state node will not be affected by this effect.
    //            if ((*fowardState) < (*tempStateNode))
    //            {
    //                // need to check if it's the same level with fowardState,
    //                if ((fowardState->forwardRuleNode != tempStateNode->forwardRuleNode))
    //                    continue;
    //            }

                if (tempStateNode->state->isSameState(*groundPs))
                {
                    // check if this effect unsatisfy this state
                    float satDegree;
                    bool unknown;
                    if (! groundPs->isSatisfied(*(tempStateNode->state ),satDegree,unknown))
                    {
                        if (! unknown)
                        {
                            delete groundPs;
                            contradictoryOtherGoal = true;
                            return -999;
                        }
                    }
                    else
                        continue;
                }

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

        bool unknownvalue, known;

        if (! checkIsGoalAchievedInRealTime(*(tempStateNode->state), sd, unknownvalue, known))
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
            bool known;

            if (! effState->isSatisfied(*(fowardState->state),satDegree, known))
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
                bool known;

                if (! effState->isSatisfied(*(tempStateNode->state ),satDegree, known))
                {
                    if (effState->permanent)
                    {
                        negativeGoal = true;
                        delete effState;
                        return 1000;
                    }
                    else
                        negateveStateNum ++;
                }
                else
                    continue;
            }

        }

        // check how many unsatisifed subgoals will be negated by this effect
        for (sait = unsatisfiedStateNodes.begin(); sait != unsatisfiedStateNodes.end(); ++ sait)
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

//            // only check the StateNode which is more backward than the input fowardState
//            // if this effect state node has a lower depth than this state node, it means the effect will happen after this state node
//            // so this state node will not be affected by this effect.
//            if ((*fowardState) < (*tempStateNode))
//            {
//                // need to check if it's the same level with fowardState,
//                if ((fowardState->forwardRuleNode != tempStateNode->forwardRuleNode))
//                    continue;
//            }

            if (tempStateNode->state->isSameState(*effState))
            {
                // check if this effect unsatisfy this state
                float satDegree;
                bool known;

                if (! effState->isSatisfied(*(tempStateNode->state ),satDegree,known))
                {
                    if (effState->permanent)
                    {
                        negativeGoal = true;
                        delete effState;
                        return 1000;
                    }
                    else
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
                                        bool &isDiffStateOwnerType, bool &preconImpossible, bool &willAddCirle, bool &contradictoryOtherGoal, bool &needRollBack)
{

    RuleNode* tmpRuleNode = new RuleNode(rule,fowardState);

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
    contradictoryOtherGoal = false;
    needRollBack = false;
    bool isRecursivePrecon0Sat,isRecursivePrecon1Sat;

    // check all the effects:
    negateveStateNum = checkEffectFitness(tmpRuleNode,fowardState,isDiffStateOwnerType,negativeGoal);


    bool hasDirectHelpRule;

    // check how many preconditions will be satisfied

    satisfiedPreconNum = checkPreconditionFitness(tmpRuleNode,fowardState,preconImpossible,willAddCirle, hasDirectHelpRule, contradictoryOtherGoal,isRecursivePrecon0Sat,isRecursivePrecon1Sat);

    if (preconImpossible && rule->isReversibleRule && rule->action->getType().getCode() == DO_NOTHING_CODE)
    {
        bool directHelp;
        rule->isRulePossibleToHelpToAchieveGoal(fowardState->state,directHelp);

        if(directHelp)
            needRollBack = true;
    }

    delete tmpRuleNode;

}

void OCPlanner::addHypotheticalLinkForStateNode(StateNode *stateNode)
{
    // Create evaluationlink used by pattern matcher
    Handle predicateNode = AtomSpaceUtil::addNode(*atomSpace,PREDICATE_NODE, stateNode->state->name().c_str());

    HandleSeq predicateListLinkOutgoings;

    // add all the stateOwners
    for (vector<ParamValue>::iterator ownerIt = stateNode->state->stateOwnerList.begin(); ownerIt != stateNode->state->stateOwnerList.end(); ++ ownerIt)
    {
        HandleSeq handles = Inquery::generateNodeForGroundedParamValue(&(*ownerIt));

        OC_ASSERT((handles.size() != 0),
                  "OCPlanner::addHypotheticalLinkForStateNode: cannot generate handle for this state owner value for state: %s is invalid.\n",
                  stateNode->state->name().c_str());

        predicateListLinkOutgoings.insert(predicateListLinkOutgoings.end(), handles.begin(),handles.end());

    }

    // add the state value
    HandleSeq handles =  Inquery::generateNodeForGroundedParamValue(&(stateNode->state->stateVariable->getValue()));

    OC_ASSERT((handles.size() != 0),
              "OCPlanner::addHypotheticalLinkForStateNode: cannot generate handle for this state value for state: %s is invalid.\n",
              stateNode->state->name().c_str());

    predicateListLinkOutgoings.insert(predicateListLinkOutgoings.end(), handles.begin(),handles.end());

    Handle predicateListLink = AtomSpaceUtil::addLink(*atomSpace,LIST_LINK, predicateListLinkOutgoings);

    HandleSeq evalLinkOutgoings;
    evalLinkOutgoings.push_back(predicateNode);
    evalLinkOutgoings.push_back(predicateListLink);
    Handle hEvalLink;

    // If the state is STATE_NOT_EQUAL_TO, need to use a NotLink wrap the EvaluationLink
    if (stateNode->state->stateType == STATE_NOT_EQUAL_TO)
       hEvalLink = AtomSpaceUtil::addLink(*atomSpace,EVALUATION_LINK, evalLinkOutgoings, true, true, TruthValue::FALSE_TV());
    else
       hEvalLink  = AtomSpaceUtil::addLink(*atomSpace,EVALUATION_LINK, evalLinkOutgoings,true);

    stateNode->hypotheticalLink = hEvalLink;

//    static int count = 1;
//    cout <<std::endl << "Added HypotheticalLink " << count++ << std::endl << atomSpace->atomAsString(hEvalLink).c_str() << std::endl;

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
void OCPlanner::deleteRuleNodeRecursively(RuleNode* ruleNode, StateNode* forwardStateNode, bool deleteThisforwardStateNode, bool deleteThisRuleNode)
{

    // For the other forward state node of this rule node which is not the given forwardStateNode,
    // does not belong to the main branch of this given forwardStateNode
    // so don't need to delete such state nodes, only need to remove them from the temporaryStateNodes ,
    // and put them into the unsatisfied list if there is any other rule node used them as preconditions

    // among all its foward state nodes, move all with forward rule link from temporaryStateNodes to unsatisfiedStateNodes, delete the ones without forward link
    vector<StateNode*>::iterator forwardStateIt;
    for (forwardStateIt = ruleNode->forwardLinks.begin(); forwardStateIt != ruleNode->forwardLinks.end(); ++ forwardStateIt)
    {
        StateNode* forwardForwardSN = ((StateNode*)(*forwardStateIt));

        deleteStateNodeInTemporaryList(forwardForwardSN);

        if(forwardForwardSN->forwardRuleNode == 0)
        {
            if (forwardForwardSN == forwardStateNode)
                continue;

            if (forwardForwardSN->hypotheticalLink != Handle::UNDEFINED)
            {
//               cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                   << atomSpace->atomAsString(forwardForwardSN->hypotheticalLink).c_str() << std::endl;

               atomSpace->removeAtom(forwardForwardSN->hypotheticalLink);
               forwardForwardSN->hypotheticalLink = Handle::UNDEFINED;

            }

            delete forwardForwardSN;
        }
        else
        {
            unsatisfiedStateNodes.push_back(forwardForwardSN);
            forwardForwardSN->backwardRuleNode = 0;
        }

    }

    ruleNode->forwardLinks.clear();

    if (forwardStateNode && deleteThisforwardStateNode)
    {
        if (forwardStateNode->hypotheticalLink != Handle::UNDEFINED)
        {
//            cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                << atomSpace->atomAsString(forwardStateNode->hypotheticalLink).c_str() << std::endl;

           atomSpace->removeAtom(forwardStateNode->hypotheticalLink);
        }
        delete forwardStateNode;
    }

    // check all its backward state nodes
    // delete them recursively
    vector<StateNode*>::iterator backwardStateIt;
    for (backwardStateIt = ruleNode->backwardLinks.begin(); backwardStateIt != ruleNode->backwardLinks.end(); ++ backwardStateIt)
    {
        StateNode* curSNode = (StateNode*)(*backwardStateIt);

        deleteStateNodeInTemporaryList(curSNode);
        deleteStateNodeInUnsatisfedNodeList(curSNode);

        if (curSNode->hypotheticalLink != Handle::UNDEFINED)
        {
//            cout <<std::endl << "Removed HypotheticalLink " << ++removedHypotheticalLinkCount << std::endl
//                << atomSpace->atomAsString(curSNode->hypotheticalLink).c_str() << std::endl;

            atomSpace->removeAtom(curSNode->hypotheticalLink);

            curSNode->hypotheticalLink = Handle::UNDEFINED;
        }

         if (curSNode->backwardRuleNode != 0)
             deleteRuleNodeRecursively(curSNode->backwardRuleNode, curSNode);
         else
         {
             delete curSNode;
         }

    }

    ruleNode->backwardLinks.clear();

    deleteRuleNodeInAllRuleNodeList(ruleNode);

    if (ruleNode && deleteThisRuleNode)
        delete ruleNode;

}

void OCPlanner::deleteStateNodeInUnsatisfedNodeList(StateNode* stateNode)
{
    list<StateNode*>::iterator it;
    for (it = unsatisfiedStateNodes.begin(); it != unsatisfiedStateNodes.end(); ++it)
    {
        if ((*it) == stateNode)
        {
            unsatisfiedStateNodes.erase(it);
            break;
        }
    }
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
        //       so the total cost of this recursive rule = Distance(x,m) + Distance(m,y)
        //       but when really calculate the cost, only the unsatisfied preconditions cost will be calculated

        vector<State*>::iterator itpre;
        for (itpre = ruleNode->originalRule->preconditionList.begin(); itpre != ruleNode->originalRule->preconditionList.end(); ++ itpre)
        {

            vector<CostHeuristic>::iterator costIt;
            for(costIt = forwardRuleNode->originalRule->CostHeuristics.begin(); costIt != forwardRuleNode->originalRule->CostHeuristics.end(); ++costIt)
            {
                State* forward_cost_state = ((CostHeuristic)(*costIt)).cost_cal_state;
                State* cost_state = new State(forward_cost_state->name(),forward_cost_state->stateVariable->getType(),forward_cost_state->stateType,
                                              forward_cost_state->stateVariable->getValue(),forward_cost_state->need_inquery,forward_cost_state->inqueryStateFun,forward_cost_state->permanent);
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

                ruleNode->AddCostHeuristic(cost_state, ((CostHeuristic)(*costIt)).cost_coefficient);

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
    bool contradictoryOtherGoal = false;
    bool isRecursivePrecon0Sat,isRecursivePrecon1Sat;

    impossible = false;

    // check all the effects:
    negateveStateNum = checkEffectFitness(ruleNode,fowardState,isDiffStateOwnerType,negativeGoal);

    // check how many preconditions will be satisfied
    satisfiedPreconNum = checkPreconditionFitness(ruleNode,fowardState,preconImpossible,willAddCirle, hasDirectHelpRule, contradictoryOtherGoal,isRecursivePrecon0Sat,isRecursivePrecon1Sat);

    // ruleNode resetbinding
    ruleNode->currentAllBindings = ruleNode->currentBindingsFromForwardState;

    fitnessScore = fitnessScore - negateveStateNum*100.0f + satisfiedPreconNum*100.0f;

    if (isDiffStateOwnerType || negativeGoal || preconImpossible || willAddCirle || contradictoryOtherGoal)
    {
        impossible = true;
        fitnessScore -= 999999;
    }

    if (hasDirectHelpRule)
        fitnessScore += 50.0f;

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

    if (number_easy_state == 0)
    {
        cout<<"Debug: groundARuleNodeBySelectingNonNumericValues(): There is no non numeric variable in this rule. Doesn't need to be grounded!"<<std::endl;
        return true;
    }

    // ToBeImproved: Is it possible that some non-need-real-time-inquery states contains some variables that need to be grounded by other need_real-time-inquery states?

    // find all the canditates meet as many as possible preconditions, and put all the candidates in ParamCandidates
    // first, find from the easy state via selecting values from the Atomspace

    int n_max = number_easy_state;
    bool* indexes = new bool[n_max];

    // ToBeImproved: in fact we don't need to generate all the possible groups of candidates at this moment,
    //               we can just allow it generateNextCandidates every time one group failed in a planning step
    // we generate the candidates by trying full combination calculation

    // record the varaible number
    unsigned int var_total_num;
    bool firstCombination = true;
    bool cannotGroundAllVaraibles = false;

    for (int n_gram = n_max; n_gram >= 1; -- n_gram)
    {
        // Use the binary method to generate all combinations:

        if (cannotGroundAllVaraibles)
            break;

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

            if (firstCombination)
            {
                var_total_num = varNames.size();
                firstCombination = false;
            }

            // means that this group of results can't ground it all the ungrounded variables
            if ( varNames.size() < var_total_num)
            {
                cannotGroundAllVaraibles = true;
                if (isLastNElementsAllTrue(indexes, n_max, n_gram))
                    break;

                generateNextCombinationGroup(indexes, n_max);

                continue;
            }

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

                    atomSpace->removeAtom(listH);

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

    delete [] indexes;

    if (ruleNode->ParamCandidates.size() == 0)
    {
        return false;
    }
    else
    {
        cout<<"Debug: groundARuleNodeBySelectingNonNumericValues() found candidate group number totally: "<<ruleNode->ParamCandidates.size()<<std::endl;

        // Till now,  all the easy states have been dealed with, now we need to deal with numberic states if any
        // we won't ground the numeric states here, because it's too time-consuming,
        // we won't give all the possible combinations of numeric values and non-numeric values for candidates

        return true;
    }
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
    RuleNode* tmpRuleNode = new RuleNode(rule,0);
    tmpRuleNode->currentAllBindings = currentbindings;

    vector<ParamValue>::iterator vit;
    float bestScore = -9999999.9;
    ParamValue bestValue = UNDEFINED_VALUE;

    for (vit = values.begin(); vit != values.end(); ++ vit)
    {
//        Vector* vector1 = boost::get<Vector>(&(*vit));

        float score = 0.0f;

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
        bool contradictoryOtherGoal;
        bool isRecursivePrecon0Sat,isRecursivePrecon1Sat;
        int satisfiedPreconNum = checkPreconditionFitness(tmpRuleNode,curStateNode,preconImpossible,willAddCirle,hasDirectHelpRule,
                                                          contradictoryOtherGoal, isRecursivePrecon0Sat, isRecursivePrecon1Sat, orginalRule);


        if (preconImpossible || willAddCirle || contradictoryOtherGoal)
            score -= 99999.9f;

        // calculate the cost
        // for recursive rules, only the unsatisfied preconditions cost will be calculated
        currentbindings.insert(std::pair<string, ParamValue>(varName,*vit));
        float cost = Rule::getCost(rule, basic_cost, costHeuristics, currentbindings,isRecursivePrecon0Sat, isRecursivePrecon1Sat);
        if (cost < -0.00001f)
        {
            logger().error("OCPlanner::selectBestNumericValueFromCandidates: this rule has not been grounded fully!" );
            return UNDEFINED_VALUE;
        }

        score -= cost*100;
        currentbindings.erase(varName);

        if (checkPrecons)
        {
            score += satisfiedPreconNum * 10.0f;

            if(hasDirectHelpRule)
                score += 5.0f;
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

bool OCPlanner::checkHasNoCoexistenceRuleInPlanningNetWork(Rule* r)
{
    if (r->noCoexisenceOtherRules.size() == 0)
        return false;

    vector<RuleNode*>::iterator ruleItInNetWork;
    vector<Rule*>::iterator nocoRuleIt;

    for (nocoRuleIt = r->noCoexisenceOtherRules.begin(); nocoRuleIt != r->noCoexisenceOtherRules.end(); ++ nocoRuleIt)
    {
        Rule* nocoRule = *nocoRuleIt;
        for (ruleItInNetWork = allRuleNodeInThisPlan.begin(); ruleItInNetWork != allRuleNodeInThisPlan.end(); ++ ruleItInNetWork)
        {
            RuleNode* ruleNode = *ruleItInNetWork;
            if (nocoRule == ruleNode->originalRule)
                return true;
        }

    }

    return false;
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
    cout << std::endl << std::endl << "RuleNode:" << ruleNode->originalRule->ruleName << " Action:" << ruleNode->originalRule->action->getName() <<  std::endl;

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

    cout << " Rule node dept = " << lastedStateNode->depth ;
}

// a bunch of rules for test, load from c++ codes
void OCPlanner::loadTestRulesFromCodes()
{
    // define a special action: do nothing
    AvatarAction* doNothingAction = new AvatarAction(ActionType::DO_NOTHING());


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
    AvatarAction* eatAction = new AvatarAction(ActionType::EAT());
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
    AvatarAction* pickupAction = new AvatarAction(ActionType::GRAB());
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

    //----------------------------Begin Rule: Move_to an object to get closed to it -----------------------------------------
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
    AvatarAction* moveToObjectAction = new AvatarAction(ActionType::MOVE_TO_OBJ());
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
    AvatarAction* walkAction = new AvatarAction(ActionType::WALK());
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
    AvatarAction* walkAction0 = new AvatarAction(ActionType::WALK());
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
    AvatarAction* buildBlockAction = new AvatarAction(ActionType::BUILD_BLOCK());
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


//    //----------------------------Begin Einstein puzzle rules---------------------------------------------------------------------


//    //----------------------------Begin Rule: The person who drinks water keeps fish------------------------------------------------------
//    // define variables:
//    ParamValue var_pet = str_var[0];
//    ParamValue var_man_x = entity_var[0];

//    // precondition 1: var_man_x is people
//    vector<ParamValue> peopleStateOwnerList1;
//    peopleStateOwnerList1.push_back(var_man_x);
//    State* peopleState1 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", peopleStateOwnerList1,false,0);


//    // precondition 1: var_man_x is people

//    // precondition 3: var_man_x  drinks water
//    vector<ParamValue> drinkWaterStateOwnerList2;
//    drinkWaterStateOwnerList2.push_back(var_man_x);
//    State* drinkWaterState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"water", drinkWaterStateOwnerList2, false, 0, true);

//    // effect1: var_man_x keeps fish
//    vector<ParamValue> keepFishStateOwnerList2;
//    keepFishStateOwnerList2.push_back(var_man_x);
//    State* keepFishState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,var_pet, keepFishStateOwnerList2, false,0,true);
//    Effect* keepFishStateEffect2 = new Effect(keepFishState2, OP_ASSIGN, "fish",true);

//    // add rule:
//    Rule* waterDrinkerKeepsFishRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f, false, true);
//    waterDrinkerKeepsFishRule->ruleName = "waterDrinkerKeepsFishRule";
//    waterDrinkerKeepsFishRule->addPrecondition(peopleState1);
//    waterDrinkerKeepsFishRule->addPrecondition(drinkWaterState2);

//    waterDrinkerKeepsFishRule->addEffect(EffectPair(1.0f,keepFishStateEffect2));

//    this->AllRules.push_back(waterDrinkerKeepsFishRule);
//    //----------------------------End Rule: The person who drinks water keeps fish--------------------------------------------------

//    //----------------------------Begin Rule: The person who keeps fish drinks water.------------------------------------------------------
//    // define variables:
//    ParamValue var_drink = str_var[0];

//    // precondition 3: var_man_x keeps fish
//    vector<ParamValue> keepFishStateOwnerList1;
//    keepFishStateOwnerList1.push_back(var_man_x);
//    State* keepFishState1 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"fish", keepFishStateOwnerList1, false,0,true);

//    // effect1: var_man_x drinks water
//    vector<ParamValue> drinkWaterStateOwnerList1;
//    drinkWaterStateOwnerList1.push_back(var_man_x);
//    State* drinkWaterState1 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,var_drink, drinkWaterStateOwnerList1, false, 0, true);
//    Effect* drinkWaterStateEffect1 = new Effect(drinkWaterState1, OP_ASSIGN, "water",true);

//    // add rule:
//    Rule* fishKeeperDrinkWaterRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f, false, true);
//    fishKeeperDrinkWaterRule->ruleName = "fishKeeperDrinkWaterRule";
//    fishKeeperDrinkWaterRule->addPrecondition(peopleState1);
//    fishKeeperDrinkWaterRule->addPrecondition(keepFishState1);

//    fishKeeperDrinkWaterRule->addEffect(EffectPair(1.0f,drinkWaterStateEffect1));

//    this->AllRules.push_back(fishKeeperDrinkWaterRule);

//    waterDrinkerKeepsFishRule->noCoexisenceOtherRules.push_back(fishKeeperDrinkWaterRule);
//    fishKeeperDrinkWaterRule->noCoexisenceOtherRules.push_back(waterDrinkerKeepsFishRule);
//    //----------------------------End Rule: The person who keeps fish drinks water--------------------------------------------------


//    //----------------------------Begin Rule: The person who drinks tea keeps cats------------------------------------------------------
//    // define variables:

//    // precondition 1: var_man_x is people

//    // precondition 2: var_man_x drinks tea
//    vector<ParamValue> drinkTeaStateOwnerList1;
//    drinkTeaStateOwnerList1.push_back(var_man_x);
//    State* drinkTeaState1 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,"tea", drinkTeaStateOwnerList1, false, 0, true);

//    // effect1: var_man_x keeps cats
//    vector<ParamValue> keepCatsStateOwnerList1;
//    keepCatsStateOwnerList1.push_back(var_man_x);
//    State* keepCatsStateState1 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,var_pet, keepCatsStateOwnerList1, false, 0, true);
//    Effect* keepCatsEffect1 = new Effect(keepCatsStateState1, OP_ASSIGN, "cats",true);

//    // add rule:
//    Rule* teaDrinkerKeepsCatsRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f, false, true);
//    teaDrinkerKeepsCatsRule->ruleName = "teaDrinkerKeepsCatsRule";
//    teaDrinkerKeepsCatsRule->addPrecondition(peopleState1);
//    teaDrinkerKeepsCatsRule->addPrecondition(drinkTeaState1);

//    teaDrinkerKeepsCatsRule->addEffect(EffectPair(1.0f,keepCatsEffect1));

//    this->AllRules.push_back(teaDrinkerKeepsCatsRule);
//    //----------------------------End Rule: The person who drinks tea keeps cats--------------------------------------------------

//    //----------------------------Begin Rule: The person who keeps cats drinks tea -----------------------------------------------------
//    // define variables:

//    // precondition 1: var_man_x is people

//    // precondition 2: var_man_x keeps cats
//    vector<ParamValue> keepCatsStateOwnerList2;
//    keepCatsStateOwnerList2.push_back(var_man_x);
//    State* keepCatsStateState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"cats", keepCatsStateOwnerList2, false, 0, true);

//    // effect1: var_man_x drinks tea
//    vector<ParamValue> drinkTeaStateOwnerList2;
//    drinkTeaStateOwnerList2.push_back(var_man_x);
//    State* drinkTeaState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,var_drink, drinkTeaStateOwnerList2, false, 0, true);
//    Effect* drinkTeaEffect2 = new Effect(drinkTeaState2, OP_ASSIGN, "tea",true);

//    // add rule:
//    Rule* catsKeeperDrinksTeaRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f, false, true);
//    catsKeeperDrinksTeaRule->ruleName = "catsKeeperDrinksTeaRule";
//    catsKeeperDrinksTeaRule->addPrecondition(peopleState1);
//    catsKeeperDrinksTeaRule->addPrecondition(keepCatsStateState2);

//    catsKeeperDrinksTeaRule->addEffect(EffectPair(1.0f,drinkTeaEffect2));

//    this->AllRules.push_back(catsKeeperDrinksTeaRule);

//    teaDrinkerKeepsCatsRule->noCoexisenceOtherRules.push_back(catsKeeperDrinksTeaRule);
//    catsKeeperDrinksTeaRule->noCoexisenceOtherRules.push_back(teaDrinkerKeepsCatsRule);
//    //----------------------------End Rule: The person who keeps cats drinks tea-------------------------------------------------

//    //----------------------------Begin Rule: closed to person who keep fish to achieve Integrity demanding goal -------------------------------------------
//    // define variables:
//    ParamValue var_fish_keeper = entity_var[0];
//    ParamValue var_achieve_integrity_goal = bool_var[0];

//    // precondition 1: var_fish_keeper is people
//    vector<ParamValue> peopleStateOwnerList;
//    peopleStateOwnerList.push_back(var_fish_keeper);
//    State* fishKeeperPeopleState = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", peopleStateOwnerList,false,0);

//    // precondition 2: var_fish_keeper keeps fish
//    vector<ParamValue> keepFishStateOwnerList;
//    keepFishStateOwnerList.push_back(var_fish_keeper);
//    State* keepFishState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,"fish", keepFishStateOwnerList, false,0, true);

//    // precondition 3: closed to var_fish_keeper
//    vector<ParamValue> closedToFishKeeperStateOwnerList;
//    closedToFishKeeperStateOwnerList.push_back(var_avatar);
//    closedToFishKeeperStateOwnerList.push_back(var_fish_keeper);
//    State* closedToFishKeeperState = new State("Distance",ActionParamType::FLOAT(),STATE_LESS_THAN ,ACCESS_DISTANCE, closedToFishKeeperStateOwnerList, true, &Inquery::inqueryDistance);

//    // energy state:
//    vector<ParamValue> IntegrityStateOwnerList;
//    IntegrityStateOwnerList.push_back(var_avatar);
//    State* IntegrityGoalState = new State("IntegrityDemandGoal",ActionParamType::BOOLEAN(),STATE_EQUAL_TO ,var_achieve_integrity_goal, IntegrityStateOwnerList);

//    // effect1: Integrity Goal Achieved
//    Effect* IntegrityGoalAchievedEffect = new Effect(IntegrityGoalState, OP_ASSIGN, SV_TRUE);

//    Rule* IntegrityRule = new Rule(doNothingAction,boost::get<Entity>(var_avatar),0.2f);
//    IntegrityRule->ruleName = "ClosedToFishKeeperIIntegrityRule";
//    IntegrityRule->addPrecondition(fishKeeperPeopleState);
//    IntegrityRule->addPrecondition(keepFishState);
//    IntegrityRule->addPrecondition(closedToFishKeeperState);

//    IntegrityRule->addEffect(EffectPair(1.0f,IntegrityGoalAchievedEffect));

//    this->AllRules.push_back(IntegrityRule);

//    //----------------------------End Rule:closed to person who keep fish to achieve Integrity demanding goal -------------------------------------------


//    //----------------------------Begin Rule: if other people do not keep pet_x, then man_1 keeps it--------------------------------------
//    // define variables:
//    ParamValue man_1 = entity_var[0];
//    ParamValue man_2 = entity_var[1];
//    ParamValue man_3 = entity_var[2];
//    ParamValue man_1_pet = str_var[3];
//    ParamValue pet_x = str_var[4];
//    ParamValue pet_y = str_var[5];
//    ParamValue pet_z = str_var[6];

//    // precondition 1 -5 : man_1-5 are people
//    vector<ParamValue> ispeopleStateOwnerList1;
//    ispeopleStateOwnerList1.push_back(man_1);
//    State* ispeopleState1 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList1,false,0);

//    vector<ParamValue> ispeopleStateOwnerList2;
//    ispeopleStateOwnerList2.push_back(man_2);
//    State* ispeopleState2 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList2, false,0);

//    vector<ParamValue> ispeopleStateOwnerList3;
//    ispeopleStateOwnerList3.push_back(man_3);
//    State* ispeopleState3 = new State("class",ActionParamType::STRING(),STATE_EQUAL_TO , "people", ispeopleStateOwnerList3,false,0);


//    // additional preconditions: man_2, man_3,  are not the same to man_1
//    vector<ParamValue> isNotSameOwnerList1;
//    isNotSameOwnerList1.push_back(man_1);
//    isNotSameOwnerList1.push_back(man_2);
//    State* isNotSameState1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameOwnerList1,true, &Inquery::inqueryIsSame);

//    vector<ParamValue> isNotSameOwnerList2;
//    isNotSameOwnerList2.push_back(man_1);
//    isNotSameOwnerList2.push_back(man_3);
//    State* isNotSameState2 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameOwnerList2,true, &Inquery::inqueryIsSame);


//    // precondition 0: pet_x y z are pets
//    vector<ParamValue> isPetStateOwnerList1;
//    isPetStateOwnerList1.push_back(pet_x);
//    State* isPetState1 = new State("is_pet",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isPetStateOwnerList1, false, 0);

//    vector<ParamValue> isPetStateOwnerList2;
//    isPetStateOwnerList2.push_back(pet_y);
//    State* isPetState2 = new State("is_pet",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isPetStateOwnerList2, false, 0);

//    vector<ParamValue> isPetStateOwnerList3;
//    isPetStateOwnerList3.push_back(pet_z);
//    State* isPetState3 = new State("is_pet",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isPetStateOwnerList3, false, 0);

//    // precondition 1 -5 : man_1-3 are people

//    // precondition: pet_y z are different from pet_x
//    vector<ParamValue> isNotSamePetOwnerList1;
//    isNotSamePetOwnerList1.push_back(pet_x);
//    isNotSamePetOwnerList1.push_back(pet_y);
//    State* isNotSamePet1 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSamePetOwnerList1,true, &Inquery::inqueryIsSame);

//    vector<ParamValue> isNotSamePetOwnerList2;
//    isNotSamePetOwnerList2.push_back(pet_x);
//    isNotSamePetOwnerList2.push_back(pet_z);
//    State* isNotSamePet2 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSamePetOwnerList2,true, &Inquery::inqueryIsSame);

//    // precondition 6-9 : other people do not keep pet_x
//    vector<ParamValue> notkeepXStateOwnerList1;
//    notkeepXStateOwnerList1.push_back(man_2);
//    State* notkeepXState1 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,pet_y, notkeepXStateOwnerList1, false,0, true);

//    vector<ParamValue> notkeepXStateOwnerList2;
//    notkeepXStateOwnerList2.push_back(man_3);
//    State* notkeepXState2 = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,pet_z, notkeepXStateOwnerList2, false,0, true);

//    // effect1: man_1 keeps pet_x
//    vector<ParamValue> keepXStateOwnerList;
//    keepXStateOwnerList.push_back(man_1);
//    State* keepXState = new State("keep_pet",ActionParamType::STRING(),STATE_EQUAL_TO ,man_1_pet, keepXStateOwnerList, false,0, true);
//    Effect* keepXEffect = new Effect(keepXState, OP_ASSIGN, pet_x,true);

//    // add rule:
//    Rule* notKeepOtherPeoplesPetRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
//    notKeepOtherPeoplesPetRule->ruleName = "notKeepOtherPeoplesPetRule";
//    notKeepOtherPeoplesPetRule->addPrecondition(isPetState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(isPetState2);
//    notKeepOtherPeoplesPetRule->addPrecondition(isPetState3);

//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState2);
//    notKeepOtherPeoplesPetRule->addPrecondition(ispeopleState3);

//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSameState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSameState2);

//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSamePet1);
//    notKeepOtherPeoplesPetRule->addPrecondition(isNotSamePet2);

//    notKeepOtherPeoplesPetRule->addPrecondition(notkeepXState1);
//    notKeepOtherPeoplesPetRule->addPrecondition(notkeepXState2);

//    notKeepOtherPeoplesPetRule->addEffect(EffectPair(1.0f,keepXEffect));

//    this->AllRules.push_back(notKeepOtherPeoplesPetRule);
//    //----------------------------End Rule:  if other 4 people do not keep pet_x, then man_1 keeps it-------------------------------------------------

//    //----------------------------Begin Rule: if other people do not drink drink_x, then man_1 drinks it--------------------------------------
//    // define variables:
//    ParamValue drink_man_1 = str_var[0];
//    ParamValue drink_x = str_var[1];
//    ParamValue drink_y = str_var[2];
//    ParamValue drink_z = str_var[3];

//    // precondition 0: drink_x y z are drink
//    vector<ParamValue> isDrinkStateOwnerList1;
//    isDrinkStateOwnerList1.push_back(drink_x);
//    State* isDrinkState1 = new State("is_drink",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isDrinkStateOwnerList1,false, 0);

//    vector<ParamValue> isDrinkStateOwnerList2;
//    isDrinkStateOwnerList2.push_back(drink_y);
//    State* isDrinkState2 = new State("is_drink",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isDrinkStateOwnerList2,false, 0);

//    vector<ParamValue> isDrinkStateOwnerList3;
//    isDrinkStateOwnerList3.push_back(drink_z);
//    State* isDrinkState3 = new State("is_drink",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "true", isDrinkStateOwnerList3,false, 0);

//    // precondition 1 -5 : man_1-3 are people and different

//    //  precondition:  drink_y and  drink_z are different
//    vector<ParamValue> isNotSameDrinkOwnerList1;
//    isNotSameDrinkOwnerList1.push_back(drink_x);
//    isNotSameDrinkOwnerList1.push_back(drink_y);
//    State* isNotSameDrinkState = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameDrinkOwnerList1,true, &Inquery::inqueryIsSame);

//    //  precondition:  drink_y and  drink_z are different
//    vector<ParamValue> isNotSameDrinkOwnerList2;
//    isNotSameDrinkOwnerList2.push_back(drink_x);
//    isNotSameDrinkOwnerList2.push_back(drink_z);
//    State* isNotSameDrinkState2 = new State("is_same",ActionParamType::BOOLEAN(),STATE_EQUAL_TO , "false", isNotSameDrinkOwnerList2,true, &Inquery::inqueryIsSame);

//    // precondition 6-9 : other people do not drink drink_x
//    vector<ParamValue> notdrinkXStateOwnerList1;
//    notdrinkXStateOwnerList1.push_back(man_2);
//    State* notdrinkXState1 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,drink_y, notdrinkXStateOwnerList1, false, 0, true);

//    vector<ParamValue> notdrinkXStateOwnerList2;
//    notdrinkXStateOwnerList2.push_back(man_3);
//    State* notdrinkXState2 = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,drink_z, notdrinkXStateOwnerList2, false, 0, true);

//    // effect1: man_1 drinks drink_x
//    vector<ParamValue> drinkXStateOwnerList;
//    drinkXStateOwnerList.push_back(man_1);
//    State* drinkXState = new State("drink",ActionParamType::STRING(),STATE_EQUAL_TO ,drink_man_1, drinkXStateOwnerList, false, 0, true);
//    Effect* drinkXEffect = new Effect(drinkXState, OP_ASSIGN, drink_x,true);


//    // add rule:
//    Rule* notDrinkOtherPeoplesDrinkRule = new Rule(doNothingAction,boost::get<Entity>(selfEntityParamValue),0.0f);
//    notDrinkOtherPeoplesDrinkRule->ruleName = "notDrinkOtherPeoplesDrinkRule";
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isDrinkState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isDrinkState2);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isDrinkState3);

//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState2);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(ispeopleState3);

//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameState2);

//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameDrinkState);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(isNotSameDrinkState2);

//    notDrinkOtherPeoplesDrinkRule->addPrecondition(notdrinkXState1);
//    notDrinkOtherPeoplesDrinkRule->addPrecondition(notdrinkXState2);

//    notDrinkOtherPeoplesDrinkRule->addEffect(EffectPair(1.0f,drinkXEffect));

//    this->AllRules.push_back(notDrinkOtherPeoplesDrinkRule);
//    //----------------------------End Rule:  if other 4 people do not drink drink_x, then man_1 drinks it--------------------------------------------------
//    //----------------------------End Einstein puzzle rules--------------------------------------------------


}


void OCPlanner::loadFacts(vector<State*> &knownStates)
{
    // test simplified Einstein puzzle facts:
//    ; The German doesn't drink milk
//    (EvaluationLink (stv 0 1)
//       (PredicateNode "drink")
//       (ListLink
//          (AvatarNode "id_German_man")
//          (ConceptNode "milk")
//       )
//    )

//    ; The British doesn't drink water
//    (EvaluationLink (stv 0 1)
//       (PredicateNode "drink")
//       (ListLink
//          (AvatarNode "id_British_man")
//          (ConceptNode "water")
//       )
//    )

//    ; The British doesn't keep dogs
//    (EvaluationLink (stv 0 1)
//       (PredicateNode "keep_pet")
//       (ListLink
//          (AvatarNode "id_British_man")
//          (ConceptNode "dogs")
//       )
//    )

//    ParamValue GermanEntity = Entity("id_German_man","avatar");
//    ParamValue BritishEntity = Entity("id_British_man","avatar");

//    vector<ParamValue> GermanDoesntDrinkMilkStateOwnerList;
//    GermanDoesntDrinkMilkStateOwnerList.push_back(GermanEntity);
//    State* GermanDoesntDrinkMilkState = new State("drink",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,"milk", GermanDoesntDrinkMilkStateOwnerList, false, 0, true);

//    knownStates.push_back(GermanDoesntDrinkMilkState);

//    vector<ParamValue> BritishDoesntDrinkWaterStateOwnerList;
//    BritishDoesntDrinkWaterStateOwnerList.push_back(BritishEntity);
//    State* BritishDoesntDrinkWaterState = new State("drink",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,"water", BritishDoesntDrinkWaterStateOwnerList, false, 0, true);
//    knownStates.push_back(BritishDoesntDrinkWaterState);

//    vector<ParamValue> BritishDoesntKeepDogsStateOwnerList;
//    BritishDoesntKeepDogsStateOwnerList.push_back(BritishEntity);
//    State* BritishDoesntKeepDogsState = new State("keep_pet",ActionParamType::STRING(),STATE_NOT_EQUAL_TO ,"dogs", BritishDoesntKeepDogsStateOwnerList, false, 0, true);
//    knownStates.push_back(BritishDoesntKeepDogsState);

}
