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
#include <opencog/embodiment/Control/PerceptionActionInterface/PetAction.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/ActionType.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h>
#include <iterator>
#include <list>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/atom_types.h>

#include <map>
#include <math.h>
#include <stdio.h>
#include <sstream>

using namespace opencog::oac;
using namespace std;


// this function need to be call after its forward rule node assigned, to calculate the depth of this state node
// the root state node depth is 0, every state node's depth is its forward rule node's forward state node' depth +1
// it its forward rule node has multiple forward state node, using the deepest one
void StateNode::calculateNodeDepth()
{
    if (! this->forwardRuleNode)
        return;

    set<StateNode*>::iterator it;

    int deepest = 0;
    for ( it = this->forwardRuleNode->forwardLinks.begin(); it != this->forwardRuleNode->forwardLinks.end();  ++ it)
    {
        int d = ((StateNode*)(*it))->depth;
        if (d > deepest)
            deepest = d;
    }

    this->depth = deepest + 1;

}

void RuleNode::updateCurrentAllBindings()
{
    currentAllBindings = currentBindingsFromForwardState;
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


// @ bool &found: return if this same state is found in temporaryStateNodes
// @ StateNode& *stateNode: the stateNode in temporaryStateNodes which satisfied or dissatisfied this goal
bool OCPlanner::checkIfThisGoalIsSatisfiedByTempStates(State& goalState, bool &found, StateNode* &satstateNode)
{
    //  check if this state has beed satisfied by the previous state nodes
    list<StateNode*>::const_iterator vit = temporaryStateNodes.begin();
    float satisfiedDegree;

    for (;vit != temporaryStateNodes.end(); vit ++)
    {
        // there are possible mutiple same state nodes describe this same state in temporaryStateNodes,
        // but the latest one is put in the front, so we can just check the first one we find
        State* vState = ((StateNode*)(*vit))->state;
        if (vState->isSameState(goalState))
        {
            found = true;
            satstateNode = ((StateNode*)(*vit));

             if (vState->isSatisfied(goalState,satisfiedDegree))
             {
                 return true;
             }
             else
             {
                 return false;
             }

        }

    }

    // cannot find this state in temporaryStateNodes
    found = false;
    satstateNode = 0;

    return false;
}

OCPlanner::OCPlanner(AtomSpace *_atomspace,OAC* _oac)
{

    atomSpace = _atomspace;
    oac = _oac;

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
        StateValue inqueryValue = f(oneGoal.stateOwnerList);
        OC_ASSERT(!(inqueryValue == UNDEFINED_VALUE),
                  "OCPlanner::checkIsGoalAchievedInRealTime: the inqueried value for state: %s is invalid.\n",
                  oneGoal.name().c_str());

        return oneGoal.isSatisfiedMe(inqueryValue,satisfiedDegree,original_state);

    }
    else // it doesn't need real time calculation, then we search for its latest evaluation link value in the atomspace
    {
        // TODO
        StateValue value = Inquery::getStateValueFromAtomspace(oneGoal);
        OC_ASSERT(!(value == UNDEFINED_VALUE),
                  "OCPlanner::checkIsGoalAchievedInRealTime: the inqueried value for state: %s is invalid.\n",
                  oneGoal.name().c_str());

        // put this state into the cache, so we don't need to search for it next time
        State curState(oneGoal.name(),oneGoal.getStateValuleType(),oneGoal.stateType,value,oneGoal.stateOwnerList);

        return curState.isSatisfied(oneGoal, satisfiedDegree,original_state);

    }

}


bool OCPlanner::doPlanning(const vector<State*>& goal, vector<PetAction> &plan)
{

    int ruleNodeCount = 0;

    curtimeStamp = oac->getPAI().getLatestSimWorldTimestamp();

    // clone a spaceMap for image all the the steps happen in the spaceMap, like building a block in some postion.
    // Cuz it only happens in imagination, not really happen, we should not really change in the real spaceMap
    SpaceServer::SpaceMap* clonedMap = spaceServer().cloneTheLatestSpaceMap();

    // Set this cloned spaceMap for Inquery
    Inquery::setSpaceMap(clonedMap);

    unsatisfiedStateNodes.clear();
    temporaryStateNodes.clear();
    imaginaryHandles.clear(); // TODO: Create imaginary atoms

    // we use the basic idea of the graph planner for plan searching:
    // alternated state layers with action layers
    // But we use backward depth-first chaining, instead of forward breadth-frist reasoning
    // Because our embodiment game world is not a simple finite boolean-state world, we cannot use a full forward breadth-frist which will be too slowly

    // Firstly, we construct the original unsatisfiedStateNodes and temporaryStateNodes from the input goals

    // this is to store the already satisfied goal states before beginning planning, in case some planning steps  will change them into unsatisfied
    // Everytime it's changed into unsatisfied, it should be put into the unsatisfiedStateNodes list and removed from goalStateNodes list
    vector<StateNode*> goalStateNodes;
    vector<State*>::const_iterator it;
    for (it = goal.begin(); it != goal.end(); ++ it)
    {
        float satisfiedDegree;
        StateNode* newStateNode = new StateNode(*it);

        newStateNode->backwardRuleNode = 0;
        newStateNode->forwardRuleNode = 0;
        newStateNode->depth = 0;

        if (checkIsGoalAchievedInRealTime(*(newStateNode->state), satisfiedDegree))
        {
            goalStateNodes.push_back(newStateNode);
            temporaryStateNodes.push_front(newStateNode);
        }
        else
        {
            unsatisfiedStateNodes.push_front(newStateNode);
        }

    }

    while(unsatisfiedStateNodes.size() != 0)
    {
        Rule* selectedRule = 0;

        curtimeStamp ++;

        // decide which state should be chosed to achieved first
        list<StateNode*>::iterator stateNodeIter;
        for (stateNodeIter = unsatisfiedStateNodes.begin(); stateNodeIter != unsatisfiedStateNodes.end();++stateNodeIter)
        {
            ((StateNode*)(*stateNodeIter))->calculateNodeDepth();
        }

        unsatisfiedStateNodes.sort();

        // the state node with deeper depth will be solved first
        StateNode* curStateNode = (StateNode*)(unsatisfiedStateNodes.back());

        // if the deepest state node has a depth of -1, it means all the required states have been satisfied
        if (curStateNode->depth == -1)
            break;

        // if we have not tried to achieve this state node before, find all the candidate rules first
        if (! curStateNode->hasFoundCandidateRules)
        {
            map<string,map<float,Rule*> >::iterator it;
            it = ruleEffectIndexes.find(curStateNode->state->name());

            // Select a rule to apply

            map<float,Rule*> rules = (map<float,Rule*>)(it->second);

            if ( rules.size() == 1)
            {
                // if there is one rule to achieve this goal, just select it
                selectedRule = (((map<float,Rule*>)(it->second)).begin())->second;
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

                map<float,Rule*> ::iterator ruleIt;

                for (ruleIt = rules.begin(); ruleIt != rules.end(); ruleIt ++)
                {
                    Rule* r = ruleIt->second;

                    // TODO: check if this rule is positive or negative for the current state

                    // Because grounding every rule is time consuming, but some cost of rule requires the calculation of grounded variables.
                    // So here we just use the basic cost of every rule as the cost value.
                    float curRuleScore = 0.5f* ruleIt->first + 0.5*(1.0f - r->getBasicCost());
                    if (r->IsRecursiveRule)
                        curRuleScore += 0.5f;

                    // the rules with higher score are put in front
                    list< pair<float,Rule*> >::iterator canIt;
                    canIt = curStateNode->candidateRules.begin();

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

                selectedRule = (curStateNode->candidateRules.begin())->second;
                curStateNode->ruleHistory.push_back((curStateNode->candidateRules.begin())->second);
                curStateNode->candidateRules.pop_front();

            }

            curStateNode->hasFoundCandidateRules = true;
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
                    return false;
                }

                if (forwardRuleNode->ParamCandidates.size() == 0)
                {
                    // we have tried all the Candidate bindings in previous steps,
                    // so it means this rule doesn't work, we have to go back to its forward state node

                    // Remove this rule from the candidate rules of all its foward state nodes
                    // and move all its foward state nodes from temporaryStateNodes to unsatisfiedStateNodes
                    set<StateNode*>::iterator forwardStateIt;
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
                            unsatisfiedStateNodes.push_front(*forwardStateIt);
                    }

                    deleteRuleNodeRecursively(forwardRuleNode);

                }
                else
                {
                    // There are still Candidate bindings for this rule node to try.

                    // check which states of the preconditions of this forwardRuleNode have been sovled , which still remand unsloved.
                    set<StateNode*>::iterator preconItor;
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
                        recordOrginalStateValuesAfterGroundARule(forwardRuleNode);

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

                                    // try to find this find this variable name in the ungrounded variable records
                                    if (ungroundVarInAState.vars.find(varName) != ungroundVarInAState.vars.end())
                                    {
                                        oneAffectRecord.push_back((StateNode*)(solvedItor->second));
                                    }

                                }

                            }

                            willBeAffecteds.push_back(oneAffectRecord);

                            if (oneAffectRecord.size() < affectLeastStateNum)
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
                        recordOrginalStateValuesAfterGroundARule(forwardRuleNode);

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
                                unsatisfiedStateNodes.push_front(affectedStateNode);

                            // the affectedStateNode has been rebinded, should not delete it as well
                            deleteRuleNodeRecursively(affectedStateNode->backwardRuleNode,affectedStateNode, false);

                        }

                        continue; // continue with the big while loop to check the next subgoal in unsatisfiedStateNodes

                    }
                }

            }
        }

        // Till now have selected one unsatisfied state and the rule to applied to try to do one step backward chaining to satisfy it

        // create a new RuleNode to apply this selected rule
        RuleNode* ruleNode = new RuleNode(selectedRule);
        ruleNode->number = ruleNodeCount ++;
        ruleNode->forwardLinks.insert(curStateNode);
        curStateNode->backwardRuleNode = ruleNode;

        // When apply a rule, we need to select proper variables to ground it.

        // To ground a rule, first, we get all the variable values from the current state node to ground it.
        // ToBeImproved:  need to groundARuleNodeFromItsForwardState when choose which rule to apply, to avoid the rules that will cause a lot of effect on the already solved states
        groundARuleNodeFromItsForwardState(ruleNode, curStateNode);

        // And then is possible to still have some variables cannot be grounded by just copy from the forward state
        // So we need to select suitable variables to ground them.
        groundARuleNodeBySelectingNonNumericValues(ruleNode);

        ruleNode->updateCurrentAllBindings();

        // ToBeImproved: currently it can only solve the numeric state with only one ungrounded Numeric variable
        selectValueForGroundingNumericState(ruleNode->originalRule,ruleNode->currentAllBindings,ruleNode);

        recordOrginalStateValuesAfterGroundARule(ruleNode);

        //  find if there are other previous states besides curStateNode will be affected by this rule
        vector<EffectPair>::iterator effectItor;

        for (effectItor = ruleNode->originalRule->effectList.begin(); effectItor != ruleNode->originalRule->effectList.end(); ++ effectItor)
        {
            Effect* e = (Effect*)(((EffectPair)(*effectItor)).second);

            State* effState =  Rule::groundAStateByRuleParamMap(e->state, ruleNode->currentAllBindings);

            // skip the current state node
            if (curStateNode->state->isSameState(*effState) )
                continue;

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
                        ruleNode->forwardLinks.insert(unsatStateNode);
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
                            unsatisfiedStateNodes.push_front(satStateNode);
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
            for (goIt = goalStateNodes.begin(); goIt != goalStateNodes.end();)
            {
                StateNode* goalNode = (StateNode*)(*goIt);
                if (effState->isSameState( *(goalNode->state) ))
                {
                    float satDegree;

                    if (! effState->isSatisfied(*(goalNode->state) ,satDegree))
                    {                        
                        // This effect dissatisfied this goal, put this goal into the unsatisfied state node list and remove it from goal list
                        unsatisfiedStateNodes.push_front(goalNode);
                        goalStateNodes.erase(goIt);
                    }
                    else
                         ++ goIt;
                }
                else
                     ++ goIt;
            }

        }

        // move the current state node from unsatisfied to satisfied list
        temporaryStateNodes.push_front(curStateNode);
        unsatisfiedStateNodes.pop_back();

        // ground all the precondition state and create new nodes for each and add the forwardEffectStates
        // and then check each precondition if it has already been satisfied, put it the unsatisfied one to the
        vector<State*>::iterator itpre;
        float satisfiedDegree;
        for (itpre = ruleNode->originalRule->preconditionList.begin(); itpre != ruleNode->originalRule->preconditionList.end(); ++ itpre)
        {
            State* ps = *itpre;
            State* groundPs = Rule::groundAStateByRuleParamMap(ps, ruleNode->currentAllBindings);

            // create a new state node for this grounded precondition state
            StateNode* newStateNode = new StateNode(groundPs);
            newStateNode->forwardRuleNode = ruleNode;
            newStateNode->backwardRuleNode = 0;
            newStateNode->forwardEffectState = ps;

            // first check if this state has beed satisfied by the previous state nodes
            bool found = false;
            StateNode* satStateNode;

            bool satByTemp = checkIfThisGoalIsSatisfiedByTempStates(*groundPs, found, satStateNode);

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
                        satStateNode->backwardRuleNode->forwardLinks.insert(newStateNode);
                        newStateNode->backwardRuleNode =  satStateNode->backwardRuleNode;
                    }

                    // ToBeImproved: Need to check if there is any dependency loop,
                    // if any precondiction in the backward branch depends on any states in the forward branch of current rule node,
                    // it means two branches depend on each other. How to deal with this case?

                    // not need to add it in the temporaryStateNodes list, because it has beed satisfied by the previous steps

                }
                else
                {
                    // add it to unsatisfied list
                    unsatisfiedStateNodes.push_front(newStateNode);
                    if (newStateNode->backwardRuleNode)
                        newStateNode->backwardRuleNode->negativeForwardLinks.insert(newStateNode);

                }
            }
            else
            {
                // cannot find this state in the temporaryStateNodes list, need to check it in real time
                // check real time
                if (! checkIsGoalAchievedInRealTime(*groundPs,satisfiedDegree))
                {
                    // add it to unsatisfied list
                    unsatisfiedStateNodes.push_front(newStateNode);

                }

            }

        }

        // execute the current rule action to change the imaginary SpaceMap if any action that involved changing space map
        executeActionInImaginarySpaceMap(ruleNode,clonedMap);

    }

    // finished planning!
    // todo: generate the action series according to the planning network we have constructed in this planning process

    // Reset the spaceMap for inquery back to the real spaceMap
    Inquery::reSetSpaceMap();

    // todo: remove all the imaginary atoms in imaginaryHandles

    return true;
}


// ToBeImproved: this function is very ugly...the right way is to add a callback function for each action to auto execute
void OCPlanner::executeActionInImaginarySpaceMap(RuleNode* ruleNode,SpaceServer::SpaceMap* iSpaceMap)
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
            iSpaceMap->updateNoneBLockEntityLocation(agentH0,targetLocation,curtimeStamp);
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
            iSpaceMap->updateNoneBLockEntityLocation(agentH,newPos,curtimeStamp);

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

    }

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
            Vector movedToVec0 = boost::get<Vector>((ruleNode->orginalGroundedStateValues)[1]);
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
            Vector movedToVec = boost::get<Vector>((ruleNode->orginalGroundedStateValues)[1]);
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
    set<StateNode*>::iterator forwardStateIt;
    for (forwardStateIt = ruleNode->forwardLinks.begin(); forwardStateIt != ruleNode->forwardLinks.end(); ++ forwardStateIt)
    {
        StateNode* curStateNode = (StateNode*)(*forwardStateIt);
        if (curStateNode == forwardStateNode)
            continue;

        deleteStateNodeInTemporaryList(curStateNode);

        if (curStateNode->forwardRuleNode != 0)
            unsatisfiedStateNodes.push_front(curStateNode);
    }

    if (forwardStateNode && deleteThisforwardStateNode)
        delete forwardStateNode;

    // check all its backward state nodes
    // delete them recursively
    set<StateNode*>::iterator backwardStateIt;
    for (backwardStateIt = ruleNode->backwardLinks.begin(); backwardStateIt != ruleNode->backwardLinks.end(); ++ backwardStateIt)
    {
        StateNode* curStateNode = (StateNode*)(*backwardStateIt);

        deleteStateNodeInTemporaryList(curStateNode);

         if (curStateNode->backwardRuleNode != 0)
             deleteRuleNodeRecursively(curStateNode->backwardRuleNode, curStateNode);
         else
             delete curStateNode;

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

// the forwardState can be a ungrounded state from other rule
// if cannot unify it , return 0
Rule* OCPlanner::unifyRuleVariableName(Rule* toBeUnifiedRule, State* forwardState )
{
    // first, find the state in the effect list of this rule which the same to this forward state
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    // Todo:  need to check if all the non-variables/consts are the same to find the exact state rather than just check the state name
    for (effectIt = toBeUnifiedRule->effectList.begin(); effectIt != toBeUnifiedRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;

        s = e->state;
        if (s->name() ==  forwardState->name())
            break;
    }

    if (effectIt == toBeUnifiedRule->effectList.end())
        return false;

    // check if all the stateOwner parameters grounded
    vector<StateValue>::iterator f_ownerIt = forwardState->stateOwnerList.begin(); // state owner list in forward state
    vector<StateValue>::iterator r_ownerIt = s->stateOwnerList.begin(); // state owner list in rule effect state

    map<string, StateValue> currentBindingsFromForwardState;

    for ( ; r_ownerIt != s->stateOwnerList.end(); ++ f_ownerIt, ++r_ownerIt)
    {
        if (Rule::isParamValueUnGrounded(*r_ownerIt))
        {
            string variableName = StateVariable::ParamValueToString((StateValue)(*r_ownerIt));
            map<string, StateValue>::iterator paraIt = currentBindingsFromForwardState.find(variableName);
            if (paraIt == currentBindingsFromForwardState.end())
                currentBindingsFromForwardState.insert(std::pair<string, StateValue>(variableName,*f_ownerIt));
        }
    }

    // unify actor
    StateValue actor;
    if (Rule::isParamValueUnGrounded(toBeUnifiedRule->actor))
    {
        string variableName = StateVariable::ParamValueToString(toBeUnifiedRule->actor);
        map<string, StateValue>::iterator paraIt = currentBindingsFromForwardState.find(variableName);
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
        State* unifiedState = Rule::groundAStateByRuleParamMap(s,currentBindingsFromForwardState);
        if (unifiedState == 0)
            return 0;

        // unify effect operator value
        StateValue opValue;
        if (Rule::isParamValueUnGrounded(e->opStateValue))
        {
            string variableName = StateVariable::ParamValueToString(toBeUnifiedRule->actor);
            map<string, StateValue>::iterator paraIt = currentBindingsFromForwardState.find(variableName);
            if (paraIt == currentBindingsFromForwardState.end())
                opValue = e->opStateValue;
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
        State* unifiedState = Rule::groundAStateByRuleParamMap(bs.goalState,currentBindingsFromForwardState);
        if (unifiedState == 0)
            return 0;

        BestNumericVariableInqueryStruct unifiedBS;
        unifiedBS.bestNumericVariableInqueryFun = bs.bestNumericVariableInqueryFun;
        unifiedBS.goalState = unifiedState;

        string varName;
        map<string, StateValue>::iterator paraIt = currentBindingsFromForwardState.find(beIt->first);
        if (paraIt == currentBindingsFromForwardState.end())
            varName = beIt->first;
        else
            varName = StateVariable::ParamValueToString(paraIt->second);

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

bool OCPlanner::groundARuleNodeFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode)
{
    // first, find the state in the effect list of this rule which the same to this forward state
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    // Todo:  need to check if all the non-variables/consts are the same to find the exact state rather than just check the state name
    for (effectIt = ruleNode->originalRule->effectList.begin(); effectIt != ruleNode->originalRule->effectList.end(); ++ effectIt)
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
            map<string, StateValue>::iterator paraIt = ruleNode->currentBindingsFromForwardState.find(variableName);
            if (paraIt == ruleNode->currentBindingsFromForwardState.end())
                ruleNode->currentBindingsFromForwardState.insert(std::pair<string, StateValue>(variableName,*f_ownerIt));
        }
    }

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
                vector<StateValue>::iterator ownerIt;
                for (ownerIt = forward_cost_state->stateOwnerList.begin(); ownerIt != forward_cost_state->stateOwnerList.end(); ++ ownerIt)
                {
                    // if this state_owner is a const, not a variable, just copy it
                    if (! Rule::isParamValueUnGrounded(*ownerIt))
                        cost_state->addOwner(*ownerIt);
                    else
                    {
                        // this state_owner is a variable, we need to change its variable name into the corresponding variable name in current rule
                        // e.g.: the forward rule is: MoveTo(pos1,pos2), cost is :Distance(pos1,pos2).
                        //       But the current rule variables are different: If ExistAPath(x,m) & ExistAPath(m,y), then ExistAPath(x,y)
                        //       so we need to make the cost in current rule like: Distance(x,m)+ Distance(m,y), using the variables x,m,y, rather than pos1, pos2

                        vector<StateValue>::iterator f_rule_ownerIt = forwardStateNode->forwardEffectState->stateOwnerList.begin();
                        vector<StateValue>::iterator cur_ownerIt = ((State*)(*itpre))->stateOwnerList.begin();
                        // This two state should be the same state just with possible different variable names
                        // So the state owners in the same order of bot stateOwnerLists should suggest the same usage
                        for ( ; f_rule_ownerIt != forwardStateNode->forwardEffectState->stateOwnerList.end(); ++ f_rule_ownerIt, ++ cur_ownerIt)
                        {
                            // need to find the state owner of this cost heuristic in the forward effect state
                            if ((*f_rule_ownerIt) == (*ownerIt))
                            {
                                // assign the state owner (variable name) in the same position of current rule precondition to the new cost_state we creat
                                cost_state->addOwner(*cur_ownerIt);

                                break;
                            }
                        }

                        // If cannot find this state owner of this cost heuristic in the forward effect state, it means this owner doesn't affect the calculation of the cost in backward rule
                        // So in this case, we just bind this state owner in the backward cost_cal_state as the binded value in the forward rule node
                        if (f_rule_ownerIt == forwardStateNode->forwardEffectState->stateOwnerList.end())
                        {
                            // find the grounded value of this variable
                            ParamGroundedMapInARule::iterator bindIt = forwardRuleNode->currentBindingsFromForwardState.find(StateVariable::ParamValueToString((StateValue)(*ownerIt)));
                            OC_ASSERT(!(bindIt == forwardRuleNode->currentBindingsFromForwardState.end()),
                                      "OCPlanner::groundARuleNodeFromItsForwardState: Cannot find the binding of this variable:\n",
                                      StateVariable::ParamValueToString((StateValue)(*ownerIt)).c_str());

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
    ParamGroundedMapInARule::iterator bindIt = ruleNode->currentBindingsFromForwardState.begin();
    for( ; paraIt != ruleNode->originalRule->paraIndexMap.end(); ++ paraIt)
    {
        // try to find this variable name in the currentBindings
        bindIt = ruleNode->currentBindingsFromForwardState.find(paraIt->first);

        // if cannot find it, it means this variable remains ungrounded, add it into the curUngroundedVariables
        if (bindIt == ruleNode->currentBindingsFromForwardState.end())
        {
            bool is_numeric_var = opencog::oac::isAVariableNumeric(paraIt->first);

            vector<paramIndex>::iterator indexIt = (paraIt->second).begin();
            for (; indexIt != (paraIt->second).end(); ++ indexIt)
            {
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

            if (candidateListHandles.size() != 0)
            {
                foreach (Handle listH, candidateListHandles)
                {
                    HandleSeq candidateHandlesInOneGroup = atomSpace->getOutgoing(listH);
                    ParamGroundedMapInARule oneGroupCandidate;
                    int index = 0;

                    foreach (Handle h, candidateHandlesInOneGroup)
                    {
                        StateValue v = Inquery::getStateValueFromHandle(varNames[index],h);
                        oneGroupCandidate.insert(std::pair<string, StateValue>(varNames[index],v));

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
                                if (checkIfThisGoalIsSatisfiedByTempStates(*groundedState,found,satStateNode))
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

    // Till now,  all the easy states have been dealed with, now we need to deal with numberic states if any
    // we won't ground the numeric states here, because it's too time-consuming,
    // we won't give all the possible combinations of numeric values and non-numeric values for candidates

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
        // find in the currentAllBindings to check if this variable has been grounded or not
        if (currentbindings.find(beIt->first) != currentbindings.end())
            continue;

        break;
    }

    if (beIt == rule->bestNumericVariableinqueryStateFuns.end())
        return false;

    // this variable has not been grouned , call its BestNumericVariableInquery to ground it
    // first, ground the state required by bestNumericVariableinqueryStateFun
    BestNumericVariableInqueryStruct& bs = (BestNumericVariableInqueryStruct&)(beIt->second);
    State* groundedState = Rule::groundAStateByRuleParamMap( bs.goalState,currentbindings);
    if (groundedState == 0)
    {
        // todo: Currently we cannot solve such problem that this state cannot be grouded by previous grouding steps
        logger().error("OCPlanner::selectValueForGroundingNumericState :the goal state needed in BestNumericVariableInquery cannot be grouned in previous grouding steps. State name is:,"
                       +  bs.goalState->name() );
        return false;
    }

    vector<StateValue> values = bs.bestNumericVariableInqueryFun(groundedState->stateOwnerList);

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

        StateNode* effectStateNode = *(ruleNode->forwardLinks.begin());

        // find the first unrecursive rule
        // you can also find it by ruleEffectIndexes
        // ToBeImproved: currently we only borrow from the first unrecursive rule found
        list< pair<float,Rule*> >::iterator canIt;
        Rule* unrecursiveRule = 0;

        for (canIt = effectStateNode->candidateRules.begin(); canIt != effectStateNode->candidateRules.end(); ++ canIt)
        {
            Rule* r = canIt->second;
            if (! r->IsRecursiveRule)
            {
                unrecursiveRule = r;
                break;
            }
        }

        if (unrecursiveRule == 0)
            return false; // cannot find a unrecursiveRule to borrow from

        // ground this unrecursiveRule by the effectStateNode
        Effect* e = rule->effectList.begin()->second;

        Rule* borrowedRule =  unifyRuleVariableName(unrecursiveRule, e->state);
        if (borrowedRule == 0)
            return false; // cannot unify the borrowed rule

        // because we have unified the rules, so the borrowed rule can use the same grounding map with
        if (! selectValueForGroundingNumericState(borrowedRule,currentbindings))
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

    //  select the best one in this vector<StateValue> that get the highest score according to the heuristics cost calculations
    StateValue bestValue;

    if (rule->CostHeuristics.size() != 0)
    {
        bestValue = selectBestNumericValueFromCandidates(rule->basic_cost, rule->CostHeuristics,currentbindings, beIt->first,values);
    }
    else if (ruleNode && (ruleNode->costHeuristics.size()!= 0) )
    {
        bestValue = selectBestNumericValueFromCandidates(0.0f, ruleNode->costHeuristics,currentbindings, beIt->first,values);
    }
    else
    {
        logger().error("OCPlanner::selectValueForGroundingNumericState: this rule doesn't have any costHeuristics for calculation!" );
        return false;
    }


    if (bestValue == UNDEFINED_VALUE)
    {
        logger().error("OCPlanner::selectValueForGroundingNumericState: failed to find the best value for grounding numeric state!" );
        return false;
    }

    //  ground the rule fully first
    currentbindings.insert(std::pair<string, StateValue>(beIt->first,bestValue));
    return true;


}


StateValue OCPlanner::selectBestNumericValueFromCandidates(float basic_cost, vector<CostHeuristic>& costHeuristics, ParamGroundedMapInARule& currentbindings, string varName, vector<StateValue>& values)
{

    vector<StateValue>::iterator vit;
    float lowestcost = 999999.9;
    StateValue bestValue = UNDEFINED_VALUE;

    for (vit = values.begin(); vit != values.end(); ++ vit)
    {
        // try to ground the rule fully first
        currentbindings.insert(std::pair<string, StateValue>(varName,*vit));

        float cost = Rule::getCost(basic_cost, costHeuristics, currentbindings);

        if (cost < -0.00001f)
        {
            logger().error("OCPlanner::selectBestNumericValueFromCandidates: this rule has not been grounded fully!" );
            return UNDEFINED_VALUE;
        }

        currentbindings.erase(varName);

        if (cost < lowestcost)
        {
            lowestcost = cost;
            bestValue = *vit;
        }
    }

    return bestValue;
}

// this function should be called after completely finished grounding a rule
void OCPlanner::recordOrginalStateValuesAfterGroundARule(RuleNode* ruleNode)
{
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    ruleNode->orginalGroundedStateValues.clear();

    for (effectIt = ruleNode->originalRule->effectList.begin(); effectIt != ruleNode->originalRule->effectList.end(); ++ effectIt)
    {
        e = effectIt->second;
        s = e->state;
        State* groundedState = Rule::groundAStateByRuleParamMap(s,ruleNode->currentAllBindings);
        ruleNode->orginalGroundedStateValues.push_back(groundedState->getStateValue());

    }
}


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
                                        var_food));

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
    existPathStateOwnerList6.push_back(var_pos_3);
    State* existPathState6 = new State("existPath",StateValuleType::BOOLEAN(),STATE_EQUAL_TO ,var_exist_path, existPathStateOwnerList6, true, &Inquery::inqueryExistPath);
    Effect* becomeExistPathEffect2 = new Effect(existPathState6, OP_ASSIGN, "true");

    // add rule:
    Rule* pathTransmitRule = new Rule(doNothingAction,boost::get<Entity>(varAvatar),0.0f);
    pathTransmitRule->addPrecondition(existPathState4);
    pathTransmitRule->addPrecondition(existPathState5);

    BestNumericVariableInqueryStruct bs;
    bs.bestNumericVariableInqueryFun = &Inquery::inqueryNearestAccessiblePosition;
    bs.goalState = existPathState6;
    pathTransmitRule->bestNumericVariableinqueryStateFuns.insert(map<string,BestNumericVariableInqueryStruct>::value_type(StateVariable::ParamValueToString(var_pos_2), bs));

    pathTransmitRule->addEffect(EffectPair(1.0f,becomeExistPathEffect2));

    this->AllRules.push_back(pathTransmitRule);

    //----------------------------End Rule: if there exist a path from pos1 to pos2, and also exist a path from pos2 to pos3, then there should exist a path from pos1 to pos3---


}




