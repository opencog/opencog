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

    // has not been found in the globalStatesCache,
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
//    globalStatesCache.clear();

    int ruleNodeCount = 0;

    // clone a spaceMap for image all the the steps happen in the spaceMap, like building a block in some postion.
    // Cuz it only happens in imagination, not really happen, we should not really change in the real spaceMap
    SpaceServer::SpaceMap* clonedMap = spaceServer().cloneTheLatestSpaceMap();

    // Set this cloned spaceMap for Inquery
    Inquery::setSpaceMap(clonedMap);

    // we use the basic idea of the graph planner for plan searching:
    // alternated state layers with action layers
    // But we use backward depth-first chaining, instead of forward breadth-frist reasoning
    // Because our embodiment game world is not a simple finite boolean-state world, we cannot use a full forward breadth-frist which will be too slowly

    // Firstly, we construct the original unsatisfiedStateNodes and temporaryStateNodes from the input goals
    list<StateNode*> unsatisfiedStateNodes; // all the state nodes that have not been satisfied till current planning step

    // all the state nodes that store the temporary state nodes generated by effect of some rule nodes,
    // all the new elements should be put_front , so the latest update of the same state will already put in front of the older history
    list<StateNode*> temporaryStateNodes;

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

        if (checkIsGoalAchieved(*(newStateNode->state), satisfiedDegree))
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

        // decide which state should be chosed to achieved first
        list<StateNode*>::iterator stateNodeIter;
        for (stateNodeIter = unsatisfiedStateNodes->nodes.begin(); stateNodeIter != unsatisfiedStateNodes->nodes.end();++stateNodeIter)
        {
            (StateNode*)(*stateNodeIter)->calculateNodeDepth();
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
                curStateNode->ruleHistory.push_back(curStateNode->candidateRules.front());
                curStateNode->candidateRules.pop_front();


            }

            curStateNode->hasFoundCandidateRules = true;
        }
        else //  we have  tried to achieve this state node before,which suggests we have found all the candidate rules
        {

            // check if there is any rule left not been tried in the candidate rules
            if (curStateNode->candidateRules.size() != 0)
            {
                selectedRule = curStateNode->candidateRules.front();
                curStateNode->ruleHistory.push_back(curStateNode->candidateRules.front());
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

                        temporaryStateNodes.erase(*forwardStateIt);
                        unsatisfiedStateNodes.insert(*forwardStateIt);
                    }

                    deleteRuleNode(forwardRuleNode);

                }
                else
                {
                    // check which states of the effects of this forwardRuleNode have been sovled , which still remand unsloved.
                    set<StateNode*>::iterator effectItor;
                    map<State*,StateNode*> solvedStateNodes; // all the state nodes in forwardRuleNode's effects that have been solved by previous planning steps
                    for (effectItor = forwardRuleNode->backwardLinks.begin(); effectItor != forwardRuleNode->backwardLinks.end(); ++ effectItor)
                    {
                        // skip the current state node
                        if ((*effectItor) == curStateNode)
                            continue;

                        if ((*effectItor)->backwardRuleNode != 0)
                        {
                            // currently , this state node has already been tried a backward rule to solve it
                            // so put it in the solvedStateNodes map
                            StateNode* sn = (StateNode*)(*effectItor);
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
                        forwardRuleNode->ParamCandidates.erase(ParamCandidates.begin());

                        std::set_union(forwardRuleNode->currentBindingsFromForwardState.begin(),forwardRuleNode->currentBindingsFromForwardState.end(),
                                       forwardRuleNode->currentBindingsViaSelecting.begin(),forwardRuleNode->currentBindingsViaSelecting.end(),
                                       inserter(forwardRuleNode->currentAllBindings,forwardRuleNode->currentAllBindings.begin()));


                        // rebind all the effect state nodes in the forward rule
                        for (effectItor = forwardRuleNode->backwardLinks.begin(); effectItor != forwardRuleNode->backwardLinks.end(); ++ effectItor)
                        {
                            reBindStateNode((*effectItor),forwardRuleNode->currentAllBindings);
                        }

                        continue;
                    }
                    else // some of the effect states of this forwardRuleNode has been solved, so we cannot simply replace the current bindings with another random bindings
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
                                solvedItor = solvedStateNodes.find(UngroundedVariablesInAState.state);
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
                        std::set_union(forwardRuleNode->currentBindingsFromForwardState.begin(),forwardRuleNode->currentBindingsFromForwardState.end(),
                                       forwardRuleNode->currentBindingsViaSelecting.begin(),forwardRuleNode->currentBindingsViaSelecting.end(),
                                       inserter(forwardRuleNode->currentAllBindings,forwardRuleNode->currentAllBindings.begin()));

                        // have to rebind the affected state nodes and delete the affected effect states' branches
                        vector<StateNode*>::iterator affectedStateNodeIt;
                        for ( affectedStateNodeIt =  (willBeAffecteds[index]).begin(); affectedStateNodeIt !=  (willBeAffecteds[index]).end(); ++ affectedStateNodeIt)
                        {
                            StateNode* affectedStateNode = (StateNode* )(*affectedStateNodeIt);
                            reBindStateNode(affectedStateNode,forwardRuleNode->currentAllBindings);

                            temporaryStateNodes.erase(affectedStateNode);
                            unsatisfiedStateNodes.insert(affectedStateNode);

                            deleteRuleNode(affectedStateNode->backwardRuleNode);
                        }

                        continue;

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
        groundARuleNodeBySelectingValues(ruleNode);

        std::set_union(ruleNode->currentBindingsFromForwardState.begin(),ruleNode->currentBindingsFromForwardState.end(),
                       ruleNode->currentBindingsViaSelecting.begin(),ruleNode->currentBindingsViaSelecting.end(),
                       inserter(ruleNode->currentAllBindings,ruleNode->currentAllBindings.begin()));

        //  find if there are other forward states besides curStateNode will be affected by this rule
        set<StateNode*>::iterator effectItor;

        for (effectItor = forwardRuleNode->backwardLinks.begin(); effectItor != forwardRuleNode->backwardLinks.end(); ++ effectItor)
        {
            // skip the current state node
            if ((*effectItor) == curStateNode)
                continue;


            State* effState =  Rule::groundAStateByRuleParamMap(((StateNode*)(*effectItor))->state, ruleNode->currentAllBindings);

            // create a new state node for it
            StateNode* newStateNode = new StateNode(effState);
            newStateNode->backwardRuleNode = ruleNode;
            newStateNode->forwardRuleNode = 0;
            newStateNode->forwardEffectState = 0;
            temporaryStateNodes.push_front(newStateNode);

            list<StateNode*>::iterator unsait;

            for (unsait = unsatisfiedStateNodes.begin(); unsait != unsatisfiedStateNodes.end(); )
            {
                if (effState->isSameState( ((StateNode*)(*unsait))->state ))
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

            // need to check in the goal state node list, if this effect change the already satisfied states in the goal list
            vector<StateNode*>::iterator goIt;
            for (goIt = goalStateNodes.begin(); goIt != goalStateNodes.end();)
            {
                StateNode* goalNode = (StateNode*)(*goIt);
                if (effState->isSameState( goalNode->state ))
                {
                    float satDegree;

                    if (! effState->isSatisfied(goalNode->state ,satDegree))
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

        // ground all the precondition state nodes and add the forwardEffectStates
        vector<State*>::iterator itpre;
        float satisfiedDegree;
        for (itpre = preconditionList.begin(); itpre != preconditionList.end(); ++ itpre)
        {
            State* ps = *itpre;
            State* groundPs = Rule::groundAStateByRuleParamMap(ps, ruleNode->currentAllBindings);

            // create a new state node for this grounded precondition state
            StateNode* newStateNode = new StateNode(groundPs);
            newStateNode->forwardRuleNode = ruleNode;
            newStateNode->backwardRuleNode = 0;
            newStateNode->forwardEffectState = ?;

            // check if this state has beed satisfied by the previous state nodes
            list<StateNode*>::const_iterator vit = temporaryStateNodes.begin();
            bool found = false;
            for (;vit != temporaryStateNodes.end(); vit ++)
            {
                // there are possible mutiple same state nodes describe this same state in temporaryStateNodes,
                // but the latest one is put in the front, so we can just check the first one we find
                State* vState = ((StateNode*)(*vit))->state;
                if (vState->isSameState(groundPs))
                {
                     if (vState->isSatisfied(groundPs,satisfiedDegree))
                     {
                         // the current state satisfied this precondition,
                         // connect this precondition to the backward rule node of the state which satisfied this precondition
                         ((StateNode*)(*vit))->backwardRuleNode->forwardLinks.insert(newStateNode);
                         newStateNode->backwardRuleNode =  ((StateNode*)(*vit))->backwardRuleNode;

                         found = true;

                         // not need to add it in the temporaryStateNodes list, because it has beed satisfied by the previous steps
                     }

                }

            }

            if (! found)
            {
                // check real time

                // to do add in unsatisfied list
            }

        }




        // Todo: put all the new unsatisfied states into the current


        // Todo: to execute the rules to change the imaginary SpaceMap if any

    }

    // finished planning!
    // todo: generate the action series according to the planning network we have constructed in this planning process

    // Reset the spaceMap for inquery back to the real spaceMap
    Inquery::reSetSpaceMap();

    return true;
}
void OCPlanner::reBindStateNode(StateNode* stateNode, ParamGroundedMapInARule& newBindings)
{
    State* newGroundedState = Rule::groundAStateByRuleParamMap(stateNode->state, newBindings);
    delete stateNode->state;
    stateNode->state = newGroundedState;
}

// delete a rule node and recursivly delete all its backward state nodes and rule nodes
void OCPlanner::deleteRuleNode(RuleNode* ruleNode)
{
    // todo
}

bool OCPlanner::groundARuleNodeFromItsForwardState(RuleNode* ruleNode, StateNode* forwardStateNode)
{
    // first, find the state in the effect list of this rule which the same to this forward state
    vector<EffectPair>::iterator effectIt;
    Effect* e;
    State* s;

    // Todo: Maybe need to check if all the non-variables/consts are the same to find the exact state rather than just check the state name
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
                                              forward_cost_state->stateVariable->getValue(),forward_cost_state->need_inquery,forward_cost_state->inqueryFun);
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


// this function should be called after groundARuleNodeFromItsForwardState
bool OCPlanner::groundARuleNodeBySelectingValues(RuleNode *ruleNode)
{
    // First find all the ungrounded variables
    findAllUngroundedVariablesInARuleNode(ruleNode);

    // If there are multiple variables need to be grounded, the curUngroundedVariables list has already decided the order for grounding

    // First, get all the non-need-real-time-inquery states as the conditions for mattern matching
    // if cannot find any solution, then remove the last condition. Repeat this, till only have one condition left

    // Because the curUngroundedVariables list has already been in order, so we just need to find out the first need-inquery one,
    // and only select variables by pattern matching for the states before

    // find the last state in the list curUngroundedVariables, which needs no real-time inquery state , and not numeric
    int number_easy_state = -1;
    list<UngroundedVariablesInAState>::iterator uvIt= ruleNode->curUngroundedVariables.begin();
    for (; uvIt != ruleNode->curUngroundedVariables.end(); ++ uvIt)
    {
        number_easy_state ++;
        if (((((UngroundedVariablesInAState&)(*uvIt)).state)->need_inquery )
                || ((UngroundedVariablesInAState&)(*uvIt)).contain_numeric_var)
            break;
        else
        {
            // generate all the evaluationlink will be used to do pattern matching for each easy state
            ((UngroundedVariablesInAState&)(*uvIt)).PMLink = Inquery::generatePMLinkFromAState((((UngroundedVariablesInAState&)(*uvIt)).state), ruleNode);
        }
    }

    // TODO: Is it possible that some non-need-real-time-inquery states contains some variables that need to be grounded by other need_real-time-inquery states?
    // currentBindingsViaSelecting
    // find all the canditates meet as many as possible preconditions
    // first try all the combinations of  these in the
    int tryTotalStateNum = number_easy_state;

    while (tryTotalStateNum > 0)
    {
        -- tryTotalStateNum;
    }

}

bool OCPlanner::selectValueForAVariableToGroundARule(RuleNode* ruleNode, string variableStr)
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


}




