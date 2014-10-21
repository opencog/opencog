/*
 * opencog/learning/PatternMiner/PatternMinerDF.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
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

#include <math.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>
#include <sstream>
#include <thread>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/query/BindLink.h>
#include <opencog/util/Config.h>
#include <opencog/util/foreach.h>
#include <opencog/util/StringManipulator.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


void PatternMiner::growPatternsDepthFirstTask()
{
    while(true)
    {
        readNextLinkLock.lock();
        cur_index ++;

        if (cur_index < allLinkNumber)
        {
            cout<< "\r" << (float)(cur_index)/(float)(allLinkNumber)*100.0f << + "% completed." ;
            std::cout.flush();
        }
        else
        {
            if (cur_index == allLinkNumber)
            {
                cout<< "\r100% completed." ;
                std::cout.flush();
            }

            readNextLinkLock.unlock();
            break;

        }

        Handle& cur_link = allLinks[cur_index];

        readNextLinkLock.unlock();

        // if this link is listlink, ignore it
        if (originalAtomSpace->getType(cur_link) == opencog::LIST_LINK)
        {
            continue;
        }

        // Add this link into observingAtomSpace
        HandleSeq outgoingLinks,outVariableNodes;

        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
        Handle newLink = observingAtomSpace->addLink(originalAtomSpace->getType(cur_link),outgoingLinks,originalAtomSpace->getTV(cur_link));

        HandleSeq observedLinks;
        observedLinks.push_back(newLink);

        // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns

        vector<HTreeNode*> _allLastGramHTreeNodes; // it's empty for the 1-gram patterns, because there is no last gram

        vector<HTreeNode*> _allThisGramHTreeNodes;

        extractAllPossiblePatternsFromInputLinksDF(observedLinks, 0, observingAtomSpace, _allLastGramHTreeNodes,_allThisGramHTreeNodes,1);

        map<HandleSeq, vector<HTreeNode*> > allLastGramLinksToPatterns; // for this cur_link
        allLastGramLinksToPatterns.insert(std::pair<HandleSeq, vector<HTreeNode*>>(observedLinks, _allThisGramHTreeNodes));

        unsigned int gram;

        for ( gram = 2; gram <= MAX_GRAM; ++ gram)
        {
            map<HandleSeq, vector<HTreeNode*> > ::iterator it = allLastGramLinksToPatterns.begin();
            map<HandleSeq, vector<HTreeNode*> > allThisGramLinksToPatterns;
            vector<set<Handle>> newConnectedLinksFoundThisGram;

            for(; it != allLastGramLinksToPatterns.end(); ++ it)
            {
                // find all the 2~MAX_GRAM gram distance neighbour links of newLink
                extendAllPossiblePatternsForOneMoreGramDF((HandleSeq&)(it->first),observingAtomSpace,gram, (vector<HTreeNode*>&)(it->second), allThisGramLinksToPatterns, newConnectedLinksFoundThisGram);
            }

            allLastGramLinksToPatterns = allThisGramLinksToPatterns;

        }

    }
}


void PatternMiner::runPatternMinerDepthFirst()
{
    // observingAtomSpace is used to copy one link everytime from the originalAtomSpace
    observingAtomSpace = new AtomSpace();

    cur_index = -1;

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i] = std::thread([this]{this->growPatternsDepthFirstTask();}); // using C++11 lambda-expression
    }

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i].join();
    }

    // release allLinks
    allLinks.clear();
    (vector<Handle>()).swap(allLinks);

    cout << "\nFinished mining 1~" << MAX_GRAM << " gram patterns.\n";

}

// allLastGramHTreeNodes is input, allFactLinksToPatterns is output - the links fact to all its pattern HTreeNodes
void PatternMiner::extendAllPossiblePatternsForOneMoreGramDF(HandleSeq &instance, AtomSpace* _fromAtomSpace, unsigned int gram,
     vector<HTreeNode*>& allLastGramHTreeNodes, map<HandleSeq, vector<HTreeNode*> >& allFactLinksToPatterns, vector<set<Handle>>& newConnectedLinksFoundThisGram)
{

    // First, extract all the variable nodes in the instance links
    // we only extend one more link on the nodes that are considered as varaibles for this pattern
    map<Handle, unsigned int> allVarNodes;

    unsigned int index = 0;
    foreach (Handle link, instance)
    {
        extractAllNodesInLink(link, allVarNodes, _fromAtomSpace, index);
        index ++;
    }

    map<Handle, unsigned int>::iterator varIt;

    for(varIt = allVarNodes.begin(); varIt != allVarNodes.end(); ++ varIt)
    {
        Handle extendNode = (Handle)(varIt->first);
        if (enable_filter_node_types_should_not_be_vars)
        {
            bool isIgnoredType = false;
            Type t = _fromAtomSpace->getType(extendNode);
            foreach (Type noType, node_types_should_not_be_vars)
            {
                if (t == noType)
                {
                    isIgnoredType = true;
                    break;
                }
            }

            if (isIgnoredType )
                continue;
        }

        // find what are the other links in the original Atomspace contain this variable
        HandleSeq incomings = _fromAtomSpace->getIncoming( (extendNode));
        // debug
        // string curvarstr = _fromAtomSpace->atomAsString((Handle)(*varIt));

        foreach(Handle incomingHandle, incomings)
        {
            Handle extendedHandle;
            // if this atom is a igonred type, get its first parent that is not in the igonred types
            if (isIgnoredType (_fromAtomSpace->getType(incomingHandle)) )
            {
                extendedHandle = getFirstNonIgnoredIncomingLink(_fromAtomSpace, incomingHandle);
                if (extendedHandle == Handle::UNDEFINED)
                    continue;
            }
            else
                extendedHandle = incomingHandle;

            // debug
            string extendedHandleStr = _fromAtomSpace->atomAsString(extendedHandle);

            if (isInHandleSeq(extendedHandle, instance))
                continue;

            // cout << "Debug: Extended link :" << extendedHandleStr << std::endl;

            if (extendedHandleStr.find("$var") != std::string::npos)
            {
               // cout << "Debug: error! The extended link contines variables!" << extendedHandleStr << std::endl;
                continue;
            }

            // Add this extendedHandle to the old pattern so as to make a new pattern
            HandleSeq originalLinks = instance;
            originalLinks.push_back(extendedHandle);

            // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns

            vector<set<Handle> >::iterator newExtendIt;
            set<Handle> originalLinksToSet(originalLinks.begin(),originalLinks.end());
            bool alreadyExtracted = false;

            // check if these links have been extracted
            for(newExtendIt = newConnectedLinksFoundThisGram.begin(); newExtendIt != newConnectedLinksFoundThisGram.end(); newExtendIt++)
            {
                set<Handle>& exitstingLinks = (set<Handle>&)(*newExtendIt);
                if (exitstingLinks == originalLinksToSet)
                {
                    alreadyExtracted = true;
                    // cout<< "Debug: these links have already been extracted! Skip them!"<<std::endl;
                    break;
                }

            }

            if (! alreadyExtracted)
            {
                newConnectedLinksFoundThisGram.push_back(originalLinksToSet);
                set<Handle> sharedNodes; // only the current extending shared node is in sharedNodes for Depth first
                sharedNodes.insert(extendNode);
                vector<HTreeNode*> allThisGramHTreeNodes;
                extractAllPossiblePatternsFromInputLinksDF(originalLinks, (unsigned int)(varIt->second), _fromAtomSpace, allLastGramHTreeNodes, allThisGramHTreeNodes, gram);
                allFactLinksToPatterns.insert(std::pair<HandleSeq, vector<HTreeNode*>>(originalLinks, allThisGramHTreeNodes));

            }

        }
    }
}

// Extract all possible patterns from the original Atomspace input links (full Combination), and add to the patternmining Atomspace
// Patterns are in the following format:
//    (InheritanceLink
//       (VariableNode )
//       (ConceptNode "Animal")

//    (InheritanceLink
//       (VariableNode )
//       (VariableNode )

//    (InheritanceLink
//       (VariableNode )
//       (VariableNode )

//    (EvaluationLink (stv 1 1)
//       (PredicateNode "like_food")
//       (ListLink
//          (VariableNode )
//          (ConceptNode "meat")
//       )
//    )
// note: for Breadth first: sharedNodes = parentInstance's sharedNodes + current shared node
//       for Depth first: sharedNodes = current shared node
// sharedNodes have to be variables , should not be const
// sharedLinkIndex is the index in the inputLinks which contains the shared node
void PatternMiner::extractAllPossiblePatternsFromInputLinksDF(vector<Handle>& inputLinks,unsigned int sharedLinkIndex, AtomSpace* _fromAtomSpace,
                                                              vector<HTreeNode*>& allLastGramHTreeNodes, vector<HTreeNode*>& allHTreeNodes, unsigned int gram)
{
    map<Handle,Handle> valueToVarMap;  // the ground value node in the _fromAtomSpace to the variable handle in pattenmining Atomspace

//    // Debug
//    cout << "Extract patterns from these links: \n";
//    foreach (Handle ih, inputLinks)
//    {
//        cout << _fromAtomSpace->atomAsString(ih) << std::endl;
//    }

    // First, extract all the nodes in the input links
    foreach (Handle link, inputLinks)
        extractAllNodesInLink(link, valueToVarMap, _fromAtomSpace);

    // Generate all the possible combinations of all the nodes: all patterns including the 1 ~ n_max variables
    // If there are too many variables in a pattern, it doesn't make much sense, so we litmit the max number of variables to half of the node number

//    OC_ASSERT( (valueToVarMap.size() > 1),
//              "PatternMiner::extractAllPossiblePatternsFromInputLinks: this group of links only has one node: %s!\n",
//               atomSpace->atomAsString(inputLinks[0]).c_str() );

    int n_max = valueToVarMap.size();
    int n_limit= valueToVarMap.size()/2.0f;
    n_limit ++;

    // sometimes there is only one variable in a link, lik:
//    (DuringLink
//      (ConceptNode "dead")
//      (ConceptNode "dead")
//    ) ; [44694]
    if (n_limit == 1)
        n_limit = 2;

    bool* indexes = new bool[n_max]; //  indexes[i]=true means this i is a variable, indexes[i]=false means this i is a const

    // Get all the shared nodes and leaves
    HandleSeqSeq oneOfEachSeqShouldBeVars;
    HandleSeq leaves, shouldNotBeVars;

    filters(inputLinks, oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, _fromAtomSpace);

    // var_num is the number of variables
    for (int var_num = 1;var_num < n_limit; ++ var_num)
    {
        // Use the binary method to generate all combinations:

        // generate the first combination
        for (int i = 0; i < var_num; ++ i)
            indexes[i] = true;

        for (int i = var_num; i <n_max; ++ i)
            indexes[i] = false;

        while (true)
        {
            // construct the pattern for this combination in the PatternMining Atomspace
            // generate the valueToVarMap for this pattern of this combination
            map<Handle,Handle>::iterator iter;

            map<Handle,Handle> patternVarMap;

            bool skip = false;

            unsigned int index = 0;
            for (iter = valueToVarMap.begin(); iter != valueToVarMap.end(); ++ iter)
            {
                if (indexes[index]) // this is considered as a variable, add it into the variable to value map
                    patternVarMap.insert(std::pair<Handle,Handle>(iter->first, iter->second));

                index ++;
            }

            if (enable_filter_links_should_connect_by_vars)
            {

                // check if in this combination, if at least one node in each Seq of oneOfEachSeqShouldBeVars is considered as variable
                bool allSeqContainsVar = true;
                foreach(HandleSeq& oneSharedSeq, oneOfEachSeqShouldBeVars)
                {
                    bool thisSeqContainsVar = false;
                    foreach (Handle& toBeSharedNode, oneSharedSeq)
                    {
                        if (patternVarMap.find(toBeSharedNode) != patternVarMap.end())
                        {
                            thisSeqContainsVar = true;
                            break;
                        }

                    }

                    if (! thisSeqContainsVar)
                    {
                        allSeqContainsVar = false;
                        break;
                    }
                }

                if (! allSeqContainsVar)
                    skip = true;
            }



            if ( (! skip) && (enable_filter_leaves_should_not_be_vars) && (gram > 1) ) // for gram > 1, any leaf should not considered as variable
            {
                foreach (Handle leaf , leaves)
                {
                    if (patternVarMap.find(leaf) != patternVarMap.end())
                    {
                        skip = true;
                        break;
                    }
                }
            }

            if ( (! skip) && (enable_filter_node_types_should_not_be_vars))
            {
                foreach (Handle noTypeNode , shouldNotBeVars)
                {
                    if (patternVarMap.find(noTypeNode) != patternVarMap.end())
                    {
                        skip = true;
                        break;
                    }
                }
            }


            if (! skip)
            {

                HandleSeq pattern, unifiedPattern;

                foreach (Handle link, inputLinks)
                {
                    HandleSeq outgoingLinks;
                    generateALinkByChosenVariables(link, patternVarMap, outgoingLinks, _fromAtomSpace);
                    Handle rebindedLink = atomSpace->addLink(atomSpace->getType(link),outgoingLinks,TruthValue::TRUE_TV());

                    pattern.push_back(rebindedLink);
                }

                // unify the pattern
                unifiedPattern = UnifyPatternOrder(pattern);

                string keyString = unifiedPatternToKeyString(unifiedPattern);

                // next, check if this pattern already exist (need lock)
                HTreeNode* newHTreeNode = 0;
                uniqueKeyLock.lock();

                map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(keyString);

                if (htreeNodeIter == keyStrToHTreeNodeMap.end())
                {
                    newHTreeNode = new HTreeNode();
                    keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(keyString, newHTreeNode));


                    cout << "A new pattern Found:\n"<< keyString << std::endl;

                    newHTreeNode->count = 1;
                    allHTreeNodes.push_back(newHTreeNode);

                }
                else
                {

                    ((HTreeNode*)(htreeNodeIter->second))->count ++;
                    allHTreeNodes.push_back((HTreeNode*)(htreeNodeIter->second));
                    cout << "Unique Key already exists. count ++ !\n\n";

                }

                uniqueKeyLock.unlock();

                // if gram > 1, this pattern is the super pattern of al the lastGramHTreeNodes
                // add ExtendRelations
                if (gram > 1)
                {
                    HTreeNode* superPatternNode;
                    if (newHTreeNode)
                        superPatternNode = newHTreeNode;
                    else
                        superPatternNode = (HTreeNode*)(htreeNodeIter->second);

                    foreach (HTreeNode* lastGramHTreeNode, allLastGramHTreeNodes)
                    {
                        ExtendRelation relation;
                        relation.extendedHTreeNode = superPatternNode;
                        relation.newExtendedLink = pattern[pattern.size()-1];
                        relation.sharedLink = lastGramHTreeNode->pattern[sharedLinkIndex];

                        lastGramHTreeNode->superPatternRelations.push_back(relation);
                    }
                }

                if (newHTreeNode)
                {
                    newHTreeNode->pattern = unifiedPattern;
                    newHTreeNode->var_num = var_num;

                    addNewPatternLock.lock();
                    (patternsForGram[gram-1]).push_back(newHTreeNode);
                    addNewPatternLock.unlock();

                }

            }


            if (isLastNElementsAllTrue(indexes, n_max, var_num))
                break;

            // generate the next combination
            generateNextCombinationGroup(indexes, n_max);
        }
    }

}
