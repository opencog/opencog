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

#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/learning/PatternMiner/types/atom_types.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;



// This file is not maintained anymore. Please use depth first mining.

void PatternMiner::extendAllPossiblePatternsForOneMoreGramBF(HandleSeq &instance, HTreeNode* curHTreeNode, unsigned int gram)
{
    // debug:
    string instanceInst = unifiedPatternToKeyString(instance, originalAtomSpace);
    if (instanceInst.find("PatternVariableNode") != std::string::npos)
    {
        cout << "Debug: error! The instance contines variables!" << instanceInst <<  "Skip it!" << std::endl;
        return;
    }

    // First, extract all the variable nodes in the instance links
    // we only extend one more link on the nodes that are considered as varaibles for this pattern
    HandleSet allVarNodes;

    HandleSeq::iterator patternLinkIt = curHTreeNode->pattern.begin();
    for (Handle link : instance)
    {
        extractAllVariableNodesInAnInstanceLink(link, *(patternLinkIt), allVarNodes);
        patternLinkIt ++;
    }

    HandleSet::iterator varIt;

    for(varIt = allVarNodes.begin(); varIt != allVarNodes.end(); ++ varIt)
    {
        // find what are the other links in the original Atomspace contain this variable
        HandleSeq incomings;
        ((Handle)(*varIt))->getIncomingSet(back_inserter(incomings));

        // debug
        string curvarstr = ((Handle)(*varIt))->toShortString();

        for (Handle incomingHandle : incomings)
        {
            Handle extendedHandle;
            // if this atom is a igonred type, get its first parent that is not in the igonred types
            if (isIgnoredType (incomingHandle->getType()) )
            {
                extendedHandle = getFirstNonIgnoredIncomingLink(originalAtomSpace, incomingHandle);
                if (extendedHandle == Handle::UNDEFINED)
                    continue;
            }
            else
                extendedHandle = incomingHandle;

            // debug
            string extendedHandleStr = extendedHandle->toShortString();

            if (isInHandleSeq(extendedHandle, instance))
                continue;

            if (extendedHandleStr.find("PatternVariableNode") != std::string::npos)
            {
               // cout << "Debug: error! The extended link contines variables!" << extendedHandleStr << std::endl;
                continue;
            }

            // Add this extendedHandle to the old pattern so as to make a new pattern
            HandleSeq originalLinks = instance;
            originalLinks.push_back(extendedHandle);

            // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns
            extractAllPossiblePatternsFromInputLinksBF(originalLinks, curHTreeNode, allVarNodes , gram);

        }
    }
}

void PatternMiner::extractAllPossiblePatternsFromInputLinksBF(HandleSeq& inputLinks,  HTreeNode* parentNode,
                                                              HandleSet& sharedNodes, unsigned int gram)
{
    HandleMap valueToVarMap;  // the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace

//    // Debug
//    cout << "Extract patterns from these links: \n";
//    for (Handle ih : inputLinks)
//    {
//        cout << originalAtomSpace->atomAsString(ih) << std::endl;
//    }

    // First, extract all the nodes in the input links
    for (Handle link : inputLinks)
        extractAllNodesInLink(link, valueToVarMap, originalAtomSpace);

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

    // Find the indexes of the sharedVarNodes. They have to be an variable, not a const.
    int sharedNum = sharedNodes.size();
    int sharedNodesIndexes[sharedNum];

    int sharedCout = 0;

    for (Handle shardNode : sharedNodes)
    {
        sharedNodesIndexes[sharedCout] = -1;
        HandleMap::iterator varIt;
        int j = 0;
        for (varIt = valueToVarMap.begin(); varIt != valueToVarMap.end(); ++varIt, j++)
        {
            if ((varIt->first) == shardNode)
            {
                sharedNodesIndexes[sharedCout] = j;
                break;
            }
        }

        OC_ASSERT( (sharedNodesIndexes[sharedCout] != -1),
                  "PatternMiner::extractAllPossiblePatternsFromInputLinks: cannot find the shared variable!\n");
        sharedCout ++;

    }

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
            HandleMap::iterator iter;

            HandleMap patternVarMap;

            bool sharedAllVar = true;

            if (sharedNum != 0)
            {
                // check if in this combination, all the sharednodes are considered as variables
                for (int k = 0; k < sharedNum; k ++)
                {
                    int sharedIndex = sharedNodesIndexes[k];
                    if (! indexes[sharedIndex])
                        sharedAllVar = false;
                }
            }

            if (sharedAllVar)
            {
                unsigned int index = 0;
                for (iter = valueToVarMap.begin(); iter != valueToVarMap.end(); ++ iter)
                {
                    if (indexes[index]) // this is considered as a variable, add it into the variable to value map
                        patternVarMap.insert(HandlePair(iter->first, iter->second));

                    index ++;
                }

                HandleSeq pattern, unifiedPattern;
                bool hasLinkContainsOnlyVars = false;

                for (Handle link : inputLinks)
                {
                    HandleSeq outgoingLinks;
                    generateALinkByChosenVariables(link, patternVarMap, outgoingLinks, originalAtomSpace);
                    Handle rebindedLink = atomSpace->add_link(link->getType(), outgoingLinks);
                    rebindedLink->setTruthValue(TruthValue::TRUE_TV());
                    if (onlyContainVariableNodes(rebindedLink, atomSpace))
                    {
                        hasLinkContainsOnlyVars = true;
                    }
                    pattern.push_back(rebindedLink);
                }

                // skip the patterns that has links that only contain variable nodes, no const nodes
                if (! hasLinkContainsOnlyVars)
                {

                    // unify the pattern
                    unsigned int unifiedLastLinkIndex;
                    HandleMap orderedVarNameMap;
                    unifiedPattern = UnifyPatternOrder(pattern, unifiedLastLinkIndex, orderedVarNameMap);

                    string keyString = unifiedPatternToKeyString(unifiedPattern);

                    // next, check if this pattern already exist (need lock)
                    HTreeNode* newHTreeNode = nullptr;
                    uniqueKeyLock.lock();

                    map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(keyString);

                    if (htreeNodeIter == keyStrToHTreeNodeMap.end())
                    {
                        newHTreeNode = new HTreeNode();
                        keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(keyString, newHTreeNode));
                    }
                    else
                    {
                        // which means the parent node is also a parent node of the found HTreeNode
                        if (parentNode)
                        {
                            set<HTreeNode*>& parentLinks= ((HTreeNode*)(htreeNodeIter->second))->parentLinks;
                            if (parentLinks.find(parentNode) == parentLinks.end())
                            {
                                parentLinks.insert(parentNode);
                                parentNode->childLinks.insert(((HTreeNode*)(htreeNodeIter->second)));
                            }
                        }
    //                    // debug
    //                    cout << "Unique Key already exists: \n" << keyString << "Skip this pattern!\n\n";
                    }

                    uniqueKeyLock.unlock();

                    if (newHTreeNode)
                    {
                        newHTreeNode->pattern = unifiedPattern;
                        newHTreeNode->var_num = var_num;

                        // Find All Instances in the original AtomSpace For this Pattern
                        findAllInstancesForGivenPatternInNestedAtomSpace(newHTreeNode);

                        if (parentNode)
                        {
                            newHTreeNode->parentLinks.insert(parentNode);
                            parentNode->childLinks.insert(newHTreeNode);
                        }
                        else
                        {
                            newHTreeNode->parentLinks.insert(this->htree->rootNode);
                            this->htree->rootNode->childLinks.insert(newHTreeNode);
                        }

                        addNewPatternLock.lock();
                        (patternsForGram[gram-1]).push_back(newHTreeNode);
                        addNewPatternLock.unlock();
                    }
                }
            }


            if (isLastNElementsAllTrue(indexes, n_max, var_num))
                break;

            // generate the next combination
            generateNextCombinationGroup(indexes, n_max);
        }
    }

}


void PatternMiner::growPatternsTaskBF()
{

    vector<HTreeNode*>& last_gram_patterns = patternsForGram[cur_gram-2];

    unsigned int total = last_gram_patterns.size();

    while(true)
    {

        patternForLastGramLock.lock();

        cur_index ++;
        cout<< "\r" + toString(((float)(cur_index)/last_gram_total_float)*100.0f) + "% completed." ;
        std::cout.flush();

        if (cur_index >= (int)total)
        {
            patternForLastGramLock.unlock();
            break;
        }

        HTreeNode* cur_growing_pattern = last_gram_patterns[cur_index];

        patternForLastGramLock.unlock();

        if(cur_growing_pattern->count < thresholdFrequency)
            continue;

        for (HandleSeq instance  : cur_growing_pattern->instances)
        {
            extendAllPossiblePatternsForOneMoreGramBF(instance, cur_growing_pattern, cur_gram);
        }

        clear_by_swap(cur_growing_pattern->instances);
    }

}

void PatternMiner::growTheFirstGramPatternsTaskBF()
{

    while (true)
    {
        patternForLastGramLock.lock();

        cur_index ++;
        cout<< "\r" + toString(((float)(cur_index)/atomspaceSizeFloat)*100.0f) + "% completed." ;
        std::cout.flush();

        if (cur_index >= (int)allLinks.size())
        {
            patternForLastGramLock.unlock();
            break;
        }

        patternForLastGramLock.unlock();

        Handle cur_link = allLinks[cur_index];

        // if this link is listlink, ignore it
        if (cur_link->getType() == opencog::LIST_LINK)
        {
            continue;
        }

        HandleSeq originalLinks;
        originalLinks.push_back(cur_link);

        // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns
        HandleSet sharedNodes;
        extractAllPossiblePatternsFromInputLinksBF(originalLinks, 0, sharedNodes, 1);

    }

}

bool compareHTreeNodeByFrequency(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeByInteractionInformation(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness(HTreeNode* node1, HTreeNode* node2);

void PatternMiner::ConstructTheFirstGramPatternsBF()
{
    int start_time = time(nullptr);

    std::cout<<"Debug: PatternMiner:  start (gram = 1) pattern mining..." << std::endl;

    cur_gram = 1;
    cur_index = -1;

    originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

    atomspaceSizeFloat = (float)(allLinks.size());

//    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
//    {
//        threads[i] = std::thread([this]{this->growTheFirstGramPatternsTaskBF();}); // using C++11 lambda-expression
//    }

//    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
//    {
//        threads[i].join();
//    }

    growTheFirstGramPatternsTaskBF();

    clear_by_swap(allLinks);

    // sort the patterns by frequency
    std::sort((patternsForGram[0]).begin(), (patternsForGram[0]).end(),compareHTreeNodeByFrequency );

    int end_time = time(nullptr);
    OutPutFrequentPatternsToFile(1, patternsForGram);

    std::cout<<"Debug: PatternMiner: done (gram = 1) pattern mining! " + toString((patternsForGram[0]).size()) + " patterns found! " << std::endl;
    printf(" Total time: %d seconds. \n", end_time - start_time);

    OutPutFrequentPatternsToFile(cur_gram, patternsForGram);

    HandleSeq allDumpNodes, allDumpLinks;
    atomSpace->get_handles_by_type(back_inserter(allDumpNodes), (Type) NODE, true );

    // Debug : out put the current dump Atomspace to a file
    ofstream dumpFile;
    string fileName = "DumpAtomspace" + toString(cur_gram) + "gram.scm";

    dumpFile.open(fileName.c_str());

    for (Handle h : allDumpNodes)
    {
        dumpFile << h->toShortString();
    }

    atomSpace->get_handles_by_type(back_inserter(allDumpLinks), (Type) LINK, true );

    for (Handle h : allDumpLinks)
    {
        dumpFile << h->toShortString();
    }

    dumpFile.close();

}

void PatternMiner::GrowAllPatternsBF()
{
    for ( cur_gram = 2; cur_gram <= MAX_GRAM; ++ cur_gram)
    {
        cur_index = -1;
        std::cout<<"Debug: PatternMiner:  start (gram = " + toString(cur_gram) + ") pattern mining..." << std::endl;

        last_gram_total_float = (float)((patternsForGram[cur_gram-2]).size());

        for (unsigned int i = 0; i < THREAD_NUM; ++ i)
        {
            threads[i] = std::thread([this]{this->growPatternsTaskBF();}); // using C++11 lambda-expression
        }

        for (unsigned int i = 0; i < THREAD_NUM; ++ i)
        {
            threads[i].join();
        }

        cout << "\nFinished mining " << cur_gram << "gram patterns.\n";


        // sort by frequency
        std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency );

        // Finished mining cur_gram patterns; output to file
        std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") frequent pattern mining!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! " << std::endl;

        OutPutFrequentPatternsToFile(cur_gram, patternsForGram);



        if (enable_Interesting_Pattern)
        {
            cout << "\nCalculating interestingness for " << cur_gram << "gram patterns";
            // evaluate the interestingness
            // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
            if (Enable_Interaction_Information)
            {
                cout << "by evaluating Interaction_Information ...\n";
               // calculate interaction information
               for (HTreeNode* htreeNode : patternsForGram[cur_gram-1])
               {
                   calculateInteractionInformation(htreeNode);
               }

               // sort by interaction information
               std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
            }

            if (Enable_surprisingness)
            {
                cout << "by evaluating surprisingness ...\n";
                // calculate surprisingness
                for (HTreeNode* htreeNode : patternsForGram[cur_gram-1])
                {
                    calculateSurprisingness(htreeNode, originalAtomSpace);
                }

                // sort by surprisingness
                std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeBySurprisingness);
            }

            // Finished mining cur_gram patterns; output to file
            std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") interesting pattern mining!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! " << std::endl;

            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1],cur_gram, true);
        }



        HandleSeq allDumpNodes, allDumpLinks;
        atomSpace->get_handles_by_type(back_inserter(allDumpNodes), (Type) NODE, true );

        // Debug : out put the current dump Atomspace to a file
        ofstream dumpFile;
        string fileName = "DumpAtomspace" + toString(cur_gram) + "gram.scm";

        dumpFile.open(fileName.c_str());

        for (Handle h : allDumpNodes)
        {
            dumpFile << h->toShortString();
        }

        atomSpace->get_handles_by_type(back_inserter(allDumpLinks), (Type) LINK, true );

        for (Handle h : allDumpLinks)
        {
            dumpFile << h->toShortString();
        }

        dumpFile.close();
    }
}

void PatternMiner::swapOneLinkBetweenTwoAtomSpaceForBindLink(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings,
                                                  HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool& containVar )
{
    containVar = false;
    HandleSeq outgoingLinks = fromLink->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
           Handle new_node = toAtomSpace->add_node(h->getType(), h->getName());
           new_node->setTruthValue(h->getTruthValue());
           outgoings.push_back(new_node);
           if (h->getType() == PATTERN_VARIABLENODE_TYPE)
           {
               containVar = true;
               if ( ! isInHandleSeq(new_node, outVariableNodes) ) // should not have duplicated variable nodes
                outVariableNodes.push_back(new_node);
           }
        }
        else
        {
             HandleSeq _OutgoingLinks;
             bool _containVar;
             swapOneLinkBetweenTwoAtomSpaceForBindLink(fromAtomSpace, toAtomSpace, h, _OutgoingLinks, outVariableNodes,linksWillBeDel,  _containVar);
             Handle _link = toAtomSpace->add_link(h->getType(), _OutgoingLinks);
             _link->setTruthValue(h->getTruthValue());
             if (_containVar)
             {
                 linksWillBeDel.push_back(_link);
                 containVar = true;
             }
             outgoings.push_back(_link);
        }
    }
}

// linksWillBeDel are all the links contain varaibles. Those links need to be deleted after run BindLink
HandleSeq PatternMiner::swapLinksBetweenTwoAtomSpaceForBindLink(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq& outVariableNodes, HandleSeq& linksWillBeDel)
{
    HandleSeq outPutLinks;

    for (Handle link : fromLinks)
    {
        HandleSeq outgoingLinks;
        bool containVar;
        swapOneLinkBetweenTwoAtomSpaceForBindLink(fromAtomSpace, toAtomSpace, link, outgoingLinks, outVariableNodes, linksWillBeDel,containVar);
        Handle toLink = toAtomSpace->add_link(link->getType(), outgoingLinks);
        toLink->setTruthValue(link->getTruthValue());
        if (containVar)
            linksWillBeDel.push_back(toLink);
        outPutLinks.push_back(toLink);
    }

    return outPutLinks;
}

// using PatternMatcher
void PatternMiner::findAllInstancesForGivenPatternBF(HTreeNode* HNode)
{

//     First, generate the Bindlink for using PatternMatcher to find all the instances for this pattern in the original Atomspace
//    (BindLink
//        ;; The variables to be bound
//        (Listlink)
//          (VariableNode "$var_1")
//          (VariableNode "$var_2")
//          ...
//        ;; The pattern to be searched for
//        (pattern)
//        (Listlink)
//            ;; The instance to be returned.
//            (result)
//            (variable Listlink)
//     )


   HandleSeq bindLinkOutgoings, variableNodes, linksWillBeDel;

   HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpaceForBindLink(atomSpace, originalAtomSpace, HNode->pattern, variableNodes, linksWillBeDel);

//   HandleSet allNodesInPattern;
//   for (unsigned int i = 0; i < HNode->pattern.size(); ++i)
//   {
//       extractAllVariableNodesInLink(HNode->pattern[i],allNodesInPattern, atomSpace);
//   }

//   HandleSeq variableNodes(allNodesInPattern.begin(), allNodesInPattern.end());

//    if (HNode->pattern.size() == 1) // this pattern only contains one link
//    {
//        implicationLinkOutgoings.push_back(patternToMatch[0]); // the pattern to match
//        implicationLinkOutgoings.push_back(patternToMatch[0]); // the results to return

//        std::cout<<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//                << originalAtomSpace->atomAsString(patternToMatch[0]).c_str() << std::endl;
//    }

   Handle hAndLink = originalAtomSpace->add_link(AND_LINK, patternToMatch);
   hAndLink->setTruthValue(TruthValue::TRUE_TV());
   Handle hOutPutListLink = originalAtomSpace->add_link(LIST_LINK, patternToMatch);
   hOutPutListLink->setTruthValue(TruthValue::TRUE_TV());

//    std::cout <<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//            << atomSpace->atomAsString(hAndLink).c_str() << std::endl;

   // add variable atoms
   Handle hVariablesListLink = originalAtomSpace->add_link(LIST_LINK, variableNodes);
   hVariablesListLink->setTruthValue(TruthValue::TRUE_TV());

   bindLinkOutgoings.push_back(hVariablesListLink);
   bindLinkOutgoings.push_back(hAndLink);
   bindLinkOutgoings.push_back(hOutPutListLink);
   Handle hBindLink = originalAtomSpace->add_link(BIND_LINK, bindLinkOutgoings);
   hBindLink->setTruthValue(TruthValue::TRUE_TV());


   string s = hBindLink->toShortString();
   // Run pattern matcher
   Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

   // Get result
   // Note: Don't forget to remove the hResultListLink and BindLink
   HandleSeq resultSet = hResultListLink->getOutgoingSet();

//     std::cout << toString(resultSet.size())  << " instances found!" << std::endl ;

   //    //debug
//    std::cout << atomSpace->atomAsString(hResultListLink) << std::endl  << std::endl;

   for (Handle listH  : resultSet)
   {
       HandleSeq instanceLinks = listH->getOutgoingSet();

       if (cur_gram == 1)
       {
           HNode->instances.push_back(instanceLinks);
       }
       else
       {
           // instance that contains duplicate links will not be added
           if (! containsDuplicateHandle(instanceLinks))
               HNode->instances.push_back(instanceLinks);
       }

 //      originalAtomSpace->removeAtom(listH);
   }

//   originalAtomSpace->removeAtom(hBindLink);
//   originalAtomSpace->removeAtom(hImplicationLink);
//   originalAtomSpace->removeAtom(hAndLink);
//   originalAtomSpace->removeAtom(hResultListLink);
//   originalAtomSpace->removeAtom(hVariablesListLink);

//   for (Handle linkToDel  : linksWillBeDel)
//   {
//       originalAtomSpace->removeAtom(linkToDel);
//   }

   HNode->count = HNode->instances.size();
}


void PatternMiner::runPatternMinerBreadthFirst()
{

    ConstructTheFirstGramPatternsBF();

    // and then generate all patterns
    GrowAllPatternsBF();

}
