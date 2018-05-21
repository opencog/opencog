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

#include <boost/range/algorithm/sort.hpp>

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/proto/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/algorithm.h>
#include <opencog/learning/PatternMiner/types/atom_types.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;

// This file is not maintained anymore. Please use depth first mining.

void PatternMiner::extendAllPossiblePatternsForOneMoreGramBF(const HandleSeq& instance, HTreeNode* curHTreeNode, unsigned int gram)
{
    // debug:
    string instanceInst = unifiedPatternToKeyString(instance);
    if (instanceInst.find("PatternVariableNode") != std::string::npos)
    {
        cout << "Debug: error! The instance contines variables!" << instanceInst <<  "Skip it!" << std::endl;
        return;
    }

    // First, extract all the variable nodes in the instance links
    // we only extend one more link on the nodes that are considered as varaibles for this pattern
    HandleSet allVarNodes;

    HandleSeq::const_iterator patternLinkIt = curHTreeNode->pattern.begin();
    for (const Handle& link : instance)
    {
        extractAllVariableNodesInAnInstanceLink(link, *patternLinkIt, allVarNodes);
        ++patternLinkIt;
    }

    for (const Handle& var : allVarNodes)
    {
        // find what are the other links in the original Atomspace contain this variable
        HandleSeq incomings;
        var->getIncomingSet(back_inserter(incomings));

        // debug
        string curvarstr = var->to_short_string();

        for (const Handle& incomingHandle : incomings)
        {
            Handle extendedHandle;
            // if this atom is a igonred type, get its first parent that is not in the igonred types
            if (isIgnoredType (incomingHandle->get_type()) )
            {
                extendedHandle = getFirstNonIgnoredIncomingLink(original_as, incomingHandle);
                if (extendedHandle == Handle::UNDEFINED)
                    continue;
            }
            else
                extendedHandle = incomingHandle;

            // debug
            string extendedHandleStr = extendedHandle->to_short_string();

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
            extractAllPossiblePatternsFromInputLinksBF(originalLinks, curHTreeNode, allVarNodes, gram);
        }
    }
}

void PatternMiner::extractAllPossiblePatternsFromInputLinksBF(const HandleSeq& inputLinks, HTreeNode* parentNode,
                                                              HandleSet& sharedNodes, unsigned int gram)
{
    HandleMap valueToVarMap;  // the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace

//    // Debug
//    cout << "Extract patterns from these links: \n";
//    for (const Handle& ih : inputLinks)
//    {
//        cout << original_as.atomAsString(ih) << std::endl;
//    }

    // First, extract all the nodes in the input links
    for (const Handle& link : inputLinks)
        associateNodesToVars(link, valueToVarMap);

    // Generate all the possible combinations of all the nodes: all patterns including the 1 ~ n_max variables
    // If there are too many variables in a pattern, it doesn't make much sense, so we litmit the max number of variables to half of the node number

//    OC_ASSERT( (valueToVarMap.size() > 1),
//              "PatternMiner::extractAllPossiblePatternsFromInputLinks: this group of links only has one node: %s!\n",
//               as->atomAsString(inputLinks[0]).c_str() );

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

    for (const Handle& shardNode : sharedNodes)
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
                for (const auto& valvar : valueToVarMap)
                {
                    if (indexes[index]) // this is considered as a variable, add it into the variable to value map
                        patternVarMap.insert(valvar);

                    index++;
                }

                HandleSeq pattern, unifiedPattern;
                bool hasLinkContainsOnlyVars = false;

                for (const Handle& link : inputLinks)
                {
                    Handle rebindedLink = substitute(link, patternVarMap);
                    if (containOnlyVariables(rebindedLink))
                        hasLinkContainsOnlyVars = true;
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
                        keyStrToHTreeNodeMap.insert({keyString, newHTreeNode});
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

        if(cur_growing_pattern->count < param.threshold_frequency)
            continue;

        for (const HandleSeq& instance  : cur_growing_pattern->instances)
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
        if (cur_link->get_type() == opencog::LIST_LINK)
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

    original_as.get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

    atomspaceSizeFloat = (float)(allLinks.size());

//    for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
//    {
//        threads[i] = std::thread([this]{this->growTheFirstGramPatternsTaskBF();}); // using C++11 lambda-expression
//    }

//    for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
//    {
//        threads[i].join();
//    }

    growTheFirstGramPatternsTaskBF();

    clear_by_swap(allLinks);

    // sort the patterns by frequency
    boost::sort(patternsForGram[0], compareHTreeNodeByFrequency );

    int end_time = time(nullptr);
    OutPutFrequentPatternsToFile(1, patternsForGram);

    std::cout<<"Debug: PatternMiner: done (gram = 1) pattern mining! " + toString((patternsForGram[0]).size()) + " patterns found! " << std::endl;
    printf(" Total time: %d seconds. \n", end_time - start_time);

    OutPutFrequentPatternsToFile(cur_gram, patternsForGram);

    HandleSeq allDumpNodes, allDumpLinks;
    as->get_handles_by_type(back_inserter(allDumpNodes), (Type) NODE, true );

    // Debug : out put the current dump Atomspace to a file
    ofstream dumpFile;
    string fileName = "DumpAtomspace" + toString(cur_gram) + "gram.scm";

    dumpFile.open(fileName.c_str());

    for (const Handle& h : allDumpNodes)
    {
        dumpFile << h->to_short_string();
    }

    as->get_handles_by_type(back_inserter(allDumpLinks), (Type) LINK, true );

    for (const Handle& h : allDumpLinks)
    {
        dumpFile << h->to_short_string();
    }

    dumpFile.close();

}

void PatternMiner::GrowAllPatternsBF()
{
    for (cur_gram = 2; cur_gram <= param.MAX_GRAM; ++ cur_gram)
    {
        cur_index = -1;
        std::cout<<"Debug: PatternMiner:  start (gram = " + toString(cur_gram) + ") pattern mining..." << std::endl;

        last_gram_total_float = (float)((patternsForGram[cur_gram-2]).size());

        for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
        {
            threads[i] = std::thread([this]{this->growPatternsTaskBF();}); // using C++11 lambda-expression
        }

        for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
        {
            threads[i].join();
        }

        cout << "\nFinished mining " << cur_gram << "gram patterns.\n";


        // sort by frequency
        boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeByFrequency);

        // Finished mining cur_gram patterns; output to file
        std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") frequent pattern mining!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! " << std::endl;

        OutPutFrequentPatternsToFile(cur_gram, patternsForGram);



        if (param.enable_interesting_pattern)
        {
            cout << "\nCalculating interestingness for " << cur_gram << "gram patterns";
            // evaluate the interestingness
            // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
            if (param.enable_interaction_information)
            {
                cout << "by evaluating Interaction_Information ...\n";
               // calculate interaction information
               for (HTreeNode* htreeNode : patternsForGram[cur_gram-1])
               {
                   calculateInteractionInformation(htreeNode);
               }

               // sort by interaction information
               boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeByInteractionInformation);
            }

            if (param.enable_surprisingness)
            {
                cout << "by evaluating surprisingness ...\n";
                // calculate surprisingness
                for (HTreeNode* htreeNode : patternsForGram[cur_gram-1])
                {
                    calculateSurprisingness(htreeNode);
                }

                // sort by surprisingness
                boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeBySurprisingness);
            }

            // Finished mining cur_gram patterns; output to file
            std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") interesting pattern mining!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! " << std::endl;

            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1],cur_gram, true);
        }

        HandleSeq allDumpNodes, allDumpLinks;
        as->get_handles_by_type(back_inserter(allDumpNodes), (Type) NODE, true );

        // Debug : out put the current dump Atomspace to a file
        ofstream dumpFile;
        string fileName = "DumpAtomspace" + toString(cur_gram) + "gram.scm";

        dumpFile.open(fileName.c_str());

        for (const Handle& h : allDumpNodes)
        {
            dumpFile << h->to_short_string();
        }

        as->get_handles_by_type(back_inserter(allDumpLinks), (Type) LINK, true );

        for (const Handle& h : allDumpLinks)
        {
            dumpFile << h->to_short_string();
        }

        dumpFile.close();
    }
}

void PatternMiner::swapOneLinkBetweenTwoAtomSpaceForBindLink(AtomSpace& to_as, const Handle& fromLink, HandleSeq& outgoings,
                                                  HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool& containVar)
{
    containVar = false;
    HandleSeq outgoingLinks = fromLink->getOutgoingSet();

    for (const Handle& h : outgoingLinks)
    {
        if (h->is_node())
        {
           Handle new_node = to_as.add_node(h->get_type(), h->get_name());
           new_node->setTruthValue(h->getTruthValue());
           outgoings.push_back(new_node);
           if (h->get_type() == PATTERN_VARIABLENODE_TYPE)
           {
               containVar = true;
               if (!isInHandleSeq(new_node, outVariableNodes)) // should not have duplicated variable nodes
                outVariableNodes.push_back(new_node);
           }
        }
        else
        {
             HandleSeq _OutgoingLinks;
             bool _containVar;
             swapOneLinkBetweenTwoAtomSpaceForBindLink(to_as, h, _OutgoingLinks, outVariableNodes, linksWillBeDel, _containVar);
             Handle _link = to_as.add_link(h->get_type(), _OutgoingLinks);
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
HandleSeq PatternMiner::swapLinksBetweenTwoAtomSpaceForBindLink(AtomSpace& to_as, const HandleSeq& fromLinks, HandleSeq& outVariableNodes, HandleSeq& linksWillBeDel)
{
    HandleSeq outPutLinks;

    for (const Handle& link : fromLinks)
    {
        HandleSeq outgoingLinks;
        bool containVar;
        swapOneLinkBetweenTwoAtomSpaceForBindLink(to_as, link, outgoingLinks, outVariableNodes, linksWillBeDel,containVar);
        Handle toLink = to_as.add_link(link->get_type(), outgoingLinks);
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

   HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpaceForBindLink(original_as, HNode->pattern, variableNodes, linksWillBeDel);

//   HandleSet allNodesInPattern;
//   for (unsigned int i = 0; i < HNode->pattern.size(); ++i)
//   {
//       extractVarNodes(HNode->pattern[i],allNodesInPattern, as);
//   }

//   HandleSeq variableNodes(allNodesInPattern.begin(), allNodesInPattern.end());

//    if (HNode->pattern.size() == 1) // this pattern only contains one link
//    {
//        implicationLinkOutgoings.push_back(patternToMatch[0]); // the pattern to match
//        implicationLinkOutgoings.push_back(patternToMatch[0]); // the results to return

//        std::cout<<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//                << original_as.atomAsString(patternToMatch[0]).c_str() << std::endl;
//    }

   Handle hAndLink = original_as.add_link(AND_LINK, patternToMatch);
   hAndLink->setTruthValue(TruthValue::TRUE_TV());
   Handle hOutPutListLink = original_as.add_link(LIST_LINK, patternToMatch);
   hOutPutListLink->setTruthValue(TruthValue::TRUE_TV());

//    std::cout <<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//            << as->atomAsString(hAndLink).c_str() << std::endl;

   // add variable atoms
   Handle hVariablesListLink = original_as.add_link(LIST_LINK, variableNodes);
   hVariablesListLink->setTruthValue(TruthValue::TRUE_TV());

   bindLinkOutgoings.push_back(hVariablesListLink);
   bindLinkOutgoings.push_back(hAndLink);
   bindLinkOutgoings.push_back(hOutPutListLink);
   Handle hBindLink = original_as.add_link(BIND_LINK, bindLinkOutgoings);
   hBindLink->setTruthValue(TruthValue::TRUE_TV());


   string s = hBindLink->to_short_string();
   // Run pattern matcher
   Handle hResultListLink = bindlink(&original_as, hBindLink);

   // Get result
   // Note: Don't forget to remove the hResultListLink and BindLink
   HandleSeq resultSet = hResultListLink->getOutgoingSet();

//     std::cout << toString(resultSet.size())  << " instances found!" << std::endl ;

   //    //debug
//    std::cout << as->atomAsString(hResultListLink) << std::endl  << std::endl;

   for (const Handle& listH  : resultSet)
   {
       HandleSeq instanceLinks = listH->getOutgoingSet();

       if (cur_gram == 1)
       {
           HNode->instances.push_back(instanceLinks);
       }
       else
       {
           // instance that contains duplicate links will not be added
           if (! containDuplicates(instanceLinks))
               HNode->instances.push_back(instanceLinks);
       }

 //      original_as.removeAtom(listH);
   }

//   original_as.removeAtom(hBindLink);
//   original_as.removeAtom(hImplicationLink);
//   original_as.removeAtom(hAndLink);
//   original_as.removeAtom(hResultListLink);
//   original_as.removeAtom(hVariablesListLink);

//   for (const Handle& linkToDel  : linksWillBeDel)
//   {
//       original_as.removeAtom(linkToDel);
//   }

   HNode->count = HNode->instances.size();
}


void PatternMiner::runPatternMinerBreadthFirst()
{

    ConstructTheFirstGramPatternsBF();

    // and then generate all patterns
    GrowAllPatternsBF();

}
