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

void PatternMiner::extendAllPossiblePatternsForOneMoreGramBF(HandleSeq &instance, HTreeNode* curHTreeNode, unsigned int gram)
{
    // debug:
    string instanceInst = unifiedPatternToKeyString(instance, originalAtomSpace);
    if (instanceInst.find("$var") != std::string::npos)
    {
        cout << "Debug: error! The instance contines variables!" << instanceInst <<  "Skip it!" << std::endl;
        return;
    }

    // First, extract all the variable nodes in the instance links
    // we only extend one more link on the nodes that are considered as varaibles for this pattern
    set<Handle> allVarNodes;

    HandleSeq::iterator patternLinkIt = curHTreeNode->pattern.begin();
    foreach (Handle link, instance)
    {
        extractAllVariableNodesInAnInstanceLink(link, *(patternLinkIt), allVarNodes);
        patternLinkIt ++;
    }

    set<Handle>::iterator varIt;

    for(varIt = allVarNodes.begin(); varIt != allVarNodes.end(); ++ varIt)
    {
        // find what are the other links in the original Atomspace contain this variable
        HandleSeq incomings = originalAtomSpace->getIncoming( ((Handle)(*varIt)));
        // debug
        string curvarstr = originalAtomSpace->atomAsString((Handle)(*varIt));

        foreach(Handle incomingHandle, incomings)
        {
            Handle extendedHandle;
            // if this atom is a igonred type, get its first parent that is not in the igonred types
            if (isIgnoredType (originalAtomSpace->getType(incomingHandle)) )
            {
                extendedHandle = getFirstNonIgnoredIncomingLink(originalAtomSpace, incomingHandle);
                if (extendedHandle == Handle::UNDEFINED)
                    continue;
            }
            else
                extendedHandle = incomingHandle;

            // debug
            string extendedHandleStr = originalAtomSpace->atomAsString(extendedHandle);

            if (isInHandleSeq(extendedHandle, instance))
                continue;

            if (extendedHandleStr.find("$var") != std::string::npos)
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

void PatternMiner::extractAllPossiblePatternsFromInputLinksBF(vector<Handle>& inputLinks,  HTreeNode* parentNode,
                                                                          set<Handle>& sharedNodes, unsigned int gram)
{
    map<Handle,Handle> valueToVarMap;  // the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace

//    // Debug
//    cout << "Extract patterns from these links: \n";
//    foreach (Handle ih, inputLinks)
//    {
//        cout << originalAtomSpace->atomAsString(ih) << std::endl;
//    }

    // First, extract all the nodes in the input links
    foreach (Handle link, inputLinks)
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

    foreach (Handle shardNode, sharedNodes)
    {
        sharedNodesIndexes[sharedCout] = -1;
        map<Handle,Handle>::iterator varIt;
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
            map<Handle,Handle>::iterator iter;

            map<Handle,Handle> patternVarMap;

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
                        patternVarMap.insert(std::pair<Handle,Handle>(iter->first, iter->second));

                    index ++;
                }

                HandleSeq pattern, unifiedPattern;
                bool hasLinkContainsOnlyVars = false;

                foreach (Handle link, inputLinks)
                {
                    HandleSeq outgoingLinks;
                    generateALinkByChosenVariables(link, patternVarMap, outgoingLinks, originalAtomSpace);
                    Handle rebindedLink = atomSpace->addLink(atomSpace->getType(link),outgoingLinks,TruthValue::TRUE_TV());
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

        if (cur_index >= total)
        {
            patternForLastGramLock.unlock();
            break;
        }

        HTreeNode* cur_growing_pattern = last_gram_patterns[cur_index];

        patternForLastGramLock.unlock();

        if(cur_growing_pattern->count < thresholdFrequency)
            continue;

        foreach (HandleSeq instance , cur_growing_pattern->instances)
        {
            extendAllPossiblePatternsForOneMoreGramBF(instance, cur_growing_pattern, cur_gram);
        }

        cur_growing_pattern->instances.clear();
        (vector<HandleSeq>()).swap(cur_growing_pattern->instances);



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

        if (cur_index >= allLinks.size())
        {
            patternForLastGramLock.unlock();
            break;
        }

        patternForLastGramLock.unlock();

        Handle cur_link = allLinks[cur_index];

        // if this link is listlink, ignore it
        if (originalAtomSpace->getType(cur_link) == opencog::LIST_LINK)
        {
            continue;
        }

        HandleSeq originalLinks;
        originalLinks.push_back(cur_link);

        // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns
        set<Handle> sharedNodes;
        extractAllPossiblePatternsFromInputLinksBF(originalLinks, 0, sharedNodes, 1);

    }

}

bool compareHTreeNodeByFrequency(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeByInteractionInformation(HTreeNode* node1, HTreeNode* node2);
bool compareHTreeNodeBySurprisingness(HTreeNode* node1, HTreeNode* node2);

void PatternMiner::ConstructTheFirstGramPatternsBF()
{
    int start_time = time(NULL);

    std::cout<<"Debug: PatternMiner:  start (gram = 1) pattern mining..." << std::endl;

    cur_gram = 1;
    cur_index = -1;

    originalAtomSpace->getHandlesByType(back_inserter(allLinks), (Type) LINK, true );

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

    // release allLinks
    allLinks.clear();
    (vector<Handle>()).swap(allLinks);

    // sort the patterns by frequency
    std::sort((patternsForGram[0]).begin(), (patternsForGram[0]).end(),compareHTreeNodeByFrequency );

    int end_time = time(NULL);
    OutPutPatternsToFile(1);

    std::cout<<"Debug: PatternMiner: done (gram = 1) pattern mining! " + toString((patternsForGram[0]).size()) + " patterns found! " << std::endl;
    printf(" Total time: %d seconds. \n", end_time - start_time);

    OutPutPatternsToFile(cur_gram);

    HandleSeq allDumpNodes, allDumpLinks;
    atomSpace->getHandlesByType(back_inserter(allDumpNodes), (Type) NODE, true );

    // Debug : out put the current dump Atomspace to a file
    ofstream dumpFile;
    string fileName = "DumpAtomspace" + toString(cur_gram) + "gram.scm";

    dumpFile.open(fileName.c_str());

    foreach(Handle h, allDumpNodes)
    {
        dumpFile << atomSpace->atomAsString(h);
    }

    atomSpace->getHandlesByType(back_inserter(allDumpLinks), (Type) LINK, true );

    foreach(Handle h, allDumpLinks)
    {
        dumpFile << atomSpace->atomAsString(h);
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

        if (enable_Frequent_Pattern)
        {
            // sort by frequency
            std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency );

            // Finished mining cur_gram patterns; output to file
            std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") frequent pattern mining!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! " << std::endl;

            OutPutPatternsToFile(cur_gram);
        }


        if (enable_Interesting_Pattern)
        {
            cout << "\nCalculating interestingness for " << cur_gram << "gram patterns";
            // evaluate the interestingness
            // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
            if (interestingness_Evaluation_method == "Interaction_Information")
            {
                cout << "by evaluating Interaction_Information ...\n";
               // calculate interaction information
               foreach(HTreeNode* htreeNode, patternsForGram[cur_gram-1])
               {
                   calculateInteractionInformation(htreeNode);
               }

               // sort by interaction information
               std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
            }
            else if (interestingness_Evaluation_method == "surprisingness")
            {
                cout << "by evaluating surprisingness ...\n";
                // calculate surprisingness
                foreach(HTreeNode* htreeNode, patternsForGram[cur_gram-1])
                {
                    calculateSurprisingness(htreeNode);
                }

                // sort by surprisingness
                std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeBySurprisingness);
            }

            // Finished mining cur_gram patterns; output to file
            std::cout<<"Debug: PatternMiner:  done (gram = " + toString(cur_gram) + ") interesting pattern mining!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! " << std::endl;

            OutPutPatternsToFile(cur_gram, true);
        }



        HandleSeq allDumpNodes, allDumpLinks;
        atomSpace->getHandlesByType(back_inserter(allDumpNodes), (Type) NODE, true );

        // Debug : out put the current dump Atomspace to a file
        ofstream dumpFile;
        string fileName = "DumpAtomspace" + toString(cur_gram) + "gram.scm";

        dumpFile.open(fileName.c_str());

        foreach(Handle h, allDumpNodes)
        {
            dumpFile << atomSpace->atomAsString(h);
        }

        atomSpace->getHandlesByType(back_inserter(allDumpLinks), (Type) LINK, true );

        foreach(Handle h, allDumpLinks)
        {
            dumpFile << atomSpace->atomAsString(h);
        }

        dumpFile.close();
    }
}

void PatternMiner::swapOneLinkBetweenTwoAtomSpaceBF(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings,
                                                  HandleSeq &outVariableNodes, HandleSeq& linksWillBeDel, bool& containVar )
{
    containVar = false;
    HandleSeq outgoingLinks = fromAtomSpace->getOutgoing(fromLink);

    foreach (Handle h, outgoingLinks)
    {
        if (fromAtomSpace->isNode(h))
        {
           Handle new_node = toAtomSpace->addNode(fromAtomSpace->getType(h), fromAtomSpace->getName(h), fromAtomSpace->getTV(h));
           outgoings.push_back(new_node);
           if (fromAtomSpace->getType(h) == VARIABLE_NODE)
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
             swapOneLinkBetweenTwoAtomSpaceBF(fromAtomSpace, toAtomSpace, h, _OutgoingLinks, outVariableNodes,linksWillBeDel,  _containVar);
             Handle _link = toAtomSpace->addLink(fromAtomSpace->getType(h),_OutgoingLinks,fromAtomSpace->getTV(h));
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
HandleSeq PatternMiner::swapLinksBetweenTwoAtomSpaceBF(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq& outVariableNodes, HandleSeq& linksWillBeDel)
{
    HandleSeq outPutLinks;

    foreach (Handle link, fromLinks)
    {
        HandleSeq outgoingLinks;
        bool containVar;
        swapOneLinkBetweenTwoAtomSpaceBF(fromAtomSpace, toAtomSpace, link, outgoingLinks, outVariableNodes, linksWillBeDel,containVar);
        Handle toLink = toAtomSpace->addLink(fromAtomSpace->getType(link),outgoingLinks,fromAtomSpace->getTV(link));
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
//        (ImplicationLink
//          ;; The pattern to be searched for
//          (pattern)
//          (Listlink)
//              ;; The instance to be returned.
//              (result)
//              (variable Listlink)
//        )
//     )


   HandleSeq implicationLinkOutgoings, bindLinkOutgoings, variableNodes, linksWillBeDel;

   HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpaceBF(atomSpace, originalAtomSpace, HNode->pattern, variableNodes, linksWillBeDel);

//   set<Handle> allNodesInPattern;
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

   Handle hAndLink = originalAtomSpace->addLink(AND_LINK, patternToMatch, TruthValue::TRUE_TV());
   Handle hOutPutListLink = originalAtomSpace->addLink(LIST_LINK, patternToMatch, TruthValue::TRUE_TV());
   implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
   implicationLinkOutgoings.push_back(hOutPutListLink); // the results to return

//    std::cout <<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//            << atomSpace->atomAsString(hAndLink).c_str() << std::endl;


   Handle hImplicationLink = originalAtomSpace->addLink(IMPLICATION_LINK, implicationLinkOutgoings, TruthValue::TRUE_TV());

   // add variable atoms
   Handle hVariablesListLink = originalAtomSpace->addLink(LIST_LINK, variableNodes, TruthValue::TRUE_TV());

   bindLinkOutgoings.push_back(hVariablesListLink);
   bindLinkOutgoings.push_back(hImplicationLink);
   Handle hBindLink = originalAtomSpace->addLink(BIND_LINK, bindLinkOutgoings, TruthValue::TRUE_TV());


   string s = originalAtomSpace->atomAsString(hBindLink);
   // Run pattern matcher
   Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

   // Get result
   // Note: Don't forget to remove the hResultListLink and BindLink
   HandleSeq resultSet = originalAtomSpace->getOutgoing(hResultListLink);

//     std::cout << toString(resultSet.size())  << " instances found!" << std::endl ;

   //    //debug
//    std::cout << atomSpace->atomAsString(hResultListLink) << std::endl  << std::endl;

   foreach (Handle listH , resultSet)
   {
       HandleSeq instanceLinks = originalAtomSpace->getOutgoing(listH);

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

//   foreach (Handle linkToDel , linksWillBeDel)
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
