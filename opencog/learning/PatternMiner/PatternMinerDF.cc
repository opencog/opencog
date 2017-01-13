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
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/atom_types.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <opencog/util/StringManipulator.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


void PatternMiner::growPatternsDepthFirstTask_old()
{
    float sq_allLinkNumber = ((float)(allLinkNumber)) * ((float)(allLinkNumber));
    while(true)
    {
        readNextLinkLock.lock();
        cur_index ++;

        if (cur_index < allLinkNumber)
        {
            cout<< "\r" << ((float)(cur_index))*((float)(cur_index))/sq_allLinkNumber*100.0f << + "% completed." ; // it's not liner
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
        if (originalAtomSpace->get_type(cur_link) == opencog::LIST_LINK)
        {
            continue;
        }

        // Add this link into observingAtomSpace
        HandleSeq outgoingLinks,outVariableNodes;

        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
        Handle newLink = observingAtomSpace->add_link(originalAtomSpace->get_type(cur_link), outgoingLinks);
        newLink->merge(originalAtomSpace->get_TV(cur_link));

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
            vector<OrderedHandleSet> newConnectedLinksFoundThisGram;

            for(; it != allLastGramLinksToPatterns.end(); ++ it)
            {
                // find all the 2~MAX_GRAM gram distance neighbour links of newLink
                extendAllPossiblePatternsForOneMoreGramDF((HandleSeq&)(it->first),observingAtomSpace,gram, (vector<HTreeNode*>&)(it->second), allThisGramLinksToPatterns, newConnectedLinksFoundThisGram);
            }

            allLastGramLinksToPatterns = allThisGramLinksToPatterns;

        }

    }
}


void PatternMiner::growPatternsDepthFirstTask(unsigned int thread_index)
{

	// The start index in allLinks for current thread
    unsigned int start_index = linksPerThread * thread_index;
    unsigned int end_index; // the last index for current thread (excluded)
    if (thread_index == THREAD_NUM - 1) // if this the last thread, it
                                        // needs to finish all the
                                        // rest of the links
        end_index = allLinkNumber;
    else
        end_index = linksPerThread * (thread_index + 1);


    cout << "Start thread " << thread_index << " from " << start_index
         << " to (excluded) " << end_index << std::endl;

    patternJsonArrays[thread_index] = json::value::array();

    float allLinkNumberfloat = ((float)(end_index - start_index));
    for(unsigned int t_cur_index = start_index; t_cur_index < end_index; ++t_cur_index)
    {
        readNextLinkLock.lock();
        cout<< "\r" << ((float)(t_cur_index - start_index))/allLinkNumberfloat*100.0f << "% completed in Thread " + toString(thread_index) + "."; // it's not liner
        std::cout.flush();

        processedLinkNum ++;
        Handle& cur_link = allLinks[t_cur_index];

        readNextLinkLock.unlock();

        // if this link is listlink, ignore it
        if (originalAtomSpace->get_type(cur_link) == opencog::LIST_LINK)
        {
            continue;
        }

        // Add this link into observingAtomSpace
        HandleSeq outgoingLinks, outVariableNodes;

        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
        Handle newLink = observingAtomSpace->add_link(originalAtomSpace->get_type(cur_link), outgoingLinks);
        newLink->merge(originalAtomSpace->get_TV(cur_link));


        // Extract all the possible patterns from this originalLink, and extend till the max_gram links, not duplicating the already existing patterns
        HandleSeq lastGramLinks;
        map<Handle,Handle> lastGramValueToVarMap;
        map<Handle,Handle> patternVarMap;

        // vector<HTreeNode*> &allHTreeNodesCurTask is only used in distributed version
        // is to store all the HTreeNode* mined in this current task, and release them after the task is finished.
        vector<HTreeNode*> allHTreeNodesCurTask;


        actualProcessedLinkLock.lock();
        actualProcessedLinkNum ++;
        actualProcessedLinkLock.unlock();

        extendAPatternForOneMoreGramRecursively(newLink, observingAtomSpace, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap,
                                                patternVarMap, false, allHTreeNodesCurTask, patternJsonArrays[thread_index]);

        // release all the HTreeNodes created in this task if it's running as a distributed worker
        if (run_as_distributed_worker)
        {
            // clean up the pattern atomspace, do not need to keep patterns in atomspace when run as a distributed worker
            if (THREAD_NUM == 1)
                atomSpace->clear(); // can only clear the atomspace when only 1 thread is used

            for(unsigned int hNodeNum = 0; hNodeNum < allHTreeNodesCurTask.size(); hNodeNum ++)
            {
                delete (allHTreeNodesCurTask[hNodeNum]);
            }
        }


    }

    if (patternJsonArrays[thread_index].size() > 0)
        sendPatternsToCentralServer(patternJsonArrays[thread_index]);

    cout<< "\r100% completed in Thread " + toString(thread_index) + ".";
    std::cout.flush();
}


//void PatternMiner::growPatternsDepthFirstTask()
//{
//    float sq_allLinkNumber = ((float)(allLinkNumber)) * ((float)(allLinkNumber));
//    while(true)
//    {
//        readNextLinkLock.lock();
//        cur_index ++;

//        if (cur_index < allLinkNumber)
//        {
//            cout<< "\r" << ((float)(cur_index))*((float)(cur_index))/sq_allLinkNumber*100.0f << + "% completed." ; // it's not liner
//            std::cout.flush();
//        }
//        else
//        {
//            if (cur_index == allLinkNumber)
//            {
//                cout<< "\r100% completed." ;
//                std::cout.flush();
//            }

//            readNextLinkLock.unlock();
//            break;

//        }

//        Handle& cur_link = allLinks[cur_index];

//        readNextLinkLock.unlock();

//        // if this link is listlink, ignore it
//        if (originalAtomSpace->getType(cur_link) == opencog::LIST_LINK)
//        {
//            continue;
//        }

//        // Add this link into observingAtomSpace
//        HandleSeq outgoingLinks,outVariableNodes;

//        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
//        Handle newLink = observingAtomSpace->addLink(originalAtomSpace->getType(cur_link),outgoingLinks,originalAtomSpace->getTV(cur_link));

//        // Extract all the possible patterns from this originalLink, and extend till the max_gram links, not duplicating the already existing patterns
//        HandleSeq lastGramLinks;
//        map<Handle,Handle> lastGramValueToVarMap;
//        map<Handle,Handle> patternVarMap;
//        set<string> cur_task_ExtractedLinks[MAX_GRAM];
//        extendAPatternForOneMoreGramRecursively(newLink, observingAtomSpace, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap, patternVarMap, cur_task_ExtractedLinks, false);



//    }
//}



void PatternMiner::runPatternMinerDepthFirst()
{
    // observingAtomSpace is used to copy one link everytime from the originalAtomSpace
    observingAtomSpace = new AtomSpace();

//    cur_DF_ExtractedLinks = new set<string>[MAX_GRAM];

    linksPerThread = allLinkNumber / THREAD_NUM;

    processedLinkNum = 0;
    actualProcessedLinkNum = 0;

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {

        threads[i] = std::thread(&PatternMiner::growPatternsDepthFirstTask,this,i);
        // threads[i] = std::thread([this]{this->growPatternsDepthFirstTask(i);}); // using C++11 lambda-expression
    }

    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i].join();
    }

    // release allLinks
    allLinks.clear();
    (HandleSeq()).swap(allLinks);

//    delete [] cur_DF_ExtractedLinks;
    delete [] threads;
    delete [] patternJsonArrays;

    cout << "\nFinished mining 1~" << MAX_GRAM << " gram patterns.\n";
    cout << "\nprocessedLinkNum = " << processedLinkNum << std::endl;

    if (run_as_distributed_worker)
        cout << "Totally "<< cur_worker_mined_pattern_num << " patterns found!\n";



}

// extendedLinkIndex is to return the index of extendedLink's patternlink in the unified pattern so as to identify where is the extended link in this pattern
// vector<HTreeNode*> &allHTreeNodesCurTask is only used in distributed version
HTreeNode* PatternMiner::extractAPatternFromGivenVarCombination(HandleSeq &inputLinks, map<Handle,Handle> &patternVarMap, HandleSeqSeq &oneOfEachSeqShouldBeVars, HandleSeq &leaves,
                                                                HandleSeq &shouldNotBeVars, HandleSeq &shouldBeVars,AtomSpace* _fromAtomSpace, unsigned int & extendedLinkIndex)
{
    HTreeNode* returnHTreeNode = 0;
    bool skip = false;
    unsigned int gram = inputLinks.size();

//    // debug
//    string inputLinksStr = "";
//    for (Handle h : inputLinks)
//        inputLinksStr += _fromAtomSpace->atomAsString(h);


    if ( shouldNotBeVars.size() > 0)
    {
        for (Handle noTypeNode  : shouldNotBeVars)
        {
            if (patternVarMap.find(noTypeNode) != patternVarMap.end())
            {
                skip = true;
                break;
            }
        }
    }

    if ((! skip) && (shouldBeVars.size() > 0))
    {
        for (Handle shouldBeVarNode  : shouldBeVars)
        {
            if (patternVarMap.find(shouldBeVarNode) == patternVarMap.end())
            {
                skip = true;
                break;
            }
        }
    }

    if ((! skip) && (enable_filter_links_should_connect_by_vars || enable_filter_not_all_first_outgoing_const))
    {

        // check if in this combination, if at least one node in each Seq of oneOfEachSeqShouldBeVars is considered as variable
        bool allSeqContainsVar = true;
        for (HandleSeq& oneSharedSeq : oneOfEachSeqShouldBeVars)
        {
            bool thisSeqContainsVar = false;
            for (Handle& toBeSharedNode : oneSharedSeq)
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
        for (Handle leaf  : leaves)
        {
            if (patternVarMap.find(leaf) != patternVarMap.end())
            {
                skip = true;
                break;
            }
        }
    }

    if (! skip)
    {

        HandleSeq pattern, unifiedPattern;

        for (Handle link : inputLinks)
        {
            HandleSeq outgoingLinks;
            generateALinkByChosenVariables(link, patternVarMap, outgoingLinks, _fromAtomSpace);
            Handle rebindedLink = atomSpace->add_link(atomSpace->get_type(link), outgoingLinks);
            rebindedLink->merge(TruthValue::TRUE_TV());

            pattern.push_back(rebindedLink);
        }

        if ( gram > 2)
        {
            if (containsLoopVariable(pattern))
            return returnHTreeNode;
        }

        // unify the pattern
        unifiedPattern = UnifyPatternOrder(pattern, extendedLinkIndex);

        // next, check if this pattern already exist (need lock)
        HTreeNode* newHTreeNode = 0;


        if (run_as_distributed_worker)
        {
            newHTreeNode = new HTreeNode();
            returnHTreeNode = newHTreeNode;
            newHTreeNode->count = 1;
            newHTreeNode->pattern = unifiedPattern;
            newHTreeNode->var_num = patternVarMap.size();
            returnHTreeNode = newHTreeNode;

        }
        else
        {
            string keyString = unifiedPatternToKeyString(unifiedPattern);

            string instancekeyString = "";
            if ( (gram > 1) && (THREAD_NUM > 1) )
            {
                // check if these fact links already been processed before or by other thread

                OrderedHandleSet originalLinksSet(inputLinks.begin(), inputLinks.end());

                for (Handle h  : originalLinksSet)
                {
                    instancekeyString +=  toString(h.value());
                    instancekeyString += "_";
                }
            }

            if (THREAD_NUM > 1)
                uniqueKeyLock.lock();

            map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(keyString);

            if (htreeNodeIter == keyStrToHTreeNodeMap.end())
            {
                newHTreeNode = new HTreeNode();
                returnHTreeNode = newHTreeNode;
                newHTreeNode->count = 1;

                if (gram > 1  && (THREAD_NUM > 1) )
                {
                    newHTreeNode->instancesUidStrings.insert(instancekeyString);
                }

                keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(keyString, newHTreeNode));

    //            cout << "A new pattern Found:\n"<< keyString << std::endl;



            }
            else
            {
                returnHTreeNode = ((HTreeNode*)(htreeNodeIter->second));

                bool alreadyExtracted = false;
                if (gram > 1  && (THREAD_NUM > 1))
                {
                    // check if these fact links already been processed before or by other thread
                    if (returnHTreeNode->instancesUidStrings.find(instancekeyString) == returnHTreeNode->instancesUidStrings.end())
                    {
                        returnHTreeNode->instancesUidStrings.insert(instancekeyString);
                    }
                    else
                    {
                        alreadyExtracted = true;
                        // debug
                        // cout << "already extracted!" << std::endl;
                    }

                }

                if (! alreadyExtracted)
                    returnHTreeNode->count ++;

    //            cout << "Unique Key already exists:" << keyString << std::endl;

            }

            if (THREAD_NUM > 1)
                uniqueKeyLock.unlock();


            if (newHTreeNode)
            {
                newHTreeNode->pattern = unifiedPattern;
                newHTreeNode->var_num = patternVarMap.size();

                if (THREAD_NUM > 1)
                    addNewPatternLock.lock();

                (patternsForGram[gram-1]).push_back(newHTreeNode);

                if (THREAD_NUM > 1)
                    addNewPatternLock.unlock();

            }
        }



    }

    return returnHTreeNode;

}

// when it's the first gram pattern: parentNode = 0, extendedNode = undefined, lastGramLinks is empty, lastGramValueToVarMap and lastGramPatternVarMap are empty
// extendedNode is the value node in original AtomSpace
// lastGramLinks is the original links the parentLink is extracted from
// patternJsonArray is only used in distributed mode, to buffer the pattern jsons to send to server
void PatternMiner::extendAPatternForOneMoreGramRecursively(const Handle &extendedLink, AtomSpace* _fromAtomSpace, const Handle &extendedNode, const HandleSeq &lastGramLinks,
                 HTreeNode* parentNode, const map<Handle,Handle> &lastGramValueToVarMap, const map<Handle,Handle> &lastGramPatternVarMap,
                 bool isExtendedFromVar, vector<HTreeNode*> &allHTreeNodesCurTask, json::value &patternJsonArray)
{

    // the ground value node in the _fromAtomSpace to the variable handle in pattenmining Atomspace
    map<Handle,Handle> valueToVarMap = lastGramValueToVarMap;

    // First, extract all the nodes in the input link
    extractAllNodesInLink(extendedLink, valueToVarMap, _fromAtomSpace);

    map<Handle,Handle> newValueToVarMap; // the new elements added in this gram

    if (parentNode == 0) // this the first gram
    {
        newValueToVarMap = valueToVarMap;
    }
    else
    {
        map<Handle,Handle>::iterator valueToVarIt = valueToVarMap.begin();
        for(; valueToVarIt != valueToVarMap.end(); valueToVarIt ++)
        {
            if (lastGramValueToVarMap.find(valueToVarIt->first) == lastGramValueToVarMap.end())
                newValueToVarMap.insert( std::pair<Handle,Handle>(valueToVarIt->first,valueToVarIt->second));
        }
    }

    // Generate all the possible combinations of all the nodes: all patterns including the 1 ~ n_max variables
    // If there are too many variables in a pattern, it doesn't make much sense, so we litmit the max number of variables to half of the node number

    unsigned int cur_pattern_gram = 1;
//    unsigned int lastGramTotalNodeNum = 0;
    unsigned int lastGramTotalVarNum = 0;


    if (parentNode)
    {
        cur_pattern_gram = parentNode->pattern.size() + 1;
//        lastGramTotalNodeNum = lastGramValueToVarMap.size();
        lastGramTotalVarNum = parentNode->var_num;
    }

    unsigned int n_max = newValueToVarMap.size();
    unsigned int n_limit= valueToVarMap.size()/2.0f - lastGramTotalVarNum;
    n_limit ++;

    // sometimes there is only one variable in a link, lik:
//    (DuringLink
//      (ConceptNode "dead")
//      (ConceptNode "dead")
//    ) ; [44694]

    if (n_limit > n_max)
        n_limit = n_max;

    if (n_limit == 1)
        n_limit = 2;



    // Get all the shared nodes and leaves
    HandleSeqSeq oneOfEachSeqShouldBeVars;
    HandleSeq leaves, shouldNotBeVars, shouldBeVars;

    HandleSeq inputLinks = lastGramLinks;
    inputLinks.push_back(extendedLink);

    //    OC_ASSERT( (valueToVarMap.size() > 1),
    //              "PatternMiner::extractAllPossiblePatternsFromInputLinks: this group of links only has one node: %s!\n",
    //               atomSpace->atomAsString(inputLinks[0]).c_str() );

    if( filters(inputLinks, oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars,_fromAtomSpace) )
        return; //already been filter out in this phrase

    bool* indexes = new bool[n_max]; //  indexes[i]=true means this i is a variable, indexes[i]=false means this i is a const

/*    // debug
    string lastGramLinksStr = "";
    for (Handle h : lastGramLinks)
        lastGramLinksStr += _fromAtomSpace->atom_as_string(h);

    string inputLinksStr = "";
    for (Handle h : inputLinks)
        inputLinksStr += _fromAtomSpace->atom_as_string(h);

        if ((inputLinksStr.find("man") != inputLinksStr.npos) && (inputLinksStr.find("soda drinker") != inputLinksStr.npos))
        {
            int i = 0; // debug
            i ++;
        }
*/

    // var_num is the number of variables
    unsigned int var_num;
    if (parentNode)
        var_num = 0;
    else
        var_num = 1;

    for (; var_num < n_limit; ++ var_num)
    {
        // Use the binary method to generate all combinations:

        // generate the first combination
        for (unsigned int i = 0; i < var_num; ++ i)
            indexes[i] = true;

        for (unsigned int i = var_num; i <n_max; ++ i)
            indexes[i] = false;

        while (true)
        {
            // construct the pattern for this combination in the PatternMining Atomspace
            // generate the patternVarMap for this pattern of this combination
            map<Handle,Handle>::iterator iter;

            // the first part is the same with its parent node
            map<Handle,Handle> patternVarMap;
            if (parentNode)
            {
                patternVarMap = lastGramPatternVarMap;

                if (! isExtendedFromVar) // need to add the extendedNode into patternVarMap
                {
                    map<Handle,Handle>::const_iterator extendedIter = lastGramValueToVarMap.find(extendedNode);
                    if (extendedIter == lastGramValueToVarMap.end())
                    {
                        cout <<"Exception: can't find extendedNode in lastGramValueToVarMap" << std::endl;
                    }

                    patternVarMap.insert(std::pair<Handle,Handle>(extendedIter->first, extendedIter->second));
                }
            }

            // And then add the second part:
            if (var_num > 0 )
            {
                unsigned int index = 0;
                for (iter = newValueToVarMap.begin(); iter != newValueToVarMap.end(); ++ iter)
                {
                    if (indexes[index]) // this is considered as a variable, add it into the variable to value map
                        patternVarMap.insert(std::pair<Handle,Handle>(iter->first, iter->second));

                    index ++;
                }
            }

            unsigned int extendedLinkIndex = 999;
            HTreeNode* thisGramHTreeNode = extractAPatternFromGivenVarCombination(inputLinks, patternVarMap, oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars,_fromAtomSpace, extendedLinkIndex);

            if (thisGramHTreeNode)
            {

                if (run_as_distributed_worker)
                {
                    allHTreeNodesCurTask.push_back(thisGramHTreeNode);
                    string curPatternKeyStr = unifiedPatternToKeyString(thisGramHTreeNode->pattern);

                    string parentKeyStr = "";

                    if (parentNode)
                    {
                        parentKeyStr = unifiedPatternToKeyString(parentNode->pattern);

                    }
                    else
                    {
                        parentKeyStr = "none";
                    }

                    addPatternsToJsonArrayBuf(curPatternKeyStr, parentKeyStr, extendedLinkIndex, patternJsonArray);
                }
                else
                {
                    // This pattern is the super pattern of all its lastGramHTreeNodes (parentNode)
                    // add an ExtendRelation

                    ExtendRelation relation;
                    relation.extendedHTreeNode = thisGramHTreeNode;
                    relation.newExtendedLink = (thisGramHTreeNode->pattern)[extendedLinkIndex];
                    relation.sharedLink = extendedLink;
                    relation.extendedNode = extendedNode;
                    // relation.isExtendedFromVar = isExtendedFromVar;

                    if (parentNode)
                        parentNode->superPatternRelations.push_back(relation);
                }

                // check if the current gram is already the MAX_GRAM
                if(cur_pattern_gram >= MAX_GRAM)
                {
                    if ( (var_num == 0) || (isLastNElementsAllTrue(indexes, n_max, var_num)))
                        break;

                    // generate the next combination
                    generateNextCombinationGroup(indexes, n_max);

                    continue;
                }


                // Extend one more gram from lastGramHTreeNode to get its superpatterns
                // There are two different super patterns: extended from a variable, and extended from a const by turning it into a variable:
                unsigned int nodeIndex = 0;
                map<Handle,Handle>::iterator niter;
                for (niter = valueToVarMap.begin(); niter != valueToVarMap.end(); ++ niter)
                {

                    Handle extendNode = (Handle)(niter->first);
                    if (enable_filter_node_types_should_not_be_vars)
                    {
                        bool isIgnoredType = false;
                        Type t = _fromAtomSpace->get_type(extendNode);
                        for (Type noType : node_types_should_not_be_vars)
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

                    bool isNewExtendedFromVar;
                    if (indexes[nodeIndex]) // this is considered as a variable
                    {   // Type 1: extended from a variable
                        isNewExtendedFromVar = true;
                    }
                    else
                    {
                        // Type 2: extended from a const by turning it into a variable
                        isNewExtendedFromVar = false;
                    }

                    // find what are the other links in the original Atomspace contain this variable
                    HandleSeq incomings;
                    extendNode->getIncomingSet(back_inserter(incomings));

                    // debug
                    // string curvarstr = _fromAtomSpace->atomAsString(extendNode);

                    for (Handle incomingHandle : incomings)
                    {
                        Handle extendedHandle;
                        // if this atom is of igonred type, get its first ancestor that is not in the igonred types
                        if (isIgnoredType (_fromAtomSpace->get_type(incomingHandle)) )
                        {
                            extendedHandle = getFirstNonIgnoredIncomingLink(_fromAtomSpace, incomingHandle);
                            if (extendedHandle == Handle::UNDEFINED)
                                continue;
                        }
                        else
                            extendedHandle = incomingHandle;


                        string extendedHandleStr = _fromAtomSpace->atom_as_string(extendedHandle);

                        if (isInHandleSeq(extendedHandle, inputLinks))
                            continue;

                        // debug
                        // cout << "Debug: Extended link :" << extendedHandleStr << std::endl;

                        if (extendedHandleStr.find("$var") != std::string::npos)
                        {
                           // cout << "Debug: error! The extended link contines variables!" << extendedHandleStr << std::endl;
                            continue;
                        }

                        // extract patterns from these child
                        extendAPatternForOneMoreGramRecursively(extendedHandle,  _fromAtomSpace, extendNode, inputLinks, thisGramHTreeNode,
                                                                valueToVarMap,patternVarMap,isNewExtendedFromVar, allHTreeNodesCurTask, patternJsonArray);
                    }

                    nodeIndex ++;
                }



            }




            if ( (var_num == 0) || (isLastNElementsAllTrue(indexes, n_max, var_num)))
                break;

            // generate the next combination
            generateNextCombinationGroup(indexes, n_max);
        }
    }

    delete [] indexes;

//    HandleSeq instance;
//    instance.push_back(startLink);

//    // First, extract all the variable nodes in the instance links
//    // we only extend one more link on the nodes that are considered as varaibles for this pattern
//    map<Handle, unsigned int> allVarNodes;

//    unsigned int index = 0;
//    for (Handle link : instance)
//    {
//        extractAllNodesInLink(link, allVarNodes, _fromAtomSpace, index);
//        index ++;
//    }


}

// allLastGramHTreeNodes is input, allFactLinksToPatterns is output - the links fact to all its pattern HTreeNodes
void PatternMiner::extendAllPossiblePatternsForOneMoreGramDF(HandleSeq &instance, AtomSpace* _fromAtomSpace, unsigned int gram,
     vector<HTreeNode*>& allLastGramHTreeNodes, map<HandleSeq, vector<HTreeNode*> >& allFactLinksToPatterns, vector<OrderedHandleSet>& newConnectedLinksFoundThisGram)
{

    // First, extract all the variable nodes in the instance links
    // we only extend one more link on the nodes that are considered as varaibles for this pattern
    map<Handle, unsigned int> allVarNodes;

    unsigned int index = 0;
    for (Handle link : instance)
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
            Type t = _fromAtomSpace->get_type(extendNode);
            for (Type noType : node_types_should_not_be_vars)
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
        HandleSeq incomings;
        extendNode->getIncomingSet(back_inserter(incomings));

        // debug
        // string curvarstr = _fromAtomSpace->atomAsString((Handle)(*varIt));

        for (Handle incomingHandle : incomings)
        {
            Handle extendedHandle;
            // if this atom is of igonred type, get its first ancestor that is not in the igonred types
            if (isIgnoredType (_fromAtomSpace->get_type(incomingHandle)) )
            {
                extendedHandle = getFirstNonIgnoredIncomingLink(_fromAtomSpace, incomingHandle);
                if (extendedHandle == Handle::UNDEFINED)
                    continue;
            }
            else
                extendedHandle = incomingHandle;


            string extendedHandleStr = _fromAtomSpace->atom_as_string(extendedHandle);

            if (isInHandleSeq(extendedHandle, instance))
                continue;

            // debug
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

            vector<OrderedHandleSet >::iterator newExtendIt;
            OrderedHandleSet originalLinksToSet(originalLinks.begin(),originalLinks.end());
            bool alreadyExtracted = false;

            // check if these links have been extracted
            for(newExtendIt = newConnectedLinksFoundThisGram.begin(); newExtendIt != newConnectedLinksFoundThisGram.end(); ++newExtendIt)
            {
                OrderedHandleSet& exitstingLinks = (OrderedHandleSet&)(*newExtendIt);
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
//                OrderedHandleSet sharedNodes; // only the current extending shared node is in sharedNodes for Depth first
//                sharedNodes.insert(extendNode);
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
void PatternMiner::extractAllPossiblePatternsFromInputLinksDF(HandleSeq& inputLinks,unsigned int sharedLinkIndex, AtomSpace* _fromAtomSpace,
                                                              vector<HTreeNode*>& allLastGramHTreeNodes, vector<HTreeNode*>& allHTreeNodes, unsigned int gram)
{
    map<Handle,Handle> valueToVarMap;  // the ground value node in the _fromAtomSpace to the variable handle in pattenmining Atomspace

//    // Debug
//    cout << "Extract patterns from these links: \n";
//    for (Handle ih : inputLinks)
//    {
//        cout << _fromAtomSpace->atomAsString(ih) << std::endl;
//    }

    // First, extract all the nodes in the input links
    for (Handle link : inputLinks)
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



    // Get all the shared nodes and leaves
    HandleSeqSeq oneOfEachSeqShouldBeVars;
    HandleSeq leaves, shouldNotBeVars, shouldBeVars;

    if (filters(inputLinks, oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars, _fromAtomSpace))
        return; // already been filter out in this phrase

    bool* indexes = new bool[n_max]; //  indexes[i]=true means this i is a variable, indexes[i]=false means this i is a const

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

            if (enable_filter_links_should_connect_by_vars || enable_filter_not_all_first_outgoing_const)
            {

                // check if in this combination, if at least one node in each Seq of oneOfEachSeqShouldBeVars is considered as variable
                bool allSeqContainsVar = true;
                for (HandleSeq& oneSharedSeq : oneOfEachSeqShouldBeVars)
                {
                    bool thisSeqContainsVar = false;
                    for (Handle& toBeSharedNode : oneSharedSeq)
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
                for (Handle leaf  : leaves)
                {
                    if (patternVarMap.find(leaf) != patternVarMap.end())
                    {
                        skip = true;
                        break;
                    }
                }
            }

            if ( (! skip) && ( shouldNotBeVars.size() > 0) )
            {
                for (Handle noTypeNode  : shouldNotBeVars)
                {
                    if (patternVarMap.find(noTypeNode) != patternVarMap.end())
                    {
                        skip = true;
                        break;
                    }
                }
            }


            if ((! skip) && (shouldBeVars.size() > 0))
            {
                for (Handle shouldBeVarNode  : shouldBeVars)
                {
                    if (patternVarMap.find(shouldBeVarNode) == patternVarMap.end())
                    {
                        skip = true;
                        break;
                    }
                }
            }



            if (! skip)
            {

                HandleSeq pattern, unifiedPattern;

                for (Handle link : inputLinks)
                {
                    HandleSeq outgoingLinks;
                    generateALinkByChosenVariables(link, patternVarMap, outgoingLinks, _fromAtomSpace);
                    Handle rebindedLink = atomSpace->add_link(atomSpace->get_type(link), outgoingLinks);
                    rebindedLink->merge(TruthValue::TRUE_TV());

                    pattern.push_back(rebindedLink);
                }

                // unify the pattern
                unsigned int unifiedLastLinkIndex;
                unifiedPattern = UnifyPatternOrder(pattern, unifiedLastLinkIndex);

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

                    for (HTreeNode* lastGramHTreeNode : allLastGramHTreeNodes)
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

    delete [] indexes;

}
