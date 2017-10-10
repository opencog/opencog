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
#include <opencog/util/Config.h>
#include <opencog/util/algorithm.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


//void PatternMiner::growPatternsDepthFirstTask_old()
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
//        if (cur_link->getType() == opencog::LIST_LINK)
//        {
//            continue;
//        }

//        // Add this link into observingAtomSpace
//        HandleSeq outgoingLinks,outVariableNodes;

//        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
//        Handle newLink = observingAtomSpace->add_link(cur_link->getType(), outgoingLinks);
//        newLink->setTruthValue(cur_link->getTruthValue());

//        HandleSeq observedLinks;
//        observedLinks.push_back(newLink);

//        // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns

//        vector<HTreeNode*> _allLastGramHTreeNodes; // it's empty for the 1-gram patterns, because there is no last gram

//        vector<HTreeNode*> _allThisGramHTreeNodes;

//        extractAllPossiblePatternsFromInputLinksDF(observedLinks, 0, observingAtomSpace, _allLastGramHTreeNodes,_allThisGramHTreeNodes,1);

//        map<HandleSeq, vector<HTreeNode*> > allLastGramLinksToPatterns; // for this cur_link
//        allLastGramLinksToPatterns.insert(std::pair<HandleSeq, vector<HTreeNode*>>(observedLinks, _allThisGramHTreeNodes));

//        unsigned int gram;


//        for ( gram = 2; gram <= MAX_GRAM; ++ gram)
//        {
//            map<HandleSeq, vector<HTreeNode*> > ::iterator it = allLastGramLinksToPatterns.begin();
//            map<HandleSeq, vector<HTreeNode*> > allThisGramLinksToPatterns;
//            vector<HandleSet> newConnectedLinksFoundThisGram;

//            for(; it != allLastGramLinksToPatterns.end(); ++ it)
//            {
//                // find all the 2~MAX_GRAM gram distance neighbour links of newLink
//                extendAllPossiblePatternsForOneMoreGramDF((HandleSeq&)(it->first),observingAtomSpace,gram, (vector<HTreeNode*>&)(it->second), allThisGramLinksToPatterns, newConnectedLinksFoundThisGram);
//            }

//            allLastGramLinksToPatterns = allThisGramLinksToPatterns;

//        }

//    }
//}


void PatternMiner::growPatternsDepthFirstTask(unsigned int thread_index)
{

	// The start index in allLinks for current thread
    unsigned int start_index = linksPerThread * thread_index;
    unsigned int end_index; // the last index for current thread (excluded)
    if (thread_index == THREAD_NUM - 1)
    {
        // if this the last thread, it
        // needs to finish all the
        // rest of the links
        if (only_mine_patterns_start_from_white_list)
            end_index = allLinksContainWhiteKeywords.size();
        else
            end_index = allLinkNumber;
    }
    else
        end_index = linksPerThread * (thread_index + 1);


    cout << "\nStart thread " << thread_index << ": will process Link number from " << start_index
         << " to (excluded) " << end_index << std::endl;


    float allLinkNumberfloat = ((float)(end_index - start_index));

    for(unsigned int t_cur_index = start_index; t_cur_index < end_index; ++t_cur_index)
    {
        if (THREAD_NUM > 1)
            readNextLinkLock.lock();

        cout<< "\r" << ((float)(t_cur_index - start_index))/allLinkNumberfloat*100.0f << "% completed in Thread " + toString(thread_index) + "."; // it's not liner
        std::cout.flush();

        processedLinkNum ++;

        Handle cur_link;

        if (only_mine_patterns_start_from_white_list)
        {
            cur_link = allLinksContainWhiteKeywords[t_cur_index];
            havenotProcessedWhiteKeywordLinks.erase(havenotProcessedWhiteKeywordLinks.find(cur_link));

        }
        else
            cur_link = allLinks[t_cur_index];


        if (THREAD_NUM > 1)
            readNextLinkLock.unlock();

        // Extract all the possible patterns from this originalLink, and extend till the max_gram links, not duplicating the already existing patterns
        HandleSeq lastGramLinks;
        map<Handle,Handle> lastGramValueToVarMap;
        map<Handle,Handle> patternVarMap;

        set<string> allNewMinedPatternsCurTask;
        bool startFromLinkContainWhiteKeyword = false;

        // allHTreeNodesCurTask is only used in distributed version;
        // is to store all the HTreeNode* mined in this current task, and release them after the task is finished.
        vector<HTreeNode*> allHTreeNodesCurTask;

        // allNewMinedPatternInfo is only used in distributed version, to store all the new mined patterns in this task for sending to the server.
        vector<MinedPatternInfo> allNewMinedPatternInfo;

        if (! only_mine_patterns_start_from_white_list)
        {
            if (use_linktype_black_list && isIgnoredType (cur_link->getType()) )
            {
                continue;
            }
            else if (use_linktype_white_list && (! isTypeInList(cur_link->getType(), linktype_white_list)))
            {
                continue;
            }

            if (use_keyword_black_list)
            {
                // if the content in this link contains content in the black list,ignore it
                if (containIgnoredContent(cur_link))
                    continue;
            }

            if (only_output_patterns_contains_white_keywords)
            {
                if (havenotProcessedWhiteKeywordLinks.find(cur_link) != havenotProcessedWhiteKeywordLinks.end())
                    startFromLinkContainWhiteKeyword = true;
            }

            // Add this link into observingAtomSpace
            HandleSeq outgoingLinks, outVariableNodes;

            swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, cur_link, outgoingLinks, outVariableNodes);
            Handle newLink = observingAtomSpace->add_link(cur_link->getType(), outgoingLinks);
            newLink->setTruthValue(cur_link->getTruthValue());

            extendAPatternForOneMoreGramRecursively(newLink, observingAtomSpace, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap,patternVarMap, false,
                                                    allNewMinedPatternsCurTask, allHTreeNodesCurTask, allNewMinedPatternInfo, thread_index,startFromLinkContainWhiteKeyword);


        }
        else
        {
            extendAPatternForOneMoreGramRecursively(cur_link, originalAtomSpace, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap,patternVarMap, false,
                                                    allNewMinedPatternsCurTask, allHTreeNodesCurTask, allNewMinedPatternInfo, thread_index,startFromLinkContainWhiteKeyword);

        }

        if (THREAD_NUM > 1)
            actualProcessedLinkLock.lock();

        actualProcessedLinkNum ++;

        if (THREAD_NUM > 1)
            actualProcessedLinkLock.unlock();
    }

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
    if (observingAtomSpace == 0)
        observingAtomSpace = new AtomSpace();

    if (THREAD_NUM > 1)
    {
        thread_DF_ExtractedLinks = new list<string>* [THREAD_NUM];
        for (unsigned int ti = 0; ti < THREAD_NUM; ti ++)
            thread_DF_ExtractedLinks[ti] = new list<string>[MAX_GRAM];

        all_thread_ExtractedLinks_pergram = new set<string>[MAX_GRAM];
    }

    if (only_mine_patterns_start_from_white_list)
        linksPerThread = allLinksContainWhiteKeywords.size() / THREAD_NUM;
    else
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

    clear_by_swap(allLinks);

    if (THREAD_NUM > 1)
    {
        for (unsigned int ti = 0; ti < THREAD_NUM; ti ++)
            delete [] (thread_DF_ExtractedLinks[ti]);

        delete [] thread_DF_ExtractedLinks;

        delete [] all_thread_ExtractedLinks_pergram;

    }

    delete [] threads;

    cout << "\nFinished mining 1~" << MAX_GRAM << " gram patterns.\n";
    cout << "\ntotalLinkNum = " << processedLinkNum << ", actualProcessedLinkNum = " << actualProcessedLinkNum << std::endl;


}

// extendedLinkIndex is to return the index of extendedLink's patternlink in the unified pattern so as to identify where is the extended link in this pattern
// vector<HTreeNode*> &allHTreeNodesCurTask is only used in distributed version
// notOutPutPattern is passed from extendAPatternForOneMoreGramRecursively, also may be modify in this function.
// it indicates if one pattern is only generated for middle process - calculate interestingness for its superpatterns, but not put in output results
HTreeNode* PatternMiner::extractAPatternFromGivenVarCombination(HandleSeq &inputLinks, map<Handle,Handle> &patternVarMap, map<Handle,Handle>& orderedVarNameMap,HandleSeqSeq &oneOfEachSeqShouldBeVars, HandleSeq &leaves,
                                                                HandleSeq &shouldNotBeVars, HandleSeq &shouldBeVars,AtomSpace* _fromAtomSpace, unsigned int & extendedLinkIndex,
                                                                set<string>& allNewMinedPatternsCurTask, bool& notOutPutPattern, bool &patternAlreadyExtractedInCurTask, bool startFromLinkContainWhiteKeyword)
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


    if ( (! skip) && (enable_filter_leaves_should_not_be_vars) && (gram > 1)) // for gram > 1, any leaf should not considered as variable
    {
        for (Handle leaf  : leaves)
        {

            if (patternVarMap.find(leaf) != patternVarMap.end())
            {
                if (GENERATE_TMP_PATTERNS)
                {
                    if (gram == MAX_GRAM) // only skip for the max gram patterns
                        skip = true;
                    else // for smaller patterns, still need to generate, just put in keyStrToHTreeNodeMap, not put in patternsForGram
                        notOutPutPattern = true;
                }
                else
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
            Handle rebindedLink = atomSpace->add_link(link->getType(), outgoingLinks);
            rebindedLink->setTruthValue(TruthValue::TRUE_TV());

            pattern.push_back(rebindedLink);
        }

        if ( gram > 2)
        {
            if (containsLoopVariable(pattern))
            return 0;
        }

        // unify the pattern

        unifiedPattern = UnifyPatternOrder(pattern, extendedLinkIndex, orderedVarNameMap);

        string keyString = unifiedPatternToKeyString(unifiedPattern);


        // check if this pattern has been found in current Link task
        if (allNewMinedPatternsCurTask.find(keyString) != allNewMinedPatternsCurTask.end())
        {
            patternAlreadyExtractedInCurTask = true;

            return keyStrToHTreeNodeMap[keyString];
        }
        else
        {
            patternAlreadyExtractedInCurTask = false;
            allNewMinedPatternsCurTask.insert(keyString);
        }



        HTreeNode* newHTreeNode = 0;

        if (is_distributed)
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

            // next, check if this pattern already exist (need lock)

            if (THREAD_NUM > 1)
                uniqueKeyLock.lock();

            map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(keyString);

            if (htreeNodeIter == keyStrToHTreeNodeMap.end())
            {
                newHTreeNode = new HTreeNode();
                returnHTreeNode = newHTreeNode;
                newHTreeNode->count = 1;


                keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(keyString, newHTreeNode));

    //            cout << "A new pattern Found:\n"<< keyString << std::endl;

            }
            else
            {
                returnHTreeNode = ((HTreeNode*)(htreeNodeIter->second));

                returnHTreeNode->count ++;

    //            cout << "Unique Key already exists:" << keyString << std::endl;

            }

            if (THREAD_NUM > 1)
                uniqueKeyLock.unlock();


            if (newHTreeNode)
            {
                if (! GENERATE_TMP_PATTERNS)
                    notOutPutPattern = false;

                newHTreeNode->pattern = unifiedPattern;
                newHTreeNode->var_num = patternVarMap.size();


                if ( (! notOutPutPattern) && only_output_patterns_contains_white_keywords)
                {
                    // check if this instance contain any white keywords
                    if (! startFromLinkContainWhiteKeyword) // if it already starts from Links that contain keywords , do not need to check
                    {
                        bool is_contain = false;
                        for (Handle link : inputLinks)
                        {
                            if (containKeywords(link->toShortString(), keyword_white_list, keyword_white_list_logic))
                            {
                                is_contain = true;
                                break;
                            }
                        }

                        if (GENERATE_TMP_PATTERNS)
                        {
                            if (! is_contain)
                                notOutPutPattern = true;
                        }
                    }
                }

                if (THREAD_NUM > 1)
                    addNewPatternLock.lock();

                if (notOutPutPattern)
                {
                    if (GENERATE_TMP_PATTERNS)
                        (tmpPatternsForGram[gram-1]).push_back(newHTreeNode);
                }
                else
                    (patternsForGram[gram-1]).push_back(newHTreeNode);

                if (THREAD_NUM > 1)
                    addNewPatternLock.unlock();

            }
        }

    }

    return returnHTreeNode;

}

// call existInOneThreadExtractedLinks and existInAllThreadExtractedLinks before this call function
// only keep 60 per thread per gram, should be enough
void PatternMiner::addThreadExtractedLinks(unsigned int _gram, unsigned int cur_thread_index, string _extractedLinkUIDs)
{
    threadExtractedLinksLock.lock();

    (thread_DF_ExtractedLinks[cur_thread_index][_gram - 1]).push_back(_extractedLinkUIDs);

    all_thread_ExtractedLinks_pergram[_gram - 1].insert(_extractedLinkUIDs);

    if ((thread_DF_ExtractedLinks[cur_thread_index][_gram - 1]).size() > 60)
    {
        set<string>::iterator uidIt = all_thread_ExtractedLinks_pergram[_gram - 1].find((thread_DF_ExtractedLinks[cur_thread_index][_gram - 1]).front());
        if (uidIt != all_thread_ExtractedLinks_pergram[_gram - 1].end())
        {
            all_thread_ExtractedLinks_pergram[_gram - 1].erase(uidIt);
        }

        (thread_DF_ExtractedLinks[cur_thread_index][_gram - 1]).pop_front();
    }

    threadExtractedLinksLock.unlock();

}

bool PatternMiner::existInAllThreadExtractedLinks(unsigned int _gram, string _extractedLinkUIDs)
{
    threadExtractedLinksLock.lock();
    if (all_thread_ExtractedLinks_pergram[_gram - 1].find(_extractedLinkUIDs) == all_thread_ExtractedLinks_pergram[_gram - 1].end())
    {
        threadExtractedLinksLock.unlock();
        return false;
    }
    else
    {
        threadExtractedLinksLock.unlock();
        return true;
    }

}

bool PatternMiner::existInOneThreadExtractedLinks(unsigned int _gram, unsigned int cur_thread_index, string _extractedLinkUIDs)
{
    for (string uids : (thread_DF_ExtractedLinks[cur_thread_index][_gram - 1]))
    {
        if (_extractedLinkUIDs == uids)
            return true;
    }

    return false;
}

// when it's the first gram pattern: parentNode = 0, extendedNode = undefined, lastGramLinks is empty, lastGramValueToVarMap and lastGramPatternVarMap are empty
// extendedNode is the value node in original AtomSpace
// lastGramLinks is the original links the parentLink is extracted from
// allNewMinedPatternsCurTask is to store all the pattern keystrings mined in current Link task, to avoid duplicate patterns being mined
// allNewMinedPatternInfo is only used in distributed mode, to store all the new mined pattern info to send to server
void PatternMiner::extendAPatternForOneMoreGramRecursively(const Handle &extendedLink, AtomSpace* _fromAtomSpace, const Handle &extendedNode, const HandleSeq &lastGramLinks,
                 HTreeNode* parentNode, const map<Handle,Handle> &lastGramValueToVarMap, const map<Handle,Handle> &lastGramPatternVarMap, bool isExtendedFromVar,
                 set<string>& allNewMinedPatternsCurTask, vector<HTreeNode*> &allHTreeNodesCurTask, vector<MinedPatternInfo> &allNewMinedPatternInfo, unsigned int thread_index,
                 bool startFromLinkContainWhiteKeyword)
{

    // the ground value node in the _fromAtomSpace to the variable handle in pattenmining Atomspace
    map<Handle,Handle> valueToVarMap = lastGramValueToVarMap;

    // First, extract all the nodes in the input link
    extractAllNodesInLink(extendedLink, valueToVarMap, _fromAtomSpace);

    map<Handle,Handle> newValueToVarMap; // the new elements added in this gram

    bool notOutPutPattern = false;

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
    unsigned int n_limit;
    unsigned int n_limit_putin_result = n_max;

    // sometimes there is only one variable in a link, like:
    //    (DuringLink
    //      (ConceptNode "dead")
    //      (ConceptNode "dead")
    //    ) ; [44694]
    n_limit_putin_result = valueToVarMap.size() * max_var_num_percent - lastGramTotalVarNum;
    n_limit_putin_result ++;

    if (n_limit_putin_result == 1)
        n_limit_putin_result = 2;

    if (GENERATE_TMP_PATTERNS)
    {
        n_limit = n_limit_putin_result + 1;
    }
    else
    {
        n_limit = n_limit_putin_result;

    }

    // sometimes an extended Link do not have any new nodes compared to its parent
    // so  that newValueToVarMap.size() can be , e.g.:
//        (EvaluationLink (stv 1.000000 1.000000)
//          (PredicateNode "spouse")
//          (ListLink (stv 1.000000 1.000000)
//            (ConceptNode "Ann_Druyan")
//            (ConceptNode "Carl_Sagan")
//          )
//        )
//        (EvaluationLink (stv 1.000000 1.000000)
//          (PredicateNode "spouse")
//          (ListLink (stv 1.000000 1.000000)
//            (ConceptNode "Carl_Sagan")
//            (ConceptNode "Ann_Druyan")
//          )
//        )
    if (n_max == 0)
    {
        n_limit_putin_result = 1;
        n_limit = 1;
    }
    else
    {
        if (n_limit_putin_result > n_max)
            n_limit_putin_result = n_max;

        if (n_limit > n_max)
            n_limit = n_max;
    }


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

//    if ((cur_pattern_gram == 2) && (n_max == 0)) // debug
//    {
//        int x = 0;
//        x ++;
//        for(Handle h : inputLinks)
//            cout << h->toShortString();

//        cout << std::endl;
//    }
//    // debug
//    string lastGramLinksStr = "";
//    for (Handle h : lastGramLinks)
//        lastGramLinksStr += h->toShortString();

//    string inputLinksStr = "";
//    for (Handle h : inputLinks)
//        inputLinksStr += h->toShortString();



    // var_num is the number of variables
    unsigned int var_num;
    if (parentNode)
        var_num = 0;
    else
        var_num = 1;

    // todo: store the var combinations to HTreeNode map for finding super/sub_patternrelation_b
    // the string is one combination of indexes to string , e.g.: "0010"
    map<string, HTreeNode*> varCombinToHTreeNode;

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

            if (GENERATE_TMP_PATTERNS)
            {
                if ((cur_pattern_gram > 1) &&(var_num >= n_limit_putin_result))
                    notOutPutPattern = true;
            }

            unsigned int extendedLinkIndex = 999;
            bool patternAlreadyExtractedInCurTask;
            map<Handle,Handle> orderedVarNameMap;

            HTreeNode* thisGramHTreeNode = extractAPatternFromGivenVarCombination(inputLinks, patternVarMap, orderedVarNameMap,oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars,
                                          _fromAtomSpace, extendedLinkIndex, allNewMinedPatternsCurTask, notOutPutPattern, patternAlreadyExtractedInCurTask, startFromLinkContainWhiteKeyword);

            if (thisGramHTreeNode)
            {

                if (is_distributed)
                {
                    allHTreeNodesCurTask.push_back(thisGramHTreeNode);

                    MinedPatternInfo pInfo;

                    pInfo.curPatternKeyStr = unifiedPatternToKeyString(thisGramHTreeNode->pattern);

                    if (parentNode)
                    {
                        pInfo.parentKeyString = unifiedPatternToKeyString(parentNode->pattern);

                    }
                    else
                    {
                        pInfo.parentKeyString = "none";
                    }

                    pInfo.extendedLinkIndex = extendedLinkIndex;

                    if (GENERATE_TMP_PATTERNS)
                        pInfo.notOutPutPattern = notOutPutPattern;
                    else
                        pInfo.notOutPutPattern = false;

                    allNewMinedPatternInfo.push_back(pInfo);


                }
                else
                {
                    // This pattern is the super pattern of all its lastGramHTreeNodes (parentNode)
                    // add an ExtendRelation

                    // check if this relation already exist
                    if (parentNode)
                    {
                        bool superPatternRelationExist = false;
                        for (ExtendRelation& existRelation : parentNode->superPatternRelations)
                        {
                            if (existRelation.extendedHTreeNode == thisGramHTreeNode)
                            {
                                // debug
//                                cout << "\nsuperPatternRelation already exist!\n";
                                superPatternRelationExist = true;
                                break;
                            }
                        }

                        if (! superPatternRelationExist)
                        {
                            ExtendRelation relation;
                            relation.extendedHTreeNode = thisGramHTreeNode;
                            relation.newExtendedLink = (thisGramHTreeNode->pattern)[extendedLinkIndex];
                            relation.sharedLink = extendedLink;
                            relation.extendedNode = extendedNode;
                            relation.isExtendedFromVar = isExtendedFromVar;
                            parentNode->superPatternRelations.push_back(relation);
                        }
                    }
                }

                if (! patternAlreadyExtractedInCurTask)
                {
                    //  track type b super-sub relations - currently only for 1-gram patterns
                    if ((cur_pattern_gram == 1) && (calculate_type_b_surprisingness) && (var_num <= n_limit_putin_result))
                    {

//                        cout << "\n-------------------Found sub type b patterns for current pattern: -----------------\n";
//                        for (Handle plink : thisGramHTreeNode->pattern)
//                            cout << plink->toShortString();
//                        cout << std::endl;
                        string indexStr = "";

                        for (unsigned int  index_i = 0; index_i < n_max ; index_i ++)
                        {
                            if (indexes[index_i]) // if this node in this index is variable
                            {
                                indexStr += "1";

                                // changed it back into a const,
                                // and try to find its type-b sub patterns in varCombinToHTreeNode

                                // first generate the sub pattern varCombin string
                                string sub_b_str = "";

                                for (unsigned int  b_index_i = 0; b_index_i < n_max ; b_index_i ++)
                                {
                                    if (indexes[b_index_i]) // if this node in this index is variable
                                    {
                                        if (b_index_i == index_i) // change it into const
                                            sub_b_str += "0";
                                        else
                                            sub_b_str += "1";
                                    }
                                    else
                                        sub_b_str  += "0";

                                }

                                // find the type b sub pattern with the sub pattern varCombin string
                                map<string, HTreeNode*>::iterator sub_b_it = varCombinToHTreeNode.find(sub_b_str);
                                if (sub_b_it != varCombinToHTreeNode.end())
                                {
                                    HTreeNode* sub_b_htreenode = (HTreeNode*)(sub_b_it->second);

                                    SuperRelation_b superb;
                                    superb.superHTreeNode = thisGramHTreeNode;

                                    unsigned int const_index = 0;
                                    Handle oldVarHandle;
                                    for (iter = newValueToVarMap.begin(); iter != newValueToVarMap.end(); ++ iter)
                                    {
                                        if (const_index == index_i)
                                        {
                                            superb.constNode = iter->first;
                                            oldVarHandle = iter->second;

                                            break;
                                        }

                                        const_index ++;
                                    }

                                    // add the super b relation into the sub_b_treednode
                                    sub_b_htreenode->superRelation_b_list.push_back(superb);


                                    //
                                    // find out what is variable name this const node became in current pattern:
                                    Handle orderedVarHandle = orderedVarNameMap[oldVarHandle];

                                    SubRelation_b onesub_b;
                                    onesub_b.subHTreeNode = sub_b_htreenode;
                                    onesub_b.constNode = superb.constNode;

                                    // find if this variable already exists in the type b sub pattern relations of current pattern
                                    map<Handle, vector<SubRelation_b>>::iterator sub_b_relation_it = thisGramHTreeNode->SubRelation_b_map.find(orderedVarHandle);
                                    if (sub_b_relation_it == thisGramHTreeNode->SubRelation_b_map.end())
                                    {
                                        vector<SubRelation_b> subRelations;
                                        subRelations.push_back(onesub_b);
                                        thisGramHTreeNode->SubRelation_b_map.insert(std::pair<Handle, vector<SubRelation_b>>(orderedVarHandle, subRelations));

                                    }
                                    else
                                    {
                                        (sub_b_relation_it->second).push_back(onesub_b);
                                    }

//                                    cout << "const node:" << superb.constNode->toShortString() << std::endl;
//                                    cout << "variable node: " << orderedVarHandle->toShortString() << std::endl;
//                                    cout << "Sub pattern: " << std::endl;
//                                    for (Handle plink : sub_b_htreenode->pattern)
//                                        cout << plink->toShortString();
//                                    cout << std::endl;

                                }
                            }
                            else // it is a const node
                                indexStr += "0";

                        }

                        varCombinToHTreeNode.insert(std::pair<string, HTreeNode*>(indexStr, thisGramHTreeNode));

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

//                    cout << "\n****************************begin**************************************" << std::endl;
//                    cout << "\nCurrent Pattern:" << unifiedPatternToKeyString(thisGramHTreeNode->pattern) ;
                    // Extend one more gram from lastGramHTreeNode to get its superpatterns
                    // There are two different super patterns: extended from a variable, and extended from a const by turning it into a variable:
                    unsigned int nodeIndex = 0;
                    map<Handle,Handle>::iterator niter;

//                    cout << "valueToVarMap:\n";
//                    for (map<Handle, Handle>::const_iterator titer = valueToVarMap.begin(); titer != valueToVarMap.end(); ++ titer)
//                    {
//                        cout << " ( " <<((Handle)(titer->first))->getName() << " , " << ((Handle)(titer->second))->getName() << " ) ";
//                    }

//                    cout << "\n\npatternVarMap:\n";
//                    for (map<Handle, Handle>::const_iterator titer = patternVarMap.begin(); titer != patternVarMap.end(); ++ titer)
//                    {
//                        cout << " ( " <<((Handle)(titer->first))->getName() << " , " << ((Handle)(titer->second))->getName() << " ) ";
//                    }

                    for (niter = valueToVarMap.begin(); niter != valueToVarMap.end(); ++ niter)
                    {
//                        cout << "nodeIndex = " << nodeIndex << std::endl;
                        Handle extendNode = (Handle)(niter->first);
                        if (enable_filter_node_types_should_not_be_vars)
                        {
                            bool isIgnoredType = false;
                            Type t = extendNode->getType();
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

                        if (patternVarMap.find(extendNode) != patternVarMap.end()) // this is considered as a variable
                        {   // Type 1: extended from a variable
                            isNewExtendedFromVar = true;
//                            cout << "\nExtended from var";
                        }
                        else
                        {
                            // Type 2: extended from a const by turning it into a variable
                            isNewExtendedFromVar = false;
//                            cout << "\nExtended from const";
                        }

                        // find what are the other links in the original Atomspace contain this variable
                        IncomingSet incomings = extendNode->getIncomingSet(_fromAtomSpace);

                        // debug
//                        string curvarstr = extendNode->toShortString();
//                        cout << "\n---------------start curvarstr = " << curvarstr << "---------------" <<std::endl;

                        for (LinkPtr incomeingPtr : incomings)
                        {
                            Handle incomingHandle = incomeingPtr->getHandle();
                            Handle extendedHandle = incomingHandle;

                            if (use_linktype_black_list && isIgnoredType (incomingHandle->getType()) )
                            {
                                // if this atom is of igonred type, get its first ancestor that is not in the igonred types
                                extendedHandle = getFirstNonIgnoredIncomingLink(_fromAtomSpace, incomingHandle);

                                if ((extendedHandle == Handle::UNDEFINED))
                                    continue;
                            }
                            else if (use_linktype_white_list && (! isTypeInList(incomingHandle->getType(), linktype_white_list)))
                            {
                                continue;
                            }

                            if (use_keyword_black_list)
                            {
                                if (keyword_black_logic_is_contain)
                                {
                                    if ((keyword_black_list.size() > 0) && containIgnoredContent(extendedHandle))
                                        continue;
                                }
                                else
                                {
                                    if ((black_keyword_Handles.size() > 0) && doesLinkContainNodesInKeyWordNodes(extendedHandle, black_keyword_Handles))
                                        continue;
                                }
                            }

                            if (isInHandleSeq(extendedHandle, inputLinks))
                                continue;

                            if (only_mine_patterns_start_from_white_list)
                            {
                                if (havenotProcessedWhiteKeywordLinks.find(extendedHandle) != havenotProcessedWhiteKeywordLinks.end())
                                    continue;
                            }

//                             string extendedHandleStr = extendedHandle->toShortString();

                            // debug
//                             cout << "Extended link :" << extendedHandleStr << std::endl;

    //                        if (extendedHandleStr.find("PatternVariableNode") != std::string::npos)
    //                        {
    //                           // cout << "Debug: error! The extended link contines variables!" << extendedHandleStr << std::endl;
    //                            continue;
    //                        }

                            // check if other thread happends to process on the same links
                            if (THREAD_NUM > 1)
                            {
                                string instancekeyString = "";

                                // check if these fact links already been processed before or by other thread

                                HandleSet originalLinksSet(inputLinks.begin(), inputLinks.end());
                                originalLinksSet.insert(extendedHandle);

                                for (Handle h  : originalLinksSet)
                                {
                                    instancekeyString +=  toString(h.value());
                                    instancekeyString += "_";
                                }

                                // if this Links combination has been processed by "other" thread but not by current thread, skip it
                                if (! existInOneThreadExtractedLinks(cur_pattern_gram + 1, thread_index, instancekeyString))
                                {
                                    if (existInAllThreadExtractedLinks(cur_pattern_gram + 1, instancekeyString))
                                    {
                                        // cout << "existInOneThreadExtractedLinks: \n" << instancekeyString << std::endl;
                                        continue;
                                    }
                                    else
                                        addThreadExtractedLinks(cur_pattern_gram + 1, thread_index, instancekeyString);
                                }

                            }

                            // extract patterns from this child
                            extendAPatternForOneMoreGramRecursively(extendedHandle,  _fromAtomSpace, extendNode, inputLinks, thisGramHTreeNode, valueToVarMap,patternVarMap,isNewExtendedFromVar,
                                                                    allNewMinedPatternsCurTask, allHTreeNodesCurTask, allNewMinedPatternInfo, thread_index, startFromLinkContainWhiteKeyword);

                        }
//                        cout << "\n---------------end curvarstr = " << curvarstr << "---------------" <<std::endl;
                        nodeIndex ++;
                    }

//                    cout << "\n****************************end**************************************" << std::endl;
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

// this function is old
// allLastGramHTreeNodes is input, allFactLinksToPatterns is output - the links fact to all its pattern HTreeNodes
//void PatternMiner::extendAllPossiblePatternsForOneMoreGramDF(HandleSeq &instance, AtomSpace* _fromAtomSpace, unsigned int gram,
//     vector<HTreeNode*>& allLastGramHTreeNodes, map<HandleSeq, vector<HTreeNode*> >& allFactLinksToPatterns, vector<HandleSet>& newConnectedLinksFoundThisGram)
//{

//    // First, extract all the variable nodes in the instance links
//    // we only extend one more link on the nodes that are considered as varaibles for this pattern
//    map<Handle, unsigned int> allVarNodes;

//    unsigned int index = 0;
//    for (Handle link : instance)
//    {
//        extractAllNodesInLink(link, allVarNodes, _fromAtomSpace, index);
//        index ++;
//    }

//    map<Handle, unsigned int>::iterator varIt;

//    for(varIt = allVarNodes.begin(); varIt != allVarNodes.end(); ++ varIt)
//    {
//        Handle extendNode = (Handle)(varIt->first);
//        if (enable_filter_node_types_should_not_be_vars)
//        {
//            bool isIgnoredType = false;
//            Type t = extendNode->getType();
//            for (Type noType : node_types_should_not_be_vars)
//            {
//                if (t == noType)
//                {
//                    isIgnoredType = true;
//                    break;
//                }
//            }

//            if (isIgnoredType )
//                continue;
//        }

//        // find what are the other links in the original Atomspace contain this variable
//        IncomingSet incomings = extendNode->getIncomingSet(_fromAtomSpace);

//        // debug
//        // string curvarstr = _fromAtomSpace->atomAsString((Handle)(*varIt));

//        for (LinkPtr incomingPtr : incomings)
//        {
//            Handle incomingHandle = incomingPtr->getHandle();
//            Handle extendedHandle;
//            // if this atom is of igonred type, get its first ancestor that is not in the igonred types
//            if (isIgnoredType (incomingHandle->getType()) )
//            {
//                extendedHandle = getFirstNonIgnoredIncomingLink(_fromAtomSpace, incomingHandle);
//                if (extendedHandle == Handle::UNDEFINED)
//                    continue;
//            }
//            else
//                extendedHandle = incomingHandle;


//            string extendedHandleStr = extendedHandle->toShortString();

//            if (isInHandleSeq(extendedHandle, instance))
//                continue;

//            // debug
//            // cout << "Debug: Extended link :" << extendedHandleStr << std::endl;

//            if (extendedHandleStr.find("PatternVariableNode") != std::string::npos)
//            {
//               // cout << "Debug: error! The extended link contines variables!" << extendedHandleStr << std::endl;
//                continue;
//            }

//            // Add this extendedHandle to the old pattern so as to make a new pattern
//            HandleSeq originalLinks = instance;
//            originalLinks.push_back(extendedHandle);

//            // Extract all the possible patterns from this originalLinks, not duplicating the already existing patterns

//            vector<HandleSet >::iterator newExtendIt;
//            HandleSet originalLinksToSet(originalLinks.begin(),originalLinks.end());
//            bool alreadyExtracted = false;

//            // check if these links have been extracted
//            for(newExtendIt = newConnectedLinksFoundThisGram.begin(); newExtendIt != newConnectedLinksFoundThisGram.end(); ++newExtendIt)
//            {
//                HandleSet& exitstingLinks = (HandleSet&)(*newExtendIt);
//                if (exitstingLinks == originalLinksToSet)
//                {
//                    alreadyExtracted = true;
//                    // cout<< "Debug: these links have already been extracted! Skip them!"<<std::endl;
//                    break;
//                }

//            }

//            if (! alreadyExtracted)
//            {
//                newConnectedLinksFoundThisGram.push_back(originalLinksToSet);
////                HandleSet sharedNodes; // only the current extending shared node is in sharedNodes for Depth first
////                sharedNodes.insert(extendNode);
//                vector<HTreeNode*> allThisGramHTreeNodes;
//                extractAllPossiblePatternsFromInputLinksDF(originalLinks, (unsigned int)(varIt->second), _fromAtomSpace, allLastGramHTreeNodes, allThisGramHTreeNodes, gram);
//                allFactLinksToPatterns.insert(std::pair<HandleSeq, vector<HTreeNode*>>(originalLinks, allThisGramHTreeNodes));

//            }

//        }
//    }
//}

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
//void PatternMiner::extractAllPossiblePatternsFromInputLinksDF(HandleSeq& inputLinks,unsigned int sharedLinkIndex, AtomSpace* _fromAtomSpace,
//                                                              vector<HTreeNode*>& allLastGramHTreeNodes, vector<HTreeNode*>& allHTreeNodes, unsigned int gram)
//{
//    map<Handle,Handle> valueToVarMap;  // the ground value node in the _fromAtomSpace to the variable handle in pattenmining Atomspace

////    // Debug
////    cout << "Extract patterns from these links: \n";
////    for (Handle ih : inputLinks)
////    {
////        cout << _fromAtomSpace->atomAsString(ih) << std::endl;
////    }

//    // First, extract all the nodes in the input links
//    for (Handle link : inputLinks)
//        extractAllNodesInLink(link, valueToVarMap, _fromAtomSpace);

//    // Generate all the possible combinations of all the nodes: all patterns including the 1 ~ n_max variables
//    // If there are too many variables in a pattern, it doesn't make much sense, so we litmit the max number of variables to half of the node number

////    OC_ASSERT( (valueToVarMap.size() > 1),
////              "PatternMiner::extractAllPossiblePatternsFromInputLinks: this group of links only has one node: %s!\n",
////               atomSpace->atomAsString(inputLinks[0]).c_str() );

//    int n_max = valueToVarMap.size();
//    int n_limit= valueToVarMap.size()/2.0f;
//    n_limit ++;

//    // sometimes there is only one variable in a link, lik:
////    (DuringLink
////      (ConceptNode "dead")
////      (ConceptNode "dead")
////    ) ; [44694]
//    if (n_limit == 1)
//        n_limit = 2;



//    // Get all the shared nodes and leaves
//    HandleSeqSeq oneOfEachSeqShouldBeVars;
//    HandleSeq leaves, shouldNotBeVars, shouldBeVars;

//    if (filters(inputLinks, oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars, _fromAtomSpace))
//        return; // already been filter out in this phrase

//    bool* indexes = new bool[n_max]; //  indexes[i]=true means this i is a variable, indexes[i]=false means this i is a const

//    // var_num is the number of variables
//    for (int var_num = 1;var_num < n_limit; ++ var_num)
//    {
//        // Use the binary method to generate all combinations:

//        // generate the first combination
//        for (int i = 0; i < var_num; ++ i)
//            indexes[i] = true;

//        for (int i = var_num; i <n_max; ++ i)
//            indexes[i] = false;

//        while (true)
//        {
//            // construct the pattern for this combination in the PatternMining Atomspace
//            // generate the valueToVarMap for this pattern of this combination
//            map<Handle,Handle>::iterator iter;

//            map<Handle,Handle> patternVarMap;

//            bool skip = false;

//            unsigned int index = 0;
//            for (iter = valueToVarMap.begin(); iter != valueToVarMap.end(); ++ iter)
//            {
//                if (indexes[index]) // this is considered as a variable, add it into the variable to value map
//                    patternVarMap.insert(std::pair<Handle,Handle>(iter->first, iter->second));

//                index ++;
//            }

//            if (enable_filter_links_should_connect_by_vars || enable_filter_not_all_first_outgoing_const)
//            {

//                // check if in this combination, if at least one node in each Seq of oneOfEachSeqShouldBeVars is considered as variable
//                bool allSeqContainsVar = true;
//                for (HandleSeq& oneSharedSeq : oneOfEachSeqShouldBeVars)
//                {
//                    bool thisSeqContainsVar = false;
//                    for (Handle& toBeSharedNode : oneSharedSeq)
//                    {
//                        if (patternVarMap.find(toBeSharedNode) != patternVarMap.end())
//                        {
//                            thisSeqContainsVar = true;
//                            break;
//                        }

//                    }

//                    if (! thisSeqContainsVar)
//                    {
//                        allSeqContainsVar = false;
//                        break;
//                    }
//                }

//                if (! allSeqContainsVar)
//                    skip = true;
//            }



//            if ( (! skip) && (enable_filter_leaves_should_not_be_vars) && (gram == MAX_GRAM) ) // for gram > 1, any leaf should not considered as variable
//            {
//                for (Handle leaf  : leaves)
//                {
//                    if (patternVarMap.find(leaf) != patternVarMap.end())
//                    {
//                        skip = true;
//                        break;
//                    }
//                }
//            }

//            if ( (! skip) && ( shouldNotBeVars.size() > 0) )
//            {
//                for (Handle noTypeNode  : shouldNotBeVars)
//                {
//                    if (patternVarMap.find(noTypeNode) != patternVarMap.end())
//                    {
//                        skip = true;
//                        break;
//                    }
//                }
//            }


//            if ((! skip) && (shouldBeVars.size() > 0))
//            {
//                for (Handle shouldBeVarNode  : shouldBeVars)
//                {
//                    if (patternVarMap.find(shouldBeVarNode) == patternVarMap.end())
//                    {
//                        skip = true;
//                        break;
//                    }
//                }
//            }



//            if (! skip)
//            {

//                HandleSeq pattern, unifiedPattern;

//                for (Handle link : inputLinks)
//                {
//                    HandleSeq outgoingLinks;
//                    generateALinkByChosenVariables(link, patternVarMap, outgoingLinks, _fromAtomSpace);
//                    Handle rebindedLink = atomSpace->add_link(link->getType(), outgoingLinks);
//                    rebindedLink->setTruthValue(TruthValue::TRUE_TV());

//                    pattern.push_back(rebindedLink);
//                }

//                // unify the pattern
//                unsigned int unifiedLastLinkIndex;
//                unifiedPattern = UnifyPatternOrder(pattern, unifiedLastLinkIndex);

//                string keyString = unifiedPatternToKeyString(unifiedPattern);

//                // next, check if this pattern already exist (need lock)
//                HTreeNode* newHTreeNode = 0;
//                uniqueKeyLock.lock();

//                map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(keyString);

//                if (htreeNodeIter == keyStrToHTreeNodeMap.end())
//                {
//                    newHTreeNode = new HTreeNode();
//                    keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(keyString, newHTreeNode));


//                    cout << "A new pattern Found:\n"<< keyString << std::endl;

//                    newHTreeNode->count = 1;
//                    allHTreeNodes.push_back(newHTreeNode);

//                }
//                else
//                {

//                    ((HTreeNode*)(htreeNodeIter->second))->count ++;
//                    allHTreeNodes.push_back((HTreeNode*)(htreeNodeIter->second));
//                    cout << "Unique Key already exists. count ++ !\n\n";

//                }

//                uniqueKeyLock.unlock();

//                // if gram > 1, this pattern is the super pattern of al the lastGramHTreeNodes
//                // add ExtendRelations
//                if (gram > 1)
//                {
//                    HTreeNode* superPatternNode;
//                    if (newHTreeNode)
//                        superPatternNode = newHTreeNode;
//                    else
//                        superPatternNode = (HTreeNode*)(htreeNodeIter->second);

//                    for (HTreeNode* lastGramHTreeNode : allLastGramHTreeNodes)
//                    {
//                        ExtendRelation relation;
//                        relation.extendedHTreeNode = superPatternNode;
//                        relation.newExtendedLink = pattern[pattern.size()-1];
//                        relation.sharedLink = lastGramHTreeNode->pattern[sharedLinkIndex];

//                        lastGramHTreeNode->superPatternRelations.push_back(relation);
//                    }
//                }

//                if (newHTreeNode)
//                {
//                    newHTreeNode->pattern = unifiedPattern;
//                    newHTreeNode->var_num = var_num;

//                    addNewPatternLock.lock();
//                    (patternsForGram[gram-1]).push_back(newHTreeNode);
//                    addNewPatternLock.unlock();

//                }

//            }


//            if (isLastNElementsAllTrue(indexes, n_max, var_num))
//                break;

//            // generate the next combination
//            generateNextCombinationGroup(indexes, n_max);
//        }
//    }

//    delete [] indexes;

//}
