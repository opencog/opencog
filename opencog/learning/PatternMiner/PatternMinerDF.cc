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

#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/proto/atom_types.h>
#include <opencog/util/Config.h>
#include <opencog/util/algorithm.h>

#include "HTree.h"
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;

void PatternMiner::growPatternsDepthFirstTask(unsigned int thread_index)
{

	// The start index in allLinks for current thread
    unsigned int start_index = linksPerThread * thread_index;
    unsigned int end_index; // the last index for current thread (excluded)
    if (thread_index == param.THREAD_NUM - 1)
    {
        // if this the last thread, it
        // needs to finish all the
        // rest of the links
        if (param.only_mine_patterns_start_from_white_list)
            end_index = allLinksContainWhiteKeywords.size();
        else
            end_index = allLinkNumber;
    }
    else
        end_index = linksPerThread * (thread_index + 1);

    cout << "\nStart thread " << thread_index
         << ": will process Link number from " << start_index
         << " to (excluded) " << end_index << std::endl;

    float allLinkNumberfloat = (float)(end_index - start_index);

    for (unsigned int t_cur_index = start_index; t_cur_index < end_index; ++t_cur_index)
    {
        if (param.THREAD_NUM > 1)
            readNextLinkLock.lock();

        cout<< "\r" << ((float)(t_cur_index - start_index))/allLinkNumberfloat*100.0f << "% completed in Thread " + toString(thread_index) + "."; // it's not liner
        std::cout.flush();

        processedLinkNum ++;

        Handle cur_link;

        if (param.only_mine_patterns_start_from_white_list)
        {
            cur_link = allLinksContainWhiteKeywords[t_cur_index];
            havenotProcessedWhiteKeywordLinks.erase(havenotProcessedWhiteKeywordLinks.find(cur_link));

        }
        else
            cur_link = allLinks[t_cur_index];


        if (param.THREAD_NUM > 1)
            readNextLinkLock.unlock();

        // Extract all the possible patterns from this originalLink, and extend till the max_gram links, not duplicating the already existing patterns
        HandleSeq lastGramLinks;
        HandleMap lastGramValueToVarMap;
        HandleMap patternVarMap;

        set<string> allNewMinedPatternsCurTask;
        bool startFromLinkContainWhiteKeyword = false;

        // allHTreeNodesCurTask is only used in distributed version;
        // is to store all the HTreeNode* mined in this current task, and release them after the task is finished.
        vector<HTreeNode*> allHTreeNodesCurTask;

        // allNewMinedPatternInfo is only used in distributed version, to store all the new mined patterns in this task for sending to the server.
        vector<MinedPatternInfo> allNewMinedPatternInfo;

        if (! param.only_mine_patterns_start_from_white_list)
        {
            if (param.use_linktype_black_list && isIgnoredType (cur_link->get_type()) )
            {
                continue;
            }
            else if (param.use_linktype_white_list && (!is_in(cur_link->get_type(), param.linktype_white_list)))
            {
                continue;
            }

            if (param.use_keyword_black_list)
            {
                // if the content in this link contains content in the black list,ignore it
                if (containIgnoredContent(cur_link))
                    continue;
            }

            if (param.only_output_patterns_contains_white_keywords)
            {
                if (havenotProcessedWhiteKeywordLinks.find(cur_link) != havenotProcessedWhiteKeywordLinks.end())
                    startFromLinkContainWhiteKeyword = true;
            }

            // Add this link into observing_as
            HandleSeq outVariableNodes;
            Handle newLink = copyAtom(*observing_as, cur_link, outVariableNodes);

            extendAPatternForOneMoreGramRecursively(newLink, *observing_as, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap,patternVarMap, false,
                                                    allNewMinedPatternsCurTask, allHTreeNodesCurTask, allNewMinedPatternInfo, thread_index, startFromLinkContainWhiteKeyword);


        }
        else
        {
            extendAPatternForOneMoreGramRecursively(cur_link, original_as, Handle::UNDEFINED, lastGramLinks, 0, lastGramValueToVarMap,patternVarMap, false,
                                                    allNewMinedPatternsCurTask, allHTreeNodesCurTask, allNewMinedPatternInfo, thread_index,startFromLinkContainWhiteKeyword);
        }

        if (param.THREAD_NUM > 1)
            actualProcessedLinkLock.lock();

        actualProcessedLinkNum ++;

        if (param.THREAD_NUM > 1)
            actualProcessedLinkLock.unlock();
    }

    cout<< "\r100% completed in Thread " + toString(thread_index) + ".";
    std::cout.flush();
}

void PatternMiner::runPatternMinerDepthFirst()
{
    // observing_as is used to copy one link everytime from the
    // original_as
    if (not observing_as)
        observing_as = new AtomSpace(); // TODO can you init this at ctor?

    if (param.THREAD_NUM > 1)
    {
        thread_DF_ExtractedLinks = new list<string>* [param.THREAD_NUM];
        for (unsigned int ti = 0; ti < param.THREAD_NUM; ti ++)
            thread_DF_ExtractedLinks[ti] = new list<string>[param.MAX_GRAM];

        all_thread_ExtractedLinks_pergram = new set<string>[param.MAX_GRAM];
    }

    if (param.only_mine_patterns_start_from_white_list)
        linksPerThread = allLinksContainWhiteKeywords.size() / param.THREAD_NUM;
    else
        linksPerThread = allLinkNumber / param.THREAD_NUM;

    processedLinkNum = 0;
    actualProcessedLinkNum = 0;

    for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
    {

        threads[i] = std::thread(&PatternMiner::growPatternsDepthFirstTask,this,i);
        // threads[i] = std::thread([this]{this->growPatternsDepthFirstTask(i);}); // using C++11 lambda-expression
    }

    for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
    {
        threads[i].join();
    }

    clear_by_swap(allLinks);

    if (param.THREAD_NUM > 1)
    {
        for (unsigned int ti = 0; ti < param.THREAD_NUM; ti ++)
            delete [] (thread_DF_ExtractedLinks[ti]);

        delete [] thread_DF_ExtractedLinks;

        delete [] all_thread_ExtractedLinks_pergram;
    }

    delete [] threads;

    cout << "\nFinished mining 1~" << param.MAX_GRAM << " gram patterns.\n";
    cout << "\ntotalLinkNum = " << processedLinkNum << ", actualProcessedLinkNum = " << actualProcessedLinkNum << std::endl;
}

// extendedLinkIndex is to return the index of extendedLink's patternlink in the unified pattern so as to identify where is the extended link in this pattern
// vector<HTreeNode*> &allHTreeNodesCurTask is only used in distributed version
// notOutPutPattern is passed from extendAPatternForOneMoreGramRecursively, also may be modify in this function.
// it indicates if one pattern is only generated for middle process - calculate interestingness for its superpatterns, but not put in output results
HTreeNode* PatternMiner::extractAPatternFromGivenVarCombination(HandleSeq &inputLinks, HandleMap &patternVarMap, HandleMap& orderedVarNameMap,HandleSeqSeq &oneOfEachSeqShouldBeVars, HandleSeq &leaves,
                                                                HandleSeq &shouldNotBeVars, HandleSeq &shouldBeVars, unsigned int & extendedLinkIndex,
                                                                set<string>& allNewMinedPatternsCurTask, bool& notOutPutPattern, bool &patternAlreadyExtractedInCurTask, bool startFromLinkContainWhiteKeyword)
{
    HTreeNode* returnHTreeNode = nullptr;
    bool skip = false;
    unsigned int gram = inputLinks.size();

    if (not shouldNotBeVars.empty())
    {
        for (const Handle& noTypeNode  : shouldNotBeVars)
        {
            if (patternVarMap.find(noTypeNode) != patternVarMap.end())
            {
                skip = true;
                break;
            }
        }
    }

    if (not skip && not shouldBeVars.empty())
    {
        for (const Handle& shouldBeVarNode  : shouldBeVars)
        {
            if (patternVarMap.find(shouldBeVarNode) == patternVarMap.end())
            {
                skip = true;
                break;
            }
        }
    }

    if ((! skip) && (param.enable_filter_links_should_connect_by_vars ||
                     param.enable_filter_not_all_first_outgoing_const))
    {

        // check if in this combination, if at least one node in each Seq of oneOfEachSeqShouldBeVars is considered as variable
        bool allSeqContainsVar = true;
        for (const HandleSeq& oneSharedSeq : oneOfEachSeqShouldBeVars)
        {
            bool thisSeqContainsVar = false;
            for (const Handle& toBeSharedNode : oneSharedSeq)
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


    if ( (! skip) && (param.enable_filter_leaves_should_not_be_vars) && (gram > 1)) // for gram > 1, any leaf should not considered as variable
    {
        for (const Handle& leaf : leaves)
        {

            if (patternVarMap.find(leaf) != patternVarMap.end())
            {
                if (GENERATE_TMP_PATTERNS)
                {
                    if (gram == param.MAX_GRAM) // only skip for the max gram patterns
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

        for (const Handle& link : inputLinks) {
            Handle nlink = substitute(link, patternVarMap);
            pattern.push_back(nlink);
        }

        if (gram > 2)
        {
            if (containLoopVariable(pattern))
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

        HTreeNode* newHTreeNode = nullptr;

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

            if (param.THREAD_NUM > 1)
                uniqueKeyLock.lock();

            map<string, HTreeNode*>::iterator htreeNodeIter = keyStrToHTreeNodeMap.find(keyString);

            if (htreeNodeIter == keyStrToHTreeNodeMap.end())
            {
                newHTreeNode = new HTreeNode();
                returnHTreeNode = newHTreeNode;
                newHTreeNode->count = 1;

                keyStrToHTreeNodeMap.insert({keyString, newHTreeNode});

    //            cout << "A new pattern Found:\n"<< keyString << std::endl;

            }
            else
            {
                returnHTreeNode = htreeNodeIter->second;

                returnHTreeNode->count ++;

    //            cout << "Unique Key already exists:" << keyString << std::endl;
            }

            if (param.THREAD_NUM > 1)
                uniqueKeyLock.unlock();

            if (newHTreeNode)
            {
                if (! GENERATE_TMP_PATTERNS)
                    notOutPutPattern = false;

                newHTreeNode->pattern = unifiedPattern;
                newHTreeNode->var_num = patternVarMap.size();

                if ( (! notOutPutPattern) && param.only_output_patterns_contains_white_keywords)
                {
                    // check if this instance contain any white keywords
                    if (! startFromLinkContainWhiteKeyword) // if it already starts from Links that contain keywords , do not need to check
                    {
                        bool is_contain = false;
                        for (const Handle& link : inputLinks)
                        {
                            if (containKeywords(link->to_short_string(), param.keyword_white_list, param.keyword_white_list_logic))
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

                if (param.THREAD_NUM > 1)
                    addNewPatternLock.lock();

                if (notOutPutPattern)
                {
                    if (GENERATE_TMP_PATTERNS)
                        (tmpPatternsForGram[gram-1]).push_back(newHTreeNode);
                }
                else
                    (patternsForGram[gram-1]).push_back(newHTreeNode);

                if (param.THREAD_NUM > 1)
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
    for (const string& uids : (thread_DF_ExtractedLinks[cur_thread_index][_gram - 1]))
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
void PatternMiner::extendAPatternForOneMoreGramRecursively(const Handle &extendedLink, AtomSpace& from_as, const Handle &extendedNode, const HandleSeq &lastGramLinks,
                 HTreeNode* parentNode, const HandleMap &lastGramValueToVarMap, const HandleMap &lastGramPatternVarMap, bool isExtendedFromVar,
                 set<string>& allNewMinedPatternsCurTask, vector<HTreeNode*> &allHTreeNodesCurTask, vector<MinedPatternInfo> &allNewMinedPatternInfo, unsigned int thread_index,
                 bool startFromLinkContainWhiteKeyword)
{
    // the ground value node in the from_as to the variable handle in pattenmining Atomspace
    HandleMap valueToVarMap = lastGramValueToVarMap;
    // std::cout << "valueToVarMap = " << oc_to_string(valueToVarMap);

    // First, extract all the nodes in the input link
    associateNodesToVars(extendedLink, valueToVarMap);

    HandleMap newValueToVarMap; // the new elements added in this gram

    bool notOutPutPattern = false;

    if (parentNode == nullptr) // this the first gram
    {
        newValueToVarMap = valueToVarMap;
    }
    else
    {
        for (const auto& p : valueToVarMap) // TODO Replace this code by map difference
        {
            if (lastGramValueToVarMap.find(p.first) == lastGramValueToVarMap.end())
                newValueToVarMap.insert(p);
        }
    }

    // Generate all the possible combinations of all the nodes: all patterns including the 1 ~ n_max variables
    // If there are too many variables in a pattern, it doesn't make much sense, so we limit the max number of variables to half of the node number

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
    unsigned int n_limit_putin_result;

    // sometimes there is only one variable in a link, like:
    //    (DuringLink
    //      (ConceptNode "dead")
    //      (ConceptNode "dead")
    //    ) ; [44694]
    n_limit_putin_result = valueToVarMap.size() * param.max_var_num_percent - lastGramTotalVarNum;
    n_limit_putin_result ++;

    if (n_limit_putin_result == 1) // TODO why?
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
    // so that newValueToVarMap.size() can be equal, e.g.:
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

    n_limit_putin_result = clamp(n_limit_putin_result, 1U, n_max);
    n_limit = clamp(n_limit, 1U, n_max);

    // Get all the shared nodes and leaves
    HandleSeqSeq oneOfEachSeqShouldBeVars;
    HandleSeq leaves, shouldNotBeVars, shouldBeVars;

    HandleSeq inputLinks = lastGramLinks;
    inputLinks.push_back(extendedLink);

    //    OC_ASSERT( (valueToVarMap.size() > 1),
    //              "PatternMiner::extractAllPossiblePatternsFromInputLinks: this group of links only has one node: %s!\n",
    //               from_as.atomAsString(inputLinks[0]).c_str() );

    if(filters(inputLinks, oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars))
        return; //already been filter out in this phrase

    bool* indexes = new bool[n_max]; //  indexes[i]=true means this i is a variable, indexes[i]=false means this i is a const

//    if ((cur_pattern_gram == 2) && (n_max == 0)) // debug
//    {
//        int x = 0;
//        x ++;
//        for (Handle h : inputLinks)
//            cout << h->to_short_string();

//        cout << std::endl;
//    }
//    // debug
//    string lastGramLinksStr = "";
//    for (const Handle& h : lastGramLinks)
//        lastGramLinksStr += h->to_short_string();

//    string inputLinksStr = "";
//    for (const Handle& h : inputLinks)
//        inputLinksStr += h->to_short_string();

    // var_num is the number of variables
    unsigned int var_num = parentNode ? 0 : 1;

    // todo: store the var combinations to HTreeNode map for finding super/sub_patternrelation_b
    // the string is one combination of indexes to string , e.g.: "0010"
    map<string, HTreeNode*> varCombinToHTreeNode;

    for (; var_num < n_limit; ++ var_num)
    {
        // Use the binary method to generate all combinations:

        // generate the first combination
        for (unsigned int i = 0; i < var_num; ++ i)
            indexes[i] = true;

        for (unsigned int i = var_num; i < n_max; ++ i)
            indexes[i] = false;

        while (true)
        {
            // construct the pattern for this combination in the PatternMining Atomspace
            // generate the patternVarMap for this pattern of this combination

            // the first part is the same as its parent node
            HandleMap patternVarMap;
            if (parentNode)
            {
                patternVarMap = lastGramPatternVarMap;

                if (! isExtendedFromVar) // need to add the extendedNode into patternVarMap
                {
                    HandleMap::const_iterator extendedIter = lastGramValueToVarMap.find(extendedNode);
                    OC_ASSERT(extendedIter != lastGramValueToVarMap.end(),
                              "can't find extendedNode in lastGramValueToVarMap");
                    patternVarMap.insert(*extendedIter);
                }
            }

            // And then add the second part:
            if (var_num > 0)
            {
                unsigned int index = 0;
                for (const auto& p : newValueToVarMap)
                {
                    if (indexes[index]) // this is considered as a variable, add it into the variable to value map
	                    patternVarMap.insert(p);
                    index++;
                }
            }

            if (GENERATE_TMP_PATTERNS)
            {
                if ((cur_pattern_gram > 1) &&(var_num >= n_limit_putin_result))
                    notOutPutPattern = true;
            }

            unsigned int extendedLinkIndex = 999;// TODO max value?
            bool patternAlreadyExtractedInCurTask;
            HandleMap orderedVarNameMap;

            HTreeNode* thisGramHTreeNode = extractAPatternFromGivenVarCombination(inputLinks, patternVarMap, orderedVarNameMap,oneOfEachSeqShouldBeVars, leaves, shouldNotBeVars, shouldBeVars,
                                          extendedLinkIndex, allNewMinedPatternsCurTask, notOutPutPattern, patternAlreadyExtractedInCurTask, startFromLinkContainWhiteKeyword);

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
                        for (const ExtendRelation& existRelation : parentNode->superPatternRelations)
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
                    if ((cur_pattern_gram == 1) && (param.calculate_type_b_surprisingness) && (var_num <= n_limit_putin_result))
                    {

//                        cout << "\n-------------------Found sub type b patterns for current pattern: -----------------\n";
//                        for (Handle plink : thisGramHTreeNode->pattern)
//                            cout << plink->to_short_string();
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
                                    for (const auto& p : newValueToVarMap)
                                    {
                                        if (const_index == index_i)
                                        {
                                            superb.constNode = p.first;
                                            oldVarHandle = p.second;

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
                                        thisGramHTreeNode->SubRelation_b_map.insert({orderedVarHandle, subRelations});

                                    }
                                    else
                                    {
                                        (sub_b_relation_it->second).push_back(onesub_b);
                                    }
                                }
                            }
                            else // it is a const node
                                indexStr += "0";

                        }

                        varCombinToHTreeNode.insert({indexStr, thisGramHTreeNode});

                    }


                    // check if the current gram is already the MAX_GRAM
                    if(cur_pattern_gram >= param.MAX_GRAM)
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

                    for (const auto& p : valueToVarMap)
                    {
                        Handle extendNode = p.first;
                        if (param.enable_filter_node_types_should_not_be_vars)
                        {
                            bool isIgnoredType = false; // TODO move this to its own function
                            Type t = extendNode->get_type();
                            for (Type noType : param.node_types_should_not_be_vars)
                            {
                                if (t == noType)
                                {
                                    isIgnoredType = true;
                                    break;
                                }
                            }

                            if (isIgnoredType)
                                continue;
                        }


                        bool isNewExtendedFromVar;

                        if (patternVarMap.find(extendNode) != patternVarMap.end()) // this is considered as a variable
                        {   // Type 1: extended from a variable
                            isNewExtendedFromVar = true;
                        }
                        else
                        {
                            // Type 2: extended from a const by turning it into a variable
                            isNewExtendedFromVar = false;
                        }

                        // find what are the other links in the original Atomspace contain this variable
                        IncomingSet incomings = extendNode->getIncomingSet(&from_as);

                        // debug
//                        string curvarstr = extendNode->to_short_string();
//                        cout << "\n---------------start curvarstr = " << curvarstr << "---------------" <<std::endl;

                        for (LinkPtr incomingPtr : incomings)
                        {
                            Handle incomingHandle = incomingPtr->get_handle();
                            Handle extendedHandle = incomingHandle;

                            if (param.use_linktype_black_list && isIgnoredType(incomingHandle->get_type()))
                            {
                                // if this atom is of igonred type, get its first ancestor that is not in the igonred types
                                extendedHandle = getFirstNonIgnoredIncomingLink(from_as, incomingHandle);

                                if ((extendedHandle == Handle::UNDEFINED))
                                    continue;
                            }
                            else if (param.use_linktype_white_list && (!is_in(incomingHandle->get_type(), param.linktype_white_list)))
                            {
                                continue;
                            }

                            if (param.use_keyword_black_list)
                            {
                                if (param.keyword_black_logic_is_contain)
                                {
                                    if (not param.keyword_black_list.empty() && containIgnoredContent(extendedHandle))
                                        continue;
                                }
                                else
                                {
                                    if (not black_keyword_Handles.empty() && doesLinkContainNodesInKeyWordNodes(extendedHandle, black_keyword_Handles))
                                        continue;
                                }
                            }

                            if (is_in(extendedHandle, inputLinks))
                                continue;

                            if (param.only_mine_patterns_start_from_white_list)
                            {
                                if (havenotProcessedWhiteKeywordLinks.find(extendedHandle) != havenotProcessedWhiteKeywordLinks.end())
                                    continue;
                            }

//                             string extendedHandleStr = extendedHandle->to_short_string();

                            // debug
//                             cout << "Extended link :" << extendedHandleStr << std::endl;

    //                        if (extendedHandleStr.find("PatternVariableNode") != std::string::npos)
    //                        {
    //                           // cout << "Debug: error! The extended link contines variables!" << extendedHandleStr << std::endl;
    //                            continue;
    //                        }

                            // check if other thread happends to process on the same links
                            if (param.THREAD_NUM > 1)
                            {
                                string instancekeyString = "";

                                // check if these fact links already been processed before or by other thread

                                HandleSet originalLinksSet(inputLinks.begin(), inputLinks.end());
                                originalLinksSet.insert(extendedHandle);

                                for (const Handle& h  : originalLinksSet)
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
                            extendAPatternForOneMoreGramRecursively(extendedHandle,  from_as, extendNode, inputLinks, thisGramHTreeNode, valueToVarMap,patternVarMap,isNewExtendedFromVar,
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
}
