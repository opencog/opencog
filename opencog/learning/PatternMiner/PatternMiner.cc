/*
 * opencog/learning/PatternMiner/PatternMiner.h
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com> Scott Jones <troy.scott.j@gmail.com>
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
#include <thread>
#include <sstream>
#include <map>
#include <iterator>
#include <opencog/atomspace/atom_types.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/foreach.h>
#include <stdlib.h>
#include <time.h>
#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;

PatternMiner::PatternMiner(AtomSpace* _originalAtomSpace): originalAtomSpace(_originalAtomSpace)
{
    htree = new HTree();
    atomSpace = new AtomSpace();

    unsigned int system_thread_num  = std::thread::hardware_concurrency();
    if (system_thread_num > 2)
        THREAD_NUM = system_thread_num - 2;
    else
        THREAD_NUM = 1;

    threads = new thread[THREAD_NUM];
}

void PatternMiner::generateIndexesOfSharedVars(Handle& link, vector<Handle>& orderedHandles, vector < vector<int> >& indexes)
{
    HandleSeq outgoingLinks = atomSpace->getOutgoing(link);
    foreach (Handle h, outgoingLinks)
    {
        if (atomSpace->isNode(h))
        {
            if (atomSpace->getType(h) == opencog::VARIABLE_NODE)
            {
                string var_name = atomSpace->getName(h);

                vector<int> indexesForCurVar;
                int index = 0;

                foreach (Handle oh,orderedHandles)
                {
                    string ohStr = atomSpace->atomAsString(oh);
                    if (ohStr.find(var_name) != std::string::npos)
                    {
                        indexesForCurVar.push_back(index);
                    }

                    index ++;
                }

                indexes.push_back(indexesForCurVar);
            }
        }
        else
            generateIndexesOfSharedVars(h,orderedHandles,indexes);
    }
}

void PatternMiner::findAndRenameVariablesForOneLink(Handle link, map<Handle,Handle>& varNameMap, HandleSeq& renameOutgoingLinks)
{

    HandleSeq outgoingLinks = atomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (atomSpace->isNode(h))
        {
           if (atomSpace->getType(h) == opencog::VARIABLE_NODE)
           {
               if (varNameMap.find(h) != varNameMap.end())
               {
                   renameOutgoingLinks.push_back(varNameMap[h]);
               }
               else
               {
                   string var_name = "$var_"  + toString(varNameMap.size() + 1);
                   Handle var_node = atomSpace->addNode(opencog::VARIABLE_NODE, var_name, TruthValue::TRUE_TV());
                   varNameMap.insert(std::pair<Handle,Handle>(h,var_node));
                   renameOutgoingLinks.push_back(var_node);

               }

           }
        }
        else
        {
             HandleSeq _renameOutgoingLinks;
             findAndRenameVariablesForOneLink(h, varNameMap, _renameOutgoingLinks);
             Handle reLink = atomSpace->addLink(atomSpace->getType(h),_renameOutgoingLinks,TruthValue::TRUE_TV());
             renameOutgoingLinks.push_back(reLink);

        }

    }

}

vector<Handle> PatternMiner::RebindVariableNames(vector<Handle>& orderedPattern)
{
    map<Handle,Handle> varNameMap;
    vector<Handle> rebindedPattern;

    foreach (Handle link, orderedPattern)
    {
        HandleSeq renameOutgoingLinks;
        findAndRenameVariablesForOneLink(link, varNameMap, renameOutgoingLinks);
        Handle rebindedLink = atomSpace->addLink(atomSpace->getType(link),renameOutgoingLinks,TruthValue::TRUE_TV());
        rebindedPattern.push_back(rebindedLink);
    }

    return rebindedPattern;
}

// the input links should be like: only specify the const node, all the variable node name should not be specified:
vector<Handle> PatternMiner::UnifyPatternOrder(vector<Handle>& inputPattern)
{

    // Step 1: take away all the variable names, make the pattern into such format string:
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

    multimap<string, Handle> nonVarStrToHandleMap;

    foreach (Handle inputH, inputPattern)
    {
        string str = atomSpace->atomAsString(inputH);
        string nonVarNameString = "";
        std::stringstream stream(str);
        string oneLine;

        while(std::getline(stream, oneLine,'\n'))
        {
            if (oneLine.find("VariableNode")==std::string::npos)
            {
                // this node is not a VariableNode, just keep this line
                nonVarNameString += oneLine;
            }
            else
            {
                // this node is an VariableNode, remove the name, just keep "VariableNode"
                nonVarNameString += "VariableNode\n";
            }
        }

        nonVarStrToHandleMap.insert(std::pair<string, Handle>(nonVarNameString,inputH));
    }

    // Step 2: sort the order of all the handls do not share the same string key with other handls
    // becasue the strings are put into a map , so they are already sorted.
    // now print the Handles that do not share the same key with other Handles into a vector, left the Handles share the same keys

    vector<Handle> orderedHandles;
    vector<string> duplicateStrs;
    multimap<string, Handle>::iterator it;
    for(it = nonVarStrToHandleMap.begin(); it != nonVarStrToHandleMap.end();)
    {
        int count = nonVarStrToHandleMap.count(it->first);
        if (count == 1)
        {
            // if this key string has only one record , just put the corresponding handle to the end of orderedHandles
            orderedHandles.push_back(it->second);
            it ++;
        }
        else
        {
            // this key string has multiple handles to it, not put these handles into the orderedHandles,
            // insteadly put this key string into duplicateStrs
            duplicateStrs.push_back(it->first);
            it = nonVarStrToHandleMap.upper_bound(it->first);
        }

    }

    // Step 3: sort the order of the handls share the same string key with other handles
    foreach (string keyString, duplicateStrs)
    {
        // get all the corresponding handles for this key string
        multimap<string, Handle>::iterator kit;
        vector<_non_ordered_pattern> sharedSameKeyPatterns;
        for (kit = nonVarStrToHandleMap.lower_bound(keyString); kit != nonVarStrToHandleMap.upper_bound(keyString);  ++ kit)
        {
            _non_ordered_pattern p;
            p.link = kit->second;
            generateIndexesOfSharedVars(p.link, orderedHandles, p.indexesOfSharedVars);
            sharedSameKeyPatterns.push_back(p);
        }

        std::sort(sharedSameKeyPatterns.begin(), sharedSameKeyPatterns.end());
        foreach (_non_ordered_pattern np, sharedSameKeyPatterns)
        {
            orderedHandles.push_back(np.link);
        }
    }

    return orderedHandles;

}

string PatternMiner::unifiedPatternToKeyString(vector<Handle>& inputPattern)
{
    string keyStr = "";
    foreach(Handle h, inputPattern)
    {
        keyStr += atomSpace->atomAsString(inputPattern);
        keyStr += "\n";
    }
}

bool PatternMiner::checkPatternExist(const string& patternKeyStr)
{
    if (keyStrToHTreeNodeMap.find(patternKeyStr) == keyStrToHTreeNodeMap.end())
        return false;
    else
        return true;

}

void generateNextCombinationGroup(bool* &indexes, int n_max)
{
    int trueCount = -1;
    int i = 0;
    for (; i < n_max - 1; ++ i)
    {
        if (indexes[i])
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

void PatternMiner::generateALinkByChosenVariables(Handle& originalLink, map<Handle,Handle>& valueToVarMap,  HandleSeq& outputOutgoings)
{
    HandleSeq outgoingLinks = originalAtomSpace->getOutgoing(originalLink);

    foreach (Handle h, outgoingLinks)
    {
        if (originalAtomSpace->isNode(h))
        {
           if (valueToVarMap.find(h) != valueToVarMap.end())
           {
               // this node is considered as a variable
               outputOutgoings.push_back(valueToVarMap[h]);
           }
           else
           {
               // this node is considered not a variable, so add its bound value node into the Pattern mining Atomspace
               Handle value_node = atomSpace->addNode(originalAtomSpace->getType(h), originalAtomSpace->getName(h), TruthValue::TRUE_TV());
               outputOutgoings.push_back(value_node);
           }
        }
        else
        {
             HandleSeq _outputOutgoings;
             generateALinkByChosenVariables(h, valueToVarMap, _outputOutgoings);
             Handle reLink = atomSpace->addLink(originalAtomSpace->getType(h),_outputOutgoings,TruthValue::TRUE_TV());
             outputOutgoings.push_back(reLink);
        }
    }
}

void PatternMiner::extractAllNodesInLink(Handle link, map<Handle,Handle>& valueToVarMap)
{
    HandleSeq outgoingLinks = originalAtomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (originalAtomSpace->isNode(h))
        {
            if (valueToVarMap.find(h) == valueToVarMap.end())
            {
                // add a variable node in Pattern miner Atomspace
                Handle varHandle = atomSpace->addNode(opencog::VARIABLE_NODE,"$var~" + toString(valueToVarMap.size()) );
                valueToVarMap.insert(std::pair<Handle,Handle>(h,varHandle));
            }
            else
            {
                extractAllNodesInLink(h,valueToVarMap);
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
HandleSeqSeq PatternMiner::extractAllPossiblePatternsFromInputLinks(vector<Handle>& inputLinks)
{
    map<Handle,Handle> valueToVarMap;
    // First, extract all the nodes in the input links
    foreach (Handle link, inputLinks)
        extractAllNodesInLink(link, valueToVarMap);

    int n_max = valueToVarMap.size();
    bool* indexes = new bool[n_max];

    OC_ASSERT( (n_max > 1),
              "PatternMiner::extractAllPossiblePatternsFromInputLinks: this group of links only has one node: %s!\n",
               atomSpace->atomAsString(inputLinks[0]).c_str() );

    // Generate all the possible combinations of all the nodes: all patterns including 1 ~ (valueToVarMap.size() - 1) variables
    for (int var_num = 1; var_num < n_max; ++ var_num)
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
            unsigned int index = 0;
            for (iter = valueToVarMap.begin(); iter != valueToVarMap.end(); ++ iter)
            {
                if (indexes[index])
                    patternVarMap.insert(std::pair<Handle,Handle>(iter->first, iter->second));

                index ++;
            }

            HandleSeq pattern, unifiedPattern;

            foreach (Handle link, inputLinks)
            {
                HandleSeq outgoingLinks;
                generateALinkByChosenVariables(link, patternVarMap, outgoingLinks);
                Handle rebindedLink = atomSpace->addLink(atomSpace->getType(link),outgoingLinks,TruthValue::TRUE_TV());
                pattern.push_back(rebindedLink);
            }

            // unify the pattern
            unifiedPattern = UnifyPatternOrder(pattern);
            string keyString = unifiedPatternToKeyString(unifiedPattern);

            // next, check if this pattern already exist, need lock

            if (isLastNElementsAllTrue(indexes, n_max, var_num))
                break;

            // generate the next combination
            generateNextCombinationGroup(indexes, n_max);
        }
    }

}

void PatternMiner::growTheFirstGramPatternsTask()
{
    static HandleSeq allLinks;
    originalAtomSpace->getHandlesByType(back_inserter(allLinks), (Type) LINK, true );

    while (allLinks.size() > 0)
    {
        allAtomListLock.lock();
        Handle cur_link = allLinks[allLinks.size() - 1];
        // if this link is listlink, ignore it
        if (originalAtomSpace->getType(cur_link) == opencog::LIST_LINK)
            continue;

        allLinks.pop_back();
        allAtomListLock.unlock();

    }

}

void PatternMiner::growAPatternTask()
{
    static unsigned cur_gram = 1;

    static HandleSeq allLinks;
    originalAtomSpace->getHandlesByType(back_inserter(allLinks), (Type) LINK, true );

    // randomly choose one link for start
    Handle startLink;

    while(true)
    {
        int startLinkIndex = rand() / allLinks.size();
        startLink = allLinks[startLinkIndex];

        // if this link is listlink, try another time until get a link that is not listlink
        if (originalAtomSpace->getType(startLink) != opencog::LIST_LINK)
            break;
    }

}

void PatternMiner::runPatternMiner(unsigned int max_gram)
{
    MAX_GRAM = max_gram;

    // first, generate the first layer patterns: patterns of 1 gram (contains only one link)
    for (unsigned int i = 0; i < THREAD_NUM; ++ i)
    {
        threads[i] = std::thread([this]{this->growTheFirstGramPatternsTask();}); // using C++11 lambda-expression
        threads[i].join();
    }

    srand(time(NULL));
}
