/*
 * opencog/learning/PatternMiner/PatternMiner.cc
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

#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;

const string PatternMiner::ignoreKeyWords[] = {"this", "that","these","those","it","he", "him", "her", "she" };

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
               // it's a variable node, rename it
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
           else
           {
               // it's a const node, just add it
               renameOutgoingLinks.push_back(h);

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

vector<Handle> PatternMiner::RebindVariableNames(vector<Handle>& orderedPattern, map<Handle,Handle>& orderedVarNameMap)
{

    vector<Handle> rebindedPattern;

    foreach (Handle link, orderedPattern)
    {
        HandleSeq renameOutgoingLinks;
        findAndRenameVariablesForOneLink(link, orderedVarNameMap, renameOutgoingLinks);
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
                nonVarNameString += "VariableNode";
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

    // in this map, the first Handle is the variable node is the original Atomspace,
    // the second Handle is the renamed ordered variable node in the Pattern Mining Atomspace.
    map<Handle,Handle> orderedVarNameMap;
    vector<Handle> rebindPattern = RebindVariableNames(orderedHandles, orderedVarNameMap);

    return rebindPattern;

}

string PatternMiner::unifiedPatternToKeyString(vector<Handle>& inputPattern, const AtomSpace *atomspace)
{
    if (atomspace == 0)
        atomspace = this->atomSpace;

    string keyStr = "";
    foreach(Handle h, inputPattern)
    {
        keyStr += Link2keyString(h,"", atomspace);
        keyStr += "\n";
    }

    return keyStr;
}

bool PatternMiner::checkPatternExist(const string& patternKeyStr)
{
    if (keyStrToHTreeNodeMap.find(patternKeyStr) == keyStrToHTreeNodeMap.end())
        return false;
    else
        return true;

}

void PatternMiner::generateNextCombinationGroup(bool* &indexes, int n_max)
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

bool PatternMiner::isLastNElementsAllTrue(bool* array, int size, int n)
{
    for (int i = size - 1; i >= size - n; i --)
    {
        if (! array[i])
            return false;
    }

    return true;
}

unsigned int combinationCalculate(int r, int n)
{
    // = n!/(r!*(n-r)!)
    int top = 1;
    for (int i = n; i > r; i-- )
    {
        top *= i;
    }

    int bottom = 1;
    for (int j = n-r; j > 1; j--)
    {
        bottom *= j;
    }

    return top/bottom;

}

 // valueToVarMap:  the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace
void PatternMiner::generateALinkByChosenVariables(Handle& originalLink, map<Handle,Handle>& valueToVarMap,  HandleSeq& outputOutgoings,AtomSpace *_fromAtomSpace)
{
    HandleSeq outgoingLinks = _fromAtomSpace->getOutgoing(originalLink);

    foreach (Handle h, outgoingLinks)
    {
        if (_fromAtomSpace->isNode(h))
        {
           if (valueToVarMap.find(h) != valueToVarMap.end())
           {
               // this node is considered as a variable
               outputOutgoings.push_back(valueToVarMap[h]);
           }
           else
           {
               // this node is considered not a variable, so add its bound value node into the Pattern mining Atomspace
               Handle value_node = atomSpace->addNode(_fromAtomSpace->getType(h), _fromAtomSpace->getName(h), TruthValue::TRUE_TV());
               outputOutgoings.push_back(value_node);
           }
        }
        else
        {
             HandleSeq _outputOutgoings;
             generateALinkByChosenVariables(h, valueToVarMap, _outputOutgoings, _fromAtomSpace);
             Handle reLink = atomSpace->addLink(_fromAtomSpace->getType(h),_outputOutgoings,TruthValue::TRUE_TV());
             outputOutgoings.push_back(reLink);
        }
    }
}

 // valueToVarMap:  the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace
// _fromAtomSpace: where is input link from
void PatternMiner::extractAllNodesInLink(Handle link, map<Handle,Handle>& valueToVarMap, AtomSpace* _fromAtomSpace)
{
    HandleSeq outgoingLinks = _fromAtomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (_fromAtomSpace->isNode(h))
        {
            if (valueToVarMap.find(h) == valueToVarMap.end())
            {
                // add a variable node in Pattern miner Atomspace
                Handle varHandle = atomSpace->addNode(opencog::VARIABLE_NODE,"$var~" + toString(valueToVarMap.size()) );
                valueToVarMap.insert(std::pair<Handle,Handle>(h,varHandle));
            }

            if ((_fromAtomSpace->getType(h) == opencog::VARIABLE_NODE))
                cout<<"Error: instance link contains variables: \n" << _fromAtomSpace->atomAsString(h)<<std::endl;

        }
        else
        {
            extractAllNodesInLink(h,valueToVarMap,_fromAtomSpace);
        }
    }
}

void PatternMiner::extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, set<Handle>& allVarNodes)
{

    HandleSeq ioutgoingLinks = originalAtomSpace->getOutgoing(instanceLink);
    HandleSeq poutgoingLinks = atomSpace->getOutgoing(patternLink);

    HandleSeq::iterator pit = poutgoingLinks.begin();

    foreach (Handle h, ioutgoingLinks)
    {
        if (originalAtomSpace->isNode(h))
        {
            if ((atomSpace->getType(*pit) == opencog::VARIABLE_NODE))
            {
                if (allVarNodes.find(h) == allVarNodes.end())
                {
                    allVarNodes.insert(h);
                }
            }
        }
        else
        {
            extractAllVariableNodesInAnInstanceLink(h,(Handle&)(*pit),allVarNodes);
        }

        pit ++;
    }

}


void PatternMiner::extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, map<Handle, unsigned int>& allVarNodes, unsigned index)
{

    HandleSeq ioutgoingLinks = originalAtomSpace->getOutgoing(instanceLink);
    HandleSeq poutgoingLinks = atomSpace->getOutgoing(patternLink);

    HandleSeq::iterator pit = poutgoingLinks.begin();

    foreach (Handle h, ioutgoingLinks)
    {
        if (originalAtomSpace->isNode(h))
        {
            if ((atomSpace->getType(*pit) == opencog::VARIABLE_NODE))
            {
                if (allVarNodes.find(h) == allVarNodes.end())
                {
                    allVarNodes.insert(std::pair<Handle, unsigned>(h,index));
                }
            }
        }
        else
        {
            extractAllVariableNodesInAnInstanceLink(h,(Handle&)(*pit),allVarNodes, index);
        }

        pit ++;
    }

}

// map<Handle, unsigned> allNodes  , unsigned is the index the first link this node belongs to
void PatternMiner::extractAllNodesInLink(Handle link, map<Handle, unsigned int>& allNodes, AtomSpace* _fromAtomSpace, unsigned index)
{
    HandleSeq outgoingLinks = _fromAtomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (_fromAtomSpace->isNode(h))
        {
            if (allNodes.find(h) == allNodes.end())
            {
                allNodes.insert(std::pair<Handle, unsigned>(h,index) );
            }
        }
        else
        {
            extractAllNodesInLink(h,allNodes,_fromAtomSpace, index);
        }
    }
}

void PatternMiner::extractAllNodesInLink(Handle link, set<Handle>& allNodes, AtomSpace* _fromAtomSpace)
{
    HandleSeq outgoingLinks = _fromAtomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (_fromAtomSpace->isNode(h))
        {
            if (allNodes.find(h) == allNodes.end())
            {
                allNodes.insert(h);
            }
        }
        else
        {
            extractAllNodesInLink(h,allNodes,_fromAtomSpace);
        }
    }
}

void PatternMiner::extractAllVariableNodesInLink(Handle link, set<Handle>& allNodes, AtomSpace* _atomSpace)
{
    HandleSeq outgoingLinks = _atomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (_atomSpace->isNode(h))
        {
            if ((_atomSpace->getType(h) == opencog::VARIABLE_NODE) && (allNodes.find(h) == allNodes.end()))
            {
                allNodes.insert(h);
            }
        }
        else
        {
            extractAllVariableNodesInLink(h,allNodes, _atomSpace);
        }
    }
}

bool PatternMiner::onlyContainVariableNodes(Handle link, AtomSpace* _atomSpace)
{
    HandleSeq outgoingLinks = _atomSpace->getOutgoing(link);

    foreach (Handle h, outgoingLinks)
    {
        if (_atomSpace->isNode(h))
        {
            if (_atomSpace->getType(h) != opencog::VARIABLE_NODE)
            {
                return false;
            }
        }
        else
        {
            if (! onlyContainVariableNodes(h, _atomSpace))
                return false;
        }
    }

    return true;
}




void PatternMiner::swapOneLinkBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings,
                                                  HandleSeq &outVariableNodes )
{
    HandleSeq outgoingLinks = fromAtomSpace->getOutgoing(fromLink);

    foreach (Handle h, outgoingLinks)
    {
        if (fromAtomSpace->isNode(h))
        {
           Handle new_node = toAtomSpace->addNode(fromAtomSpace->getType(h), fromAtomSpace->getName(h), fromAtomSpace->getTV(h));
           outgoings.push_back(new_node);
           if (fromAtomSpace->getType(h) == VARIABLE_NODE)
           {
               if ( ! isInHandleSeq(new_node, outVariableNodes) ) // should not have duplicated variable nodes
                outVariableNodes.push_back(new_node);
           }
        }
        else
        {
             HandleSeq _OutgoingLinks;

             swapOneLinkBetweenTwoAtomSpace(fromAtomSpace, toAtomSpace, h, _OutgoingLinks, outVariableNodes);
             Handle _link = toAtomSpace->addLink(fromAtomSpace->getType(h),_OutgoingLinks,fromAtomSpace->getTV(h));

             outgoings.push_back(_link);
        }
    }
}

// linksWillBeDel are all the links contain varaibles. Those links need to be deleted after run BindLink
HandleSeq PatternMiner::swapLinksBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq& outVariableNodes)
{
    HandleSeq outPutLinks;

    foreach (Handle link, fromLinks)
    {
        HandleSeq outgoingLinks;

        swapOneLinkBetweenTwoAtomSpace(fromAtomSpace, toAtomSpace, link, outgoingLinks, outVariableNodes);
        Handle toLink = toAtomSpace->addLink(fromAtomSpace->getType(link),outgoingLinks,fromAtomSpace->getTV(link));

        outPutLinks.push_back(toLink);
    }

    return outPutLinks;
}

 // using PatternMatcher
void PatternMiner::findAllInstancesForGivenPatternInNestedAtomSpace(HTreeNode* HNode)
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

    HandleSeq  implicationLinkOutgoings, bindLinkOutgoings;

    // HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpace(atomSpace, originalAtomSpace, HNode->pattern, variableNodes, linksWillBeDel);

//    if (HNode->pattern.size() == 1) // this pattern only contains one link
//    {
//        implicationLinkOutgoings.push_back(patternToMatch[0]); // the pattern to match
//        implicationLinkOutgoings.push_back(patternToMatch[0]); // the results to return

//        std::cout<<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//                << originalAtomSpace->atomAsString(patternToMatch[0]).c_str() << std::endl;
//    }

    Handle hAndLink = atomSpace->addLink(AND_LINK, HNode->pattern, TruthValue::TRUE_TV());
    Handle hOutPutListLink = atomSpace->addLink(LIST_LINK, HNode->pattern, TruthValue::TRUE_TV());
    implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
    implicationLinkOutgoings.push_back(hOutPutListLink); // the results to return

//    std::cout <<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
//            << atomSpace->atomAsString(hAndLink).c_str() << std::endl;


    Handle hImplicationLink = atomSpace->addLink(IMPLICATION_LINK, implicationLinkOutgoings, TruthValue::TRUE_TV());

    // add variable atoms
    set<Handle> allVariableNodesInPattern;
    for (unsigned int i = 0; i < HNode->pattern.size(); ++i)
    {
        extractAllVariableNodesInLink(HNode->pattern[i],allVariableNodesInPattern, atomSpace);
    }

    HandleSeq variableNodes;
    foreach (Handle varh, allVariableNodesInPattern)
    {
        Handle v = atomSpace->addNode(VARIABLE_NODE, atomSpace->getName(varh));
        variableNodes.push_back(v);
    }

    Handle hVariablesListLink = atomSpace->addLink(LIST_LINK, variableNodes, TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = atomSpace->addLink(BIND_LINK, bindLinkOutgoings, TruthValue::TRUE_TV());


    string s = atomSpace->atomAsString(hBindLink);

    // Run pattern matcher
    Handle hResultListLink = bindlink(atomSpace, hBindLink);


    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = atomSpace->getOutgoing(hResultListLink);

//     std::cout << toString(resultSet.size())  << " instances found!" << std::endl ;

    //    //debug
//    std::cout << atomSpace->atomAsString(hResultListLink) << std::endl  << std::endl;

    atomSpace->removeAtom(hResultListLink);

    foreach (Handle listH , resultSet)
    {
        HandleSeq instanceLinks = atomSpace->getOutgoing(listH);

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

        atomSpace->removeAtom(listH);
    }

    atomSpace->removeAtom(hBindLink);
    atomSpace->removeAtom(hImplicationLink);
    atomSpace->removeAtom(hAndLink);
    atomSpace->removeAtom(hOutPutListLink);

//    atomSpace->removeAtom(hVariablesListLink);

    HNode->count = HNode->instances.size();
}


void PatternMiner::removeLinkAndItsAllSubLinks(AtomSpace* _atomspace, Handle link)
{

//    //debug
//    std::cout << "Remove atom: " << _atomspace->atomAsString(link) << std::endl;

    HandleSeq Outgoings = _atomspace->getOutgoing(link);
    foreach (Handle h, Outgoings)
    {
        if (_atomspace->isLink(h))
        {
            removeLinkAndItsAllSubLinks(_atomspace,h);

        }
    }
    _atomspace->removeAtom(link);

}



bool PatternMiner::containsDuplicateHandle(HandleSeq &handles)
{
    for (unsigned int i = 0; i < handles.size(); i ++)
    {
        for (unsigned int j = i+1; j < handles.size(); j ++)
        {
            if (handles[j] == handles[i])
                return true;
        }
    }

    return false;
}


bool PatternMiner::isInHandleSeq(Handle handle, HandleSeq &handles)
{
    foreach(Handle h, handles)
    {
        if (handle == h)
            return true;
    }

    return false;
}

bool PatternMiner::isInHandleSeqSeq(Handle handle, HandleSeqSeq &handleSeqs)
{
    foreach(HandleSeq handles, handleSeqs)
    {
        if (isInHandleSeq(handle,handles))
            return true;
    }

    return false;
}

bool PatternMiner::isIgnoredType(Type type)
{
    foreach (Type t, ignoredTypes)
    {
        if (t == type)
            return true;
    }

    return false;
}

Handle PatternMiner::getFirstNonIgnoredIncomingLink(AtomSpace *atomspace, Handle& handle)
{
    Handle cur_h = handle;
    while(true)
    {
        HandleSeq incomings = atomspace->getIncoming(cur_h);
        if (incomings.size() == 0)
            return Handle::UNDEFINED;

        if (isIgnoredType (atomspace->getType(incomings[0])))
        {
            cur_h = incomings[0];
            continue;
        }
        else
            return incomings[0];

    }

}


bool compareHTreeNodeByFrequency(HTreeNode* node1, HTreeNode* node2)
{
    return (node1->count > node2->count);
}

bool compareHTreeNodeByInteractionInformation(HTreeNode* node1, HTreeNode* node2)
{
    return (node1->interactionInformation > node2->interactionInformation);
}

bool compareHTreeNodeBySurprisingness(HTreeNode* node1, HTreeNode* node2)
{
    if (node1->nI_Surprisingness - node2->nI_Surprisingness > FLOAT_MIN_DIFF)
        return true;
    else if (node2->nI_Surprisingness - node1->nI_Surprisingness > FLOAT_MIN_DIFF)
        return false;

    return (node1->var_num < node2->var_num);
}

void PatternMiner::OutPutPatternsToFile(unsigned int n_gram, bool is_interesting_pattern)
{
    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;
    if (is_interesting_pattern)
        fileName = "InterestingPatterns_" + toString(n_gram) + "gram.scm";
    else
        fileName = "FrequentPatterns_" + toString(n_gram) + "gram.scm";
    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());
    vector<HTreeNode*> &patternsForThisGram = patternsForGram[n_gram-1];

    if (is_interesting_pattern)
        resultFile << "Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;
    else
        resultFile << "Frequent Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;

    foreach(HTreeNode* htreeNode, patternsForThisGram)
    {
//        if (htreeNode->count < 2)
//            continue;

        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        if (is_interesting_pattern)
        {
            if (interestingness_Evaluation_method == "Interaction_Information")
                resultFile << " InteractionInformation = " << toString(htreeNode->interactionInformation);
            else if (interestingness_Evaluation_method == "surprisingness")
                resultFile << " Surprisingness = " << toString(htreeNode->nI_Surprisingness);
        }

        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

//        foreach (Handle link, htreeNode->pattern)
//        {
//            resultFile << atomSpace->atomAsString(link);
//        }
    }

    resultFile.close();


}


// Each HandleSeq in HandleSeqSeq oneOfEachSeqShouldBeVars is a list of nodes that connect two links in inputLinks,
// at least one node in a HandleSeq should be variable, which means two connected links in inputLinks should be connected by at least a variable
// leaves are those nodes that are not connected to any other links in inputLinks, they should be
void PatternMiner::filters(HandleSeq& inputLinks, HandleSeqSeq& oneOfEachSeqShouldBeVars, HandleSeq& leaves, HandleSeq& shouldNotBeVars, AtomSpace* _atomSpace)
{
    if(inputLinks.size() < 2)
        return;

    set<Handle> allNodesInEachLink[inputLinks.size()];
    for (unsigned int i = 0; i < inputLinks.size(); ++i)
    {
        extractAllNodesInLink(inputLinks[i],allNodesInEachLink[i], _atomSpace);
    }

    if (enable_filter_links_should_connect_by_vars)
    {
        // find the common nodes which are shared among inputLinks
        for (unsigned i = 0; i < inputLinks.size() - 1; i ++)
        {
            for (unsigned j = i+1; j < inputLinks.size(); j ++)
            {
                vector<Handle> commonNodes;
                // get the common nodes in allNodesInEachLink[i] and allNodesInEachLink[j]
                std::set_intersection(allNodesInEachLink[i].begin(), allNodesInEachLink[i].end(),
                                      allNodesInEachLink[j].begin(), allNodesInEachLink[j].end(),
                                      std::back_inserter(commonNodes));

                if (commonNodes.size() > 0)
                    oneOfEachSeqShouldBeVars.push_back(commonNodes);

            }

        }
    }

    if ( (enable_filter_leaves_should_not_be_vars) || (enable_filter_node_types_should_not_be_vars) )
    {

        for (unsigned i = 0; i < inputLinks.size(); i ++)
        {
            foreach (Handle node, allNodesInEachLink[i])
            {

                // find leaves
                if (enable_filter_leaves_should_not_be_vars)
                {
                    bool is_leaf = true;

                    // try to find if this node exist in other links of inputLinks
                    for (unsigned j = 0; j < inputLinks.size(); j ++)
                    {
                        if (j == i)
                            continue;

                        if (allNodesInEachLink[j].find(node) != allNodesInEachLink[j].end())
                        {
                            is_leaf = false;
                            break;
                        }

                    }

                    if (is_leaf)
                        leaves.push_back(node);
                }

                // check if this node is in node_types_should_not_be_vars
                if (enable_filter_node_types_should_not_be_vars)
                {
                    Type t = _atomSpace->getType(node);
                    foreach(Type noType, node_types_should_not_be_vars)
                    {
                        if (t == noType)
                        {
                            shouldNotBeVars.push_back(node);
                            break;
                        }
                    }
                }
            }
        }
    }


}

// return true if the inputLinks are disconnected
// when the inputLinks are connected, the outputConnectedGroups has only one group, which is the same as inputLinks
bool PatternMiner::splitDisconnectedLinksIntoConnectedGroups(HandleSeq& inputLinks, HandleSeqSeq& outputConnectedGroups)
{
    if(inputLinks.size() < 2)
        return false;

    set<Handle> allNodesInEachLink[inputLinks.size()];
    for (unsigned int i = 0; i < inputLinks.size(); ++i)
    {
        extractAllVariableNodesInLink(inputLinks[i],allNodesInEachLink[i], atomSpace);
    }

    int i = -1;
    foreach (Handle link, inputLinks)
    {
        i ++;

        if (isInHandleSeqSeq(link, outputConnectedGroups))
            continue;

        // This link is not in outputConnectedGroups, which means none of its previous links connect to this link.
        // So, make a new group.
        HandleSeq newGroup;
        newGroup.push_back(link);

        // Only need to scan the links after this link
        for (unsigned int j = i+1; j < inputLinks.size(); j++)
        {
            foreach(Handle node, allNodesInEachLink[i])
            {
                if (allNodesInEachLink[j].find(node) != allNodesInEachLink[j].end())
                {
                    // they share same node -> they are connected.
                    newGroup.push_back(inputLinks[j]);
                    break;
                }
            }
        }

        outputConnectedGroups.push_back(newGroup);

    }

    return  (outputConnectedGroups.size() > 1);

}

double PatternMiner::calculateEntropyOfASubConnectedPattern(string& connectedSubPatternKey, HandleSeq& connectedSubPattern)
{
    // try to find if it has a correponding HtreeNode
    map<string, HTreeNode*>::iterator subPatternNodeIter = keyStrToHTreeNodeMap.find(connectedSubPatternKey);
    if (subPatternNodeIter != keyStrToHTreeNodeMap.end())
    {
        // it's in the H-Tree, add its entropy
        HTreeNode* subPatternNode = (HTreeNode*)subPatternNodeIter->second;
        // cout << "CalculateEntropy: Found in H-tree! h = log" << subPatternNode->count << " ";
        return log2(subPatternNode->count);
    }
    else
    {
        // can't find its HtreeNode, have to calculate its frequency again by calling pattern matcher
        // Todo: need to decide if add this missing HtreeNode into H-Tree or not

        HTreeNode* newHTreeNode = new HTreeNode();
        keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(connectedSubPatternKey, newHTreeNode));
        newHTreeNode->pattern = connectedSubPattern;

        // Find All Instances in the original AtomSpace For this Pattern
        findAllInstancesForGivenPatternInNestedAtomSpace(newHTreeNode);
        // cout << "CalculateEntropy: Not found in H-tree! call pattern matcher again! h = log" << newHTreeNode->count << " ";

        return log2(newHTreeNode->count);

    }
}


void PatternMiner::calculateInteractionInformation(HTreeNode* HNode)
{
    // this pattern has HandleSeq HNode->pattern.size() gram
    // the formula of interaction information I(XYZ..) =    sign*( H(X) + H(Y) + H(Z) + ...)
    //                                                   + -sign*( H(XY) + H(YZ) + H(XZ) + ...)
    //                                                   +  sign*( H(XYZ) ...)
    // H(X) is the entropy of X
    // Because in our sistuation, for each pattern X, we only care about how many instance it has, we don't record how many times each instance repeats.
    // Let C(X) as the count of instances for patten X, so each instance x for pattern X has an equal chance  1/C(X) of frequency.
    // Therefore, ,  H(X) =  (1/C(X))*log2(C(X))*1/C(X) = log2(C(X)). e.g. if a pattern appears 8 times in a corpus, its entropy is log2(8)

    // First, find all its subpatterns (its parent nodes in the HTree).
    // For the subpatterns those are mising in the HTree, need to call Pattern Matcher to find its frequency again.

//    std::cout << "=================Debug: calculateInteractionInformation for pattern: ====================\n";
//    foreach (Handle link, HNode->pattern)
//    {
//        std::cout << atomSpace->atomAsString(link);
//    }

//    std::cout << "II = ";

    // Start from the last gram:
    int maxgram = HNode->pattern.size();
    int sign;
    if (maxgram%2)
        sign = 1;
    else
        sign = -1;

    double II = sign * log2(HNode->count);
//    std::cout << "H(curpattern) = log" << HNode->count << "="  << II << " sign=" << sign << std::endl;


    for (int gram = maxgram-1; gram > 0; gram --)
    {
         sign *= -1;
//         std::cout << "start subpatterns of gram = " << gram << std::endl;

         bool* indexes = new bool[maxgram];

         // generate the first combination
         for (int i = 0; i < gram; ++ i)
             indexes[i] = true;

         for (int i = gram; i < maxgram; ++ i)
             indexes[i] = false;

         while(true)
         {
             HandleSeq subPattern;
             for (int index = 0; index < maxgram; index ++)
             {
                 if (indexes[index])
                     subPattern.push_back(HNode->pattern[index]);
             }

             HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern);
             string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

//             std::cout<< "Subpattern: " << subPatternKey;

             // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
             HandleSeqSeq splittedSubPattern;
             if (splitDisconnectedLinksIntoConnectedGroups(unifiedSubPattern, splittedSubPattern))
             {
//                 std::cout<< " is disconnected! splitted it into connected parts: \n" ;
                 // The splitted parts are disconnected, so they are independent. So the entroy = the sum of each part.
                 // e.g. if ABC is disconneted, and it's splitted into connected subgroups by splitDisconnectedLinksIntoConnectedGroups,
                 // for example: AC, B  then H(ABC) = H(AC) + H(B)
                 foreach(HandleSeq aConnectedSubPart, splittedSubPattern)
                 {
                     // Unify it again
                     HandleSeq unifiedConnectedSubPattern = UnifyPatternOrder(aConnectedSubPart);
                     string connectedSubPatternKey = unifiedPatternToKeyString(unifiedConnectedSubPattern);
//                     cout << "a splitted part: " << connectedSubPatternKey;
                     double h = calculateEntropyOfASubConnectedPattern(connectedSubPatternKey, unifiedConnectedSubPattern);
                     II += sign*h;
//                     cout << "sign="<<sign << " h =" << h << std::endl << std::endl;

                 }

             }
             else
             {
//                 std::cout<< " is connected! \n" ;
                 double h =calculateEntropyOfASubConnectedPattern(subPatternKey, unifiedSubPattern);
                 II += sign*h;
//                 cout << "sign="<<sign << " h =" << h << std::endl << std::endl;
             }


             if (isLastNElementsAllTrue(indexes, maxgram, gram))
                 break;

             // generate the next combination
             generateNextCombinationGroup(indexes, maxgram);
         }


    }

    HNode->interactionInformation = II;
//    std::cout<< "\n total II = " << II << "\n" ;

}

// try to use H-tree first
//void PatternMiner::calculateInteractionInformation(HTreeNode* HNode)
//{
//    // this pattern has HandleSeq HNode->pattern.size() gram
//    // the formula of interaction information I(XYZ..) =    sign*( H(X) + H(Y) + H(Z) + ...)
//    //                                                   + -sign*( H(XY) + H(YZ) + H(XZ) + ...)
//    //                                                   +  sign*( H(XYZ) ...)
//    // H(X) is the entropy of X
//    // Because in our sistuation, for each pattern X, we only care about how many instance it has, we don't record how many times each instance repeats.
//    // Let C(X) as the count of instances for patten X, so each instance x for pattern X has an equal chance  1/C(X) of frequency.
//    // Therefore, ,  H(X) =  (1/C(X))*log2(C(X))*1/C(X) = log2(C(X)). e.g. if a pattern appears 8 times in a corpus, its entropy is log2(8)

//    // First, find all its subpatterns (its parent nodes in the HTree).
//    // For the subpatterns those are mising in the HTree, need to call Pattern Matcher to find its frequency again.

//    std::cout << "=================Debug: calculateInteractionInformation for pattern: ====================\n";
//    foreach (Handle link, HNode->pattern)
//    {
//        std::cout << atomSpace->atomAsString(link);
//    }

//    std::cout << "II = ";

//    // Start from the last gram:
//    int maxgram = HNode->pattern.size();
//    double II = - log2(HNode->instances.size());
//    std::cout << "-H(curpattern) = log" << HNode->instances.size() << "="  << II  << std::endl;


//    set<HTreeNode*> lastlevelNodes;

//    lastlevelNodes.insert(HNode);

//    int sign = 1;
//    for (int gram = maxgram-1; gram > 0; gram --)
//    {
//         std::cout << "start subpatterns of gram = " << gram << std::endl;
//        // get how many subpatterns this gram should have
//        unsigned int combinNum = combinationCalculate(gram, maxgram);
//        set<HTreeNode*> curlevelNodes;
//        set<string> keystrings;

//        foreach(HTreeNode* curNode, lastlevelNodes)
//        {
//            foreach(HTreeNode* pNode, curNode->parentLinks)
//            {
//                if (curlevelNodes.find(pNode) == curlevelNodes.end())
//                {
//                    std::cout << "H(subpattern): \n";
//                    foreach (Handle link, pNode->pattern)
//                    {
//                        std::cout << atomSpace->atomAsString(link);
//                    }

//                    std::cout << "= " <<  sign << "*" << "log" << pNode->instances.size() << "=" << sign*log2(pNode->instances.size()) << "\n";

//                    II += sign*log2(pNode->instances.size());
//                    curlevelNodes.insert(pNode);
//                    keystrings.insert(unifiedPatternToKeyString(pNode->pattern));
//                }
//            }
//        }

//        if (curlevelNodes.size() < combinNum)
//        {
//             std::cout<<"Debug: calculateInteractionInformation:  missing sub patterns in the H-Tree, need to regenerate them." << std::endl;
//             bool* indexes = new bool[maxgram];

//             // generate the first combination
//             for (int i = 0; i < gram; ++ i)
//                 indexes[i] = true;

//             for (int i = gram; i < maxgram; ++ i)
//                 indexes[i] = false;

//             while(true)
//             {
//                 HandleSeq subPattern;
//                 for (int index = 0; index < maxgram; index ++)
//                 {
//                     if (indexes[index])
//                         subPattern.push_back(HNode->pattern[index]);
//                 }

//                 HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern);
//                 string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

//                 std::cout<< "Subpattern: " << subPatternKey;

//                 if (keystrings.find(subPatternKey)== keystrings.end())
//                 {
//                     std::cout<< " not found above! \n" ;

//                     // a missing subpattern
//                     // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
//                     HandleSeqSeq splittedSubPattern;
//                     if (splitDisconnectedLinksIntoConnectedGroups(unifiedSubPattern, splittedSubPattern))
//                     {
//                         std::cout<< " is disconnected! splitted it into connected parts: \n" ;
//                         // The splitted parts are disconnected, so they are independent. So the entroy = the sum of each part.
//                         // e.g. if ABC is disconneted, and it's splitted into connected subgroups by splitDisconnectedLinksIntoConnectedGroups,
//                         // for example: AC, B  then H(ABC) = H(AC) + H(B)
//                         foreach(HandleSeq aConnectedSubPart, splittedSubPattern)
//                         {
//                             // Unify it again
//                             HandleSeq unifiedConnectedSubPattern = UnifyPatternOrder(aConnectedSubPart);
//                             string connectedSubPatternKey = unifiedPatternToKeyString(unifiedConnectedSubPattern);
//                             cout << "a splitted part: " << connectedSubPatternKey;
//                             double h = calculateEntropyOfASubConnectedPattern(connectedSubPatternKey, unifiedConnectedSubPattern);
//                             II += sign*h;
//                             cout << "sign="<<sign << " h =" << h << std::endl << std::endl;

//                         }

//                     }
//                     else
//                     {
//                         std::cout<< " is connected! \n" ;
//                         double h =calculateEntropyOfASubConnectedPattern(subPatternKey, unifiedSubPattern);
//                         II += sign*h;
//                         cout << "sign="<<sign << " h =" << h << std::endl << std::endl;
//                     }

//                 }
//                 else
//                 {
//                     std::cout<< " already calculated above! skip! \n\n" ;
//                 }


//                 if (isLastNElementsAllTrue(indexes, maxgram, gram))
//                     break;

//                 // generate the next combination
//                 generateNextCombinationGroup(indexes, maxgram);
//             }
//        }


//        lastlevelNodes.clear();
//        lastlevelNodes = curlevelNodes;
//        sign *= -1;
//    }

//    HNode->interactionInformation = II;
//    std::cout<< "\n total II = " << II << "\n" ;

//}

unsigned int PatternMiner::getCountOfAConnectedPattern(string& connectedPatternKey, HandleSeq& connectedPattern)
{
    uniqueKeyLock.lock();
    // try to find if it has a correponding HtreeNode
    map<string, HTreeNode*>::iterator patternNodeIter = keyStrToHTreeNodeMap.find(connectedPatternKey);

    if (patternNodeIter != keyStrToHTreeNodeMap.end())
    {
        uniqueKeyLock.unlock();

        HTreeNode* patternNode = (HTreeNode*)patternNodeIter->second;

        return patternNode->count;
    }
    else
    {
        // can't find its HtreeNode, have to calculate its frequency again by calling pattern matcher
        // Todo: need to decide if add this missing HtreeNode into H-Tree or not

        HTreeNode* newHTreeNode = new HTreeNode();
        keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(connectedPatternKey, newHTreeNode));
        uniqueKeyLock.unlock();

        newHTreeNode->pattern = connectedPattern;

        // Find All Instances in the original AtomSpace For this Pattern
        findAllInstancesForGivenPatternInNestedAtomSpace(newHTreeNode);
//        cout << "Not found in H-tree! call pattern matcher again! count = " << newHTreeNode->count << std::endl;
        return newHTreeNode->count;

    }


}

bool PatternMiner::isALinkOneInstanceOfGivenPattern(Handle &instanceLink, Handle& patternLink, AtomSpace* instanceLinkAtomSpace)
{
    if (instanceLinkAtomSpace->getType(instanceLink) != atomSpace->getType(patternLink))
        return false;

    HandleSeq outComingsOfPattern = atomSpace->getOutgoing(patternLink);
    HandleSeq outComingsOfInstance = instanceLinkAtomSpace->getOutgoing(instanceLink);

    if (outComingsOfPattern.size() != outComingsOfInstance.size())
        return false;

    for(unsigned int i = 0; i < outComingsOfPattern.size(); i ++)
    {
        bool pIsLink = atomSpace->isLink(outComingsOfPattern[i]);
        bool iIsLink = instanceLinkAtomSpace->isLink(outComingsOfInstance[i]);

        if (pIsLink && iIsLink)
        {
            if (isALinkOneInstanceOfGivenPattern(outComingsOfInstance[i], outComingsOfPattern[i], instanceLinkAtomSpace))
                continue;
            else
                return false;
        }
        else if ( (!pIsLink)&&(!iIsLink) )
        {
            // they are both nodes

            // If this node is a variable, skip it
            if (atomSpace->getType(outComingsOfPattern[i]) == VARIABLE_NODE)
                continue;
            else
            {
                if (outComingsOfInstance[i] == outComingsOfPattern[i])
                    continue;
                else
                    return false;
            }
        }
        else
            return false; // this two atoms are of different type
    }

    return true;



}

void PatternMiner::reNameNodesForALink(Handle& inputLink, Handle& nodeToBeRenamed, Handle& newNamedNode,HandleSeq& renameOutgoingLinks,
                                       AtomSpace* _fromAtomSpace, AtomSpace* _toAtomSpace)
{
    HandleSeq outgoingLinks = _fromAtomSpace->getOutgoing(inputLink);

    foreach (Handle h, outgoingLinks)
    {

        if (_fromAtomSpace->isNode(h))
        {
           if (h == nodeToBeRenamed)
           {
               renameOutgoingLinks.push_back(newNamedNode);
           }
           else
           {
               // it's a not the node to be renamed, just add it
               renameOutgoingLinks.push_back(h);
           }
        }
        else
        {
             HandleSeq _renameOutgoingLinks;
             reNameNodesForALink(h, nodeToBeRenamed, newNamedNode, _renameOutgoingLinks, _fromAtomSpace, _toAtomSpace);
             Handle reLink = _toAtomSpace->addLink(_fromAtomSpace->getType(h),_renameOutgoingLinks,_fromAtomSpace->getTV(h));
             renameOutgoingLinks.push_back(reLink);
        }

    }

}

// the return links is in Pattern mining AtomSpace
// toBeExtendedLink is the link contains leaf in the pattern
// varNode is the variableNode of leaf
void PatternMiner::getOneMoreGramExtendedLinksFromGivenLeaf(Handle& toBeExtendedLink, Handle& leaf, Handle& varNode,
                                                            HandleSeq& outPutExtendedPatternLinks, AtomSpace* _fromAtomSpace)
{
    HandleSeq incomings = _fromAtomSpace->getIncoming(leaf);

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

        // skip this extendedHandle if it's one instance of toBeExtendedLink
        if (isALinkOneInstanceOfGivenPattern(extendedHandle, toBeExtendedLink, _fromAtomSpace))
            continue;

        // Rebind this extendedHandle into Pattern Mining Atomspace
        HandleSeq renameOutgoingLinks;
        reNameNodesForALink(extendedHandle, leaf, varNode, renameOutgoingLinks, _fromAtomSpace, atomSpace);
        Handle reLink = atomSpace->addLink(_fromAtomSpace->getType(extendedHandle),renameOutgoingLinks,_fromAtomSpace->getTV(extendedHandle));
        outPutExtendedPatternLinks.push_back(reLink);
    }

}

// make sure only input 2~4 gram patterns
void PatternMiner::calculateSurprisingness( HTreeNode* HNode)
{
    std::cout << "=================Debug: calculate I_Surprisingness for pattern: ====================\n";
    foreach (Handle link, HNode->pattern)
    {
        std::cout << atomSpace->atomAsString(link);
    }
//    std::cout << "count of this pattern = " << HNode->count << std::endl;
//    std::cout << std::endl;

    unsigned int gram = HNode->pattern.size();
    // get the predefined combination:
    // vector<vector<vector<unsigned int>>>
    float minProbability = 9999999999.00000f;
    float maxProbability = 0.0000000f;
//    int comcount = 0;

    foreach(vector<vector<unsigned int>>&  oneCombin, components_ngram[gram-2])
    {
        int com_i = 0;
//        std::cout <<" -----Combination " << comcount++ << "-----" << std::endl;
        float total_p = 1.0f;

        bool containsComponentDisconnected = false;
        foreach (vector<unsigned int>& oneComponent, oneCombin)
        {
            HandleSeq subPattern;
            foreach(unsigned int index, oneComponent)
            {
                subPattern.push_back(HNode->pattern[index]);
            }

            HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern);
            string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

//            std::cout<< "Subpattern: " << subPatternKey;

            // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
            HandleSeqSeq splittedSubPattern;
            if (splitDisconnectedLinksIntoConnectedGroups(unifiedSubPattern, splittedSubPattern))
            {
//                std::cout<< " is disconnected! skip it \n" ;
                containsComponentDisconnected = true;
                break;
            }
            else
            {
//                std::cout<< " is connected!" ;
                unsigned int component_count = getCountOfAConnectedPattern(subPatternKey, unifiedSubPattern);
//                cout << ", count = " << component_count;
                float p_i = ((float)(component_count)) / atomspaceSizeFloat;

//                cout << ", p = " << component_count  << " / " << (int)atomspaceSizeFloat << " = " << p_i << std::endl;
                total_p *= p_i;
                std::cout << std::endl;
            }

            com_i ++;

        }

        if (containsComponentDisconnected)
            continue;


//        cout << "\n ---- total_p = " << total_p << " ----\n" ;


        if (total_p < minProbability)
            minProbability = total_p;

        if (total_p > maxProbability)
            maxProbability = total_p;

    }

//    cout << "\nIn all the probability calculated by all possible component combinations, maxProbability  = " << maxProbability << ", minProbability = " << minProbability << std::endl;
    float p = ((float)HNode->count)/atomspaceSizeFloat;
//    cout << "For this pattern itself: p = " <<  HNode->count << " / " <<  (int)atomspaceSizeFloat << " = " << p << std::endl;

    float surprisingness_max = (p - maxProbability) / p;
    float surprisingness_min = (minProbability - p) / p;
//    cout << "\np - maxProbability = " << surprisingness_max << "; minProbability - p = " << surprisingness_min << std::endl;

    if (surprisingness_max >= surprisingness_min)
        HNode->nI_Surprisingness = surprisingness_max;
    else
        HNode->nI_Surprisingness = surprisingness_min;

//    cout << "nI_Surprisingness = " << HNode->nI_Surprisingness  << std::endl;

    if (gram == MAX_GRAM ) // can't calculate II_Surprisingness for MAX_GRAM patterns, becasue it required gram +1 patterns
        return;

    std::cout << "=================Debug: calculate II_Surprisingness for pattern: ====================\n";

//    Handle newVarNode = atomSpace->addNode(VARIABLE_NODE, "$var_" + toString(HNode->var_num));

//    // first , get all its superpatterns
//    foreach (Handle toBeExtendLink, HNode->pattern)
//    {
//        set<Handle> allNodes;
//        extractAllNodesInLink(toBeExtendLink, allNodes, atomSpace);

//        HandleSeq extendedPattern;
//        foreach (Handle link, HNode->pattern)
//        {
//            if (link != toBeExtendLink)
//                extendedPattern.push_back(link);
//        }

//        foreach(Handle toBeExtendNode, allNodes)
//        {
//            if ( atomSpace->getType(toBeExtendNode) == VARIABLE_NODE)
//                continue; // only extend leaf

//            HandleSeq extendedHandles;
//            if (Pattern_mining_mode == "Breadth_First")
//                getOneMoreGramExtendedLinksFromGivenLeaf(toBeExtendLink, toBeExtendNode, newVarNode, extendedHandles, originalAtomSpace);
//            else // (Pattern_mining_mode == "Depth_First")
//                getOneMoreGramExtendedLinksFromGivenLeaf(toBeExtendLink, toBeExtendNode, newVarNode, extendedHandles, observingAtomSpace);

//            // reBind the toBeExtendLink too, toBeExtendNode -> newVarNode
//            HandleSeq reToBeExtendLinkOutgoings;
//            reNameNodesForALink(toBeExtendLink,toBeExtendNode, newVarNode, reToBeExtendLinkOutgoings, atomSpace, atomSpace);
//            Handle reToBeExtendLink = atomSpace->addLink(atomSpace->getType(toBeExtendLink), reToBeExtendLinkOutgoings, atomSpace->getTV(toBeExtendLink));
//            extendedPattern.push_back(reToBeExtendLink);

//            // note that extendedLink has been already added into Pattern Mining Atomspace with new variable node

//            foreach(Handle extendedLink, extendedHandles)
//            {
//                // add this extendedLink into pattern, get an n+1 gram pattern
//                extendedPattern.push_back(extendedLink);

//                HandleSeq unifiedExtendedPattern = UnifyPatternOrder(extendedPattern);
//                string keyStr = unifiedPatternToKeyString(unifiedExtendedPattern);
//                int count = getCountOfAConnectedPattern(keyStr, unifiedExtendedPattern);
//            }

//        }
//    }



}

// in vector<vector<vector<unsigned int>>> the  <unsigned int> is the index in pattern HandleSeq : 0~n
void PatternMiner::generateComponentCombinations(string componentsStr, vector<vector<vector<unsigned int>>> &componentCombinations)
{
    // "0,12|1,02|2,01|0,1,2"

    vector<string> allCombinationsStrs = StringManipulator::split(componentsStr,"|");

    foreach(string oneCombinStr, allCombinationsStrs)
    {
        // "0,12"
        vector<vector<unsigned int>> oneCombin;

        vector<string> allComponentStrs = StringManipulator::split(oneCombinStr,",");
        foreach(string oneComponentStr, allComponentStrs)
        {
            vector<unsigned int> oneComponent;
            for(std::string::size_type i = 0; i < oneComponentStr.size(); ++i)
            {
                oneComponent.push_back((unsigned int)(oneComponentStr[i] - '0'));
            }

            oneCombin.push_back(oneComponent);

        }

        componentCombinations.push_back(oneCombin);
    }


}

PatternMiner::PatternMiner(AtomSpace* _originalAtomSpace, unsigned int max_gram): originalAtomSpace(_originalAtomSpace)
{
    htree = new HTree();
    atomSpace = new AtomSpace( _originalAtomSpace);

    unsigned int system_thread_num  = std::thread::hardware_concurrency();

    if (system_thread_num > 1)
        THREAD_NUM = system_thread_num - 1;
    else
        THREAD_NUM = 1;

     // use all the threads in this machine
     THREAD_NUM = system_thread_num;


    threads = new thread[THREAD_NUM];

    MAX_GRAM = max_gram;
    cur_gram = 0;

    ignoredTypes[0] = LIST_LINK;

    enable_Frequent_Pattern = config().get_bool("Enable_Frequent_Pattern");
    enable_Interesting_Pattern = config().get_bool("Enable_Interesting_Pattern");
    interestingness_Evaluation_method = config().get("Interestingness_Evaluation_method");

    assert(enable_Frequent_Pattern || enable_Interesting_Pattern);
    //The options are "Interaction_Information", "surprisingness"
    assert( (interestingness_Evaluation_method == "Interaction_Information") || (interestingness_Evaluation_method == "surprisingness") );

    enable_filter_leaves_should_not_be_vars = config().get_bool("enable_filter_leaves_should_not_be_vars");
    enable_filter_links_should_connect_by_vars = config().get_bool("enable_filter_links_should_connect_by_vars");
    enable_filter_node_types_should_not_be_vars =  config().get_bool("enable_filter_node_types_should_not_be_vars");

    if (enable_filter_node_types_should_not_be_vars)
    {
        string node_types_str = config().get("node_types_should_not_be_vars");
        node_types_str.erase(std::remove(node_types_str.begin(), node_types_str.end(), ' '), node_types_str.end());
        vector<string> typeStrs = StringManipulator::split(node_types_str,",");
        foreach(string typestr, typeStrs)
        {
            node_types_should_not_be_vars.push_back( classserver().getType(typestr) );
        }
    }

    // vector < vector<HTreeNode*> > patternsForGram
    for (unsigned int i = 0; i < max_gram; ++i)
    {
        vector<HTreeNode*> patternVector;
        patternsForGram.push_back(patternVector);
    }

    // define (hard coding) all the possible subcomponent combinations for 2~4 gram patterns
    string gramNcomponents[3];
    // for 2 gram patterns [01], the only possible combination is [0][1]
    gramNcomponents[0] = "0,1";

    // for 3 gram patterns [012], the possible combinations are [0][12],[1][02],[2][01],[0][1][2]
    gramNcomponents[1] = "0,12|1,02|2,01|0,1,2";

    // for 4 gram patterns [0123], the possible combinations are:
    // [0][123],[1][023],[2][013], [3][012], [01][23],[02][13],[03][12],
    // [0][1][23],[0][2][13],[0][3][12],[1][2][03],[1][3][02],[2][3][01], [0][1][2][3]
    gramNcomponents[2] = "0,123|1,023|2,013|3,012|01,23|02,13|03,12|0,1,23|0,2,13|0,3,12|1,2,03|1,3,02|2,3,01|0,1,2,3";

    // generate vector<vector<vector<unsigned int>>> components_ngram[3] from above hard coding combinations for 2~4 grams

    int ngram = 0;
    foreach(string componentCombinsStr, gramNcomponents)
    {
        generateComponentCombinations(componentCombinsStr, this->components_ngram[ngram]);
        ngram ++;
    }

    std::cout<<"Debug: PatternMiner init finished! " + toString(THREAD_NUM) + " threads used!" << std::endl;
}

PatternMiner::~PatternMiner()
{
    delete htree;
    delete atomSpace;
}

void PatternMiner::runPatternMiner(unsigned int _thresholdFrequency)
{
    thresholdFrequency = _thresholdFrequency;

    Pattern_mining_mode = config().get("Pattern_mining_mode"); // option: Breadth_First , Depth_First
    assert( (Pattern_mining_mode == "Breadth_First") || (Pattern_mining_mode == "Depth_First"));

    std::cout<<"Debug: PatternMining start! Max gram = " + toString(this->MAX_GRAM) << ", mode = " << Pattern_mining_mode << std::endl;

    int start_time = time(NULL);

    originalAtomSpace->getHandlesByType(back_inserter(allLinks), (Type) LINK, true );

    allLinkNumber = (int)(allLinks.size());
    atomspaceSizeFloat = (float)(allLinkNumber);

    std::cout<<"Corpus size: "<< allLinkNumber << " links in total. \n";

    if (Pattern_mining_mode == "Breadth_First")
        runPatternMinerBreadthFirst();
    else
    {
        runPatternMinerDepthFirst();

        if (enable_Frequent_Pattern)
        {
            for(unsigned int gram = 1; gram <= MAX_GRAM; gram ++)
            {
                // sort by frequency
                std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeByFrequency );

                // Finished mining gram patterns; output to file
                std::cout<<"Debug: PatternMiner:  done (gram = " + toString(gram) + ") frequent pattern mining!" + toString((patternsForGram[gram-1]).size()) + " patterns found! " << std::endl;

                OutPutPatternsToFile(gram);
            }
        }


        if (enable_Interesting_Pattern)
        {
            for(unsigned int gram = 2; gram <= MAX_GRAM; gram ++)
            {
                cout << "\nCalculating interestingness for " << gram << "gram patterns";
                // evaluate the interestingness
                // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
                if (interestingness_Evaluation_method == "Interaction_Information")
                {
                    cout << "by evaluating Interaction_Information ...\n";
                   // calculate interaction information
                   foreach(HTreeNode* htreeNode, patternsForGram[gram-1])
                   {
                       calculateInteractionInformation(htreeNode);
                   }

                   // sort by interaction information
                   std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeByInteractionInformation);
                }
                else if (interestingness_Evaluation_method == "surprisingness")
                {
                    cout << "by evaluating surprisingness ...\n";
                    // calculate surprisingness
                    foreach(HTreeNode* htreeNode, patternsForGram[gram-1])
                    {
                        calculateSurprisingness(htreeNode);
                    }

                    // sort by surprisingness
                    std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeBySurprisingness);
                }

                // Finished mining gram patterns; output to file
                std::cout<<"Debug: PatternMiner:  done (gram = " + toString(gram) + ") interesting pattern mining!" + toString((patternsForGram[gram-1]).size()) + " patterns found! " << std::endl;

                OutPutPatternsToFile(gram, true);
            }
        }
    }

    int end_time = time(NULL);
    printf("Pattern Mining Finish one round! Total time: %d seconds. \n", end_time - start_time);
    std::cout<<"Corpus size: "<< allLinkNumber << " links in total. \n";

//   testPatternMatcher2();

//   selectSubsetFromCorpus();

}



void PatternMiner::selectSubsetFromCorpus(vector<string>& topics, unsigned int gram)
{
    // select a subset for test topics from the huge ConceptNet corpus
    _selectSubsetFromCorpus(topics,gram);
}

std::string PatternMiner::Link2keyString(Handle& h, std::string indent, const AtomSpace *atomspace)
{
    if (atomspace == 0)
        atomspace = this->atomSpace;

    std::stringstream answer;
    std::string more_indent = indent + "  ";

    answer << indent  << "(" << classserver().getTypeName(atomspace->getType(h)) << " ";

    if (atomspace->isNode(h))
    {
        answer << atomspace->getName(h);
    }

    answer << ")" << "\n";

    if (atomspace->isLink(h))
    {
        HandleSeq outgoings = atomspace->getOutgoing(h);
        foreach(Handle outgoing, outgoings)
            answer << Link2keyString(outgoing, more_indent, atomspace);
    }

    return answer.str();
}

void PatternMiner::testPatternMatcher1()
{
    originalAtomSpace->getHandlesByType(back_inserter(allLinks), (Type) LINK, true );
    std::cout<<"Debug: PatternMiner total link numer = "  << allLinks.size() << std::endl;

//(BindLink (stv 1.000000 1.000000)
//  (ListLink (stv 1.000000 1.000000)
//    (VariableNode "$var_1") ; [66]
//    (VariableNode "$var_2") ; [265]
//    (VariableNode "$var_3") ; [673]
//    (VariableNode "$var_4") ; [729]
//  ) ; [734]
//  (ImplicationLink (stv 1.000000 1.000000)
//    (AndLink (stv 1.000000 1.000000)
//      (EvaluationLink (stv 1.000000 1.000000)
//        (VariableNode "$var_1") ; [66]
//        (ListLink (stv 1.000000 1.000000)
//          (ObjectNode "Bob") ; [13]
//          (VariableNode "$var_2") ; [265]
//        ) ; [331]
//      ) ; [332]
//      (EvaluationLink (stv 1.000000 1.000000)
//        (VariableNode "$var_1") ; [66]
//        (ListLink (stv 1.000000 1.000000)
//          (VariableNode "$var_3") ; [673]
//          (VariableNode "$var_2") ; [265]
//        ) ; [930]
//      ) ; [931]
//      (InheritanceLink (stv 1.000000 1.000000)
//        (VariableNode "$var_3") ; [673]
//        (VariableNode "$var_4") ; [729]
//      ) ; [3482]
//    ) ; [5611]
//    ...
    HandleSeq variableNodes, implicationLinkOutgoings, bindLinkOutgoings, patternToMatch;

    Handle varHandle1 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_1" );
    Handle varHandle2 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_2" );
    Handle varHandle3 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_3" );
    Handle varHandle4 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_4" );


    variableNodes.push_back(varHandle1);
    variableNodes.push_back(varHandle2);
    variableNodes.push_back(varHandle3);
    variableNodes.push_back(varHandle4);

    // The first EvaluationLink
    HandleSeq listlinkOutgoings1, evalLinkOutgoings1;
    Handle bobNode = originalAtomSpace->addNode(opencog::OBJECT_NODE, "Bob" );
    listlinkOutgoings1.push_back(bobNode);
    listlinkOutgoings1.push_back(varHandle2);
    Handle listlink1 = originalAtomSpace->addLink(LIST_LINK, listlinkOutgoings1, TruthValue::TRUE_TV());
    evalLinkOutgoings1.push_back(varHandle1);
    evalLinkOutgoings1.push_back(listlink1);
    Handle evalLink1 = originalAtomSpace->addLink(EVALUATION_LINK, evalLinkOutgoings1, TruthValue::TRUE_TV());

    // The second EvaluationLink
    HandleSeq listlinkOutgoings2, evalLinkOutgoings2;
    listlinkOutgoings2.push_back(varHandle3);
    listlinkOutgoings2.push_back(varHandle2);
    Handle listlink2 = originalAtomSpace->addLink(LIST_LINK, listlinkOutgoings2, TruthValue::TRUE_TV());
    evalLinkOutgoings2.push_back(varHandle1);
    evalLinkOutgoings2.push_back(listlink2);
    Handle evalLink2 = originalAtomSpace->addLink(EVALUATION_LINK, evalLinkOutgoings2, TruthValue::TRUE_TV());

    // The InheritanceLink
    HandleSeq inherOutgoings;
    inherOutgoings.push_back(varHandle3);
    inherOutgoings.push_back(varHandle4);
    Handle inherLink = originalAtomSpace->addLink(INHERITANCE_LINK, inherOutgoings, TruthValue::TRUE_TV());

    patternToMatch.push_back(evalLink1);
    patternToMatch.push_back(evalLink2);
    patternToMatch.push_back(inherLink);

    Handle hAndLink = originalAtomSpace->addLink(AND_LINK, patternToMatch, TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
    implicationLinkOutgoings.push_back(hAndLink); // the results to return

    Handle hImplicationLink = originalAtomSpace->addLink(IMPLICATION_LINK, implicationLinkOutgoings, TruthValue::TRUE_TV());

    // add variable atoms
    Handle hVariablesListLink = originalAtomSpace->addLink(LIST_LINK, variableNodes, TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = originalAtomSpace->addLink(BIND_LINK, bindLinkOutgoings, TruthValue::TRUE_TV());

    std::cout <<"Debug: PatternMiner::testPatternMatcher for pattern:" << std::endl
              << originalAtomSpace->atomAsString(hBindLink).c_str() << std::endl;


    // Run pattern matcher
    Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = originalAtomSpace->getOutgoing(hResultListLink);

    std::cout << toString(resultSet.size())  << " instances found:" << std::endl ;

    //debug
    std::cout << originalAtomSpace->atomAsString(hResultListLink) << std::endl  << std::endl;

    originalAtomSpace->removeAtom(hResultListLink);
    originalAtomSpace->removeAtom(hBindLink);


}

void PatternMiner::testPatternMatcher2()
{

//    (BindLink (stv 1.000000 1.000000)
//      (ListLink (stv 1.000000 1.000000)
//        (VariableNode "$var_1") ; [63]
//        (VariableNode "$var_2") ; [545]
//        (VariableNode "$var_3") ; [2003]
//      ) ; [4575]
//      (ImplicationLink (stv 1.000000 1.000000)
//        (AndLink (stv 1.000000 1.000000)
//          (EvaluationLink (stv 1.000000 1.000000)
//            (VariableNode "$var_1") ; [63]
//            (ListLink (stv 1.000000 1.000000)
//              (ObjectNode "LiMing") ; [15]
//              (ConceptNode "tea") ; [20]
//            ) ; [44]
//          ) ; [4569]
//          (EvaluationLink (stv 1.000000 1.000000)
//            (VariableNode "$var_1") ; [63]
//            (ListLink (stv 1.000000 1.000000)
//              (VariableNode "$var_2") ; [545]
//              (VariableNode "$var_3") ; [2003]
//            ) ; [4570]
//          ) ; [4571]
//          (InheritanceLink (stv 1.000000 1.000000)
//            (VariableNode "$var_2") ; [545]
//            (ConceptNode "human") ; [3]
//          ) ; [4572]
//        ) ; [4573]
//        (AndLink (stv 1.000000 1.000000)
//        .....the same as the pattern
//      ) ; [4574]
//    ) ; [4576]

    HandleSeq variableNodes, implicationLinkOutgoings, bindLinkOutgoings, patternToMatch, resultOutgoings;

    Handle varHandle1 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_1" );
    Handle varHandle2 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_2" );
    Handle varHandle3 = originalAtomSpace->addNode(opencog::VARIABLE_NODE,"$var_3" );

    variableNodes.push_back(varHandle1);
    variableNodes.push_back(varHandle2);
    variableNodes.push_back(varHandle3);

    // The first EvaluationLink
    HandleSeq listlinkOutgoings1, evalLinkOutgoings1;
    Handle LiMingNode = originalAtomSpace->addNode(opencog::OBJECT_NODE, "LiMing" );
    listlinkOutgoings1.push_back(LiMingNode);
    Handle teaNode = originalAtomSpace->addNode(opencog::CONCEPT_NODE, "tea" );
    listlinkOutgoings1.push_back(teaNode);
    Handle listlink1 = originalAtomSpace->addLink(LIST_LINK, listlinkOutgoings1, TruthValue::TRUE_TV());
    evalLinkOutgoings1.push_back(varHandle1);
    evalLinkOutgoings1.push_back(listlink1);
    Handle evalLink1 = originalAtomSpace->addLink(EVALUATION_LINK, evalLinkOutgoings1, TruthValue::TRUE_TV());

    // The second EvaluationLink
    HandleSeq listlinkOutgoings2, evalLinkOutgoings2;
    listlinkOutgoings2.push_back(varHandle2);
    listlinkOutgoings2.push_back(varHandle3);
    Handle listlink2 = originalAtomSpace->addLink(LIST_LINK, listlinkOutgoings2, TruthValue::TRUE_TV());
    evalLinkOutgoings2.push_back(varHandle1);
    evalLinkOutgoings2.push_back(listlink2);
    Handle evalLink2 = originalAtomSpace->addLink(EVALUATION_LINK, evalLinkOutgoings2, TruthValue::TRUE_TV());

    // The InheritanceLink
    HandleSeq inherOutgoings;
    Handle humanNode = originalAtomSpace->addNode(opencog::CONCEPT_NODE, "human" );
    inherOutgoings.push_back(varHandle2);
    inherOutgoings.push_back(humanNode);
    Handle inherLink = originalAtomSpace->addLink(INHERITANCE_LINK, inherOutgoings, TruthValue::TRUE_TV());

    patternToMatch.push_back(evalLink1);
    patternToMatch.push_back(evalLink2);
    patternToMatch.push_back(inherLink);

    Handle hAndLink = originalAtomSpace->addLink(AND_LINK, patternToMatch, TruthValue::TRUE_TV());

    // add variable atoms
    Handle hVariablesListLink = originalAtomSpace->addLink(LIST_LINK, variableNodes, TruthValue::TRUE_TV());

    resultOutgoings.push_back(hVariablesListLink);
    resultOutgoings.push_back(hAndLink);

    Handle hListLinkResult = originalAtomSpace->addLink(LIST_LINK, resultOutgoings, TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
    implicationLinkOutgoings.push_back(hListLinkResult); // the results to return

    Handle hImplicationLink = originalAtomSpace->addLink(IMPLICATION_LINK, implicationLinkOutgoings, TruthValue::TRUE_TV());


    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = originalAtomSpace->addLink(BIND_LINK, bindLinkOutgoings, TruthValue::TRUE_TV());

    std::cout <<"Debug: PatternMiner::testPatternMatcher for pattern:" << std::endl
              << originalAtomSpace->atomAsString(hBindLink).c_str() << std::endl;


    // Run pattern matcher
    Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = originalAtomSpace->getOutgoing(hResultListLink);

    std::cout << toString(resultSet.size())  << " instances found:" << std::endl ;

    //debug
    std::cout << originalAtomSpace->atomAsString(hResultListLink) << std::endl  << std::endl;

    originalAtomSpace->removeAtom(hResultListLink);
    originalAtomSpace->removeAtom(hBindLink);


}

set<Handle> PatternMiner::_getAllNonIgnoredLinksForGivenNode(Handle keywordNode, set<Handle>& allSubsetLinks)
{
    set<Handle> newHandles;
    HandleSeq incomings = originalAtomSpace->getIncoming(keywordNode);

    foreach (Handle incomingHandle, incomings)
    {
        Handle newh = incomingHandle;

        // if this atom is a igonred type, get its first parent that is not in the igonred types
        if (isIgnoredType (originalAtomSpace->getType(incomingHandle)) )
        {
            newh = getFirstNonIgnoredIncomingLink(originalAtomSpace, incomingHandle);

            if ((newh == Handle::UNDEFINED) || containIgnoredContent(newh ))
                continue;
        }

        if (allSubsetLinks.find(newh) == allSubsetLinks.end())
            newHandles.insert(newh);

    }

    return newHandles;
}

set<Handle> PatternMiner::_extendOneLinkForSubsetCorpus(set<Handle>& allNewLinksLastGram, set<Handle>& allSubsetLinks)
{
    set<Handle> allNewConnectedLinksThisGram;
    // only extend the links in allNewLinksLastGram. allNewLinksLastGram is a part of allSubsetLinks
    foreach(Handle link, allNewLinksLastGram)
    {
        // find all nodes in this link
        set<Handle> allNodes;
        extractAllNodesInLink(link, allNodes, originalAtomSpace);

        foreach (Handle neighborNode, allNodes)
        {
            string content = originalAtomSpace->getName(neighborNode);
            if (isIgnoredContent(content))
                continue;

            set<Handle> newConnectedLinks;
            newConnectedLinks = _getAllNonIgnoredLinksForGivenNode(neighborNode, allSubsetLinks);
            allNewConnectedLinksThisGram.insert(newConnectedLinks.begin(),newConnectedLinks.end());
            allSubsetLinks.insert(newConnectedLinks.begin(),newConnectedLinks.end());
        }

    }

    return allNewConnectedLinksThisGram;
}

// must load the corpus before calling this function
void PatternMiner::_selectSubsetFromCorpus(vector<string>& subsetKeywords, unsigned int max_connection)
{
    std::cout << "\nSelecting a subset from loaded corpus in Atomspace for the following topics:" << std::endl ;
    set<Handle> allSubsetLinks;
    string topicsStr = "";

    foreach (string keyword, subsetKeywords)
    {
        std::cout << keyword << std::endl;
        Handle keywordNode = originalAtomSpace->addNode(opencog::CONCEPT_NODE,keyword);
        set<Handle> newConnectedLinks = _getAllNonIgnoredLinksForGivenNode(keywordNode, allSubsetLinks);
        allSubsetLinks.insert(newConnectedLinks.begin(), newConnectedLinks.end());
        topicsStr += "-";
        topicsStr += keyword;

    }

    unsigned int order = 0;
    set<Handle> allNewConnectedLinksThisGram = allSubsetLinks;

    while (order < max_connection)
    {
        allNewConnectedLinksThisGram = _extendOneLinkForSubsetCorpus(allNewConnectedLinksThisGram, allSubsetLinks);
        order ++;
    }

    ofstream subsetFile;

    string fileName = "SubSet" + topicsStr + ".scm";

    subsetFile.open(fileName.c_str());

    // write the first line to enable unicode
    std::cout <<  "(setlocale LC_CTYPE \"\")" << std::endl ;

    foreach(Handle h, allSubsetLinks)
    {
        if (containIgnoredContent(h))
            continue;

        subsetFile << originalAtomSpace->atomAsString(h);
    }

    subsetFile.close();

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl ;
}

bool PatternMiner::isIgnoredContent(string keyword)
{
    foreach (string ignoreWord, ignoreKeyWords)
    {
        if (keyword == ignoreWord)
            return true;
    }

    return false;
}

bool PatternMiner::containIgnoredContent(Handle link )
{
    string str = originalAtomSpace->atomAsString(link);

    foreach (string ignoreWord, ignoreKeyWords)
    {
        string ignoreStr = "\"" + ignoreWord + "\"";
        if (str.find(ignoreStr) != std::string::npos)
            return true;
    }

    return false;
}



