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

#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/embodiment/atom_types.h>
//#include <opencog/atoms/bind/BindLink.h>
#include <opencog/query/BindLinkAPI.h>
#include <opencog/util/Config.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


bool isInStringVector(string _item, vector<string> _vector)
{
    for (string s : _vector)
    {
        if (s == _item)
            return true;
    }

    return false;
}

void PatternMiner::generateIndexesOfSharedVars(Handle& link, HandleSeq& orderedHandles, vector < vector<int> >& indexes)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();
    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if (h->getType() == opencog::VARIABLE_NODE)
            {
                string var_name = h->getName();

                vector<int> indexesForCurVar;
                int index = 0;

                for (Handle oh : orderedHandles)
                {
                    string ohStr = oh->toShortString();
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

    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {

        if (h->isNode())
        {
           if (h->getType() == opencog::VARIABLE_NODE)
           {
               // it's a variable node, rename it
               if (varNameMap.find(h) != varNameMap.end())
               {
                   renameOutgoingLinks.push_back(varNameMap[h]);
               }
               else
               {
                   string var_name = "$var_"  + toString(varNameMap.size() + 1);
                   Handle var_node = atomSpace->add_node(opencog::VARIABLE_NODE, var_name);
                   // XXX why do we need to set the TV ???
                   var_node->merge(TruthValue::TRUE_TV());
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
             Handle reLink = atomSpace->add_link(h->getType(),_renameOutgoingLinks);
             // XXX why do we need to set the TV ???
             reLink->merge(TruthValue::TRUE_TV());
             renameOutgoingLinks.push_back(reLink);
        }

    }

}

HandleSeq PatternMiner::RebindVariableNames(HandleSeq& orderedPattern, map<Handle,Handle>& orderedVarNameMap)
{

    HandleSeq rebindedPattern;

    for (Handle link : orderedPattern)
    {
        HandleSeq renameOutgoingLinks;
        findAndRenameVariablesForOneLink(link, orderedVarNameMap, renameOutgoingLinks);
        Handle rebindedLink = atomSpace->add_link(link->getType(),renameOutgoingLinks);
        // XXX why do we need to set the TV ???
        rebindedLink->merge(TruthValue::TRUE_TV());
        rebindedPattern.push_back(rebindedLink);
    }

    return rebindedPattern;
}

// the input links should be like: only specify the const node, all the variable node name should not be specified:
// unifiedLastLinkIndex is to return where the last link in the input pattern is now in the ordered pattern
// because the last link in input pattern is the externed link from last gram pattern
HandleSeq PatternMiner::UnifyPatternOrder(HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex)
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

    for (Handle inputH : inputPattern)
    {
        string str = inputH->toShortString();
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

    HandleSeq orderedHandles;
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
    for (string keyString : duplicateStrs)
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
        for (_non_ordered_pattern np : sharedSameKeyPatterns)
        {
            orderedHandles.push_back(np.link);
        }
    }

    // find out where the last link in the input pattern is now in the ordered pattern
    Handle lastLink = inputPattern[inputPattern.size()-1];
    unsigned int lastLinkIndex = 0;
    for (Handle h : orderedHandles)
    {
        if (h == lastLink)
        {
            unifiedLastLinkIndex = lastLinkIndex;
            break;
        }

        ++ lastLinkIndex;

    }

    // in this map, the first Handle is the variable node is the original Atomspace,
    // the second Handle is the renamed ordered variable node in the Pattern Mining Atomspace.
    map<Handle,Handle> orderedVarNameMap;

    HandleSeq rebindPattern = RebindVariableNames(orderedHandles, orderedVarNameMap);

    return rebindPattern;

}

string PatternMiner::unifiedPatternToKeyString(HandleSeq& inputPattern, const AtomSpace *atomspace)
{
    if (atomspace == 0)
        atomspace = this->atomSpace;

    string keyStr = "";
    for (Handle h : inputPattern)
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
    HandleSeq outgoingLinks = originalLink->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
           if (valueToVarMap.find(h) != valueToVarMap.end())
           {
               // this node is considered as a variable
               outputOutgoings.push_back(valueToVarMap[h]);
           }
           else
           {
               // this node is considered not a variable, so add its bound value node into the Pattern mining Atomspace
               Handle value_node = atomSpace->add_node(h->getType(), h->getName());
               // XXX why do we need to set the TV ???
               value_node->merge(TruthValue::TRUE_TV());
               outputOutgoings.push_back(value_node);
           }
        }
        else
        {
             HandleSeq _outputOutgoings;
             generateALinkByChosenVariables(h, valueToVarMap, _outputOutgoings, _fromAtomSpace);
             Handle reLink = atomSpace->add_link(h->getType(),_outputOutgoings);
             // XXX why do we need to set the TV ???
             reLink->merge(TruthValue::TRUE_TV());
             outputOutgoings.push_back(reLink);
        }
    }
}

 // valueToVarMap:  the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace
// _fromAtomSpace: where is input link from
void PatternMiner::extractAllNodesInLink(Handle link, map<Handle,Handle>& valueToVarMap, AtomSpace* _fromAtomSpace)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if (valueToVarMap.find(h) == valueToVarMap.end())
            {
                // add a variable node in Pattern miner Atomspace
                Handle varHandle = atomSpace->add_node(opencog::VARIABLE_NODE,"$var~" + toString(valueToVarMap.size()) );
                valueToVarMap.insert(std::pair<Handle,Handle>(h,varHandle));
            }

            if ((h->getType() == opencog::VARIABLE_NODE))
                cout<<"Error: instance link contains variables: \n" << h->toShortString() <<std::endl;

        }
        else
        {
            extractAllNodesInLink(h,valueToVarMap,_fromAtomSpace);
        }
    }
}

void PatternMiner::extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, OrderedHandleSet& allVarNodes)
{

    HandleSeq ioutgoingLinks = instanceLink->getOutgoingSet();
    HandleSeq poutgoingLinks = patternLink->getOutgoingSet();

    HandleSeq::iterator pit = poutgoingLinks.begin();

    for (Handle h : ioutgoingLinks)
    {
        if (h->isNode())
        {
            if (((*pit)->getType() == opencog::VARIABLE_NODE))
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

    HandleSeq ioutgoingLinks = instanceLink->getOutgoingSet();
    HandleSeq poutgoingLinks = patternLink->getOutgoingSet();

    HandleSeq::iterator pit = poutgoingLinks.begin();

    for (Handle h : ioutgoingLinks)
    {
        if (h->isNode())
        {
            if (((*pit)->getType() == opencog::VARIABLE_NODE))
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
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
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

void PatternMiner::extractAllNodesInLink(Handle link, OrderedHandleSet& allNodes, AtomSpace* _fromAtomSpace)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
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

void PatternMiner::extractAllVariableNodesInLink(Handle link, OrderedHandleSet& allNodes, AtomSpace* _atomSpace)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if ((h->getType() == opencog::VARIABLE_NODE) && (allNodes.find(h) == allNodes.end()))
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
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if (h->getType() != opencog::VARIABLE_NODE)
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
    HandleSeq outgoingLinks = fromLink->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
           Handle new_node = toAtomSpace->add_node(h->getType(), h->getName());
           new_node->merge(h->getTruthValue());
           outgoings.push_back(new_node);
           if (h->getType() == VARIABLE_NODE)
           {
               if ( ! isInHandleSeq(new_node, outVariableNodes) ) // should not have duplicated variable nodes
                outVariableNodes.push_back(new_node);
           }
        }
        else
        {
             HandleSeq _OutgoingLinks;

             swapOneLinkBetweenTwoAtomSpace(fromAtomSpace, toAtomSpace, h, _OutgoingLinks, outVariableNodes);
             Handle _link = toAtomSpace->add_link(h->getType(), _OutgoingLinks);
             _link->merge(h->getTruthValue());

             outgoings.push_back(_link);
        }
    }
}

// linksWillBeDel are all the links contain varaibles. Those links need to be deleted after run BindLink
HandleSeq PatternMiner::swapLinksBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq& outVariableNodes)
{
    HandleSeq outPutLinks;

    for (Handle link : fromLinks)
    {
        HandleSeq outgoingLinks;

        swapOneLinkBetweenTwoAtomSpace(fromAtomSpace, toAtomSpace, link, outgoingLinks, outVariableNodes);
        Handle toLink = toAtomSpace->add_link(link->getType(), outgoingLinks);
        toLink->merge(link->getTruthValue());

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
//                << (patternToMatch[0])->toShortString().c_str() << std::endl;
//    }

    AtomSpace* _atomSpace = originalAtomSpace;

    Handle hAndLink = _atomSpace->add_link(AND_LINK, HNode->pattern);
    // XXX why do we need to set the TV ???
    // hAndLink->merge(TruthValue::TRUE_TV());
    // Handle hOutPutListLink = _atomSpace->add_link(AND_LINK, HNode->pattern);
    // XXX why do we need to set the TV ???
    // hOutPutListLink->merge(TruthValue::TRUE_TV());
    implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
    implicationLinkOutgoings.push_back(hAndLink); // the results to return

    std::cout <<"Debug: PatternMiner::findAllInstancesForGivenPattern for pattern:" << std::endl
            << hAndLink->toShortString().c_str() << std::endl;


    Handle hImplicationLink = _atomSpace->add_link(IMPLICATION_LINK, implicationLinkOutgoings);
   // hImplicationLink->merge(TruthValue::TRUE_TV());

    // add variable atoms
    OrderedHandleSet allVariableNodesInPattern;
    for (unsigned int i = 0; i < HNode->pattern.size(); ++i)
    {
        extractAllVariableNodesInLink(HNode->pattern[i],allVariableNodesInPattern, _atomSpace);
    }

    HandleSeq variableNodes;
    for (Handle varh : allVariableNodesInPattern)
    {
        Handle v = _atomSpace->add_node(VARIABLE_NODE, varh->getName());
        variableNodes.push_back(v);
    }

    Handle hVariablesListLink = _atomSpace->add_link(LIST_LINK, variableNodes);
    // XXX why do we need to set the TV ???
    // hVariablesListLink->merge(TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = _atomSpace->add_link(BIND_LINK, bindLinkOutgoings);
    // XXX why do we need to set the TV ???
    // hBindLink->merge(TruthValue::TRUE_TV());


    string s = hBindLink->toShortString();

    // Run pattern matcher
    Handle hResultListLink = opencog::bindlink(_atomSpace, hBindLink);


    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

     std::cout << toString(resultSet.size())  << " instances found!" << std::endl ;

     HNode->count = resultSet.size();

    //    //debug
//    std::cout << hResultListLink->toShortString() << std::endl  << std::endl;

    _atomSpace->remove_atom(hResultListLink);


    for (Handle listH  : resultSet)
    {

        if  (Pattern_mining_mode == "Breadth_First")
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
        }

        _atomSpace->remove_atom(listH);
    }


    _atomSpace->remove_atom(hBindLink);
    _atomSpace->remove_atom(hImplicationLink);
    _atomSpace->remove_atom(hAndLink);
    //_atomSpace->remove_atom(hOutPutListLink);

    _atomSpace->remove_atom(hVariablesListLink);


}


void PatternMiner::removeLinkAndItsAllSubLinks(AtomSpace* _atomspace, Handle link)
{

//    //debug
//    std::cout << "Remove atom: " << link->toShortString() << std::endl;

    HandleSeq Outgoings = link->getOutgoingSet();
    for (Handle h : Outgoings)
    {
        if (h->isLink())
        {
            removeLinkAndItsAllSubLinks(_atomspace,h);

        }
    }
    _atomspace->remove_atom(link);

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
    for (Handle h : handles)
    {
        if (handle == h)
            return true;
    }

    return false;
}

bool PatternMiner::isInHandleSeqSeq(Handle handle, HandleSeqSeq &handleSeqs)
{
    for (HandleSeq handles : handleSeqs)
    {
        if (isInHandleSeq(handle,handles))
            return true;
    }

    return false;
}

bool PatternMiner::isIgnoredType(Type type)
{
    for (Type t : ignoredLinkTypes)
    {
        if (t == type)
            return true;
    }

    return false;
}

bool PatternMiner::isIgnoredContent(string keyword)
{
    for (string ignoreWord : keyword_black_list)
    {
        if (keyword == ignoreWord)
            return true;
    }

    return false;
}



bool PatternMiner::containIgnoredContent(Handle link )
{
    string str = link->toShortString();

    for (string ignoreWord : keyword_black_list)
    {
        string ignoreStr = "\"" + ignoreWord + "\"";
        if (str.find(ignoreStr) != std::string::npos)
            return true;
    }

    return false;
}


bool PatternMiner::add_Ignore_Link_Type(Type _type)
{
    if (isIgnoredType(_type))
        return false; // already in the ignore link list

    ignoredLinkTypes.push_back(_type);
    return true;
}

bool PatternMiner::remove_Ignore_Link_Type(Type _type)
{
    vector<Type>::iterator it;
    for (it = ignoredLinkTypes.begin(); it != ignoredLinkTypes.end(); it ++)
    {
        if ((Type)(*it) == _type)
        {
           ignoredLinkTypes.erase(it);
           return true;
        }
    }

    return false;
}

bool PatternMiner::add_keyword_to_black_list(string _keyword)
{
    if (isIgnoredContent(_keyword))
        return false; // already in the ignore keyword list

    keyword_black_list.push_back(_keyword);
    return true;
}

bool PatternMiner::remove_keyword_from_black_list(string _keyword)
{
    vector<string>::iterator it;
    for (it = keyword_black_list.begin(); it != keyword_black_list.end(); it ++)
    {
        if ((string)(*it) == _keyword)
        {
           keyword_black_list.erase(it);
           return true;
        }
    }

    return false;
}

bool PatternMiner::add_keyword_to_white_list(string _keyword)
{
    if (isInStringVector(_keyword, keyword_white_list))
        return false; // already exist

    keyword_white_list.push_back(_keyword);
    return true;
}

bool PatternMiner::remove_keyword_from_white_list(string _keyword)
{
    vector<string>::iterator it;
    for (it = keyword_white_list.begin(); it != keyword_white_list.end(); it ++)
    {
        if ((string)(*it) == _keyword)
        {
           keyword_white_list.erase(it);
           return true;
        }
    }

    return false;
}

Handle PatternMiner::getFirstNonIgnoredIncomingLink(AtomSpace *atomspace, Handle& handle)
{
    Handle cur_h = handle;
    while(true)
    {
        HandleSeq incomings;
        cur_h->getIncomingSet(back_inserter(incomings));
        if (incomings.size() == 0)
            return Handle::UNDEFINED;

        if (isIgnoredType ((incomings[0])->getType()))
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
    if ( (node1->nI_Surprisingness +  node1->nII_Surprisingness) - (node2->nI_Surprisingness + node2->nII_Surprisingness) > FLOAT_MIN_DIFF)
        return true;
    else if ((node2->nI_Surprisingness + node2->nII_Surprisingness) - (node1->nI_Surprisingness +  node1->nII_Surprisingness) > FLOAT_MIN_DIFF)
        return false;

    return (node1->var_num < node2->var_num);
}

bool compareHTreeNodeBySurprisingness_I(HTreeNode* node1, HTreeNode* node2)
{
    if ( node1->nI_Surprisingness - node2->nI_Surprisingness  > FLOAT_MIN_DIFF)
        return true;
    else if (node2->nI_Surprisingness - node1->nI_Surprisingness > FLOAT_MIN_DIFF)
        return false;

    return (node1->var_num < node2->var_num);
}

bool compareHTreeNodeBySurprisingness_II(HTreeNode* node1, HTreeNode* node2)
{
    if ( node1->nII_Surprisingness - node2->nII_Surprisingness > FLOAT_MIN_DIFF)
        return true;
    else if ( node2->nII_Surprisingness - node1->nII_Surprisingness > FLOAT_MIN_DIFF)
        return false;

    return (node1->var_num < node2->var_num);
}


// only used by Surprisingness evaluation mode
void PatternMiner::OutPutFinalPatternsToFile(unsigned int n_gram)
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName  = "FinalTopPatterns_" + toString(n_gram) + "gram.scm";
    std::cout << "\nDebug: PatternMiner: writing (gram = "
              << n_gram << ") final top patterns to file "
              << fileName << std::endl;

    resultFile.open(fileName.c_str());
    vector<HTreeNode*> &patternsForThisGram = finalPatternsForGram[n_gram-1];


    resultFile << "Interesting Pattern Mining final results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;


    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << ", SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << ", SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);


        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;


    }

    resultFile.close();


}



void PatternMiner::OutPutFrequentPatternsToFile(unsigned int n_gram, vector < vector<HTreeNode*> >& _patternsForGram, string _fileNamebasic)
{

    // out put the n_gram frequent patterns to a file, in the order of frequency
    ofstream resultFile;

    string fileName = _fileNamebasic + "FrequentPatterns_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing  (gram = " + toString(n_gram) + ") frequent patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());
    vector<HTreeNode*> &patternsForThisGram = _patternsForGram[n_gram-1];

    resultFile << "Frequent Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

//        for (Handle link : htreeNode->pattern)
//        {
//            resultFile << link->toShortString();
//        }
    }

    resultFile.close();


}

void PatternMiner::OutPutInterestingPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, int surprisingness, string _fileNamebasic) // surprisingness 1 or 2, it is default 0 which means Interaction_Information
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName = _fileNamebasic;

    if (interestingness_Evaluation_method == "Interaction_Information")
        fileName += "Interaction_Information_" + toString(n_gram) + "gram.scm";
    else if (surprisingness == 1)
        fileName += "SurprisingnessI_" + toString(n_gram) + "gram.scm";
    else
        fileName += "SurprisingnessII_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") interesting patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());


    resultFile << "Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        if (interestingness_Evaluation_method == "Interaction_Information")
            resultFile << " InteractionInformation = " << toString(htreeNode->interactionInformation);
        else if (interestingness_Evaluation_method == "surprisingness")
        {
            if (surprisingness == 1)
                resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);
            else
                resultFile << " SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);
        }


        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

//        for (Handle link : htreeNode->pattern)
//        {
//            resultFile << link->toShortString();
//        }
    }

    resultFile.close();


}


// call this function only after sort by frequency
void PatternMiner::OutPutStaticsToCsvFile(unsigned int n_gram)
{
    // out put to csv file
    ofstream csvFile;
    string csvfileName = "PatternStatics_" + toString(n_gram) + "gram.csv";

    vector<HTreeNode*> &patternsForThisGram = patternsForGram[n_gram-1];

    std::cout<<"\nDebug: PatternMiner: writing  (gram = " + toString(n_gram) + ") pattern statics to csv file " + csvfileName << std::endl;

    csvFile.open(csvfileName.c_str());


    csvFile << "Frequency,Surprisingness_I,Surprisingness_II" << std::endl;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

        csvFile << htreeNode->count << "," << htreeNode->nI_Surprisingness << ",";

        if (htreeNode->superPatternRelations.size() > 0)
            csvFile << htreeNode->nII_Surprisingness;
        else
            csvFile << "unknown";

        csvFile << std::endl;
    }

    csvFile.close();
}

void PatternMiner::OutPutLowFrequencyHighSurprisingnessPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram)
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;


    fileName = "LowFrequencyHighSurprisingness_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") LowFrequencyHighSurprisingness patterns to file " + fileName << std::endl;


    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if ( (htreeNode->count < 4) && (htreeNode->count > 1))
            resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeBySurprisingness_I);

    resultFile.open(fileName.c_str());

    resultFile << "Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << "This file contains the pattern with Frequency < 4, sort by Surprisingness_I"  << std::endl;


    for (HTreeNode* htreeNode : resultPatterns)
    {
        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << " SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;


    }

    resultFile.close();


}

void PatternMiner::OutPutHighFrequencyHighSurprisingnessPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, unsigned int min_frequency)
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;


    fileName = "HighFrequencyHighSurprisingness_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") HighFrequencyHighSurprisingness patterns to file " + fileName << std::endl;

    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count > min_frequency)
            resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeBySurprisingness_I);


    resultFile.open(fileName.c_str());


    resultFile << "Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << "This file contains the pattern with Frequency > " << min_frequency << ", sort by Surprisingness_I"  << std::endl;


    for (HTreeNode* htreeNode : resultPatterns)
    {
        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << " SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;


    }

    resultFile.close();


}

void PatternMiner::OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(vector<HTreeNode*> &patternsForThisGram,unsigned int n_gram, float min_surprisingness_I, float max_surprisingness_II)
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;


    fileName = "HighSurprisingILowSurprisingnessII" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") HighSurprisingILowSurprisingnessII patterns to file " + fileName << std::endl;

    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->superPatternRelations.size() == 0)
            continue;

        if (htreeNode->count < 10)
            continue;

        if ( (htreeNode->nI_Surprisingness > min_surprisingness_I) && (htreeNode->nII_Surprisingness < max_surprisingness_II) )
            resultPatterns.push_back(htreeNode);
    }


    resultFile.open(fileName.c_str());


    resultFile << "Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << "This file contains the pattern with Surprising_I > " << min_surprisingness_I << ", and Surprisingness_II < "  << max_surprisingness_II << std::endl;


    // std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeBySurprisingness_I);

    for (HTreeNode* htreeNode : resultPatterns)
    {
        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << " SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;


    }

    resultFile.close();


}



// To exclude this kind of patterns:
// $var_3 doesn't really can be a variable, all the links contains it doesn't contains any const nodes, so actually $var_3 is a leaf
// we call variable nodes like $var_3 as "loop variable"
//(InheritanceLink )
//  (ConceptNode Broccoli)
//  (VariableNode $var_1)

//(InheritanceLink )
//  (ConceptNode dragonfruit)
//  (VariableNode $var_2)

//(InheritanceLink )
//  (VariableNode $var_2)
//  (VariableNode $var_3)

//(InheritanceLink )
//  (VariableNode $var_1)
//  (VariableNode $var_3)

struct VariableInLinks
{
    HandleSeq onlyContainsVarLinks;
    HandleSeq containsConstLinks;
};

bool PatternMiner::containsLoopVariable(HandleSeq& inputPattern)
{
    // Need no check when gram < 3, it will already be filtered by the leaves filter
    if (inputPattern.size() < 3)
        return false;


    // First find those links that only contains variables, without any const
    map<string, VariableInLinks> varInLinksMap;

    bool allLinksContainsConst = true;

    for (Handle inputH : inputPattern)
    {
        string str = inputH->toShortString();
        std::stringstream stream(str);
        string oneLine;
        bool containsOnlyVars = true;
        vector<string> allVarsInThis_link;

        while(std::getline(stream, oneLine,'\n'))
        {
            if (oneLine.find("Node") == std::string::npos) // this line is a link, contains no nodes
                continue;

            std::size_t pos_var = oneLine.find("VariableNode");
            if (pos_var == std::string::npos)
            {
                // this node is a const node
                containsOnlyVars = false;
            }
            else
            {
                string keyVarStr = oneLine.substr(pos_var);
                // this node is a VariableNode
                allVarsInThis_link.push_back(keyVarStr);
            }
        }

        if (containsOnlyVars)
        {
            allLinksContainsConst = false;
        }

        for(string varStr : allVarsInThis_link)
        {
            map<string, VariableInLinks>::iterator it = varInLinksMap.find(varStr);
            if ( it == varInLinksMap.end() )
            {
                VariableInLinks varLinks;
                if (containsOnlyVars)
                    varLinks.onlyContainsVarLinks.push_back(inputH);
                else
                    varLinks.containsConstLinks.push_back(inputH);

                varInLinksMap.insert(std::pair<string, VariableInLinks>(varStr, varLinks));
            }
            else
            {
                VariableInLinks& varLinks = it->second;
                if (containsOnlyVars)
                    varLinks.onlyContainsVarLinks.push_back(inputH);
                else
                    varLinks.containsConstLinks.push_back(inputH);
            }

        }

    }

    if (allLinksContainsConst)
        return false;

    map<string, VariableInLinks>::iterator it;
    for (it = varInLinksMap.begin(); it != varInLinksMap.end(); ++ it)
    {
         VariableInLinks& varLinks = it->second;
         if (varLinks.containsConstLinks.size() == 0)
             return true;

    }

    return false;

}


// Each HandleSeq in HandleSeqSeq oneOfEachSeqShouldBeVars is a list of nodes that connect two links in inputLinks,
// at least one node in a HandleSeq should be variable, which means two connected links in inputLinks should be connected by at least a variable
// leaves are those nodes that are not connected to any other links in inputLinks, they should be
// some will be filter out in this phrase, return true to filter out
bool PatternMiner::filters(HandleSeq& inputLinks, HandleSeqSeq& oneOfEachSeqShouldBeVars, HandleSeq& leaves, HandleSeq& shouldNotBeVars, HandleSeq& shouldBeVars, AtomSpace* _atomSpace)
{
    if(inputLinks.size() < 2)
        return false;

    OrderedHandleSet allNodesInEachLink[inputLinks.size()];

    OrderedHandleSet all2ndOutgoingsOfInherlinks;

    // map<predicate, set<value> >
    map<Handle, OrderedHandleSet > predicateToValueOfEvalLinks;

    OrderedHandleSet  all1stOutgoingsOfEvalLinks;


    for (unsigned int i = 0; i < inputLinks.size(); ++i)
    {
        extractAllNodesInLink(inputLinks[i],allNodesInEachLink[i], _atomSpace);

        if (enable_filter_not_inheritant_from_same_var)
        {
            // filter: Any two InheritanceLinks should not share their secondary outgoing nodes
            if ((inputLinks[i])->getType() == INHERITANCE_LINK)
            {
                Handle secondOutgoing = inputLinks[i]->getOutgoingSet()[1];
                if (all2ndOutgoingsOfInherlinks.find(secondOutgoing) == all2ndOutgoingsOfInherlinks.end())
                    all2ndOutgoingsOfInherlinks.insert(secondOutgoing);
                else
                    return true;
            }
        }

        if (enable_filter_not_same_var_from_same_predicate || enable_filter_not_all_first_outgoing_const|| enable_filter_first_outgoing_evallink_should_be_var)
        {
            // this filter: Any two EvaluationLinks with the same predicate should not share the same secondary outgoing nodes
            if ((inputLinks[i])->getType() == EVALUATION_LINK)
            {
                HandleSeq outgoings = inputLinks[i]->getOutgoingSet();
                Handle predicateNode = outgoings[0];

                // value node is the last node of the list link
                HandleSeq outgoings2 = outgoings[1]->getOutgoingSet();
                Handle valueNode = outgoings2[outgoings2.size() - 1];

                if (enable_filter_not_same_var_from_same_predicate)
                {
                    map<Handle, OrderedHandleSet >::iterator it = predicateToValueOfEvalLinks.find(predicateNode);
                    if (it != predicateToValueOfEvalLinks.end())
                    {
                        OrderedHandleSet& values = (it->second);
                        if (values.find(valueNode) != values.end())
                            return true;
                        else
                            values.insert(valueNode);
                    }
                    else
                    {
                        OrderedHandleSet newValues;
                        newValues.insert(valueNode);
                        predicateToValueOfEvalLinks.insert(std::pair<Handle, OrderedHandleSet >(predicateNode,newValues));
                    }
                }

                // this filter: at least one of all the 1st outgoings of Evaluationlinks should be var
                if ( enable_filter_first_outgoing_evallink_should_be_var || enable_filter_not_all_first_outgoing_const)
                {
                    if (outgoings2.size() > 1)
                    {
                        if(all1stOutgoingsOfEvalLinks.find(outgoings2[0]) == all1stOutgoingsOfEvalLinks.end())
                            all1stOutgoingsOfEvalLinks.insert(outgoings2[0]);
                    }
                }

            }
        }
    }

    if (all1stOutgoingsOfEvalLinks.size() > 0)
    {
        // this filter: all the first outgoing nodes of all evaluation links should be variables
        if (enable_filter_first_outgoing_evallink_should_be_var)
            std::copy(all1stOutgoingsOfEvalLinks.begin(), all1stOutgoingsOfEvalLinks.end(), std::back_inserter(shouldBeVars));
        else if ((enable_filter_not_all_first_outgoing_const) )
        {
            // if enable_filter_first_outgoing_evallink_should_be_var is true, there is no need to enable this filter below
            HandleSeq all1stOutgoingsOfEvalLinksSeq(all1stOutgoingsOfEvalLinks.begin(), all1stOutgoingsOfEvalLinks.end());
            oneOfEachSeqShouldBeVars.push_back(all1stOutgoingsOfEvalLinksSeq);
        }
    }


    if (enable_filter_links_should_connect_by_vars)
    {
        // find the common nodes which are shared among inputLinks
        for (unsigned i = 0; i < inputLinks.size() - 1; i ++)
        {
            for (unsigned j = i+1; j < inputLinks.size(); j ++)
            {
                HandleSeq commonNodes;
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
            for (Handle node : allNodesInEachLink[i])
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
                    Type t = node->getType();
                    for (Type noType : node_types_should_not_be_vars)
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

    return false;

}

// return true if the inputLinks are disconnected
// when the inputLinks are connected, the outputConnectedGroups has only one group, which is the same as inputLinks
bool PatternMiner::splitDisconnectedLinksIntoConnectedGroups(HandleSeq& inputLinks, HandleSeqSeq& outputConnectedGroups)
{
    if(inputLinks.size() < 2)
        return false;

    OrderedHandleSet allNodesInEachLink[inputLinks.size()];
    for (unsigned int i = 0; i < inputLinks.size(); ++i)
    {
        extractAllVariableNodesInLink(inputLinks[i],allNodesInEachLink[i], atomSpace);
    }

    int i = -1;
    for (Handle link : inputLinks)
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
            for (Handle node : allNodesInEachLink[i])
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
    int count = getCountOfAConnectedPattern(connectedSubPatternKey, connectedSubPattern);
    if (count == 0)
    {
        cout << "\nwarning: cannot find subpattern: \n" << connectedSubPatternKey << std::endl;
        count = 1;
    }


    return log2(count);

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
//    for (Handle link : HNode->pattern)
//    {
//        std::cout << link->toShortString();
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

             unsigned int unifiedLastLinkIndex;
             HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex);
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
                 for (HandleSeq aConnectedSubPart : splittedSubPattern)
                 {
                     // Unify it again
                     unsigned int _unifiedLastLinkIndex;
                     HandleSeq unifiedConnectedSubPattern = UnifyPatternOrder(aConnectedSubPart, _unifiedLastLinkIndex);
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

         delete [] indexes;

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
//    for (Handle link : HNode->pattern)
//    {
//        std::cout << link->toShortString();
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

//        for (HTreeNode* curNode : lastlevelNodes)
//        {
//            for (HTreeNode* pNode : curNode->parentLinks)
//            {
//                if (curlevelNodes.find(pNode) == curlevelNodes.end())
//                {
//                    std::cout << "H(subpattern): \n";
//                    for (Handle link : pNode->pattern)
//                    {
//                        std::cout << link->toShortString();
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
//                         for (HandleSeq aConnectedSubPart : splittedSubPattern)
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

        uniqueKeyLock.unlock();
        return 0; // just skip it
//        // can't find its HtreeNode, have to calculate its frequency again by calling pattern matcher
//        // Todo: need to decide if add this missing HtreeNode into H-Tree or not

//        // cout << "Exception: can't find a subpattern: \n" << connectedPatternKey << std::endl;
//        if (is_distributed)
//        {
//            uniqueKeyLock.unlock();

//            return 0;
//        }
//        else
//        {

//            HTreeNode* newHTreeNode = new HTreeNode();
//            keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(connectedPatternKey, newHTreeNode));
//            uniqueKeyLock.unlock();

//            newHTreeNode->pattern = connectedPattern;

//            // Find All Instances in the original AtomSpace For this Pattern
//            findAllInstancesForGivenPatternInNestedAtomSpace(newHTreeNode);
//    //        cout << "Not found in H-tree! call pattern matcher again! count = " << newHTreeNode->count << std::endl;
//            return newHTreeNode->count;
//        }

    }


}

bool PatternMiner::isALinkOneInstanceOfGivenPattern(Handle &instanceLink, Handle& patternLink, AtomSpace* instanceLinkAtomSpace)
{
    if (instanceLink->getType() != patternLink->getType())
        return false;

    HandleSeq outComingsOfPattern = patternLink->getOutgoingSet();
    HandleSeq outComingsOfInstance = instanceLink->getOutgoingSet();

    if (outComingsOfPattern.size() != outComingsOfInstance.size())
        return false;

    for(unsigned int i = 0; i < outComingsOfPattern.size(); i ++)
    {
        bool pis_link = (outComingsOfPattern[i])->isLink();
        bool iis_link = (outComingsOfInstance[i])->isLink();

        if (pis_link && iis_link)
        {
            if (isALinkOneInstanceOfGivenPattern(outComingsOfInstance[i], outComingsOfPattern[i], instanceLinkAtomSpace))
                continue;
            else
                return false;
        }
        else if ( (!pis_link)&&(!iis_link) )
        {
            // they are both nodes

            // If this node is a variable, skip it
            if ((outComingsOfPattern[i])->getType() == VARIABLE_NODE)
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
    HandleSeq outgoingLinks = inputLink->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {

        if (h->isNode())
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
             Handle reLink = _toAtomSpace->add_link(h->getType(), _renameOutgoingLinks);
             reLink->merge(h->getTruthValue());
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
    HandleSeq incomings;
    leaf->getIncomingSet(back_inserter(incomings));

    for (Handle incomingHandle : incomings)
    {
        Handle extendedHandle;
        // if this atom is a igonred type, get its first parent that is not in the igonred types
        if (isIgnoredType (incomingHandle->getType()) )
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
        Handle reLink = atomSpace->add_link(extendedHandle->getType(), renameOutgoingLinks);
        reLink->merge(extendedHandle->getTruthValue());
        outPutExtendedPatternLinks.push_back(reLink);
    }

}

// make sure only input 2~4 gram patterns, calculate nSurprisingness_I and nSurprisingness_II
void PatternMiner::calculateSurprisingness( HTreeNode* HNode, AtomSpace *_fromAtomSpace)
{

//    // debug
//    if (HNode->nI_Surprisingness != 0 || HNode->nII_Surprisingness != 0)
//        std::cout << "Exception: This pattern has been calculateSurprisingness before!\n";

    if (HNode->count == 0)
        HNode->count = 1;

    if (HNode->count < 2)
    {

        HNode->nII_Surprisingness = 0.0f;
        HNode->nI_Surprisingness = 0.0f;
        return;
    }

//    std::cout << "=================Debug: calculate I_Surprisingness for pattern: ====================\n";
//    for (Handle link : HNode->pattern)
//    {
//        std::cout << link->toShortString();
//    }
//     std::cout << "count of this pattern = " << HNode->count << std::endl;
//     std::cout << std::endl;

    unsigned int gram = HNode->pattern.size();
    // get the predefined combination:
    // vector<vector<vector<unsigned int>>>
    // int comcount = 0;

    float p = ((float)HNode->count)/atomspaceSizeFloat;
    float min_diff = 999999999.9f;
    // cout << "For this pattern itself: p = " <<  HNode->count << " / " <<  (int)atomspaceSizeFloat << " = " << p << std::endl;

    for (vector<vector<unsigned int>>&  oneCombin : components_ngram[gram-2])
    {
        int com_i = 0;
        // std::cout <<" -----Combination " << comcount++ << "-----" << std::endl;
        float total_p = 1.0f;

        bool containsComponentDisconnected = false;
        bool subComponentNotFound = false;

        for (vector<unsigned int>& oneComponent : oneCombin)
        {
            HandleSeq subPattern;
            for (unsigned int index : oneComponent)
            {
                subPattern.push_back(HNode->pattern[index]);
            }

            unsigned int unifiedLastLinkIndex;
            HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex);
            string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

            // std::cout<< "Subpattern: " << subPatternKey;

            // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
            HandleSeqSeq splittedSubPattern;
            if (splitDisconnectedLinksIntoConnectedGroups(unifiedSubPattern, splittedSubPattern))
            {
                // std::cout<< " is disconnected! skip it \n" ;
                containsComponentDisconnected = true;
                break;
            }
            else
            {
                // std::cout<< " is connected!" ;
                unsigned int component_count = getCountOfAConnectedPattern(subPatternKey, unifiedSubPattern);

                if (component_count == 0)
                {
                    // one of the subcomponents is missing, skip this combination
                    subComponentNotFound = true;
                    break;
                }

                // cout << ", count = " << component_count;
                float p_i = ((float)(component_count)) / atomspaceSizeFloat;

                // cout << ", p = " << component_count  << " / " << (int)atomspaceSizeFloat << " = " << p_i << std::endl;
                total_p *= p_i;
                // std::cout << std::endl;
            }

            com_i ++;

        }

        if (containsComponentDisconnected || subComponentNotFound)
            continue;


        // cout << "\n ---- total_p = " << total_p << " ----\n" ;

        float diff = total_p - p;
        if (diff < 0)
            diff = - diff;

        // cout << "diff  = total_p - p " << diff << " \n" ;

        diff = diff / total_p;

        if (diff < min_diff)
            min_diff = diff;


        // cout << "diff / total_p = " << diff << " \n" ;

    }


    HNode->nI_Surprisingness = min_diff;

    // debug:
//    cout << "nI_Surprisingness = " << HNode->nI_Surprisingness  << std::endl;

    if (gram == MAX_GRAM ) // can't calculate II_Surprisingness for MAX_GRAM patterns, becasue it required gram +1 patterns
        return;

//    std::cout << "=================Debug: calculate II_Surprisingness for pattern: ====================\n";

    // II_Surprisingness is to evaluate how easily the frequency of this pattern can be infered from any of  its superpatterns
    // for all its super patterns
    if (HNode->superPatternRelations.size() == 0)
    {
        HNode->nII_Surprisingness  = 0.000000000f;
        // debug:
//        cout << "This node has no super patterns, give it Surprisingness_II value: 0.0 \n";
    }
    else
    {

        float minSurprisingness_II = 999999999.9f;
        vector<ExtendRelation>::iterator oneSuperRelationIt;
        for(oneSuperRelationIt = HNode->superPatternRelations.begin();  oneSuperRelationIt != HNode->superPatternRelations.end(); ++ oneSuperRelationIt)
        {
            ExtendRelation& curSuperRelation = *oneSuperRelationIt;

            // There are two types of super patterns: one is extended from a variable, one is extended from a const (turnt into a variable)
            // Only type two is the super pattern we are considering:

            // type one : extended from a variable,  the extended node itself is considered as a variable in the pattern A
            //            {
            //                // Ap is A's one supper pattern, E is the link pattern that extended
            //                // e.g.: M is the size of corpus
            //                // A:  ( var_1 is from CAR ) && ( var_1 is horror ) , P(A) = 100/M
            //                // A1: ( var_1 is from CAR ), A2: ( var_1 is horror )
            //                // Ap: ( var_1 is from CAR ) && ( var_1 is horror ) && ( var_1 is male ) , P(Ap) = 99/M
            //                // E:  ( var_1 is male )
            //                // Different from the super pattern type two bellow, E adds one more condition to var_1, so P(Ap) should be < or = P(A).
            //                // Surprisingness_II (A from Ap) =  min{|P(A) - P(Ap)*(P(Ai)/P(Ai&E))|} / P(A)
            //                // Actually in this example, both A and Ap are quite interesting


            // type two : extended from a const node , the const node is changed into a variable

            // Ap is A's one supper pattern, E is the link pattern that extended
            // e.g.: M is the size of corpus
            // A:  ( Lily eat var_1 ) && ( var_1 is vegetable ) , P(A) = 4/M
            // Ap: ( var_2 eat var_1 ) && ( var_1 is vegetable ) && ( var_2 is woman ) , P(Ap) = 20/M
            // E:  ( var_2 is woman ) , P(E) = 5/M
            // Surprisingness_II (A from Ap) =  |P(A) - P(Ap)*P(Lily)/P(E)| / P(A)
            // when the TruthValue of each atom is not taken into account, because every atom is unique, any P(Const atom) = 1/M , so that P(Lily) = 1/M
            // So: Surprisingness_II (A from Ap) =  |P(A) - P(Ap)/Count(E)| / P(A)

            // Note that becasue of unifying patern, the varible order in A, Ap, E can be different
            // HandleSeq& patternAp = curSuperRelation.extendedHTreeNode->pattern;

            float p_Ap = ((float )(curSuperRelation.extendedHTreeNode->count))/atomspaceSizeFloat;

            HandleSeq patternE;
            patternE.push_back(curSuperRelation.newExtendedLink);
            // unify patternE
            unsigned int unifiedLastLinkIndex;
            HandleSeq unifiedPatternE = UnifyPatternOrder(patternE, unifiedLastLinkIndex);
            string patternEKey = unifiedPatternToKeyString(unifiedPatternE, atomSpace);

            unsigned int patternE_count = getCountOfAConnectedPattern(patternEKey, unifiedPatternE);

//            if (patternE_count == 0)
//                cout << "Warning: In calculate II_Surprisingness, Extended Link is missing: \n" << patternEKey << std::endl;

            float p_ApDivByCountE = p_Ap / ( (float)(patternE_count) );

            float Surprisingness_II;
            if (p_ApDivByCountE > p)
                Surprisingness_II = p_ApDivByCountE - p;
            else
                Surprisingness_II = p - p_ApDivByCountE;

            if (Surprisingness_II < minSurprisingness_II)
                minSurprisingness_II = Surprisingness_II;

            // debug
//                cout << "For Super pattern: -------extended from a const----------------- " << std::endl;
//                cout << unifiedPatternToKeyString(curSuperRelation.extendedHTreeNode->pattern, atomSpace);
//                cout << "P(Ap) = " << p_Ap << std::endl;
//                cout << "The extended link pattern:  " << std::endl;
//                cout << patternEKey;
//                cout << "Count(E) = " << patternE_count << std::endl;
//                cout << "Surprisingness_II = |P(A) -P(Ap)/Count(E)| = " << Surprisingness_II << std::endl;




        }

//        // debug
//        cout << "Min Surprisingness_II  = " << minSurprisingness_II;

        if (HNode->superPatternRelations.size() > 0)
            HNode->nII_Surprisingness = minSurprisingness_II/p;
        else
        {
            HNode->nII_Surprisingness = -1.0f;
            // num_of_patterns_without_superpattern_cur_gram ++; // todo: need a lock
        }
    }



//    // debug:
//    cout << " nII_Surprisingness = Min Surprisingness_II / p = " << HNode->nII_Surprisingness  << std::endl;

//    std::cout << "=================Debug: end calculate II_Surprisingness ====================\n";

}

// in vector<vector<vector<unsigned int>>> the  <unsigned int> is the index in pattern HandleSeq : 0~n
void PatternMiner::generateComponentCombinations(string componentsStr, vector<vector<vector<unsigned int>>> &componentCombinations)
{
    // "0,12|1,02|2,01|0,1,2"

    vector<string> allCombinationsStrs;
    boost::split(allCombinationsStrs, componentsStr, boost::is_any_of("|"));

    for (string oneCombinStr : allCombinationsStrs)
    {
        // "0,12"
        vector<vector<unsigned int>> oneCombin;
        vector<string> allComponentStrs;
        boost::split(allComponentStrs, oneCombinStr, boost::is_any_of(","));

        for (string oneComponentStr : allComponentStrs)
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

PatternMiner::PatternMiner(AtomSpace* _originalAtomSpace): originalAtomSpace(_originalAtomSpace)
{

    reSetAllSettingsFromConfig();

    initPatternMiner();


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
    for (string componentCombinsStr : gramNcomponents)
    {
        generateComponentCombinations(componentCombinsStr, this->components_ngram[ngram]);
        ngram ++;
    }

    // std::cout<<"Debug: PatternMiner init finished! " + toString(THREAD_NUM) + " threads used!" << std::endl;

}

PatternMiner::~PatternMiner()
{
    cleanUpPatternMiner();
}

// make sure it is called after reSetAllSettingsFromConfig
void PatternMiner::initPatternMiner()
{
    htree = new HTree();
    atomSpace = new AtomSpace(originalAtomSpace);

    //    unsigned int system_thread_num  = std::thread::hardware_concurrency();

    //    if (system_thread_num > 1)
    //        THREAD_NUM = system_thread_num - 1;
    //    else
    //        THREAD_NUM = 1;

    //     // use all the threads in this machine
    //     THREAD_NUM = system_thread_num;

    THREAD_NUM = 1;

    threads = new thread[THREAD_NUM];

    is_distributed = false;

    cur_gram = 0;

    ignoredLinkTypes.clear();
    ignoredLinkTypes.push_back(LIST_LINK);

    // vector < vector<HTreeNode*> > patternsForGram
    for (unsigned int i = 0; i < MAX_GRAM; ++i)
    {
        vector<HTreeNode*> patternVector;
        patternsForGram.push_back(patternVector);

        vector<HTreeNode*> finalPatternVector;
        finalPatternsForGram.push_back(finalPatternVector);

    }

}

void PatternMiner::reSetAllSettingsFromConfig()
{
    int max_gram = config().get_int("Pattern_Max_Gram");
    MAX_GRAM = (unsigned int)max_gram;

    enable_Frequent_Pattern = config().get_bool("Enable_Frequent_Pattern");
    enable_Interesting_Pattern = config().get_bool("Enable_Interesting_Pattern");
    interestingness_Evaluation_method = config().get("Interestingness_Evaluation_method");

    assert(enable_Frequent_Pattern || enable_Interesting_Pattern);
    //The options are "Interaction_Information", "surprisingness"
    assert( (interestingness_Evaluation_method == "Interaction_Information") || (interestingness_Evaluation_method == "surprisingness") );


    thresholdFrequency = config().get_int("Frequency_threshold");

    use_keyword_black_list = config().get_bool("use_keyword_black_list");
    use_keyword_white_list = config().get_bool("use_keyword_white_list");

    assert( ! (use_keyword_black_list & use_keyword_white_list) );

    string keyword_black_list_str  = config().get("keyword_black_list");
    keyword_black_list_str .erase(std::remove(keyword_black_list_str .begin(), keyword_black_list_str .end(), ' '), keyword_black_list_str .end());
    boost::split(keyword_black_list, keyword_black_list_str , boost::is_any_of(","));

    string keyword_white_list_str  = config().get("keyword_white_list");
    keyword_white_list_str .erase(std::remove(keyword_white_list_str .begin(), keyword_white_list_str .end(), ' '), keyword_white_list_str .end());
    boost::split(keyword_white_list, keyword_white_list_str , boost::is_any_of(","));


    string keyword_white_list_logic_str = config().get("keyword_white_list_logic");

    if ( (keyword_white_list_logic_str == "AND") or (keyword_white_list_logic_str == "and") or (keyword_white_list_logic_str == "And")  )
        keyword_white_list_logic = QUERY_LOGIC::AND;
    else
        keyword_white_list_logic = QUERY_LOGIC::OR;

    enable_filter_leaves_should_not_be_vars = config().get_bool("enable_filter_leaves_should_not_be_vars");
    enable_filter_links_should_connect_by_vars = config().get_bool("enable_filter_links_should_connect_by_vars");
    enable_filter_node_types_should_not_be_vars =  config().get_bool("enable_filter_node_types_should_not_be_vars");
    enable_filter_not_inheritant_from_same_var = config().get_bool("enable_filter_not_inheritant_from_same_var");
    enable_filter_not_all_first_outgoing_const = config().get_bool("enable_filter_not_all_first_outgoing_const");
    enable_filter_not_same_var_from_same_predicate = config().get_bool("enable_filter_not_same_var_from_same_predicate");
    enable_filter_first_outgoing_evallink_should_be_var = config().get_bool("enable_filter_first_outgoing_evallink_should_be_var");

    if (enable_filter_node_types_should_not_be_vars)
    {
        node_types_should_not_be_vars.clear();
        string node_types_str = config().get("node_types_should_not_be_vars");
        node_types_str.erase(std::remove(node_types_str.begin(), node_types_str.end(), ' '), node_types_str.end());
        vector<string> typeStrs;
        boost::split(typeStrs, node_types_str, boost::is_any_of(","));

        for (string typestr : typeStrs)
        {
            node_types_should_not_be_vars.push_back( classserver().getType(typestr) );
        }
    }
}

// release everything
void PatternMiner::cleanUpPatternMiner()
{

    if (htree)
        delete htree;

    if (atomSpace)
        delete atomSpace;

//    if (originalAtomSpace)
//        delete originalAtomSpace;

    if (observingAtomSpace)
        delete observingAtomSpace;

//    if (threads)
//        delete threads;

    ignoredLinkTypes.clear();

    for( std::pair<string, HTreeNode*> OnePattern : keyStrToHTreeNodeMap)
    {
        delete ((HTreeNode*)(OnePattern.second));
    }

    std::map <string, HTreeNode*> emptykeyStrToHTreeNodeMap;
    keyStrToHTreeNodeMap.swap(emptykeyStrToHTreeNodeMap);

    unsigned int patternsForGramSize = patternsForGram.size();
    for (unsigned int i = 0; i < patternsForGramSize; ++i)
    {
        std::vector<HTreeNode*> emptyVector;
        (patternsForGram[i]).swap(emptyVector);
    }

    vector < vector<HTreeNode*> > emptypatternsForGram;
    patternsForGram.swap(emptypatternsForGram);

    unsigned int finalPatternsForGramSize = finalPatternsForGram.size();
    for (unsigned int i = 0; i < finalPatternsForGramSize; ++i)
    {
        std::vector<HTreeNode*> emptyVector;
        (finalPatternsForGram[i]).swap(emptyVector);
    }

    vector < vector<HTreeNode*> > emptyfinalPatternsForGram;
    finalPatternsForGram.swap(emptyfinalPatternsForGram);


}

void PatternMiner::resetPatternMiner(bool resetAllSettingsFromConfig)
{
    if (resetAllSettingsFromConfig)
        reSetAllSettingsFromConfig();

    cleanUpPatternMiner();
    initPatternMiner();

    if (resetAllSettingsFromConfig)
        cout <<  "\nPatternMiner reset with all settings resetting from config file!" << std::endl;
    else
        cout <<  "\nPatternMiner reset, keeping all the current settings!" << std::endl;
}

void PatternMiner::runPatternMiner(bool exit_program_after_finish)
{

    if (keyStrToHTreeNodeMap.size() > 0)
    {
        cleanUpPatternMiner();
        initPatternMiner();
    }


    Pattern_mining_mode = config().get("Pattern_mining_mode"); // option: Breadth_First , Depth_First
    assert( (Pattern_mining_mode == "Breadth_First") || (Pattern_mining_mode == "Depth_First"));

    std::cout <<"\nDebug: PatternMining start! Max gram = "
              << this->MAX_GRAM << ", mode = " << Pattern_mining_mode << std::endl;

    int start_time = time(NULL);

    originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

    allLinkNumber = (int)(allLinks.size());
    atomspaceSizeFloat = (float)(allLinkNumber);

    std::cout << "Using " << THREAD_NUM << " threads. \n";
    std::cout << "Corpus size: "<< allLinkNumber << " links in total. \n\n";

    runPatternMinerDepthFirst();

    std::cout<<"PatternMiner:  mining finished!\n";


    if (enable_Frequent_Pattern)
    {
        std::cout<<"PatternMiner:  done frequent pattern mining for 1 to "<< MAX_GRAM <<"gram patterns!\n";

        for(unsigned int gram = 1; gram <= MAX_GRAM; gram ++)
        {
            // sort by frequency
            std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeByFrequency );

            // Finished mining gram patterns; output to file
            std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGram[gram-1]).size()) + " patterns found! ";

            OutPutFrequentPatternsToFile(gram, patternsForGram);

            std::cout<< std::endl;
        }
    }


    if (enable_Interesting_Pattern && (MAX_GRAM >1))
    {
        runInterestingnessEvaluation();
    }


    int end_time = time(NULL);
    printf("\nPattern Mining Finished! Total time: %d seconds. \n", end_time - start_time);


    if (exit_program_after_finish)
    {
        std::cout << "Pattern Miner application quited!" << std::endl;
        std::exit(EXIT_SUCCESS);
    }

//   testPatternMatcher2();

//   selectSubsetFromCorpus();

}

void PatternMiner::runInterestingnessEvaluation()
{
    for(cur_gram = 2; cur_gram <= MAX_GRAM; cur_gram ++)
    {

        cout << "\nCalculating interestingness for " << cur_gram << " gram patterns by evaluating " << interestingness_Evaluation_method << std::endl;
        cur_index = -1;
        threads = new thread[THREAD_NUM];
        num_of_patterns_without_superpattern_cur_gram = 0;

        for (unsigned int i = 0; i < THREAD_NUM; ++ i)
        {
            threads[i] = std::thread([this]{this->evaluateInterestingnessTask();}); // using C++11 lambda-expression
        }

        for (unsigned int i = 0; i < THREAD_NUM; ++ i)
        {
            threads[i].join();
        }

        delete [] threads;

        std::cout<<"PatternMiner:  done (gram = " + toString(cur_gram) + ") interestingness evaluation!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! ";
        std::cout<<"Outputting to file ... ";

        if (interestingness_Evaluation_method == "Interaction_Information")
        {
            // sort by interaction information
            std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 0);
        }
        else if (interestingness_Evaluation_method == "surprisingness")
        {
            // sort by surprisingness_I first
            std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeBySurprisingness_I);
            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram,1);

            if (cur_gram == MAX_GRAM)
                break;

            vector<HTreeNode*> curGramPatterns = patternsForGram[cur_gram-1];


            // and then sort by surprisingness_II
            std::sort(curGramPatterns.begin(), curGramPatterns.end(),compareHTreeNodeBySurprisingness_II);
            OutPutInterestingPatternsToFile(curGramPatterns,cur_gram,2);

            // OutPutStaticsToCsvFile(cur_gram);

            // Get the min threshold of surprisingness_II
            int threshold_index_II;
            threshold_index_II = SURPRISINGNESS_II_TOP_THRESHOLD * (float)(curGramPatterns.size() - num_of_patterns_without_superpattern_cur_gram);
            int looptimes = 0;
            while (true)
            {

                surprisingness_II_threshold = (curGramPatterns[threshold_index_II])->nII_Surprisingness;
                if (surprisingness_II_threshold <= 0.00000f)
                {
                    if (++ looptimes > 8)
                    {
                        surprisingness_II_threshold = 0.00000f;
                        break;
                    }

                    threshold_index_II = ((float)threshold_index_II) * SURPRISINGNESS_II_TOP_THRESHOLD;
                }
                else
                    break;
            }


            cout<< "surprisingness_II_threshold for " << cur_gram << " gram = "<< surprisingness_II_threshold;

            // go through the top N patterns of surprisingness_I, pick the patterns with surprisingness_II higher than threshold
            int threshold_index_I = SURPRISINGNESS_I_TOP_THRESHOLD * (float)(curGramPatterns.size());
            for (int p = 0; p <= threshold_index_I; p ++)
            {
                HTreeNode* pNode = (patternsForGram[cur_gram-1])[p];

                // for patterns have no superpatterns, nII_Surprisingness == -1.0, which should be taken into account
                if ( (pNode->nII_Surprisingness < 0 ) || (pNode->nII_Surprisingness > surprisingness_II_threshold ) )
                    finalPatternsForGram[cur_gram-1].push_back(pNode);
            }

            // OutPutLowFrequencyHighSurprisingnessPatternsToFile(patternsForGram[cur_gram-1], cur_gram);

            // OutPutHighFrequencyHighSurprisingnessPatternsToFile(patternsForGram[cur_gram-1], cur_gram,  15);

            // OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 100.0f, 0.51f);

            // sort by frequency
            std::sort((finalPatternsForGram[cur_gram-1]).begin(), (finalPatternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency );

            OutPutFinalPatternsToFile(cur_gram);

        }

        std::cout<< std::endl;
    }
}

bool PatternMiner::containWhiteKeywords(string& str, QUERY_LOGIC logic)
{
    if (logic == QUERY_LOGIC::OR)
    {
        for (string keyword : keyword_white_list)
        {
            if (str.find(keyword) != std::string::npos)
                return true;
        }

        return false;
    }
    else // QUERY_LOGIC::AND
    {
        for (string keyword : keyword_white_list)
        {
            if (str.find(keyword) == std::string::npos)
                return false;
        }

        return true;
    }
}

void PatternMiner::applyWhiteListKeywordfilterAfterMining()
{
    if (patternsForGram[0].size() < 1)
    {
        std::cout<<"\nPatternMiner:  this filter should be applied after mining! Please run pattern miner first!" << std::endl;
        return;
    }

    if (keyword_white_list.size() < 1)
    {
        std::cout<<"\nPatternMiner:  white key word list is empty! Please set it first!" << std::endl;
        return;
    }

    string logic;
    if (keyword_white_list_logic == QUERY_LOGIC::OR)
       logic = "OR";
    else
       logic = "AND";

    string keywordlist = "";
    cout<<"\nPatternMiner:  applying keyword white list (" << logic << ") filter: ";
    for (string keyword : keyword_white_list)
    {
        keywordlist += (keyword + "-");
        cout << keyword << " ";
    }
    cout << std::endl;

    string fileNameBasic = "WhiteKeyWord-" + logic + "-" + keywordlist;

    vector < vector<HTreeNode*> > patternsForGramFiltered;

    for(unsigned int gram = 1; gram <= MAX_GRAM; gram ++)
    {
        vector<HTreeNode*> patternVector;
        patternsForGramFiltered.push_back(patternVector);

        for (HTreeNode* htreeNode : patternsForGram[gram-1])
        {
            if (htreeNode->count < thresholdFrequency)
                break;

            string patternStr = unifiedPatternToKeyString(htreeNode->pattern);
            if (containWhiteKeywords(patternStr, keyword_white_list_logic))
                patternsForGramFiltered[gram-1].push_back(htreeNode);

        }

        // Finished mining gram patterns; output to file
        std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGramFiltered[gram-1]).size()) + " patterns found after filtering! ";

        OutPutFrequentPatternsToFile(gram, patternsForGramFiltered, fileNameBasic);

        std::cout<< std::endl;
    }

    if (enable_Interesting_Pattern && (MAX_GRAM >1))
    {
        for(cur_gram = 2; cur_gram <= MAX_GRAM; cur_gram ++)
        {

            if (interestingness_Evaluation_method == "Interaction_Information")
            {
                // sort by interaction information
                std::sort((patternsForGramFiltered[cur_gram-1]).begin(), (patternsForGramFiltered[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
                OutPutInterestingPatternsToFile(patternsForGramFiltered[cur_gram-1], cur_gram, 0, fileNameBasic);
            }
            else if (interestingness_Evaluation_method == "surprisingness")
            {
                // sort by surprisingness_I first
                std::sort((patternsForGramFiltered[cur_gram-1]).begin(), (patternsForGramFiltered[cur_gram-1]).end(),compareHTreeNodeBySurprisingness_I);
                OutPutInterestingPatternsToFile(patternsForGramFiltered[cur_gram-1], cur_gram, 1, fileNameBasic);

                if (cur_gram == MAX_GRAM)
                    break;

                // sort by surprisingness_II first
                std::sort((patternsForGramFiltered[cur_gram-1]).begin(), (patternsForGramFiltered[cur_gram-1]).end(),compareHTreeNodeBySurprisingness_II);
                OutPutInterestingPatternsToFile(patternsForGramFiltered[cur_gram-1], cur_gram, 2, fileNameBasic);


            }
        }
    }


    std::cout << "\napplyWhiteListKeywordfilterAfterMining finished!" << std::endl;

}

void PatternMiner::evaluateInterestingnessTask()
{

    while(true)
    {
        if (THREAD_NUM > 1)
            readNextPatternLock.lock();

        cur_index ++;

        if ((unsigned int)cur_index < (patternsForGram[cur_gram-1]).size())
        {
            cout<< "\r" << (float)(cur_index)/(float)((patternsForGram[cur_gram-1]).size())*100.0f << + "% completed." ;
            std::cout.flush();
        }
        else
        {
            if ((unsigned int)cur_index == (patternsForGram[cur_gram-1]).size())
            {
                cout<< "\r100% completed." ;
                std::cout.flush();
            }

            if (THREAD_NUM > 1)
                readNextPatternLock.unlock();

            break;

        }

        HTreeNode* htreeNode = patternsForGram[cur_gram - 1][cur_index];

        if (THREAD_NUM > 1)
            readNextPatternLock.unlock();

        // evaluate the interestingness
        // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"
        if (interestingness_Evaluation_method == "Interaction_Information")
        {
           calculateInteractionInformation(htreeNode);
        }
        else if (interestingness_Evaluation_method == "surprisingness")
        {
           calculateSurprisingness(htreeNode, observingAtomSpace);
        }

    }
}

// select a subset for topics from the corpus
void PatternMiner::selectSubsetFromCorpus(vector<string>& topics, unsigned int gram)
{
    _selectSubsetFromCorpus(topics,gram);
}

std::string PatternMiner::Link2keyString(Handle& h, std::string indent, const AtomSpace *atomspace)
{
    if (atomspace == 0)
        atomspace = this->atomSpace;

    std::stringstream answer;
    std::string more_indent = indent + LINE_INDENTATION;

    answer << indent  << "(" << classserver().getTypeName(h->getType()) << " ";

    if (h->isNode())
    {
        answer << h->getName();
    }

    answer << ")" << "\n";

    if (h->isLink())
    {
        HandleSeq outgoings = h->getOutgoingSet();
        for (Handle outgoing : outgoings)
            answer << Link2keyString(outgoing, more_indent, atomspace);
    }

    return answer.str();
}

void PatternMiner::testPatternMatcher1()
{
    originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );
    std::cout <<"Debug: PatternMiner total link number = "
              << allLinks.size() << std::endl;

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

    Handle varHandle1 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_1" );
    Handle varHandle2 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_2" );
    Handle varHandle3 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_3" );
    Handle varHandle4 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_4" );


    variableNodes.push_back(varHandle1);
    variableNodes.push_back(varHandle2);
    variableNodes.push_back(varHandle3);
    variableNodes.push_back(varHandle4);

    // The first EvaluationLink
    HandleSeq listlinkOutgoings1, evalLinkOutgoings1;
    Handle bobNode = originalAtomSpace->add_node(opencog::NODE, "Bob" );
    listlinkOutgoings1.push_back(bobNode);
    listlinkOutgoings1.push_back(varHandle2);
    Handle listlink1 = originalAtomSpace->add_link(LIST_LINK, listlinkOutgoings1);
    // XXX why do we need to set the TV ???
    listlink1->merge(TruthValue::TRUE_TV());
    evalLinkOutgoings1.push_back(varHandle1);
    evalLinkOutgoings1.push_back(listlink1);
    Handle evalLink1 = originalAtomSpace->add_link(EVALUATION_LINK, evalLinkOutgoings1);
    // XXX why do we need to set the TV ???
    evalLink1->merge(TruthValue::TRUE_TV());

    // The second EvaluationLink
    HandleSeq listlinkOutgoings2, evalLinkOutgoings2;
    listlinkOutgoings2.push_back(varHandle3);
    listlinkOutgoings2.push_back(varHandle2);
    Handle listlink2 = originalAtomSpace->add_link(LIST_LINK, listlinkOutgoings2);
    // XXX why do we need to set the TV ???
    listlink2->merge(TruthValue::TRUE_TV());
    evalLinkOutgoings2.push_back(varHandle1);
    evalLinkOutgoings2.push_back(listlink2);
    Handle evalLink2 = originalAtomSpace->add_link(EVALUATION_LINK, evalLinkOutgoings2);
    // XXX why do we need to set the TV ???
    evalLink2->merge(TruthValue::TRUE_TV());

    // The InheritanceLink
    HandleSeq inherOutgoings;
    inherOutgoings.push_back(varHandle3);
    inherOutgoings.push_back(varHandle4);
    Handle inherLink = originalAtomSpace->add_link(INHERITANCE_LINK, inherOutgoings);
    // XXX why do we need to set the TV ???
    inherLink->merge(TruthValue::TRUE_TV());

    patternToMatch.push_back(evalLink1);
    patternToMatch.push_back(evalLink2);
    patternToMatch.push_back(inherLink);

    Handle hAndLink = originalAtomSpace->add_link(AND_LINK, patternToMatch);
    // XXX why do we need to set the TV ???
    hAndLink->merge(TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
    implicationLinkOutgoings.push_back(hAndLink); // the results to return

    Handle hImplicationLink = originalAtomSpace->add_link(IMPLICATION_LINK, implicationLinkOutgoings);
    // XXX why do we need to set the TV ???
    hImplicationLink->merge(TruthValue::TRUE_TV());

    // add variable atoms
    Handle hVariablesListLink = originalAtomSpace->add_link(LIST_LINK, variableNodes);
    // XXX why do we need to set the TV ???
    hVariablesListLink->merge(TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = originalAtomSpace->add_link(BIND_LINK, bindLinkOutgoings);
    // XXX why do we need to set the TV ???
    hBindLink->merge(TruthValue::TRUE_TV());

    std::cout <<"Debug: PatternMiner::testPatternMatcher for pattern:" << std::endl
              << hBindLink->toShortString().c_str() << std::endl;


    // Run pattern matcher
    Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

    std::cout << toString(resultSet.size())  << " instances found:" << std::endl ;

    //debug
    std::cout << hResultListLink->toShortString() << std::endl  << std::endl;

    originalAtomSpace->remove_atom(hResultListLink);
    originalAtomSpace->remove_atom(hBindLink);


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

    Handle varHandle1 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_1" );
    Handle varHandle2 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_2" );
    Handle varHandle3 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_3" );

    variableNodes.push_back(varHandle1);
    variableNodes.push_back(varHandle2);
    variableNodes.push_back(varHandle3);

    // The first EvaluationLink
    HandleSeq listlinkOutgoings1, evalLinkOutgoings1;
    Handle LiMingNode = originalAtomSpace->add_node(opencog::NODE, "LiMing" );
    listlinkOutgoings1.push_back(LiMingNode);
    Handle teaNode = originalAtomSpace->add_node(opencog::CONCEPT_NODE, "tea" );
    listlinkOutgoings1.push_back(teaNode);
    Handle listlink1 = originalAtomSpace->add_link(LIST_LINK, listlinkOutgoings1);
    // XXX why do we need to set the TV ???
    listlink1->merge(TruthValue::TRUE_TV());
    evalLinkOutgoings1.push_back(varHandle1);
    evalLinkOutgoings1.push_back(listlink1);
    Handle evalLink1 = originalAtomSpace->add_link(EVALUATION_LINK, evalLinkOutgoings1);
    // XXX why do we need to set the TV ???
    evalLink1->merge(TruthValue::TRUE_TV());

    // The second EvaluationLink
    HandleSeq listlinkOutgoings2, evalLinkOutgoings2;
    listlinkOutgoings2.push_back(varHandle2);
    listlinkOutgoings2.push_back(varHandle3);
    Handle listlink2 = originalAtomSpace->add_link(LIST_LINK, listlinkOutgoings2);
    // XXX why do we need to set the TV ???
    listlink2->merge(TruthValue::TRUE_TV());
    evalLinkOutgoings2.push_back(varHandle1);
    evalLinkOutgoings2.push_back(listlink2);
    Handle evalLink2 = originalAtomSpace->add_link(EVALUATION_LINK, evalLinkOutgoings2);
    // XXX why do we need to set the TV ???
    evalLink2->merge(TruthValue::TRUE_TV());

    // The InheritanceLink
    HandleSeq inherOutgoings;
    Handle humanNode = originalAtomSpace->add_node(opencog::CONCEPT_NODE, "human" );
    inherOutgoings.push_back(varHandle2);
    inherOutgoings.push_back(humanNode);
    Handle inherLink = originalAtomSpace->add_link(INHERITANCE_LINK, inherOutgoings);
    // XXX why do we need to set the TV ???
    inherLink->merge(TruthValue::TRUE_TV());

    patternToMatch.push_back(evalLink1);
    patternToMatch.push_back(evalLink2);
    patternToMatch.push_back(inherLink);

    Handle hAndLink = originalAtomSpace->add_link(AND_LINK, patternToMatch);
    // XXX why do we need to set the TV ???
    hAndLink->merge(TruthValue::TRUE_TV());

    // add variable atoms
    Handle hVariablesListLink = originalAtomSpace->add_link(LIST_LINK, variableNodes);
    // XXX why do we need to set the TV ???
    hVariablesListLink->merge(TruthValue::TRUE_TV());

    resultOutgoings.push_back(hVariablesListLink);
    resultOutgoings.push_back(hAndLink);

    Handle hListLinkResult = originalAtomSpace->add_link(LIST_LINK, resultOutgoings);
    // XXX why do we need to set the TV ???
    hListLinkResult->merge(TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hAndLink); // the pattern to match
    implicationLinkOutgoings.push_back(hListLinkResult); // the results to return

    Handle hImplicationLink = originalAtomSpace->add_link(IMPLICATION_LINK, implicationLinkOutgoings);
    // XXX why do we need to set the TV ???
    hImplicationLink->merge(TruthValue::TRUE_TV());


    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = originalAtomSpace->add_link(BIND_LINK, bindLinkOutgoings);
    // XXX why do we need to set the TV ???
    hBindLink->merge(TruthValue::TRUE_TV());

    std::cout <<"Debug: PatternMiner::testPatternMatcher for pattern:" << std::endl
              << hBindLink->toShortString().c_str() << std::endl;


    // Run pattern matcher
    Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

    std::cout << toString(resultSet.size())  << " instances found:" << std::endl ;

    //debug
    std::cout << hResultListLink->toShortString() << std::endl  << std::endl;

    originalAtomSpace->remove_atom(hResultListLink);
    originalAtomSpace->remove_atom(hBindLink);


}

OrderedHandleSet PatternMiner::_getAllNonIgnoredLinksForGivenNode(Handle keywordNode, OrderedHandleSet& allSubsetLinks)
{
    OrderedHandleSet newHandles;
    HandleSeq incomings;
    keywordNode->getIncomingSet(back_inserter(incomings));

    for (Handle incomingHandle : incomings)
    {
        Handle newh = incomingHandle;

        // if this atom is a igonred type, get its first parent that is not in the igonred types
        if (isIgnoredType (incomingHandle->getType()) )
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

OrderedHandleSet PatternMiner::_extendOneLinkForSubsetCorpus(OrderedHandleSet& allNewLinksLastGram, OrderedHandleSet& allSubsetLinks)
{
    OrderedHandleSet allNewConnectedLinksThisGram;
    // only extend the links in allNewLinksLastGram. allNewLinksLastGram is a part of allSubsetLinks
    for (Handle link : allNewLinksLastGram)
    {
        // find all nodes in this link
        OrderedHandleSet allNodes;
        extractAllNodesInLink(link, allNodes, originalAtomSpace);

        for (Handle neighborNode : allNodes)
        {
            if (neighborNode->getType() == PREDICATE_NODE)
                continue;

            string content = neighborNode->getName();
            if (isIgnoredContent(content))
                continue;

            OrderedHandleSet newConnectedLinks;
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
    std::cout << "\nSelecting a subset from loaded corpus in Atomspace for the following keywords within " << max_connection << " distance:" << std::endl ;
    OrderedHandleSet allSubsetLinks;
    string topicsStr = "";

    for (string keyword : subsetKeywords)
    {
        std::cout << keyword << std::endl;
        Handle keywordNode = originalAtomSpace->add_node(opencog::CONCEPT_NODE,keyword);
        OrderedHandleSet newConnectedLinks = _getAllNonIgnoredLinksForGivenNode(keywordNode, allSubsetLinks);

        allSubsetLinks.insert(newConnectedLinks.begin(), newConnectedLinks.end());
        topicsStr += "-";
        topicsStr += keyword;

    }

    unsigned int order = 0;
    OrderedHandleSet allNewConnectedLinksThisGram = allSubsetLinks;

    while (order < max_connection)
    {
        allNewConnectedLinksThisGram = _extendOneLinkForSubsetCorpus(allNewConnectedLinksThisGram, allSubsetLinks);
        order ++;
    }

    ofstream subsetFile;

    string fileName = "SubSet" + topicsStr + ".scm";

    subsetFile.open(fileName.c_str());

    // write the first line to enable unicode
    subsetFile <<  "(setlocale LC_CTYPE \"\")" << std::endl ;

    for (Handle h : allSubsetLinks)
    {
        if (containIgnoredContent(h))
            continue;

        subsetFile << h->toShortString();
    }

    subsetFile.close();

    std::cout << "\nDone! Subset size: " << allSubsetLinks.size() << " Links in total. The subset has been written to file:  " << fileName << std::endl ;
}

// recursively function
bool PatternMiner::loadOutgoingsIntoAtomSpaceFromString(stringstream& outgoingStream, AtomSpace *_atomSpace, HandleSeq &outgoings, string parentIndent)
{
    string line;
    string curIndent = parentIndent + LINE_INDENTATION;

    while(getline(outgoingStream, line))
    {

        std::size_t nonIndentStartPos = line.find("(");
        string indent = line.substr(0, nonIndentStartPos);
        string nonIndentSubStr = line.substr(nonIndentStartPos + 1);
        std::size_t typeEndPos = nonIndentSubStr.find(" ");
        string atomTypeStr = nonIndentSubStr.substr(0, typeEndPos);
        string linkOrNodeStr = atomTypeStr.substr(atomTypeStr.size() - 4, 4);
        Type atomType = classserver().getType(atomTypeStr);
        if (NOTYPE == atomType)
        {
            cout << "Warning: loadOutgoingsIntoAtomSpaceFromString: Not a valid typename: " << atomTypeStr << std::endl;
            return false;

        }

        if (indent == curIndent)
        {
            if (linkOrNodeStr == "Node")
            {
                std::size_t nodeNameEndPos = nonIndentSubStr.find(")");
                string nodeName = nonIndentSubStr.substr(typeEndPos + 1, nodeNameEndPos - typeEndPos - 1);
                Handle node = _atomSpace->add_node(atomType, nodeName);
                outgoings.push_back(node);
            }
            else if (linkOrNodeStr == "Link")
            {
                // call this function recursively
                HandleSeq childOutgoings;
                if (! loadOutgoingsIntoAtomSpaceFromString(outgoingStream, _atomSpace, childOutgoings, curIndent))
                    return false;

                Handle link = _atomSpace->add_link(atomType, childOutgoings);
                outgoings.push_back(link);
            }
            else
            {
                cout << "Warning: loadOutgoingsIntoAtomSpaceFromString: Not a Node, neighter a Link: " << linkOrNodeStr << std::endl;
                return false;

            }

        }
        else if (indent.size() < curIndent.size())
        {
            return true;
        }
        else
        {
            // exception
            cout << "Warning: loadOutgoingsIntoAtomSpaceFromString: Indent wrong: " << line << std::endl;
            return false;


        }

    }

    return true;
}


// a patternStr is sent from a distributed worker via json, it's the keystring of a pattern
// the server need to load the string into links into AtomSpace
// e.g. a patternStr =
//  (InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode human)\n\n
//  (EvaluationLink )\n
//    (PredicateNode like_drink)
//    (Listlink )\n
//      (VariableNode $var_1)\n
//      (ConceptNode soda)\n\n
//  (InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode ugly)\n\n

//(AndLink)\n
//  (InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode human)\n
//  (EvaluationLink )\n
//    (PredicateNode like_drink)
//    (Listlink )\n
//      (VariableNode $var_1)\n
//      (ConceptNode soda)\n\n
//(InheritanceLink )\n
//    (VariableNode $var_1)\n
//    (ConceptNode ugly)\n\n
HandleSeq PatternMiner::loadPatternIntoAtomSpaceFromString(string patternStr, AtomSpace *_atomSpace)
{

    std::vector<std::string> strs;
    boost::algorithm::split_regex( strs, patternStr, boost::regex( "\n\n" ) ) ;

    HandleSeq pattern;

//    // debug
//    if (strs.size() > 2)
//    {
//        int i = 0;
//        i++;
//    }

    for (string linkStr : strs) // load each link
    {
        if (linkStr == "")
            continue;

            HandleSeq rootOutgoings;

            std::size_t firstLineEndPos = linkStr.find("\n");
            std::string rootOutgoingStr = linkStr.substr(firstLineEndPos + 1);
            stringstream outgoingStream(rootOutgoingStr);

            if (! loadOutgoingsIntoAtomSpaceFromString(outgoingStream, _atomSpace, rootOutgoings))
            {
                cout << "Warning: loadPatternIntoAtomSpaceFromString: Parse pattern string error: " << linkStr << std::endl;
                HandleSeq emptyPattern;
                return emptyPattern;
            }

            std::size_t typeEndPos = linkStr.find(" ");
            string atomTypeStr = linkStr.substr(1, typeEndPos - 1);
            string linkOrNodeStr = atomTypeStr.substr(atomTypeStr.size() - 4, 4);

            if (linkOrNodeStr != "Link")
            {

                cout << "Warning: loadPatternIntoAtomSpaceFromString: Not a Link: " << linkOrNodeStr << std::endl;
                HandleSeq emptyPattern;
                return emptyPattern;

            }

            Type atomType = classserver().getType(atomTypeStr);
            if (NOTYPE == atomType)
            {

                cout << "Warning: loadPatternIntoAtomSpaceFromString: Not a valid typename: " << atomTypeStr << std::endl;
                HandleSeq emptyPattern;
                return emptyPattern;

            }

            Handle rootLink = _atomSpace->add_link(atomType, rootOutgoings);
            pattern.push_back(rootLink);


    }

    // debug:
    // static int pattern_num = 0;
    // string patternToStr = "";

    // for(Handle h : pattern)
    // {
    //    patternToStr += h->toShortString();
    //    patternToStr += "\n";
    // }

    // cout << "\nAdded pattern: NO." << pattern_num << "\n" << patternToStr;
    // pattern_num ++;

    return pattern;
}


void PatternMiner::loadPatternsFromResultFile(string fileName)
{
    ifstream resultFile;
    resultFile.open(fileName.c_str());

    if (resultFile.is_open())
        std::cout << "\nLoading patterns from  " << fileName << std::endl ;
    else
        std::cout << "\nCannot find file " << fileName << "!" << std::endl ;


    // read the first line
    string firstLine;
    std::getline(resultFile, firstLine);

    // get the pattern number
    int lastSpace = firstLine.find_last_of(" ");
    string patternumStr = firstLine.substr(lastSpace + 1, firstLine.size() - (lastSpace + 1));
    int expectedPatternNumber = atoi(patternumStr.c_str());
    std::cout << "Expected pattern number = " << expectedPatternNumber << std::endl ;

    // loop to load every pattern
//    Pattern: Frequency = 5
//    (InheritanceLink )
//      (VariableNode $var_1)
//      (ConceptNode human)

//    (InheritanceLink )
//      (VariableNode $var_1)
//      (ConceptNode man)

//    (InheritanceLink )
//      (VariableNode $var_1)
//      (ConceptNode ugly)

    string patternStr = "";
    int frequency = 0;
    string lastLine = "";
    unsigned int loadedPatternNum = 0;
    bool patternStart = false;

    for (std::string line; std::getline(resultFile, line); )
    {
        if (patternStart && (line == "") && (lastLine == "")) // one pattern end, load it
        {
            // add this new found pattern into the Atomspace
            HandleSeq patternHandleSeq = loadPatternIntoAtomSpaceFromString(patternStr, atomSpace);

            if (patternHandleSeq.size() == 0)
            {

                cout << "Warning: Invalid pattern string: " << patternStr << std::endl;
                return;

            }

            // create a new HTreeNode
            HTreeNode* newHTreeNode = new HTreeNode();
            newHTreeNode->pattern = patternHandleSeq;
            newHTreeNode->count = frequency;

            keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(patternStr, newHTreeNode));
            (patternsForGram[patternHandleSeq.size()-1]).push_back(newHTreeNode);
            loadedPatternNum ++;
            patternStr = "";
            patternStart = false;
        }
        else if (line.find("Pattern:") != string::npos) // one pattern start
        {
            int numberStart = line.find("Frequency = ") + 12;

            string numStr = line.substr(numberStart, line.size() - numberStart);
            frequency = atoi(numStr.c_str());
            patternStart = true;
        }
        else if (patternStart)// in the middle of one pattern
        {
            if (line == "")
                patternStr += "\n";
            else
                patternStr += (line + "\n");
        }

        lastLine = line;
    }

    resultFile.close();

    std::cout << "\nDone! " << loadedPatternNum <<  " patterns loaded in total!" << std::endl ;

}


