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

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <opencog/util/Config.h>

#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/atom_types.h>
#include <opencog/learning/PatternMiner/types/atom_types.h>
#include <opencog/query/BindLinkAPI.h>

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

void PatternMiner::generateIndexesOfSharedVars(Handle& link, HandleSeq& orderedHandles, vector < vector< std::pair<int,std::size_t> > >& indexes)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();
    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if (h->getType() == opencog::PATTERN_VARIABLENODE_TYPE)
            {
                string var_name = h->getName();

                vector< std::pair<int,std::size_t> > indexesForCurVar; // vector <handleindex,varposintthehandle>
                int index = 0;

                for (Handle oh : orderedHandles)
                {
                    string ohStr = oh->toShortString();
                    std::size_t pos = ohStr.find(var_name) ;
                    if (pos != std::string::npos)
                    {
                        indexesForCurVar.push_back(std::pair<int,std::size_t>(index,pos));
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
           if (h->getType() == opencog::PATTERN_VARIABLENODE_TYPE)
           {
               // it's a variable node, rename it
               if (varNameMap.find(h) != varNameMap.end())
               {
                   renameOutgoingLinks.push_back(varNameMap[h]);
               }
               else
               {
                   string var_name = "$var_"  + toString(varNameMap.size() + 1);
                   Handle var_node = atomSpace->add_node(opencog::PATTERN_VARIABLENODE_TYPE, var_name);

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

        rebindedPattern.push_back(rebindedLink);
    }

    return rebindedPattern;
}


void PatternMiner::ReplaceConstNodeWithVariableForOneLink(Handle link, Handle constNode, Handle newVariableNode, HandleSeq& renameOutgoingLinks)
{

    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {

        if (h->isNode())
        {
           if (h == constNode)
           {
               // it's the const node, replace it
               renameOutgoingLinks.push_back(newVariableNode);
           }
           else
           {
               // it's not that const node to be replaced, just add it
               renameOutgoingLinks.push_back(h);
           }
        }
        else
        {
             HandleSeq _renameOutgoingLinks;
             ReplaceConstNodeWithVariableForOneLink(h, constNode, newVariableNode, _renameOutgoingLinks);
             Handle reLink = atomSpace->add_link(h->getType(),_renameOutgoingLinks);

             renameOutgoingLinks.push_back(reLink);
        }

    }

}

HandleSeq PatternMiner::ReplaceConstNodeWithVariableForAPattern(HandleSeq& pattern, Handle constNode, Handle newVariableNode)
{

    HandleSeq rebindedPattern;

    for (Handle link : pattern)
    {
        HandleSeq renameOutgoingLinks;
        ReplaceConstNodeWithVariableForOneLink(link, constNode, newVariableNode, renameOutgoingLinks);

        Handle rebindedLink = atomSpace->add_link(link->getType(),renameOutgoingLinks);

        rebindedPattern.push_back(rebindedLink);
    }

    return rebindedPattern;
}

// the input links should be like: only specify the const node, all the variable node name should not be specified:
// unifiedLastLinkIndex is to return where the last link in the input pattern is now in the ordered pattern
// because the last link in input pattern is the externed link from last gram pattern
// in orderedVarNameMap, the first Handle is the variable node in the input unordered pattern,
// the second Handle is the renamed ordered variable node in the output ordered pattern.
HandleSeq PatternMiner::UnifyPatternOrder(HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex, map<Handle,Handle>& orderedVarNameMap)
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
               value_node->setTruthValue(TruthValue::TRUE_TV());
               outputOutgoings.push_back(value_node);
           }
        }
        else
        {
             HandleSeq _outputOutgoings;
             generateALinkByChosenVariables(h, valueToVarMap, _outputOutgoings, _fromAtomSpace);
             Handle reLink = atomSpace->add_link(h->getType(),_outputOutgoings);
             // XXX why do we need to set the TV ???
             reLink->setTruthValue(TruthValue::TRUE_TV());
             outputOutgoings.push_back(reLink);
        }
    }
}

 // valueToVarMap:  the ground value node in the orginal Atomspace to the variable handle in pattenmining Atomspace
// _fromAtomSpace: where the input link is from
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
                Handle varHandle = atomSpace->add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var~" + toString(valueToVarMap.size()) );
                valueToVarMap.insert(std::pair<Handle,Handle>(h,varHandle));
            }

            if ((h->getType() == opencog::PATTERN_VARIABLENODE_TYPE))
                cout<<"Error: instance link contains variables: \n" << h->toShortString() <<std::endl;

        }
        else
        {
            extractAllNodesInLink(h,valueToVarMap,_fromAtomSpace);
        }
    }
}

void PatternMiner::extractAllVariableNodesInAnInstanceLink(Handle& instanceLink, Handle& patternLink, HandleSet& allVarNodes)
{

    HandleSeq ioutgoingLinks = instanceLink->getOutgoingSet();
    HandleSeq poutgoingLinks = patternLink->getOutgoingSet();

    HandleSeq::iterator pit = poutgoingLinks.begin();

    for (Handle h : ioutgoingLinks)
    {
        if (h->isNode())
        {
            if (((*pit)->getType() == opencog::PATTERN_VARIABLENODE_TYPE))
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
            if (((*pit)->getType() == opencog::PATTERN_VARIABLENODE_TYPE))
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

void PatternMiner::extractAllNodesInLink(Handle link, HandleSet& allNodes, AtomSpace* _fromAtomSpace)
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

void PatternMiner::extractAllConstNodesInALink(Handle link, HandleSet& allConstNodes, AtomSpace* _atomSpace)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if ((h->getType() != opencog::PATTERN_VARIABLENODE_TYPE) && (allConstNodes.find(h) == allConstNodes.end()))
            {
                allConstNodes.insert(h);
            }
        }
        else
        {
            extractAllVariableNodesInLink(h,allConstNodes, _atomSpace);
        }
    }
}


void PatternMiner::extractAllVariableNodesInLink(Handle link, HandleSet& allNodes, AtomSpace* _atomSpace)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if ((h->getType() == opencog::PATTERN_VARIABLENODE_TYPE) && (allNodes.find(h) == allNodes.end()))
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
            if (h->getType() != opencog::PATTERN_VARIABLENODE_TYPE)
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

bool PatternMiner::containVariableNodes(Handle link, AtomSpace* _atomSpace)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if (h->getType() == opencog::PATTERN_VARIABLENODE_TYPE)
            {
                return true;
            }
        }
        else
        {
            if (containVariableNodes(h, _atomSpace))
                return true;
        }
    }

    return false;
}


void PatternMiner::swapOneLinkBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, Handle& fromLink, HandleSeq& outgoings,
                                                  HandleSeq &outVariableNodes )
{
    HandleSeq outgoingLinks = fromLink->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            Handle new_node;

            if (h->getType() == PATTERN_VARIABLENODE_TYPE)
            {
                new_node = toAtomSpace->add_node(VARIABLE_NODE, h->getName());
                if ( ! isInHandleSeq(new_node, outVariableNodes) ) // should not have duplicated variable nodes
                 outVariableNodes.push_back(new_node);
            }
            else
                new_node = toAtomSpace->add_node(h->getType(), h->getName());

           outgoings.push_back(new_node);

        }
        else
        {
             HandleSeq _OutgoingLinks;

             swapOneLinkBetweenTwoAtomSpace(fromAtomSpace, toAtomSpace, h, _OutgoingLinks, outVariableNodes);
             Handle _link = toAtomSpace->add_link(h->getType(), _OutgoingLinks);
             _link->setTruthValue(h->getTruthValue());

             outgoings.push_back(_link);
        }
    }
}

HandleSeq PatternMiner::swapLinksBetweenTwoAtomSpace(AtomSpace* fromAtomSpace, AtomSpace* toAtomSpace, HandleSeq& fromLinks, HandleSeq& outVariableNodes)
{
    HandleSeq outPutLinks;

    for (Handle link : fromLinks)
    {
        HandleSeq outgoingLinks;

        swapOneLinkBetweenTwoAtomSpace(fromAtomSpace, toAtomSpace, link, outgoingLinks, outVariableNodes);
        Handle toLink = toAtomSpace->add_link(link->getType(), outgoingLinks);
        toLink->setTruthValue(link->getTruthValue());

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
//        (VariableList)
//          (VariableNode "$var_1")
//          (VariableNode "$var_2")
//          ...
//        (AndLink
//          ;; The pattern to be searched for
//          (pattern)
//          (Listlink)
//              ;; The instance to be returned.
//        (result)
//        )
//     )

//    HandleSeq allAtomSpaceLinks;
//    originalAtomSpace->get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
//    std::cout <<"Debug: PatternMiner total link number = "
//              << allAtomSpaceLinks.size() << std::endl;

    AtomSpace* _atomSpace = originalAtomSpace;

    HandleSeq  bindLinkOutgoings, variableNodes;

//  HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpaceForBindLink(atomSpace, _atomSpace, HNode->pattern, variableNodes, linksWillBeDel);
    HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpace(atomSpace, _atomSpace, HNode->pattern, variableNodes);

    Handle hAndLink = _atomSpace->add_link(AND_LINK, patternToMatch);

//    // add variable atoms
//    HandleSet allVariableNodesInPattern;
//    for (unsigned int i = 0; i < patternToMatch.size(); ++i)
//    {
//        extractAllVariableNodesInLink(patternToMatch[i],allVariableNodesInPattern, _atomSpace);
//    }


//    for (Handle varh : allVariableNodesInPattern)
//    {
//        Handle v = _atomSpace->add_node(VARIABLE_NODE, varh->getName());
//        variableNodes.push_back(v);
//    }

    Handle hVariablesListLink = _atomSpace->add_link(VARIABLE_LIST, variableNodes);

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hAndLink); // the pattern to match
    bindLinkOutgoings.push_back(hAndLink); // the results to return

    Handle hBindLink = _atomSpace->add_link(BIND_LINK, bindLinkOutgoings);

//    std::cout << std::endl << hBindLink->toShortString() << std::endl;

    // Run pattern matcher
    Handle hResultListLink = opencog::bindlink(_atomSpace, hBindLink);


    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

    HNode->count = resultSet.size();

    //    //debug
//    std::cout << hResultListLink->toShortString() << std::endl  << std::endl;

//    if (HNode->pattern.size() == 2)
//    cout << "\nRemoving hResultListLink\n" << hResultListLink->toShortString() << std::endl;
    _atomSpace->remove_atom(hResultListLink);

//    int count = 0;
    for (Handle listH  : resultSet)
    {

//        if  (Pattern_mining_mode == "Breadth_First")
//        {
//            HandleSeq instanceLinks = listH->getOutgoingSet();
//            if (cur_gram == 1)
//            {
//                HNode->instances.push_back(instanceLinks);
//            }
//            else
//            {
//                // instance that contains duplicate links will not be added
//                if (! containsDuplicateHandle(instanceLinks))
//                    HNode->instances.push_back(instanceLinks);
//            }
//        }

//        if (! containVariableNodes(listH, _atomSpace))
//            count ++;

//        if (HNode->pattern.size() == 2)
//        cout << "\nRemoving listH \n" << listH->toShortString() << std::endl;
        _atomSpace->remove_atom(listH);
    }

//    HNode->count = count;

//     std::cout << HNode->count << " instances found!" << std::endl ;

//    if (HNode->pattern.size() == 2)
//    cout << "\nRemoving hBindLink\n" << hBindLink->toShortString() << std::endl;
    _atomSpace->remove_atom(hBindLink);

//    if (HNode->pattern.size() == 2)
//    cout << "\nRemoving hAndLink" << hAndLink->toShortString() << std::endl;
    _atomSpace->remove_atom(hAndLink);

//    for (Handle patternLink : linksWillBeDel) // delete the patterns links contains variables
//    {
//        _atomSpace->remove_atom(patternLink);
//    }

    for (Handle varh : variableNodes)
    {
        _atomSpace->remove_atom(varh,true);
    }

//    allAtomSpaceLinks.clear();
//    originalAtomSpace->get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
//    std::cout <<"After: PatternMiner total link number = "
//              << allAtomSpaceLinks.size() << std::endl;

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
    for (Type t : linktype_black_list)
    {
        if (t == type)
            return true;
    }

    return false;
}

bool PatternMiner::isTypeInList(Type type, vector<Type> &typeList)
{
    for (Type t : typeList)
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

bool PatternMiner::doesLinkContainNodesInKeyWordNodes(const Handle& link, const HandleSet& keywordNodes)
{
    HandleSeq outgoingLinks = link->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->isNode())
        {
            if (keywordNodes.find(h) != keywordNodes.end())
                return true;
        }
        else
        {
            if (doesLinkContainNodesInKeyWordNodes(h, keywordNodes))
                return true;
        }
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


bool PatternMiner::add_linktype_to_white_list(Type _type)
{
    if (isTypeInList(_type, linktype_white_list))
        return false; // already in the ignore link list

    linktype_white_list.push_back(_type);
    return true;
}

bool PatternMiner::remove_linktype_from_white_list(Type _type)
{
    vector<Type>::iterator it;
    for (it = linktype_white_list.begin(); it != linktype_white_list.end(); it ++)
    {
        if ((Type)(*it) == _type)
        {
           linktype_white_list.erase(it);
           return true;
        }
    }

    return false;
}

bool PatternMiner::add_Ignore_Link_Type(Type _type)
{
    if (isIgnoredType(_type))
        return false; // already in the ignore link list

    linktype_black_list.push_back(_type);
    return true;
}

bool PatternMiner::remove_Ignore_Link_Type(Type _type)
{
    vector<Type>::iterator it;
    for (it = linktype_black_list.begin(); it != linktype_black_list.end(); it ++)
    {
        if ((Type)(*it) == _type)
        {
           linktype_black_list.erase(it);
           return true;
        }
    }

    return false;
}

bool PatternMiner::add_link_type_to_same_link_types_not_share_second_outgoing(Type _type)
{
    for (Type t : same_link_types_not_share_second_outgoing)
    {
        if (t == _type)
            return false; //  exist
    }

    same_link_types_not_share_second_outgoing.push_back(_type);
    return true;
}

bool PatternMiner::remove_link_type_from_same_link_types_not_share_second_outgoing(Type _type)
{
    vector<Type>::iterator it;
    for (it = same_link_types_not_share_second_outgoing.begin(); it != same_link_types_not_share_second_outgoing.end(); it ++)
    {
        if ((Type)(*it) == _type)
        {
           same_link_types_not_share_second_outgoing.erase(it);
           return true;
        }
    }

    return false; // not exist

}

bool PatternMiner::add_node_type_to_node_types_should_not_be_vars(Type _type)
{
    for (Type t : node_types_should_not_be_vars)
    {
        if (t == _type)
            return false; //  exist
    }

    node_types_should_not_be_vars.push_back(_type);
    return true;
}

bool PatternMiner::remove_node_type_from_node_types_should_not_be_vars(Type _type)
{
    vector<Type>::iterator it;
    for (it = node_types_should_not_be_vars.begin(); it != node_types_should_not_be_vars.end(); it ++)
    {
        if ((Type)(*it) == _type)
        {
           node_types_should_not_be_vars.erase(it);
           return true;
        }
    }

    return false; // not exist
}

bool PatternMiner::add_keyword_to_black_list(string _keyword)
{
    if (_keyword == "")
            return false;

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
        IncomingSet incomings = cur_h->getIncomingSet(atomspace);
        if (incomings.size() == 0)
            return Handle::UNDEFINED;

        Handle incomingHandle = (incomings[0])->getHandle();
        if (isIgnoredType (incomingHandle->getType()))
        {
            cur_h = incomingHandle;
            continue;
        }
        else
            return incomingHandle;

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
    if (USE_ABS_SURPRISINGNESS)
    {
        if ( node1->nI_Surprisingness - node2->nI_Surprisingness  > FLOAT_MIN_DIFF)
            return true;
        else if (node2->nI_Surprisingness - node1->nI_Surprisingness > FLOAT_MIN_DIFF)
            return false;
    }
    else
    {
        if ( std::abs(node1->nI_Surprisingness) - std::abs(node2->nI_Surprisingness)  > FLOAT_MIN_DIFF)
            return true;
        else if (std::abs(node2->nI_Surprisingness) - std::abs(node1->nI_Surprisingness) > FLOAT_MIN_DIFF)
            return false;
    }

    return (node1->var_num < node2->var_num);
}

bool compareHTreeNodeBySurprisingness_II(HTreeNode* node1, HTreeNode* node2)
{

    if ((node1->superPatternRelations.size() != 0) && (node2->superPatternRelations.size() != 0))
    {
        if ( node1->nII_Surprisingness - node2->nII_Surprisingness > FLOAT_MIN_DIFF)
            return true;
        else if ( node2->nII_Surprisingness - node1->nII_Surprisingness > FLOAT_MIN_DIFF)
            return false;
    }

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


    resultFile << ";Interesting Pattern Mining final results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;


    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

//        if ((htreeNode->superPatternRelations.size() == 0))
//            continue;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

//        resultFile << endl << ";Pattern: PatternValues = " << ((htreeNode->quotedPatternLink->getValue(PatternValuesHandle)))->toShortString() << endl;

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << ", SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << ", SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;
        if (if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->toShortString();
        else
        {
            for (Handle link : htreeNode->pattern)
            {
                resultFile << link->toShortString();
            }
        }

        resultFile << std::endl;

    }

    resultFile << std::endl;
    resultFile.close();


}


void PatternMiner::OutPutAllEntityNumsToFile()
{
    // out put to csv file
    ofstream csvFile;
    string csvfileName = "AllEntityNums.csv";

    std::cout<<"\nDebug: PatternMiner: writing AllEntityNums to csv file " + csvfileName << std::endl;

    csvFile.open(csvfileName.c_str());

    csvFile << "predicates,AllEntityNum" << std::endl;

    map<string, unsigned int>::const_iterator numit;
    for (numit = allEntityNumMap.begin(); numit != allEntityNumMap.end(); numit ++)
    {
        csvFile << numit->first << "," << numit->second << std::endl;
    }

    csvFile.close();
}



void PatternMiner::OutPutFrequentPatternsToFile(unsigned int n_gram, vector < vector<HTreeNode*> >& _patternsForGram, string _fileNamebasic)
{

    // out put the n_gram frequent patterns to a file, in the order of frequency
    ofstream resultFile;

    string fileName = _fileNamebasic + "FrequentPatterns_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing  (gram = " + toString(n_gram) + ") frequent patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());
    vector<HTreeNode*> &patternsForThisGram = _patternsForGram[n_gram-1];

    resultFile << ";Frequent Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;
    num_of_patterns_with_1_frequency[n_gram-1] = 0;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {

        if (htreeNode->count == 1)
            num_of_patterns_with_1_frequency[n_gram-1] ++;

        if (htreeNode->count < thresholdFrequency)
            continue;

        // resultFile << endl << ";Pattern: PatternValues = " << ((htreeNode->quotedPatternLink->getValue(PatternValuesHandle)))->toShortString() << endl;
        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        if (if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->toShortString();
        else
        {
            for (Handle link : htreeNode->pattern)
            {
                resultFile << link->toShortString();
            }
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();


}

void PatternMiner::OutPutInterestingPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, int surprisingness, string _fileNamebasic) // surprisingness 1 or 2, it is default 0 which means Interaction_Information
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName = _fileNamebasic;

    if (surprisingness == 0)
        fileName += "Interaction_Information_" + toString(n_gram) + "gram.scm";
    else if (surprisingness == 1)
        fileName += "SurprisingnessI_" + toString(n_gram) + "gram.scm";
    else
        fileName += "SurprisingnessII_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") interesting patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());


    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

        if ((surprisingness != 0) && OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }


        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        if (surprisingness == 0)
            resultFile << " InteractionInformation = " << toString(htreeNode->interactionInformation);
        else
        {
            resultFile << ", SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

            resultFile << ", SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);
        }

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        if (if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->toShortString();
        else
        {
            for (Handle link : htreeNode->pattern)
            {
                resultFile << link->toShortString();
            }
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
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


    csvFile << "Frequency,Surprisingness_I,Surprisingness_II,nII_Surprisingness_b" << std::endl;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < thresholdFrequency)
            continue;

        csvFile << htreeNode->count << "," << htreeNode->nI_Surprisingness << ","

                << htreeNode->nII_Surprisingness << "," << htreeNode->nII_Surprisingness_b ;
//        if (htreeNode->superPatternRelations.size() > 0)
//            csvFile << htreeNode->nII_Surprisingness;
//        else
//            csvFile << "unknown";

        csvFile << std::endl;
    }

    csvFile.close();
}


void PatternMiner::OutPutLowFrequencyHighSurprisingnessPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, unsigned int max_frequency_index)
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "LowFrequencyHighSurprisingness_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") LowFrequencyHighSurprisingness patterns to file " + fileName << std::endl;


    vector<HTreeNode*> resultPatterns;

    for (unsigned int i = patternsForThisGram.size() - 1; i > max_frequency_index; i --)
    {
        HTreeNode* htreeNode = patternsForThisGram[i];

        resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeBySurprisingness_I);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency < "<< (patternsForThisGram[max_frequency_index])->count <<", sort by Surprisingness_I"  << std::endl;


    for (HTreeNode* htreeNode : resultPatterns)
    {
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);


        resultFile << ", SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (Handle link : htreeNode->pattern)
        {
            resultFile << link->toShortString();
        }

        resultFile << std::endl;

    }

    resultFile << std::endl;
    resultFile.close();


}

void PatternMiner::OutPutHighFrequencyHighSurprisingnessPatternsToFile(vector<HTreeNode*> &patternsForThisGram, unsigned int n_gram, unsigned int min_frequency_index)
{

    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;


    fileName = "HighFrequencyHighSurprisingness_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") HighFrequencyHighSurprisingness patterns to file " + fileName << std::endl;

    vector<HTreeNode*> resultPatterns;

    for (unsigned int i = 0; i < min_frequency_index; i ++)
    {
        HTreeNode* htreeNode = patternsForThisGram[i];
        resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeBySurprisingness_I);

    resultFile.open(fileName.c_str());


    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency > " << (patternsForThisGram[min_frequency_index])->count << ", sort by Surprisingness_I"  << std::endl;


    for (HTreeNode* htreeNode : resultPatterns)
    {
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);


        resultFile << ", SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (Handle link : htreeNode->pattern)
        {
            resultFile << link->toShortString();
        }

        resultFile << std::endl;


    }

    resultFile << std::endl;
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
//        if (htreeNode->superPatternRelations.size() == 0)
//            continue;

        if (htreeNode->count < thresholdFrequency)
            continue;


        if ( (htreeNode->nI_Surprisingness > min_surprisingness_I) && (htreeNode->nII_Surprisingness < max_surprisingness_II) )
            resultPatterns.push_back(htreeNode);
    }


    resultFile.open(fileName.c_str());


    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Surprising_I > " << min_surprisingness_I << ", and Surprisingness_II < "  << max_surprisingness_II << std::endl;


    // std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeBySurprisingness_I);

    for (HTreeNode* htreeNode : resultPatterns)
    {
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

        resultFile << endl << "Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << " SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (Handle link : htreeNode->pattern)
        {
            resultFile << link->toShortString();
        }

        resultFile << std::endl;


    }

    resultFile << std::endl;
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

    HandleSet allNodesInEachLink[inputLinks.size()];

    HandleSet all2ndOutgoingsOfInherlinks;

    // map<predicate, set<value> >
    map<Handle, HandleSet > predicateToValueOfEvalLinks;

    HandleSet  all1stOutgoingsOfEvalLinks;


    for (unsigned int i = 0; i < inputLinks.size(); ++i)
    {
        extractAllNodesInLink(inputLinks[i],allNodesInEachLink[i], _atomSpace);

        if (inputLinks.size() == 1)
            break;

        if (enable_filter_links_of_same_type_not_share_second_outgoing)
        {
            for (Type t : same_link_types_not_share_second_outgoing)
            {
                // filter: Any two Links of the same type in the  should not share their secondary outgoing nodes
                if (((inputLinks[i])->getType() == t))
                {
                    Handle secondOutgoing = inputLinks[i]->getOutgoingSet()[1];
                    if (all2ndOutgoingsOfInherlinks.find(secondOutgoing) == all2ndOutgoingsOfInherlinks.end())
                        all2ndOutgoingsOfInherlinks.insert(secondOutgoing);
                    else
                        return true;
                }
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
                    map<Handle, HandleSet >::iterator it = predicateToValueOfEvalLinks.find(predicateNode);
                    if (it != predicateToValueOfEvalLinks.end())
                    {
                        HandleSet& values = (it->second);
                        if (values.find(valueNode) != values.end())
                            return true;
                        else
                            values.insert(valueNode);
                    }
                    else
                    {
                        HandleSet newValues;
                        newValues.insert(valueNode);
                        predicateToValueOfEvalLinks.insert(std::pair<Handle, HandleSet >(predicateNode,newValues));
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
        else if (enable_filter_not_all_first_outgoing_const)
        {
            // if enable_filter_first_outgoing_evallink_should_be_var is true, there is no need to enable this filter below
            HandleSeq all1stOutgoingsOfEvalLinksSeq(all1stOutgoingsOfEvalLinks.begin(), all1stOutgoingsOfEvalLinks.end());
            oneOfEachSeqShouldBeVars.push_back(all1stOutgoingsOfEvalLinksSeq);
        }
    }


    if ((inputLinks.size() > 1) && enable_filter_links_should_connect_by_vars)
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


    // only check node_types_should_not_be_vars for 1 gram patterns
    if (enable_filter_leaves_should_not_be_vars || enable_filter_node_types_should_not_be_vars)
    {

        for (unsigned i = 0; i < inputLinks.size(); i ++)
        {
            for (Handle node : allNodesInEachLink[i])
            {

                // find leaves
                if ((inputLinks.size() > 1) && enable_filter_leaves_should_not_be_vars)
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

    HandleSet allNodesInEachLink[inputLinks.size()];
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
        // cout << "\nwarning: cannot find subpattern: \n" << connectedSubPatternKey << std::endl;
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

    double biggestEntroy = 0.0;

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
             map<Handle,Handle> orderedVarNameMap;
             HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex,orderedVarNameMap);
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
                     map<Handle,Handle> suborderedVarNameMap;
                     HandleSeq unifiedConnectedSubPattern = UnifyPatternOrder(aConnectedSubPart, _unifiedLastLinkIndex,suborderedVarNameMap);
                     string connectedSubPatternKey = unifiedPatternToKeyString(unifiedConnectedSubPattern);
//                     cout << "a splitted part: " << connectedSubPatternKey;
                     double h = calculateEntropyOfASubConnectedPattern(connectedSubPatternKey, unifiedConnectedSubPattern);
                     II += sign*h;

                     if (h > biggestEntroy)
                         biggestEntroy = h;
//                     cout << "sign="<<sign << " h =" << h << std::endl << std::endl;

                 }

             }
             else
             {
//                 std::cout<< " is connected! \n" ;
                 double h =calculateEntropyOfASubConnectedPattern(subPatternKey, unifiedSubPattern);
                 II += sign*h;

                 if (h > biggestEntroy)
                     biggestEntroy = h;
//                 cout << "sign="<<sign << " h =" << h << std::endl << std::endl;
             }


             if (isLastNElementsAllTrue(indexes, maxgram, gram))
                 break;

             // generate the next combination
             generateNextCombinationGroup(indexes, maxgram);
         }

         delete [] indexes;

    }

    // Normalize the interaction information
    if (biggestEntroy > FLOAT_MIN_DIFF)
        HNode->interactionInformation = II / biggestEntroy;
    else
        HNode->interactionInformation = 0.0;
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
        // return 0; // just skip it
        // can't find its HtreeNode, have to calculate its frequency again by calling pattern matcher
        // Todo: need to decide if add this missing HtreeNode into H-Tree or not

        // cout << "Exception: can't find a subpattern: \n" << connectedPatternKey << std::endl;
        if (is_distributed)
        {
            uniqueKeyLock.unlock();
            return 0;
        }
        else
        {

            if (PATTERN_VARIABLENODE_TYPE == PATTERN_VARIABLE_NODE)
            {
                uniqueKeyLock.unlock();
                return 0;
            }
            else
            {
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
            if ((outComingsOfPattern[i])->getType() == PATTERN_VARIABLENODE_TYPE)
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
             reLink->setTruthValue(h->getTruthValue());
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
    IncomingSet incomings = leaf->getIncomingSet(_fromAtomSpace);

    for (LinkPtr incomingPtr : incomings)
    {
        Handle incomingHandle  = incomingPtr->getHandle();
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
        reLink->setTruthValue(extendedHandle->getTruthValue());
        outPutExtendedPatternLinks.push_back(reLink);
    }

}

unsigned int PatternMiner::getAllEntityCountWithSamePredicatesForAPattern(HandleSeq& pattern)
{

    if (pattern.size() == 1)
    {
        if (pattern[0]->getType() == EVALUATION_LINK)
        {
            Handle predicate = pattern[0]->getOutgoingAtom(0);

            string predicateName = predicate->getName();

//            cout << "/npredicate: " << predicateName << std::endl;

            map<string,unsigned int>::iterator eit = allEntityNumMap.find(predicateName);
            if (eit != allEntityNumMap.end())
            {
//                cout << "alredy exists: " << eit->second << std::endl;
                return eit->second;
            }
            else
            {
                IncomingSet allEvals = predicate->getIncomingSet(originalAtomSpace);
                allEntityNumMap.insert(std::pair<string,int>(predicateName,allEvals.size()));
//                cout << "Found: " << allEvals.size() << " entities." << std::endl;
                return allEvals.size();
            }
        }
        else
        {
            cout << "warning: this pattern contains " << classserver().getTypeName(pattern[0]->getType())
                 << "\nUSE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE is for the corpus that only contains EvalutionLinks." << std::endl;
            return 0;
        }
    }
    else
    {

        if (pattern.size() == 3)
        {
            int x = 0;
            x ++;
        }
        HandleSeq allPredicateNodes;

        for (Handle l : pattern)
        {
            if (l->getType() == EVALUATION_LINK)
            {
                Handle predicate = l->getOutgoingAtom(0);
                allPredicateNodes.push_back(predicate);
            }
            else
            {
                cout << "warning: this pattern contains " << classserver().getTypeName(l->getType())
                     << "\nUSE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE is for the corpus that only contains EvalutionLinks." << std::endl;
                return 0;
            }
        }

        std::sort(allPredicateNodes.begin(), allPredicateNodes.end());
        string predicateWords = "";
        for (Handle predicate : allPredicateNodes)
        {
            predicateWords += predicate->getName();
            predicateWords += " ";
        }

//        cout << "/npredicates: " << predicateWords << std::endl;

        map<string,unsigned int>::iterator eit = allEntityNumMap.find(predicateWords);
        if (eit != allEntityNumMap.end())
        {
//            cout << "alredy exists: " << eit->second << std::endl;
            return eit->second;
        }

        vector<HandleSet> allEntitiesForEachPredicate;

        for (Handle predicate : allPredicateNodes)
        {
            HandleSet allEntitiesForThisPredicate;

            IncomingSet allEvals = predicate->getIncomingSet(originalAtomSpace);
            for (LinkPtr incomeingPtr : allEvals)
            {
                Handle evalLink = incomeingPtr->getHandle();
                Handle listLink = evalLink->getOutgoingAtom(1);
                Handle entityNode = listLink->getOutgoingAtom(0);
                allEntitiesForThisPredicate.insert(entityNode);
            }

            allEntitiesForEachPredicate.push_back(allEntitiesForThisPredicate);

        }

        HandleSeq commonLinks;
        // get the common Links in allEntitiesForEachPredicate
        std::set_intersection(allEntitiesForEachPredicate[0].begin(), allEntitiesForEachPredicate[0].end(),
                              allEntitiesForEachPredicate[1].begin(), allEntitiesForEachPredicate[1].end(),
                              std::back_inserter(commonLinks));

        if (commonLinks.size() == 0)
            return 0;

        for (unsigned int i = 2; i < pattern.size(); ++ i)
        {
            HandleSeq newCommonLinks;
            // get the common Links in allEntitiesForEachPredicate
            std::set_intersection(allEntitiesForEachPredicate[i].begin(), allEntitiesForEachPredicate[i].end(),
                                  commonLinks.begin(), commonLinks.end(),
                                  std::back_inserter(newCommonLinks));

            if (newCommonLinks.size() == 0)
                return 0;

            commonLinks.swap(newCommonLinks);
        }

        allEntityNumMap.insert(std::pair<string,int>(predicateWords,commonLinks.size()));
//        cout << "Found: " << commonLinks.size() << " entities." << std::endl;
        return commonLinks.size();
    }

//    HandleSeq allEntityPattern;
//    int var_index_num = 1;
//    Handle entityVar = atomSpace->add_node(VARIABLE_NODE, "$var_1");

//    for (Handle l : pattern)
//    {
//        var_index_num ++;

//        if (l->getType() == EVALUATION_LINK)
//        {
//            Handle conceptNode = l->getOutgoingAtom(0);

//            Handle valueNode = atomSpace->add_node(VARIABLE_NODE, "$var_" + toString(var_index_num));
//            Handle newListLink = atomSpace->add_link(LIST_LINK, entityVar, valueNode);

//            Handle newEvalLink = atomSpace->add_link(EVALUATION_LINK, conceptNode, newListLink);

//            allEntityPattern.push_back(newEvalLink);
//        }
//        else
//        {
//            cout << "warning: this pattern contains " << classserver().getTypeName(l->getType())
//                 << "\USE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE is for the corpus that only contains EvalutionLinks." << std::endl;
//            return;
//        }
//    }

//    unsigned int unifiedLastLinkIndex;
//    HandleSeq unifiedPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex);
//    string unifiedPatternKey = unifiedPatternToKeyString(unifiedPattern);
//    cout << "\allEntityPattern: \n" << unifiedPatternKey << std::endl;

//    uniqueKeyLock.lock();
//    // try to find if it has a correponding HtreeNode
//    map<string, HTreeNode*>::iterator patternNodeIter = keyStrToHTreeNodeMap.find(connectedPatternKey);



}

// II_Surprisingness_b can be calulate for all input grams, including 1 gram and max_gram
// only calculate 2~4 gram patterns for nSurprisingness_I and nSurprisingness_II
void PatternMiner::calculateSurprisingness( HTreeNode* HNode, AtomSpace *_fromAtomSpace)
{

//    // debug
//    if (HNode->nI_Surprisingness != 0 || HNode->nII_Surprisingness != 0)
//        std::cout << "Exception: This pattern has been calculateSurprisingness before!\n";

//    if (HNode->count == 0) // this should not happen
//        HNode->count = 1;

    if (HNode->count < thresholdFrequency)
    {

        HNode->nII_Surprisingness = 0.0f;
        HNode->nI_Surprisingness = 0.0f;
        HNode->nII_Surprisingness_b = 0.0f;
        return;
    }

    if (CALCULATE_TYPE_B_SURPRISINGNESS)
        calculateTypeBSurprisingness(HNode, _fromAtomSpace);


    unsigned int gram = HNode->pattern.size();

    if (gram == 1)
        return;


//    std::cout << "=================Debug: calculate I_Surprisingness for pattern: ====================\n";
//    for (Handle link : HNode->pattern)
//    {
//        std::cout << link->toShortString();
//    }
//     std::cout << "count of this pattern = " << HNode->count << std::endl;
//     std::cout << std::endl;


    // get the predefined combination:
    // vector<vector<vector<unsigned int>>>
//    int comcount = 0;

    HNode->surprisingnessInfo = "";

    float p;
    unsigned int allNum;

    if (USE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE) // this setting only for the corpus that only contains EvalutionLinks
    {
        // generate the allPattern for this pattern, e.g.:
//        (EvaluationLink (stv 1.000000 1.000000)
//          (PredicateNode "BirthYearCat")
//          (ListLink (stv 1.000000 1.000000)
//            (VariableNode "$var_1")
//            (ConceptNode "1975-1980")
//          )
//        )
//        the allPattern: it is to get all the entities that have the same predicate
//        (EvaluationLink (stv 1.000000 1.000000)
//          (PredicateNode "BirthYearCat")
//          (ListLink (stv 1.000000 1.000000)
//            (VariableNode "$var_1")
//            (VariableNode "$var_2")
//          )
//        )
        unsigned int allEntityCount = getAllEntityCountWithSamePredicatesForAPattern(HNode->pattern);
        if (allEntityCount == 0)
        {
            cout << "error: cannot find instances for this allEntityCount." << std::endl;
            return;
        }
        else
        {
            p = ((float)HNode->count) / ((float)allEntityCount);
            allNum = allEntityCount;
            // cout << "allEntityCount = " << allEntityCount << std::endl;

            if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
            {
                HNode->surprisingnessInfo += ";p = " + toString(HNode->count) + "/" + toString(allEntityCount) + " = " + toString(p) + "\n";
            }
        }
    }
    else
    {
        p = ((float)HNode->count)/atomspaceSizeFloat;
        allNum = atomspaceSizeFloat;
    }

    float abs_min_diff = 999999999.9f;
    float min_diff = 999999999.9f;
    // cout << "For this pattern itself: p = " <<  HNode->count << " / " <<  (int)atomspaceSizeFloat << " = " << p << std::endl;

    for (vector<vector<unsigned int>>&  oneCombin : components_ngram[gram-2])
    {
        int com_i = 0;
        // std::cout <<" -----Combination " << comcount++ << "-----" << std::endl;
        float total_p = 1.0f;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            HNode->surprisingnessInfo += "\n; {";
        }

        bool containsComponentDisconnected = false;
        bool subComponentNotFound = false;

        for (vector<unsigned int>& oneComponent : oneCombin)
        {
            if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
            {
                HNode->surprisingnessInfo += " [";
            }

            HandleSeq subPattern;
            for (unsigned int index : oneComponent)
            {
                subPattern.push_back(HNode->pattern[index]);

                if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
                {
                    HNode->surprisingnessInfo += toString(index);
                }
            }
            if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
            {
                HNode->surprisingnessInfo += "]=";
            }

            unsigned int unifiedLastLinkIndex;
            map<Handle,Handle> orderedVarNameMap;
            HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex, orderedVarNameMap);
            string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

//            std::cout<< "Subpattern: " << subPatternKey;

            // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
            HandleSeqSeq splittedSubPattern;
            if (splitDisconnectedLinksIntoConnectedGroups(unifiedSubPattern, splittedSubPattern))
            {
//                std::cout<< " is disconnected! skip it \n" ;
                containsComponentDisconnected = true;

                if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
                {
                    HNode->surprisingnessInfo += "D ";
                }

                break;
            }
            else
            {
//                std::cout<< " is connected!" ;
                unsigned int component_count = getCountOfAConnectedPattern(subPatternKey, unifiedSubPattern);

                if (component_count == 0)
                {
                    // one of the subcomponents is missing, skip this combination
                    subComponentNotFound = true;
                    break;
                }

                float p_i;
//                cout << ", count = " << component_count;
                if (USE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE) // this setting only for the corpus that only contains EvalutionLinks
                {
                    unsigned int allEntityCount = getAllEntityCountWithSamePredicatesForAPattern(unifiedSubPattern);
                    if (allEntityCount == 0)
                    {
                        cout << "error: cannot find instances for this allEntityCount." << std::endl;
                        return;
                    }
                    else
                    {
                        p_i = ((float)(component_count)) / ((float)allEntityCount);
//                        cout << "allEntityCount = " << allEntityCount;
//                        cout << ", p = " << component_count  << " / " << allEntityCount << " = " << p_i << std::endl;

                        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
                        {
                            HNode->surprisingnessInfo += toString(component_count) + "/" + toString(allEntityCount) + "=" + toString(p_i);
                        }
                    }
                }
                else
                {
                    p_i = ((float)(component_count)) / atomspaceSizeFloat;

                    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
                    {
                        HNode->surprisingnessInfo += toString(component_count) + "/" + toString(atomspaceSizeFloat) + "=" + toString(p_i);
                    }
                }

                // cout << ", p = " << component_count  << " / " << (int)atomspaceSizeFloat << " = " << p_i << std::endl;
                total_p *= p_i;
//                std::cout << std::endl;

            }

            com_i ++;

        }

        if (containsComponentDisconnected || subComponentNotFound)
            continue;


//        cout << "\n ---- total_p = " << total_p << " ----\n" ;

        float diff = p - total_p;
        diff = diff / total_p;

        float abs_diff = diff;

        if (abs_diff < 0)
            abs_diff = - abs_diff;

//        cout << "diff  = p - total_p" << diff << " \n" ;

        if (abs_diff < abs_min_diff)
        {
            abs_min_diff = abs_diff;
            min_diff = diff;
        }

//        cout << "diff / total_p = " << diff << " \n" ;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            HNode->surprisingnessInfo += ", expect = " + toString(total_p) + "x" + toString(allNum) + " = "
                    + toString((total_p * ((float)allNum))) + ", nDiff = " + toString(diff) + "}";
        }

    }


    if (USE_ABS_SURPRISINGNESS)
        HNode->nI_Surprisingness = abs_min_diff;
    else
        HNode->nI_Surprisingness = min_diff;

    // debug:
//    cout << "nI_Surprisingness = " << HNode->nI_Surprisingness  << std::endl;

    if (gram == MAX_GRAM ) // can't calculate II_Surprisingness for MAX_GRAM patterns, becasue it required gram +1 patterns
        return;


    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile << "=================Debug: calculate II_Surprisingness for pattern: ====================\n";
        surpringnessIICalfile << "Frequency = " << HNode->count << " p = " << HNode->count << "/" << allNum << " = " << p << std::endl;
        for (Handle link : HNode->pattern)
        {
            surpringnessIICalfile << link->toShortString();
        }

        surpringnessIICalfile << std::endl;
    }

    // II_Surprisingness is to evaluate how easily the frequency of this pattern can be infered from any of  its superpatterns
    // for all its super patterns
    if (HNode->superPatternRelations.size() == 0)
    {
        HNode->nII_Surprisingness  = 999999999.9;
        // debug:
//        cout << "This node has no super patterns, give it Surprisingness_II value: -1.0 \n";
    }
    else
    {

        HNode->nII_Surprisingness  = 999999999.9f;
        float minSurprisingness_II = 999999999.9f;
        vector<ExtendRelation>::iterator oneSuperRelationIt;
        unsigned int actualProcessedRelationNum = 0;
        for(oneSuperRelationIt = HNode->superPatternRelations.begin();  oneSuperRelationIt != HNode->superPatternRelations.end(); ++ oneSuperRelationIt)
        {
            ExtendRelation& curSuperRelation = *oneSuperRelationIt;

            // There are two types of super patterns: one is extended from a variable, one is extended from a const (turnt into a variable)

            // type one : extended from a variable,  the extended node itself is considered as a variable in the pattern A
            //            {
            //                // Ap is A's one supper pattern, E is the link pattern that extended
            //                // e.g.: M is the size of corpus
            //                // A:  ( var_1 is alien ) && ( var_1 is horror ) , P(A) = 100/M
            //                // A1: ( var_1 is alien ), A2: ( var_1 is horror )
            //                // Ap: ( var_1 is alien ) && ( var_1 is horror ) && ( var_1 is male ) , P(Ap) = 99/M
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
            // When Count (E) = 1, it means this super pattern is not really more generalized, so this super pattern should be skipped.

            // Note that becasue of unifying patern, the varible order in A, Ap, E can be different
            // HandleSeq& patternAp = curSuperRelation.extendedHTreeNode->pattern;

            if (curSuperRelation.isExtendedFromVar)
                continue;

            float p_Ap = ((float )(curSuperRelation.extendedHTreeNode->count))/((float)allNum);

            HandleSeq patternE;
            patternE.push_back(curSuperRelation.newExtendedLink);
            // unify patternE
            unsigned int unifiedLastLinkIndex;
            map<Handle,Handle> EorderedVarNameMap;
            HandleSeq unifiedPatternE = UnifyPatternOrder(patternE, unifiedLastLinkIndex, EorderedVarNameMap);

            string patternEKey = unifiedPatternToKeyString(unifiedPatternE, atomSpace);

            unsigned int patternE_count = getCountOfAConnectedPattern(patternEKey, unifiedPatternE);

           if (patternE_count == 1)
               continue; // This super pattern is not really more generalized, skipped it!

            float p_ApDivByCountE = p_Ap / ( (float)(patternE_count) );

            float Surprisingness_II;
            if (p_ApDivByCountE > p)
                Surprisingness_II = p_ApDivByCountE - p;
            else
                Surprisingness_II = p - p_ApDivByCountE;

            if (Surprisingness_II < minSurprisingness_II)
                minSurprisingness_II = Surprisingness_II;

             if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
             {
                if (curSuperRelation.isExtendedFromVar)
                    surpringnessIICalfile << "For Super pattern: -------extended from a variable----------------- " << std::endl;
                else
                    surpringnessIICalfile << "For Super pattern: -------extended from a const----------------- " << std::endl;

                for (Handle link : curSuperRelation.extendedHTreeNode->pattern)
                {
                    surpringnessIICalfile << link->toShortString();
                }
                //surpringnessIICalfile << unifiedPatternToKeyString(curSuperRelation.extendedHTreeNode->pattern, atomSpace);
                surpringnessIICalfile << "P(Ap) = " << curSuperRelation.extendedHTreeNode->count << "/" << allNum << " = " << p_Ap << std::endl;
                surpringnessIICalfile << "The extended link pattern:  " << std::endl;
                surpringnessIICalfile << patternEKey;
                surpringnessIICalfile << "Count(E) = " << patternE_count << std::endl;
                surpringnessIICalfile << "Surprisingness_II = |P(A) -P(Ap)/Count(E)| = " << Surprisingness_II << std::endl;
             }

             actualProcessedRelationNum ++;


        }

//        // debug
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            if (actualProcessedRelationNum > 0)
                surpringnessIICalfile << "Min Surprisingness_II  = " << minSurprisingness_II;
            else
                surpringnessIICalfile << "actualProcessedRelationNum = 0. Min Surprisingness_II  = -1.0";
        }

        if ((HNode->superPatternRelations.size() > 0) && actualProcessedRelationNum)
        {
            HNode->nII_Surprisingness = minSurprisingness_II/p;
        }
        else
        {
            HNode->nII_Surprisingness = 999999999.9f;
            // num_of_patterns_without_superpattern_cur_gram ++; // todo: need a lock
        }
    }


//    // debug:
    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile << " nII_Surprisingness = Min Surprisingness_II / p = " << HNode->nII_Surprisingness  << std::endl;
        surpringnessIICalfile << "=================Debug: end calculate II_Surprisingness ====================\n\n";
    }

}


void PatternMiner::calculateTypeBSurprisingness( HTreeNode* HNode, AtomSpace *_fromAtomSpace)
{

        // Currently II_Surprisingness_b only for 1-gram patterns
        // because the tracking of type b super-sub relations for 2-gram or bigger patterns is too costly
        // If really wants to calculate II_Surprisingness_b for 2-gram or biger patterns,
        // need to generate the type b relations when calculating II_Surprisingness_b. But it is very very costly.
        // need to turn on GENERATE_TYPE_B_RELATION_WHEN_CALCULATE_SURPRISINGNESS.
    //    std::cout << "=================Debug: calculate II_Surprisingness_b for 1-gram patterns: ====================\n";
        //    for (Handle link : HNode->pattern)
        //    {
        //        std::cout << link->toShortString();
        //    }
        //     std::cout << "count of this pattern = " << HNode->count << std::endl;
        //     std::cout << std::endl;
        // Surpringness II also can be calculated via more general patterns of the same gram, e.g.
        // nII_Surprisingness_b(A) = min{Surprisingness_b from all super patterns} = min{Count(A) / Count(S)} = 1.0 - 18 / 98
        // Here we should only consider the number of subpatterns of S, not the exact Frequency of each subpatterns,
        // because even if there is country that occurs 70 times, and other countries only occurs 3 times or less,
        // if there are a lot of countries have the same pattern with A, then S is still a generailized pattern.
        //
        //    ;Pattern A: Frequency = 18
        // A1:(EvaluationLink
        //      (PredicateNode "birthPlace")
        //      (ListLink
        //        (VariableNode "$var_1")
        //        (ConceptNode "United_States")
        //      )
        //    )
        // A2:(EvaluationLink
        //      (PredicateNode "deathPlace")
        //      (ListLink
        //        (VariableNode "$var_1")
        //        (ConceptNode "United_States")
        //      )
        //    )
        //
        //    ;Pattern S: Frequency = 98
        // S1:(EvaluationLink
        //      (PredicateNode "birthPlace")
        //      (ListLink
        //        (VariableNode "$var_1")
        //        (VariableNode "$var_2")
        //      )
        //    )
        // S2:(EvaluationLink
        //      (PredicateNode "deathPlace")
        //      (ListLink
        //        (VariableNode "$var_1")
        //        (VariableNode "$var_2")
        //      )
        //    )

        if ((HNode->pattern.size() > 1) && (! GENERATE_TYPE_B_RELATION_WHEN_CALCULATE_SURPRISINGNESS) )
        {
            return;
        }

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            surpringnessIICalfile << "=================Debug: calculate II_Surprisingness_b for pattern: ====================\n";
            surpringnessIICalfile << "Count = " << HNode->count << std::endl;
            for (Handle link : HNode->pattern)
            {
                surpringnessIICalfile << link->toShortString();
            }

            surpringnessIICalfile << std::endl;
        }

        // 1-gram patterns already has superRelation_b_list and SubRelation_b_map
        // so first, find this type b relations for 2-gram and bigger patterns
        if (HNode->pattern.size() > 1)
        {
            // generate all the super patterns of same gram of this pattern for 2-gram or bigger patterns
            // By changing one const node into a variable node, if this pattern exist, then it is one super pattern of this pattern
            set<Handle> allConstNodes;
            for (Handle link : HNode->pattern)
                extractAllConstNodesInALink(link, allConstNodes, atomSpace);

            string var_name = "$var_"  + toString(HNode->var_num + 1);
            Handle var_node = atomSpace->add_node(opencog::PATTERN_VARIABLENODE_TYPE, var_name);

            for (Handle constNode : allConstNodes)
            {
                // replace this const node with a new variable node
                HandleSeq oneSuperPattern = ReplaceConstNodeWithVariableForAPattern(HNode->pattern, constNode, var_node);

                // only try to find it from mined patterns, will not query it by pattern matcher

                unsigned int unifiedLastLinkIndex;
                map<Handle,Handle> suborderedVarNameMap;
                HandleSeq unifiedSuperPattern = UnifyPatternOrder(oneSuperPattern, unifiedLastLinkIndex, suborderedVarNameMap);

                string superPatternKey = unifiedPatternToKeyString(unifiedSuperPattern);

                // todo: need a lock here?
                map<string, HTreeNode*>::iterator patternNodeIter = keyStrToHTreeNodeMap.find(superPatternKey);
                if (patternNodeIter != keyStrToHTreeNodeMap.end())
                {
                    HTreeNode* superPatternNode = (HTreeNode*)patternNodeIter->second;

                    SuperRelation_b superb;
                    superb.superHTreeNode = superPatternNode;
                    superb.constNode = constNode;

                    HNode->superRelation_b_list.push_back(superb);

                    Handle unified_var_node = suborderedVarNameMap[var_node];

                    if (superPatternNode->SubRelation_b_map.find(unified_var_node) == superPatternNode->SubRelation_b_map.end())
                    {
                        vector<SubRelation_b> sub_blist;

                        SubRelation_b sub_b;
                        sub_b.constNode = constNode;
                        sub_b.subHTreeNode = HNode;

                        sub_blist.push_back(sub_b);
                        superPatternNode->SubRelation_b_map.insert(std::pair<Handle, vector<SubRelation_b>>(unified_var_node, sub_blist));
                    }
                }

            }

        }


        // calculate II_Surprisingness_b
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            surpringnessIICalfile << "=================Debug: calculate II_Surprisingness_b for pattern: ====================\n";
            surpringnessIICalfile << "Count = " << HNode->count << std::endl;
            for (Handle link : HNode->pattern)
            {
                surpringnessIICalfile << link->toShortString();
            }

            surpringnessIICalfile << std::endl;
        }

        double min_II_Surprisingness_b = 1.0;

        for (SuperRelation_b& superb : HNode->superRelation_b_list)
        {
            // calculate II_Surprisingness_b
            double II_Surprisingness_b = ((float)HNode->count) / ((float)superb.superHTreeNode->count);
            if (II_Surprisingness_b < min_II_Surprisingness_b)
                min_II_Surprisingness_b = II_Surprisingness_b;

            if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
            {
                surpringnessIICalfile << "Count(S) = " << superb.superHTreeNode->count << ", II_Surprisingness_b = "
                                      << HNode->count << " / " << superb.superHTreeNode->count << " = " << II_Surprisingness_b;
            }

            surpringnessIICalfile << "-----------end super pattern :---------------\n";

        }

        HNode->nII_Surprisingness_b = min_II_Surprisingness_b;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            surpringnessIICalfile << "\nmin_II_Surprisingness_b = " << min_II_Surprisingness_b
                                  << "\n=================Debug: calculate I_Surprisingness for pattern: ====================\n";
        }
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

    std::cout<<"Debug: PatternMiner init finished! " << std::endl;

}

PatternMiner::~PatternMiner()
{
    cleanUpPatternMiner();
}

void PatternMiner::addAtomTypesFromString(string node_types_str, vector<Type>& typeListToAddTo)
{

    node_types_str.erase(std::remove(node_types_str.begin(), node_types_str.end(), ' '), node_types_str.end());
    vector<string> typeStrs;
    boost::split(typeStrs, node_types_str, boost::is_any_of(","));

    for (string typestr : typeStrs)
    {
        Type atomType = classserver().getType(typestr);
        if (atomType == NOTYPE)
        {
            cout << "\nCannot find Node Type: " << typestr << " in config file.\n";
            continue;
        }
        typeListToAddTo.push_back(atomType);
    }
}

// make sure it is called after reSetAllSettingsFromConfig
void PatternMiner::initPatternMiner()
{
    htree = new HTree();
    atomSpace = new AtomSpace(originalAtomSpace);

//    FrequencyHandle = atomSpace->add_node(CONCEPT_NODE, "Frequency");
//    InteractionInformationHandle = atomSpace->add_node(CONCEPT_NODE, "InteractionInformation");
//    SurprisingnessIHandle = atomSpace->add_node(CONCEPT_NODE, "SurprisingnessIHandle");
//    SurprisingnessIIHandle = atomSpace->add_node(CONCEPT_NODE, "SurprisingnessIIHandle");

    PatternValuesHandle = atomSpace->add_node(CONCEPT_NODE, "PatternValues");

    threads = new thread[THREAD_NUM];

    is_distributed = false;

    cur_gram = 0;

    htree = 0;

    observingAtomSpace = 0;


//    if (threads)
//        delete threads;


    // vector < vector<HTreeNode*> > patternsForGram
    for (unsigned int i = 0; i < MAX_GRAM; ++i)
    {
        vector<HTreeNode*> patternVector;
        patternsForGram.push_back(patternVector);

        vector<HTreeNode*> finalPatternVector;
        finalPatternsForGram.push_back(finalPatternVector);

        vector<HTreeNode*> tmpPatternVector;
        tmpPatternsForGram.push_back(tmpPatternVector);

    }

}

void PatternMiner::reSetAllSettingsFromConfig()
{
    int max_gram = config().get_int("Pattern_Max_Gram");
    MAX_GRAM = (unsigned int)max_gram;

    enable_Interesting_Pattern = config().get_bool("Enable_Interesting_Pattern");
    Enable_Interaction_Information = config().get_bool("Enable_Interaction_Information");
    Enable_surprisingness = config().get_bool("Enable_surprisingness");

    THREAD_NUM = config().get_int("Max_thread_num");
    unsigned int system_thread_num  = std::thread::hardware_concurrency();
    if (THREAD_NUM > system_thread_num - 1)
    {
        cout << "\nThere is only " << system_thread_num << " cores in this machine, so the Max_thread_num = "
             << THREAD_NUM << " will not be used." << system_thread_num - 1 << " threads will be used instead." << std::endl;

        THREAD_NUM = system_thread_num - 1;
    }

    thresholdFrequency = config().get_int("Frequency_threshold");

    if_quote_output_pattern = config().get_bool("if_quote_output_pattern");
    string quotedTypeStr = config().get("output_pattern_quoted_linktype");
//    cout << "quotedTypeStr = " << quotedTypeStr << std::endl;
    output_pattern_quoted_linktype = classserver().getType(quotedTypeStr);
    if (output_pattern_quoted_linktype == NOTYPE)
    {
        cout << "\nError: output_pattern_quoted_linktype : "<< quotedTypeStr << " in config file doesn't exist!" << std::endl;
    }

    use_keyword_black_list = config().get_bool("use_keyword_black_list");
    use_keyword_white_list = config().get_bool("use_keyword_white_list");

    keyword_black_logic_is_contain = config().get_bool("keyword_black_logic_is_contain");

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


    use_linktype_black_list = config().get_bool("use_linktype_black_list");
    use_linktype_white_list = config().get_bool("use_linktype_white_list");

    // use_linktype_black_list and use_linktype_white_list should not both be true
    assert((! use_linktype_black_list) || (! use_linktype_white_list));

    linktype_black_list.clear();
    string linktype_black_list_str = config().get("linktype_black_list");
    addAtomTypesFromString(linktype_black_list_str, linktype_black_list);

    linktype_white_list.clear();
    string linktype_white_list_str = config().get("linktype_white_list");
    addAtomTypesFromString(linktype_white_list_str, linktype_white_list);

    enable_filter_leaves_should_not_be_vars = config().get_bool("enable_filter_leaves_should_not_be_vars");
    enable_filter_links_should_connect_by_vars = config().get_bool("enable_filter_links_should_connect_by_vars");
    enable_filter_node_types_should_not_be_vars =  config().get_bool("enable_filter_node_types_should_not_be_vars");
    enable_filter_links_of_same_type_not_share_second_outgoing = config().get_bool("enable_filter_links_of_same_type_not_share_second_outgoing");
    enable_filter_not_all_first_outgoing_const = config().get_bool("enable_filter_not_all_first_outgoing_const");
    enable_filter_not_same_var_from_same_predicate = config().get_bool("enable_filter_not_same_var_from_same_predicate");
    enable_filter_first_outgoing_evallink_should_be_var = config().get_bool("enable_filter_first_outgoing_evallink_should_be_var");


    node_types_should_not_be_vars.clear();
    string node_types_str = config().get("node_types_should_not_be_vars");
    addAtomTypesFromString(node_types_str, node_types_should_not_be_vars);


    same_link_types_not_share_second_outgoing.clear();
    string link_types_str = config().get("same_link_types_not_share_second_outgoing");
    addAtomTypesFromString(link_types_str, same_link_types_not_share_second_outgoing);

    only_mine_patterns_start_from_white_list = config().get_bool("only_mine_patterns_start_from_white_list");
    only_mine_patterns_start_from_white_list_contain = config().get_bool("only_mine_patterns_start_from_white_list_contain");

    only_output_patterns_contains_white_keywords = config().get_bool("only_output_patterns_contains_white_keywords");

}

// release everything
void PatternMiner::cleanUpPatternMiner()
{

    if (htree != nullptr)
    {
        delete htree;
        htree = 0;
    }

    if (atomSpace != nullptr)
    {
        delete atomSpace;
        atomSpace = nullptr;
    }

//    if (originalAtomSpace)
//        delete originalAtomSpace;

    if (observingAtomSpace != nullptr)
    {
        delete observingAtomSpace;
        observingAtomSpace = nullptr;
    }


//    if (threads)
//        delete threads;

    linktype_black_list.clear();

    for( std::pair<string, HTreeNode*> OnePattern : keyStrToHTreeNodeMap)
    {
        delete ((HTreeNode*)(OnePattern.second));
    }

    std::map <string, HTreeNode*> emptykeyStrToHTreeNodeMap;
    keyStrToHTreeNodeMap.swap(emptykeyStrToHTreeNodeMap);

    // clear patternsForGram
    unsigned int patternsForGramSize = patternsForGram.size();
    for (unsigned int i = 0; i < patternsForGramSize; ++i)
    {
        std::vector<HTreeNode*> emptyVector;
        (patternsForGram[i]).swap(emptyVector);
    }

    vector < vector<HTreeNode*> > emptypatternsForGram;
    patternsForGram.swap(emptypatternsForGram);


    // clear finalPatternsForGram
    unsigned int finalPatternsForGramSize = finalPatternsForGram.size();
    for (unsigned int i = 0; i < finalPatternsForGramSize; ++i)
    {
        std::vector<HTreeNode*> emptyVector;
        (finalPatternsForGram[i]).swap(emptyVector);
    }

    vector < vector<HTreeNode*> > emptyFinalPatternsForGram;
    finalPatternsForGram.swap(emptyFinalPatternsForGram);


    // clear tmpPatternsForGram
    unsigned int tmpPatternsForGramSize = tmpPatternsForGram.size();
    for (unsigned int i = 0; i < tmpPatternsForGramSize; ++i)
    {
        std::vector<HTreeNode*> emptyVector;
        (tmpPatternsForGram[i]).swap(emptyVector);
    }

    vector < vector<HTreeNode*> > emptytmpPatternsForGram;
    tmpPatternsForGram.swap(emptytmpPatternsForGram);


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

// Quote a pattern with a pattern link, the format is as below:
//(MinedPatternLink
//   pattern links
//)
// The order of "PatternValues" store in the MinedPatternLink are as below:
//    (Frequency, InteractionInformation, Surprisingness_I, Surprisingness_II)
void PatternMiner::quoteAPattern(HTreeNode* hTreeNode)
{
//    HandleSeq quoteOutgoings;
//    Handle frequencyValue = atomSpace->add_node(NUMBER_NODE, toString(hTreeNode->count));
//    Handle iiValue = atomSpace->add_node(NUMBER_NODE, toString(hTreeNode->interactionInformation));
//    Handle SurprisingnessiValue = atomSpace->add_node(NUMBER_NODE, toString(hTreeNode->nI_Surprisingness));
//    Handle SurprisingnessiiValue = atomSpace->add_node(NUMBER_NODE, toString(hTreeNode->nII_Surprisingness));
//    Handle andLink = atomSpace->add_link(AND_LINK,hTreeNode->pattern);

//    quoteOutgoings.push_back(frequencyValue);
//    quoteOutgoings.push_back(iiValue);
//    quoteOutgoings.push_back(SurprisingnessiValue);
//    quoteOutgoings.push_back(SurprisingnessiiValue);
//    quoteOutgoings.push_back(andLink);

    Handle quotedPatternLink = hTreeNode->quotedPatternLink = atomSpace->add_link(output_pattern_quoted_linktype, hTreeNode->pattern);
    std::vector<double> valuelist;
    valuelist.push_back((double)hTreeNode->count);
    valuelist.push_back((double)hTreeNode->interactionInformation);
    valuelist.push_back((double)hTreeNode->nI_Surprisingness);
    valuelist.push_back((double)hTreeNode->nII_Surprisingness);
    ProtoAtomPtr pv = createFloatValue(valuelist);
    quotedPatternLink->setValue(PatternValuesHandle, pv);
}

void PatternMiner::quoteAllThePatternSForGram(unsigned int gram)
{
    cout << "Quoting all " << gram << "-gram patterns with " << classserver().getTypeName(output_pattern_quoted_linktype) << std::endl;
    for (HTreeNode* hTreeNode : patternsForGram[gram - 1])
    {
        quoteAPattern(hTreeNode);
    }
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

    allLinks.clear();
    originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );

    allLinkNumber = (int)(allLinks.size());
    atomspaceSizeFloat = (float)(allLinkNumber);

    black_keyword_Handles.clear();
    if (use_keyword_black_list && (! keyword_black_logic_is_contain))
    {
        for (string keyword : keyword_black_list)
        {
            // std::cout << keyword << std::endl;
            Handle keywordNode = originalAtomSpace->get_node(opencog::CONCEPT_NODE,keyword);
            if (keywordNode != Handle::UNDEFINED)
            {
                if (black_keyword_Handles.find(keywordNode) == black_keyword_Handles.end())
                    black_keyword_Handles.insert(keywordNode);
            }

            keywordNode = originalAtomSpace->get_node(opencog::PREDICATE_NODE,keyword);

            if (keywordNode != Handle::UNDEFINED)
            {
                if (black_keyword_Handles.find(keywordNode) == black_keyword_Handles.end())
                    black_keyword_Handles.insert(keywordNode);
            }

        }
    }

    std::cout << "Using " << THREAD_NUM << " threads. \n";
    std::cout << "Corpus size: "<< allLinkNumber << " links in total. \n\n";

    if (only_mine_patterns_start_from_white_list || only_output_patterns_contains_white_keywords)
    {
        allLinksContainWhiteKeywords.clear();
        havenotProcessedWhiteKeywordLinks.clear();

        if (only_mine_patterns_start_from_white_list)
        {
            cout << "\nOnly mine patterns start from white list: logic = ";
            if (only_mine_patterns_start_from_white_list_contain)
                cout << " Nodes contain keyword." << std::endl;
            else
                cout << " Nodes'label equal to keyword." << std::endl;
        }

        for (string keyword : keyword_white_list)
        {
            std::cout << keyword << std::endl;
        }

        if (use_keyword_black_list)
        {
            cout << "\nuse_keyword_black_list is also enable, so avoid links that contain any nodes that ";
            if (keyword_black_logic_is_contain)
                cout << "contain";
            else
                cout << "equal to";
            cout << " any of the following black keywords:\n";
            for (string bkeyword : keyword_black_list)
            {
                std::cout << bkeyword << std::endl;
            }
        }

        if (use_linktype_black_list)
        {
            cout << "\nuse_linktype_black_list is also enable, so avoid links of these types:\n ";

            for (Type linkTpe : linktype_black_list)
            {
                std::cout << classserver().getTypeName(linkTpe) << std::endl;
            }
        }
        else if (use_linktype_white_list)
        {
            cout << "\nuse_linktype_white_list is also enable, so only find links of these types:\n ";

            for (Type linkTpe : linktype_white_list)
            {
                std::cout << classserver().getTypeName(linkTpe) << std::endl;
            }
        }

        cout << "\n\nFinding Links...\n";


        findAllLinksContainKeyWords(keyword_white_list, 0, only_mine_patterns_start_from_white_list_contain, havenotProcessedWhiteKeywordLinks);

        std::copy(havenotProcessedWhiteKeywordLinks.begin(), havenotProcessedWhiteKeywordLinks.end(), std::back_inserter(allLinksContainWhiteKeywords));
        cout << "Found " << allLinksContainWhiteKeywords.size() << " Links contians the keywords!\n";

    }

    runPatternMinerDepthFirst();

    std::cout<<"PatternMiner:  mining finished!\n";

    if (if_quote_output_pattern)
        quoteAllThePatternSForGram(1);

    if (enable_Interesting_Pattern && (MAX_GRAM >1))
    {
        runInterestingnessEvaluation();
    }
    else if (MAX_GRAM > 1)
    {
        for(unsigned int gram = 2; gram <= MAX_GRAM; gram ++)
            quoteAllThePatternSForGram(gram);
    }

    // out put all patterns with a frequency above the thresthold
    num_of_patterns_with_1_frequency = new unsigned int [MAX_GRAM];

    for(unsigned int gram = 1; gram <= MAX_GRAM; gram ++)
    {
        // sort by frequency
        std::sort((patternsForGram[gram-1]).begin(), (patternsForGram[gram-1]).end(),compareHTreeNodeByFrequency );

        // Finished mining gram patterns; output to file
        std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGram[gram-1]).size()) + " patterns found! ";

        OutPutFrequentPatternsToFile(gram, patternsForGram);

        if (GENERATE_TMP_PATTERNS && (tmpPatternsForGram[gram-1].size() > 0))
        {
            std::sort((tmpPatternsForGram[gram-1]).begin(), (tmpPatternsForGram[gram-1]).end(),compareHTreeNodeByFrequency );

            OutPutFrequentPatternsToFile(gram, tmpPatternsForGram, "tmpPatterns");
        }

        std::cout<< std::endl;
    }


    int end_time = time(NULL);
    printf("\nPattern Mining Finished! Total time: %d seconds. \n", end_time - start_time);


    if (exit_program_after_finish)
    {
        std::cout << "Pattern Miner application quited!" << std::endl;
        std::exit(EXIT_SUCCESS);
    }



}

void PatternMiner::runInterestingnessEvaluation()
{
    if (USE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE)
        allEntityNumMap.clear();
    else if (USE_QUERY_ALL_ENTITY_COUNT)
    {
        cout << "USE_QUERY_ALL_ENTITY_COUNT is enable. Querying all entity number ...\n";
        HandleSeq allEvalLinks;
        originalAtomSpace->get_handles_by_type(back_inserter(allEvalLinks), (Type) EVALUATION_LINK, false);
        HandleSet allEntityHandles;
        for (Handle evalLink : allEvalLinks)
        {
            Handle listLink = evalLink->getOutgoingAtom(1);
            Handle entityHandle = listLink->getOutgoingAtom(0);
            if (allEntityHandles.find(entityHandle) == allEntityHandles.end())
            {
                allEntityHandles.insert(entityHandle);
            }
        }

        cout << "All entity number = " << allEntityHandles.size() << std::endl;
        atomspaceSizeFloat = (float)(allEntityHandles.size());
    }

    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile.open("surpringnessII_calcualtion_info.scm");
    }

    for(cur_gram = 2; cur_gram <= MAX_GRAM; cur_gram ++)
    {

        cout << "\nCalculating";
        if (Enable_Interaction_Information)
            cout << " Interaction_Information ";

        if (Enable_surprisingness)
            cout << " Surprisingness ";

        cout << "for " << cur_gram << " gram patterns." << std::endl;

        cur_index = -1;
        threads = new thread[THREAD_NUM];
        num_of_patterns_without_superpattern_cur_gram = 0;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
            surpringnessIICalfile << ";*************** surpringnessII calculation process info for " + toString(cur_gram) + " gram patterns.***************" << endl;

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

        if (if_quote_output_pattern)
            quoteAllThePatternSForGram(cur_gram);

        if (Enable_Interaction_Information)
        {
            // sort by interaction information
            std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 0);
        }

        if (Enable_surprisingness)
        {
            // sort by frequency
            std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency);

//            int max_frequency_threshold_index = FREQUENCY_BOTTOM_THRESHOLD * ((float)(patternsForGram[cur_gram-1].size()));
//            OutPutLowFrequencyHighSurprisingnessPatternsToFile(patternsForGram[cur_gram-1], cur_gram, max_frequency_threshold_index);

//            int min_frequency_threshold_index = FREQUENCY_TOP_THRESHOLD * ((float)(patternsForGram[cur_gram-1].size() - num_of_patterns_with_1_frequency[cur_gram-1]));
//            OutPutHighFrequencyHighSurprisingnessPatternsToFile(patternsForGram[cur_gram-1], cur_gram,  min_frequency_threshold_index);

            // sort by surprisingness_I first
            std::sort((patternsForGram[cur_gram-1]).begin(), (patternsForGram[cur_gram-1]).end(),compareHTreeNodeBySurprisingness_I);
            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram,1);

            if (cur_gram == MAX_GRAM)
                break;

            vector<HTreeNode*> curGramPatterns = patternsForGram[cur_gram-1];

            // and then sort by surprisingness_II
            std::sort(curGramPatterns.begin(), curGramPatterns.end(),compareHTreeNodeBySurprisingness_II);
            OutPutInterestingPatternsToFile(curGramPatterns,cur_gram,2);

            OutPutStaticsToCsvFile(cur_gram);

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

//            OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 100.0f, 0.51f);

            // sort by frequency
            std::sort((finalPatternsForGram[cur_gram-1]).begin(), (finalPatternsForGram[cur_gram-1]).end(),compareHTreeNodeByFrequency );

            OutPutFinalPatternsToFile(cur_gram);

        }

        std::cout<< std::endl;
    }

    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE && USE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE)
        OutPutAllEntityNumsToFile();

    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        surpringnessIICalfile.close();
}


void PatternMiner::queryPatternsWithFrequencySurprisingnessIRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                                  float min_surprisingness_I, float max_surprisingness_I, int gram)
{
    // out put the gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "GivenFrequencySurprisingnessI_" + toString(gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(gram) + ") patterns to file " + fileName << std::endl;
    std::cout<<"Frequency range = [" << min_frequency << ", " << max_frequency << "] "  << std::endl;
    std::cout<<"Surprisingness_I range = [" << toString(min_surprisingness_I) << ", " << toString(max_surprisingness_I) << "]"  << std::endl;


    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForGram[gram - 1])
    {
        if ((htreeNode->count >= min_frequency) && (htreeNode->count <= max_frequency) && (htreeNode->nI_Surprisingness >= min_surprisingness_I) && (htreeNode->nI_Surprisingness <= max_surprisingness_I))
            resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] ,"
               << "Surprisingness_I range = [" << toString(min_surprisingness_I) << ", " << toString(max_surprisingness_I) << "]" << std::endl;


    for (HTreeNode* htreeNode : resultPatterns)
    {
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        string SurprisingnessI = toString(htreeNode->nI_Surprisingness);
        if (SurprisingnessI != "0")
            resultFile << " SurprisingnessI = " << SurprisingnessI;

        string SurprisingnessII = toString(htreeNode->nII_Surprisingness);
//        if (SurprisingnessII != "0")
        resultFile << " SurprisingnessII = " << SurprisingnessII;

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (Handle link : htreeNode->pattern)
        {
            resultFile << link->toShortString();
        }

        resultFile << std::endl;

    }

    resultFile << std::endl;
    resultFile.close();

    cout <<"\nDone!" << std::endl;
}


void PatternMiner::queryPatternsWithSurprisingnessIAndIIRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                               float min_surprisingness_I, float max_surprisingness_I,
                                                               float min_surprisingness_II, float max_surprisingness_II,int gram)
{
    // out put the gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "GivenFrequencySurprisingnessIAndII_" + toString(gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(gram) + ") patterns to file " + fileName << std::endl;
    std::cout<<"Frequency range = [" << min_frequency << ", " << max_frequency << "] "  << std::endl;
    std::cout<<"Surprisingness_I range = [" << toString(min_surprisingness_I) << ", " << toString(max_surprisingness_I) << "]"  << std::endl;
    std::cout<<"Surprisingness_II range = [" << toString(min_surprisingness_II) << ", " << toString(max_surprisingness_II) << "]"  << std::endl;



    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForGram[gram - 1])
    {
        if ((htreeNode->count >= min_frequency) && (htreeNode->count <= max_frequency) &&
            (htreeNode->nI_Surprisingness >= min_surprisingness_I) && (htreeNode->nI_Surprisingness <= max_surprisingness_I) &&
            (htreeNode->nII_Surprisingness >= min_surprisingness_II) && (htreeNode->nII_Surprisingness <= max_surprisingness_II)
           )
            resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] ,"
               << "Surprisingness_I range = [" << toString(min_surprisingness_I) << ", " << toString(max_surprisingness_I) << "]"
               << "Surprisingness_II range = [" << toString(min_surprisingness_II) << ", " << toString(max_surprisingness_II) << "]" << std::endl;



    for (HTreeNode* htreeNode : resultPatterns)
    {
        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        string SurprisingnessI = toString(htreeNode->nI_Surprisingness);
        if (SurprisingnessI != "0")
            resultFile << " SurprisingnessI = " << SurprisingnessI;

        string SurprisingnessII = toString(htreeNode->nII_Surprisingness);
//        if (SurprisingnessII != "0")
        resultFile << " SurprisingnessII = " << SurprisingnessII;

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (Handle link : htreeNode->pattern)
        {
            resultFile << link->toShortString();
        }

        resultFile << std::endl;

    }

    resultFile << std::endl;
    resultFile.close();

    cout <<"\nDone!" << std::endl;
}

void PatternMiner::queryPatternsWithFrequencyAndInteractionInformationRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                               float min_ii, float max_ii, int gram)
{
    // out put the gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "GivenFrequencyInteractionInformation_" + toString(gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(gram) + ") patterns to file " + fileName << std::endl;
    std::cout<<"Frequency range = [" << min_frequency << ", " << max_frequency << "] "  << std::endl;
    std::cout<<"InteractionInformation range = [" << toString(min_ii) << ", " << toString(max_ii) << "]"  << std::endl;

    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForGram[gram - 1])
    {
        if ((htreeNode->count >= min_frequency) && (htreeNode->count <= max_frequency) &&
            (htreeNode->interactionInformation >= min_ii) && (htreeNode->interactionInformation <= max_ii)
           )
            resultPatterns.push_back(htreeNode);
    }

    std::sort(resultPatterns.begin(), resultPatterns.end(),compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] ,"
               << "InteractionInformation range = [" << toString(min_ii) << ", " << toString(max_ii) << "]."  << std::endl;

    for (HTreeNode* htreeNode : resultPatterns)
    {
//        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
//        {
//            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
//        }

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " InteractionInformation = " << htreeNode->interactionInformation;

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (Handle link : htreeNode->pattern)
        {
            resultFile << link->toShortString();
        }

        resultFile << std::endl;

    }

    resultFile << std::endl;
    resultFile.close();

    cout <<"\nDone!" << std::endl;
}


bool PatternMiner::containWhiteKeywords(const string& str, QUERY_LOGIC logic)
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

bool PatternMiner::containKeywords(const string& str, vector<string>& keywords, QUERY_LOGIC logic)
{
    if (logic == QUERY_LOGIC::OR)
    {
        for (string keyword : keywords)
        {
            if (str.find(keyword) != std::string::npos)
                return true;
        }

        return false;
    }
    else // QUERY_LOGIC::AND
    {
        for (string keyword : keywords)
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

            if (use_keyword_black_list && containKeywords(patternStr, keyword_black_list, QUERY_LOGIC::OR))
                continue;

            if (containWhiteKeywords(patternStr, keyword_white_list_logic))
                patternsForGramFiltered[gram-1].push_back(htreeNode);

        }

        // Finished mining gram patterns; output to file
        std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGramFiltered[gram-1]).size()) + " patterns found after filtering! ";
        std::sort((patternsForGramFiltered[gram-1]).begin(), (patternsForGramFiltered[gram-1]).end(),compareHTreeNodeByFrequency);
        OutPutFrequentPatternsToFile(gram, patternsForGramFiltered, fileNameBasic);

        std::cout<< std::endl;
    }

    if (enable_Interesting_Pattern && (MAX_GRAM >1))
    {
        for(cur_gram = 2; cur_gram <= MAX_GRAM; cur_gram ++)
        {

            if (Enable_Interaction_Information)
            {
                // sort by interaction information
                std::sort((patternsForGramFiltered[cur_gram-1]).begin(), (patternsForGramFiltered[cur_gram-1]).end(),compareHTreeNodeByInteractionInformation);
                OutPutInterestingPatternsToFile(patternsForGramFiltered[cur_gram-1], cur_gram, 0, fileNameBasic);
            }

            if (Enable_surprisingness)
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

        if (Enable_surprisingness)
        {
           calculateSurprisingness(htreeNode, observingAtomSpace);
        }


        if (Enable_Interaction_Information)
        {

           if (cur_gram == 3)
           {
               int x = 0;
               x ++;
           }
           calculateInteractionInformation(htreeNode);
        }

    }
}

// select a subset for topics from the corpus
void PatternMiner::selectSubsetFromCorpus(vector<string>& topics, unsigned int gram, bool if_contian_logic)
{
    _selectSubsetFromCorpus(topics,gram, if_contian_logic);
}

bool checkIfObjectIsAPerson(Handle& obj, HandleSeq& listLinks)
{
    for (Handle l : listLinks)
    {
        // only check ListLinks with h as the first outgoing
        if ( l->getOutgoingAtom(0) != obj)
            continue;

        HandleSeq evals;
        l->getIncomingSet(back_inserter(evals));

        for (Handle eval : evals)
        {
            Handle predicate = eval->getOutgoingAtom(0);
            string predicateStr = predicate->getName();
            if ((predicateStr ==  "nationality") || (predicateStr == "predecessor") || (predicateStr == "successor") || (predicateStr == "religion") ||
                    (predicateStr == "occupation") || (predicateStr == "birthPlace") || (predicateStr == "party") || (predicateStr == "almaMater")  ||
                    (predicateStr == "relation") || (predicateStr == "child") || (predicateStr == "parent") )
                return true;
        }
    }

    return false;
}

// Note: the following steps are used to select a subset related to "president","chairman","vicePresident","primeMinister","vicePrimeMinister"
// 1. (use-modules (opencog patternminer))
// 2. (clear)
// 3. (load "allkeyObjects.scm")
// 4. (pm-load-all-DBpediaKeyNodes)
// 5. (load "DBPedia.scm")
// 6. (pm-select-subset-for-DBpedia)
void PatternMiner::loandAllDBpediaKeyNodes()
{
    originalAtomSpace->get_handles_by_type(back_inserter(allDBpediaKeyNodes), CONCEPT_NODE);
    cout << allDBpediaKeyNodes.size() << " nodes loaded!" << std::endl;
}

void PatternMiner::selectSubsetForDBpedia()
{
    HandleSet subsetLinks;

    // Handle newPredicate = originalAtomSpace->add_node(PREDICATE_NODE, "position");

    string titles[] = {"president","chairman","vicePresident","primeMinister","vicePrimeMinister"};

    cout << "\nselecting president related links from DBpedia ... " << std::endl;
    // int x = 0;
    for (Handle h : allDBpediaKeyNodes)
    {
//        string objname = h->getName();
//        if (objname == "Stanley_Crooks")
//        {
//            x ++;
//            cout << x << std::endl;
//        }


        bool is_person = false;
        bool already_check_is_person = false;

        HandleSeq listLinks;
        h->getIncomingSet(back_inserter(listLinks));

        for (Handle l : listLinks)
        {
            // only process ListLinks with h as the first outgoing
            if ( l->getOutgoingAtom(0) != h)
                continue;

            HandleSeq evals;
            l->getIncomingSet(back_inserter(evals));


            for (Handle eval : evals)
            {

//                // if the valueNode only has one connection, do not keep it
//                Handle valueNode = l->getOutgoingAtom(1);
//                if (valueNode->getIncomingSetSize() < 2)
//                {
//                    cout << valueNode->getName() << " only has one connection, skip it!" << std::endl;
//
//                    continue;
//                }

                Handle predicate = eval->getOutgoingAtom(0);
                string predicateStr = predicate->getName();

                if (use_keyword_black_list && isIgnoredContent(predicateStr))
                {
                    continue;
                }

                bool skip = false;
                for (string title : titles)
                {
                    if (predicateStr == title)
                    {
                        // if the object has a birthplace, it means that it is a person, do not keep it; only keep the orgs
                        if (! already_check_is_person)
                        {

                            is_person = checkIfObjectIsAPerson(h, listLinks);
                            already_check_is_person = true;
                        }


                        if (is_person)
                            skip = true;
                        else
                        {
                            cout << h->getName() << " is a not a person, keep it!\n";
                        }

                        break;


//                        Handle titlelistLink = originalAtomSpace->add_link(LIST_LINK, valueNode, predicate);
//                        originalAtomSpace->add_link(EVALUATION_LINK, newPredicate, titlelistLink);
//                        originalAtomSpace->remove_atom(eval);

                    }
                }

                if ( (! skip) && (subsetLinks.find(eval) == subsetLinks.end()))
                    subsetLinks.insert(eval);

            }
        }



    }

    cout << "\nDone!" << subsetLinks.size() << " links selected! Writing to file ..." << std::endl;

    ofstream subsetFile;

    string fileName = "DBPediaSubSet.scm";

    subsetFile.open(fileName.c_str());

    // write the first line to enable unicode
    subsetFile <<  "(setlocale LC_CTYPE \"\")" << std::endl ;

    for (Handle h : subsetLinks)
    {
        subsetFile << h->toShortString();
    }

    subsetFile.close();

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl ;

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

void PatternMiner::testPatternMatcher()
{
    HandleSeq allAtomSpaceLinks;
    originalAtomSpace->get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
    std::cout <<"Debug: PatternMiner total link number = "
              << allAtomSpaceLinks.size() << std::endl;

    HandleSeq variableNodes, bindLinkOutgoings, patternToMatch;

    Handle varHandle1 = originalAtomSpace->add_node(opencog::VARIABLE_NODE,"$var_1" );
//    Handle varHandle2 = originalAtomSpace->add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var_2" );
//    Handle varHandle3 = originalAtomSpace->add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var_3" );
//    Handle varHandle4 = originalAtomSpace->add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var_4" );

    variableNodes.push_back(varHandle1);

//    variableNodes.push_back(varHandle2);
//    variableNodes.push_back(varHandle3);
//    variableNodes.push_back(varHandle4);

//    (BindLink
//      (VariableList
//          (VariableNode "$var_1")
//      )
//        (AndLink
//          (InheritanceLink (stv 1.000000 1.000000)
//            (VariableNode "$var_1") ; [542779886756298254][1]
//            (ConceptNode "is_NOTFC4_0906_IsHandicapped") ; [1081512319175429390][1]
//          ) ; [10229665919725499268][1]
//          (EvaluationLink (stv 1.000000 1.000000)
//            (PredicateNode "has_FC4_0906_SicknessDayOfWeek") ; [4132443157726361136][1]
//            (ListLink (stv 1.000000 1.000000)
//              (VariableNode "$var_1") ; [542779886756298254][1]
//              (ConceptNode "Monday") ; [5669163256683509528][1]
//            ) ; [14759843185427703993][1]
//          ) ; [14331675976334620512][1]
//        ) ; [11278096353514021003][1]
//      (ListLink
//          (VariableNode "$var_1")
//      )
//    )

    // The  EvaluationLink
    HandleSeq listlinkOutgoings1, evalLinkOutgoings1;
    Handle conceptNode = originalAtomSpace->add_node(opencog::CONCEPT_NODE, "Male" );
    listlinkOutgoings1.push_back(varHandle1);
    listlinkOutgoings1.push_back(conceptNode);

    Handle listlink1 = originalAtomSpace->add_link(LIST_LINK, listlinkOutgoings1);
    Handle PredicateNode = originalAtomSpace->add_node(opencog::PREDICATE_NODE, "has_FC4_0906_Gender" );
    evalLinkOutgoings1.push_back(PredicateNode);
    evalLinkOutgoings1.push_back(listlink1);
    Handle evalLink1 = originalAtomSpace->add_link(EVALUATION_LINK, evalLinkOutgoings1);

    HandleSeq listlinkOutgoings2, evalLinkOutgoings2;
    Handle conceptNode2 = originalAtomSpace->add_node(opencog::CONCEPT_NODE, "Larex Personeelsbemiddeling" );
    listlinkOutgoings2.push_back(varHandle1);
    listlinkOutgoings2.push_back(conceptNode2);

    Handle listlink2 = originalAtomSpace->add_link(LIST_LINK, listlinkOutgoings2);
    Handle PredicateNode2 = originalAtomSpace->add_node(opencog::PREDICATE_NODE, "has_FC4_0906_EmployerName" );
    evalLinkOutgoings2.push_back(PredicateNode2);
    evalLinkOutgoings2.push_back(listlink2);
    Handle evalLink2 = originalAtomSpace->add_link(EVALUATION_LINK, evalLinkOutgoings2);


//    // The InheritanceLink
//    HandleSeq inherOutgoings;
//    Handle NotHandicappedNode = originalAtomSpace->add_node(opencog::CONCEPT_NODE, "soda drinker" );
//    inherOutgoings.push_back(varHandle1);
//    inherOutgoings.push_back(NotHandicappedNode);
//    Handle inherLink = originalAtomSpace->add_link(INHERITANCE_LINK, inherOutgoings);

    patternToMatch.push_back(evalLink1);
    patternToMatch.push_back(evalLink2);

    Handle hAndLink = originalAtomSpace->add_link(AND_LINK, patternToMatch);

    //Handle resultList = originalAtomSpace->add_link(LIST_LINK, variableNodes);

    // add variable atoms
    Handle hVariablesListLink = originalAtomSpace->add_link(VARIABLE_LIST, variableNodes);

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hAndLink);
    bindLinkOutgoings.push_back(hAndLink);
    Handle hBindLink = originalAtomSpace->add_link(BIND_LINK, bindLinkOutgoings);

    std::cout <<"Debug: PatternMiner::testPatternMatcher for pattern:" << std::endl
              << hBindLink->toShortString().c_str() << std::endl;

    // Run pattern matcher
    Handle hResultListLink = bindlink(originalAtomSpace, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

    std::cout << toString(resultSet.size())  << " instances found:" << std::endl ;

    originalAtomSpace->remove_atom(hResultListLink);

    for (Handle resultLink : resultSet)
        originalAtomSpace->remove_atom(resultLink);

    //debug
    std::cout << hResultListLink->toShortString() << std::endl  << std::endl;

    originalAtomSpace->remove_atom(hBindLink);
    // originalAtomSpace->remove_atom(resultList);
    originalAtomSpace->remove_atom(hVariablesListLink);
    originalAtomSpace->remove_atom(hAndLink);
    originalAtomSpace->remove_atom(evalLink1);
    originalAtomSpace->remove_atom(evalLink2);
    originalAtomSpace->remove_atom(listlink1);
    originalAtomSpace->remove_atom(listlink2);


    allAtomSpaceLinks.clear();
    originalAtomSpace->get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
    std::cout <<"After Pattern Matcher: PatternMiner total link number = "
              << allAtomSpaceLinks.size() << std::endl;
}


HandleSet PatternMiner::_getAllNonIgnoredLinksForGivenNode(Handle keywordNode, HandleSet& allSubsetLinks)
{
    HandleSet newHandles;
    IncomingSet incomings = keywordNode->getIncomingSet(originalAtomSpace);

    // cout << "\n " << incomings.size() << " incomings found for keyword: " << keywordNode->toShortString() << std::endl;
    for (LinkPtr incomingPtr : incomings)
    {
        Handle incomingHandle = incomingPtr->getHandle();
        Handle newh = incomingHandle;

        // if this atom is a igonred type, get its first parent that is not in the igonred types
        if (use_linktype_black_list && isIgnoredType (newh->getType()) )
        {
            newh = getFirstNonIgnoredIncomingLink(originalAtomSpace, newh);

            if ((newh == Handle::UNDEFINED))
                continue;
        }
        else if (use_linktype_white_list && (! isTypeInList(newh->getType(), linktype_white_list)))
        {
            continue;
        }

        if (use_keyword_black_list)
        {
            // if the content in this link contains content in the black list,ignore it
            if (keyword_black_logic_is_contain)
            {
                if (containIgnoredContent(newh))
                    continue;
            }
            else
            {
                if (doesLinkContainNodesInKeyWordNodes(newh, black_keyword_Handles))
                    continue;
            }
        }

        if (allSubsetLinks.find(newh) == allSubsetLinks.end())
            newHandles.insert(newh);

    }

    return newHandles;
}

HandleSet PatternMiner::_extendOneLinkForSubsetCorpus(HandleSet& allNewLinksLastGram, HandleSet& allSubsetLinks, HandleSet& extractedNodes)
{
    HandleSet allNewConnectedLinksThisGram;
    // only extend the links in allNewLinksLastGram. allNewLinksLastGram is a part of allSubsetLinks
    for (Handle link : allNewLinksLastGram)
    {
        // find all nodes in this link
        HandleSet allNodes;
        extractAllNodesInLink(link, allNodes, originalAtomSpace);

        for (Handle neighborNode : allNodes)
        {
            if (neighborNode->getType() == PREDICATE_NODE)
                continue;

            string content = neighborNode->getName();
            if (isIgnoredContent(content))
                continue;

            if (extractedNodes.find(neighborNode) != extractedNodes.end())
                continue;
            else
                extractedNodes.insert(neighborNode);

            HandleSet newConnectedLinks;
            newConnectedLinks = _getAllNonIgnoredLinksForGivenNode(neighborNode, allSubsetLinks);
            allNewConnectedLinksThisGram.insert(newConnectedLinks.begin(),newConnectedLinks.end());
            allSubsetLinks.insert(newConnectedLinks.begin(),newConnectedLinks.end());
        }

    }

    return allNewConnectedLinksThisGram;
}

// allSubsetLinks is  output
void PatternMiner::findAllLinksContainKeyWords(vector<string>& subsetKeywords, unsigned int max_connection, bool logic_contain, HandleSet& allSubsetLinks)
{
    allSubsetLinks.clear();
    HandleSet extractedNodes;

    if (allLinks.size() == 0)
    {
        originalAtomSpace->get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );
    }


    if (logic_contain)
    {

        for (Handle link : allLinks)
        {
            Handle newh = link;

            // if this atom is a igonred type, get its first parent that is not in the igonred types
            if (use_linktype_black_list && isIgnoredType (link->getType()) )
            {
                newh = getFirstNonIgnoredIncomingLink(originalAtomSpace, link);

                if ((newh == Handle::UNDEFINED))
                    continue;
            }
            else if (use_linktype_white_list && (! isTypeInList(link->getType(), linktype_white_list)))
            {
                continue;
            }


            if (use_keyword_black_list)
            {
                // if the content in this link contains content in the black list,ignore it
                if (keyword_black_logic_is_contain)
                {
                    if (containIgnoredContent(newh))
                        continue;
                }
                else
                {
                    if (doesLinkContainNodesInKeyWordNodes(newh, black_keyword_Handles))
                        continue;
                }
            }

            if (allSubsetLinks.find(newh) == allSubsetLinks.end())
            {
                if (containKeywords(newh->toShortString(), subsetKeywords, QUERY_LOGIC::OR))
                    allSubsetLinks.insert(newh);
//                else
//                {
//                    if (only_mine_patterns_start_from_white_list)
//                    {
//                        // add this Link into the observingAtomSpace
//                        HandleSeq outgoingLinks, outVariableNodes;

//                        swapOneLinkBetweenTwoAtomSpace(originalAtomSpace, observingAtomSpace, newh, outgoingLinks, outVariableNodes);
//                        Handle newLink = observingAtomSpace->add_link(newh->getType(), outgoingLinks);
//                        newLink->setTruthValue(newh->getTruthValue());
//                        linkNumLoadedIntoObservingAtomSpace ++;
//                    }
//                }
            }
        }


    }
    else
    {
        for (string keyword : subsetKeywords)
        {
            // std::cout << keyword << std::endl;
            Handle keywordNode = originalAtomSpace->get_node(opencog::CONCEPT_NODE,keyword);
            if (keywordNode == Handle::UNDEFINED)
                keywordNode = originalAtomSpace->get_node(opencog::PREDICATE_NODE,keyword);

            if (keywordNode == Handle::UNDEFINED)
                continue;

            if (extractedNodes.find(keywordNode) == extractedNodes.end())
                extractedNodes.insert(keywordNode);
            else
                continue;

            HandleSet newConnectedLinks = _getAllNonIgnoredLinksForGivenNode(keywordNode, allSubsetLinks);

            allSubsetLinks.insert(newConnectedLinks.begin(), newConnectedLinks.end());

        }

    }

    unsigned int order = 0;
    HandleSet allNewConnectedLinksThisGram = allSubsetLinks;

    while (order < max_connection)
    {
        allNewConnectedLinksThisGram = _extendOneLinkForSubsetCorpus(allNewConnectedLinksThisGram, allSubsetLinks, extractedNodes);
        order ++;
    }

}

void PatternMiner::selectSubsetAllEntityLinksContainsKeywords(vector<string>& subsetKeywords)
{
    std::cout << "\nSelecting a subset from loaded corpus in Atomspace for the Entities contain following value keywords." << std::endl ;
    HandleSet allSubsetLinks;
    string topicsStr = "";

    for (string keyword : subsetKeywords)
    {
        std::cout << keyword << std::endl;
        topicsStr += "-";
        topicsStr += keyword;
    }

    findAllLinksContainKeyWords(subsetKeywords, 0, false, allSubsetLinks);

    HandleSet allEntityNodes;
    for (Handle link : allSubsetLinks)
    {  
        Handle firstOutgoing = link->getOutgoingAtom(1);
        Handle entityNode;
        if (firstOutgoing->isLink())
            entityNode = firstOutgoing->getOutgoingAtom(0);
        else
            entityNode = firstOutgoing;

        if (allEntityNodes.find(entityNode) == allEntityNodes.end())
            allEntityNodes.insert(entityNode);
    }


    int allEntityNum = allEntityNodes.size();
    std::cout << allEntityNum <<" entities has the predicate value found! Now find all the other Links contain these entities ..." << std::endl;

    int processEntityNum = 0;
    for (Handle entityNode : allEntityNodes)
    {
        HandleSet allLinks = _getAllNonIgnoredLinksForGivenNode(entityNode, allSubsetLinks);

        allSubsetLinks.insert(allLinks.begin(), allLinks.end());
        processEntityNum ++;

        cout<< "\r" << ((float)(processEntityNum ))/((float)allEntityNum)*100.0f << "% completed."; // it's not liner
        std::cout.flush();

    }

    std::cout << "\n " << allSubsetLinks.size() << " Links found! Writing to file ..."  << std::endl ;

    ofstream subsetFile;

    string fileName = "SubSetEntityLinks" + topicsStr + ".scm";

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

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl ;
}

// must load the corpus before calling this function
// logic_contain = true will find all the Nodes with a label contains any of the keywords,e.g.
// keyword = Premier , Node "32nd Premier of New South Wales" will be found if logic_contain = true;
// only Node "Premier" will be found if logic_contain = false
void PatternMiner::_selectSubsetFromCorpus(vector<string>& subsetKeywords, unsigned int max_connection, bool logic_contain)
{
    std::cout << "\nSelecting a subset from loaded corpus in Atomspace for the following keywords within " << max_connection << " distance:" << std::endl ;
    HandleSet allSubsetLinks;
    string topicsStr = "";

    for (string keyword : subsetKeywords)
    {
        std::cout << keyword << std::endl;
        topicsStr += "-";
        topicsStr += keyword;
    }

    findAllLinksContainKeyWords(subsetKeywords, max_connection, logic_contain, allSubsetLinks);

    std::cout << "\n " << allSubsetLinks.size() << " Links found! Writing to file ..."  << std::endl ;

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

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl ;
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
        if (linkStr == "") continue;

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


// recursively function , normal atom format
//  (PredicateNode "country") ; [4408301568758128182][1]
//  (ListLink (stv 1.000000 1.000000)
//    (VariableNode "$var_1") ; [541152609547258039][2]
//    (ConceptNode "United_States") ; [6203771557970593342][1]
//  ) ; [15240751338816460712][2]
//) ; [14692539656916913941][2]
bool PatternMiner::loadOutgoingsIntoAtomSpaceFromAtomString(stringstream& outgoingStream, AtomSpace *_atomSpace, HandleSeq &outgoings, string parentIndent)
{
    string line;
    string curIndent = parentIndent + LINE_INDENTATION;

    while(getline(outgoingStream, line))
    {

        std::size_t nonIndentStartPos = line.find("(");
        if (nonIndentStartPos == string::npos)
            return true;
        string indent = line.substr(0, nonIndentStartPos);
        string nonIndentSubStr = line.substr(nonIndentStartPos + 1);
        std::size_t typeEndPos = nonIndentSubStr.find(" ");
        string atomTypeStr = nonIndentSubStr.substr(0, typeEndPos);
        string linkOrNodeStr = atomTypeStr.substr(atomTypeStr.size() - 4, 4);
        Type atomType = classserver().getType(atomTypeStr);
        if (NOTYPE == atomType)
        {
            cout << "Warning: loadOutgoingsIntoAtomSpaceFromAtomString: Not a valid typename: " << atomTypeStr << std::endl;
            return false;

        }

        if (indent == curIndent)
        {
            if (linkOrNodeStr == "Node")
            {
                std::size_t nodeNameEndPos = nonIndentSubStr.find_last_of("\"");
                string nodeName = nonIndentSubStr.substr(typeEndPos + 2, nodeNameEndPos - typeEndPos - 2);
                Handle node = _atomSpace->add_node(atomType, nodeName);
                outgoings.push_back(node);
            }
            else if (linkOrNodeStr == "Link")
            {
                // call this function recursively
                HandleSeq childOutgoings;
                if (! loadOutgoingsIntoAtomSpaceFromAtomString(outgoingStream, _atomSpace, childOutgoings, curIndent))
                    return false;

                Handle link = _atomSpace->add_link(atomType, childOutgoings);
                outgoings.push_back(link);
            }
            else
            {
                cout << "Warning: loadOutgoingsIntoAtomSpaceFromAtomString: Not a Node, neighter a Link: " << linkOrNodeStr << std::endl;
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
            cout << "Warning: loadOutgoingsIntoAtomSpaceFromAtomString: Indent wrong: " << line << std::endl;
            return false;


        }

    }

    return true;
}

// the input patternStr is in the format of normal Atom format, e.g.:
//(EvaluationLink (stv 1.000000 1.000000)
//  (PredicateNode "country") ; [4408301568758128182][1]
//  (ListLink (stv 1.000000 1.000000)
//    (VariableNode "$var_1") ; [541152609547258039][2]
//    (ConceptNode "United_States") ; [6203771557970593342][1]
//  ) ; [15240751338816460712][2]
//) ; [14692539656916913941][2]
//(EvaluationLink (stv 1.000000 1.000000)
//  (PredicateNode "governmentType") ; [7065069092094160337][1]
//  (ListLink (stv 1.000000 1.000000)
//    (VariableNode "$var_2") ; [1433406047102624628][2]
//    (ConceptNode "Mayor-council_government") ; [6067926743140439366][1]
//  ) ; [16879153852749076749][2]
//) ; [11770550072390833013][2]
//(EvaluationLink (stv 1.000000 1.000000)
//  (PredicateNode "isPartOf") ; [2521175578444903070][1]
//  (ListLink (stv 1.000000 1.000000)
//    (VariableNode "$var_2") ; [1433406047102624628][2]
//    (VariableNode "$var_1") ; [541152609547258039][2]
//  ) ; [11352379719155895422][2]
//) ; [13092614614903350611][2]
HandleSeq PatternMiner::loadPatternIntoAtomSpaceFromFileString(string patternStr, AtomSpace *_atomSpace)
{

    std::vector<std::string> strs;
    boost::algorithm::split_regex( strs, patternStr, boost::regex( "\n\\)\n" ) ) ;

    HandleSeq pattern;

    for (string linkStr : strs) // load each link
    {
        if (linkStr == "") continue;

        HandleSeq rootOutgoings;

        std::size_t firstLineEndPos = linkStr.find("\n"); //(EvaluationLink (stv 1.000000 1.000000)\n
        std::string rootOutgoingStr = linkStr.substr(firstLineEndPos + 1);
        stringstream outgoingStream(rootOutgoingStr);

        if (! loadOutgoingsIntoAtomSpaceFromAtomString(outgoingStream, _atomSpace, rootOutgoings))
        {
            cout << "Warning: loadPatternIntoAtomSpaceFromFileString: Parse pattern string error: " << linkStr << std::endl;
            HandleSeq emptyPattern;
            return emptyPattern;
        }

        std::size_t typeEndPos = linkStr.find(" ");
        string atomTypeStr = linkStr.substr(1, typeEndPos - 2);
        string linkOrNodeStr = atomTypeStr.substr(atomTypeStr.size() - 4, 4);

        if (linkOrNodeStr != "Link")
        {

            cout << "Warning: loadPatternIntoAtomSpaceFromFileString: Not a Link: " << linkOrNodeStr << std::endl;
            HandleSeq emptyPattern;
            return emptyPattern;

        }

        Type atomType = classserver().getType(atomTypeStr);
        if (NOTYPE == atomType)
        {

            cout << "Warning: loadPatternIntoAtomSpaceFromFileString: Not a valid typename: " << atomTypeStr << std::endl;
            HandleSeq emptyPattern;
            return emptyPattern;

        }

        Handle rootLink = _atomSpace->add_link(atomType, rootOutgoings);
        pattern.push_back(rootLink);
    }

    // debug:
//     static int pattern_num = 0;
//     string patternToStr = "";

//     for(Handle h : pattern)
//     {
//        patternToStr += h->toShortString();
//        patternToStr += "\n";
//     }

//     cout << "\nAdded pattern: NO." << pattern_num << "\n" << patternToStr;
//     pattern_num ++;

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

    string patternStr = "";
    int frequency = 0;
    float surprisingnessI = -9999999.99f;
    float surprisingnessII = -9999999.99f;
    float interactioninformation = -9999999.99f;
    bool hasSurprisingnessI = false;
    bool hasSurprisingnessII = false;
    bool hasInteractionInformation = false;

    string lastLine = "";
    unsigned int loadedPatternNum = 0;
    bool patternStart = false;

    for (std::string line; std::getline(resultFile, line); )
    {
        //cout <<"\nline: " << line << std::endl;
        if (patternStart && (line == "") && (lastLine == "")) // one pattern end, load it
        {

            // add this new found pattern into the Atomspace
            HandleSeq patternHandleSeq = loadPatternIntoAtomSpaceFromFileString(patternStr, atomSpace);

            if (patternHandleSeq.size() == 0)
            {

                cout << "Warning: Invalid pattern string: " << patternStr << std::endl;
                return;

            }

            // create a new HTreeNode
            HTreeNode* newHTreeNode = new HTreeNode();
            newHTreeNode->pattern = patternHandleSeq;
            newHTreeNode->count = frequency;

            if (hasSurprisingnessI)
                newHTreeNode->nI_Surprisingness = surprisingnessI;

            if (hasSurprisingnessII)
                newHTreeNode->nII_Surprisingness = surprisingnessII;

            if (hasInteractionInformation)
                newHTreeNode->interactionInformation = interactioninformation;

            keyStrToHTreeNodeMap.insert(std::pair<string, HTreeNode*>(patternStr, newHTreeNode));
            (patternsForGram[patternHandleSeq.size()-1]).push_back(newHTreeNode);
            loadedPatternNum ++;
            patternStr = "";
            patternStart = false;
        }
        else if (line.find("Pattern:") != string::npos) // one pattern start
        {
            frequency = 0;

            int frequencyStart = line.find("Frequency = ") + 12;
            string frequencyStr = line.substr(frequencyStart, line.size() - frequencyStart);
            frequency = atoi(frequencyStr.c_str());

            if (line.find("SurprisingnessI = ") != string::npos)
            {
                int surprisingnessIStart = line.find("SurprisingnessI = ") + 18;
                string surprisingnessIStr = line.substr(surprisingnessIStart, line.size() - surprisingnessIStart);
                surprisingnessI = atof(surprisingnessIStr.c_str());
                hasSurprisingnessI = true;
            }
            else
                hasSurprisingnessI = false;

            if (line.find("SurprisingnessII = ") != string::npos)
            {
                int surprisingnessIIStart = line.find("SurprisingnessII = ") + 19;
                string surprisingnessIIStr = line.substr(surprisingnessIIStart, line.size() - surprisingnessIIStart);
                surprisingnessII = atof(surprisingnessIIStr.c_str());
                hasSurprisingnessII = true;
            }
            else
                hasSurprisingnessII = false;

            if (line.find("InteractionInformation = ") != string::npos)
            {
                int interactionInformationStart = line.find("InteractionInformation = ") + 25;
                string interactionInformationStr = line.substr(interactionInformationStart, line.size() - interactionInformationStart);
                interactioninformation = atof(interactionInformationStr.c_str());
                hasInteractionInformation = true;
            }
            else
                hasInteractionInformation = false;

            patternStart = true;
        }
        else if (patternStart)// in the middle of one pattern
        {
            if (line != "")
            {

                if (line.find(";") != string::npos)
                {
                    string subline = line.substr(0, line.find_last_of(";") - 1);
                    patternStr += (subline + "\n");

                }
                else if (line.find("(stv") != string::npos)
                {
                    string subline = line.substr(0, line.find("(stv ") - 1);
                    patternStr += (subline + "\n");
                }
                else
                    patternStr += (line + "\n");

            }
        }

        lastLine = line;
    }

    resultFile.close();

    std::cout << "\nDone! " << loadedPatternNum <<  " patterns loaded in total!" << std::endl ;

}


