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

#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm/find.hpp>
#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>

#include <opencog/util/algorithm.h>

#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/proto/atom_types.h>
#include <opencog/query/BindLinkAPI.h>

#include "PatternMiner.h"

using namespace opencog::PatternMining;
using namespace opencog;


void PatternMiner::generateIndexesOfSharedVars(const Handle& link, const HandleSeq& orderedHandles, vector<vector<std::pair<int,std::size_t>>>& indexes)
{
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
        {
            if (h->get_type() == opencog::PATTERN_VARIABLENODE_TYPE)
            {
                string var_name = h->get_name();

                vector<std::pair<int, std::size_t>> indexesForCurVar; // vector<handleindex,varposintthehandle>
                int index = 0;

                for (const Handle& oh : orderedHandles)
                {
                    string ohStr = oh->to_short_string();
                    std::size_t pos = ohStr.find(var_name) ;
                    if (pos != std::string::npos)
                    {
                        indexesForCurVar.emplace_back(index, pos);
                    }

                    index++;
                }

                indexes.push_back(indexesForCurVar);
            }
        }
        else
            generateIndexesOfSharedVars(h, orderedHandles, indexes);
    }
}

HandleSeq PatternMiner::findAndRenameVariables(const Handle& link, HandleMap& varNameMap,
                                               const std::map<Handle, Type>& orderedTmpLinkToType)
{
    HandleSeq renameOutgoingLinks;
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
        {
           if (h->get_type() == opencog::PATTERN_VARIABLENODE_TYPE)
           {
               // It's a variable node, rename it if not in varNameMap
               if (varNameMap.find(h) != varNameMap.end())
               {
                   renameOutgoingLinks.push_back(varNameMap[h]);
               }
               else
               {
                   // TODO move this in its own method. Why is it
                   // different that the one in
                   // PatternMiner::associateNodesToVars with $var~ ?
                   string var_name = "$var_"  + toString(varNameMap.size() + 1);
                   Handle var_node = as->add_node(opencog::PATTERN_VARIABLENODE_TYPE, var_name);

                   varNameMap.insert(HandlePair(h, var_node));
                   renameOutgoingLinks.push_back(var_node);
               }
           }
           else
           {
               // It's a const node, just add it
               renameOutgoingLinks.push_back(h);
           }
        }
        else
        {
             HandleSeq _renameOutgoingLinks =
                 findAndRenameVariables(h, varNameMap, orderedTmpLinkToType);

             Handle reLink;

             // TODO probably need to re-enable that

//             if (enable_unify_unordered_links && not orderedTmpLinkToType.empty())
//             {

//                std::map<Handle, Type>::const_iterator typeIt = orderedTmpLinkToType.find(h);
//                if (typeIt != orderedTmpLinkToType.end())
//                    reLink = as->add_link(typeIt->second, _renameOutgoingLinks);
//                else
//                    reLink = as->add_link(h->get_type(),_renameOutgoingLinks);
//             }
//             else
             reLink = as->add_link(h->get_type(), _renameOutgoingLinks);

             renameOutgoingLinks.push_back(reLink);
        }
    }
    return renameOutgoingLinks;
}

HandleSeq PatternMiner::RebindVariableNames(const HandleSeq& orderedPattern, HandleMap& orderedVarNameMap, const std::map<Handle,Type>& orderedTmpLinkToType)
{
    HandleSeq rebindedPattern;

    for (const Handle& h : orderedPattern)
    {
        HandleSeq renameOutgoingLinks =
	        findAndRenameVariables(h, orderedVarNameMap, orderedTmpLinkToType);

        Handle reLink;

        // TODO probably need to re-enable that (either here or in
        // findAndRenameVariables)

//        if (enable_unify_unordered_links && not orderedTmpLinkToType.empty())
//        {

//           std::map<Handle,Type>::iterator typeIt = orderedTmpLinkToType.find(h);
//           if (typeIt != orderedTmpLinkToType.end())
//               reLink = as->add_link((Type)(typeIt->second),renameOutgoingLinks);
//           else
//               reLink = as->add_link(h->get_type(),renameOutgoingLinks);
//        }
//        else
        reLink = as->add_link(h->get_type(), renameOutgoingLinks);

        rebindedPattern.push_back(reLink);
    }

    return rebindedPattern;
}

void PatternMiner::ReplaceConstNodeWithVariableForOneLink(Handle link, Handle constNode, Handle newVariableNode, HandleSeq& renameOutgoingLinks)
{
    for (const Handle& h : link->getOutgoingSet())
    {

        if (h->is_node())
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
             Handle reLink = as->add_link(h->get_type(),_renameOutgoingLinks);

             renameOutgoingLinks.push_back(reLink);
        }
    }
}

HandleSeq PatternMiner::ReplaceConstNodeWithVariableForAPattern(const HandleSeq& pattern, Handle constNode, Handle newVariableNode)
{

    HandleSeq rebindedPattern;

    for (const Handle& link : pattern)
    {
        HandleSeq renameOutgoingLinks;
        ReplaceConstNodeWithVariableForOneLink(link, constNode, newVariableNode, renameOutgoingLinks);

        Handle rebindedLink = as->add_link(link->get_type(),renameOutgoingLinks);

        rebindedPattern.push_back(rebindedLink);
    }

    return rebindedPattern;
}

HandleSeq PatternMiner::UnifyPatternOrder(const HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex, HandleMap& orderedVarNameMap)
{
    HandleSeq orderedHandles;

    std::map<Handle, Type> orderedTmpLinkToType;

    if (param.enable_unify_unordered_links)
    {
        // check for unordered links and unify them first

        HandleSeq orderedOutgoings;
        for (const Handle& link : inputPattern)
        {
            Handle reLink = UnifyOneLinkForUnorderedLink(link, orderedTmpLinkToType);
            orderedOutgoings.push_back(reLink);
        }

        orderedHandles = _UnifyPatternOrder(orderedOutgoings, unifiedLastLinkIndex);
    }
    else
        orderedHandles = _UnifyPatternOrder(inputPattern, unifiedLastLinkIndex);

    HandleSeq rebindPattern = RebindVariableNames(orderedHandles, orderedVarNameMap, orderedTmpLinkToType);

    return rebindPattern;
}

HandleSeq PatternMiner::_UnifyPatternOrder(const HandleSeq& inputPattern, unsigned int& unifiedLastLinkIndex)
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

    // TODO: Instead of converting into a string, why not create a
    // new pattern where the VariableNode name are empty?
    for (const Handle& inputH : inputPattern)
    {
        stringstream stream(inputH->to_short_string());
        string nonVarNameString;
        string oneLine;

        while (getline(stream, oneLine,'\n'))
        {
            if (oneLine.find("VariableNode") == string::npos)
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

        nonVarStrToHandleMap.insert({nonVarNameString, inputH});
    }

    // Step 2: sort the order of all the handles do not share the same string key with other handles
    // because the strings are put into a map, so they are already sorted.
    // now print the Handles that do not share the same key with other Handles into a vector, left the Handles share the same keys

    HandleSeq orderedHandles;
    vector<string> duplicateStrs;
    multimap<string, Handle>::iterator it;
    for (it = nonVarStrToHandleMap.begin(); it != nonVarStrToHandleMap.end();)
    {
        int count = nonVarStrToHandleMap.count(it->first);
        if (count == 1)
        {
            // if this key string has only one record, just put the corresponding handle to the end of orderedHandles
            orderedHandles.push_back(it->second);
            ++it;
        }
        else
        {
            // this key string has multiple handles to it, not put these handles into the orderedHandles,
            // instead put this key string into duplicateStrs
            duplicateStrs.push_back(it->first);
            it = nonVarStrToHandleMap.upper_bound(it->first);
        }

    }

    // Step 3: sort the order of the handles share the same string key with other handles
    for (const string& keyString : duplicateStrs)
    {
        // get all the corresponding handles for this key string
        multimap<string, Handle>::iterator kit;
        vector<_non_ordered_pattern> sharedSameKeyPatterns;
        for (kit = nonVarStrToHandleMap.lower_bound(keyString); kit != nonVarStrToHandleMap.upper_bound(keyString); ++kit)
        {
            _non_ordered_pattern p;
            p.link = kit->second;
            generateIndexesOfSharedVars(p.link, orderedHandles, p.indexesOfSharedVars);
            sharedSameKeyPatterns.push_back(p);
        }

        boost::sort(sharedSameKeyPatterns);
        for (const _non_ordered_pattern& np : sharedSameKeyPatterns)
        {
            orderedHandles.push_back(np.link);
        }
    }

    // find out where the last link in the input pattern is now in the ordered pattern
    Handle lastLink = inputPattern.back();
    unsigned int lastLinkIndex = 0;
    for (const Handle& h : orderedHandles)
    {
        if (h == lastLink)
        {
            unifiedLastLinkIndex = lastLinkIndex;
            break;
        }

        ++lastLinkIndex;
    }

    return orderedHandles;
}


// This function should only be called when enable_unify_unordered_links = true
// when a link in a pattern is unordered type, need to unify it, the order and var names
// they could be nested, so they need to be unify recursively, e.g.:
//SetLink
//   AndLink
//      AndLink
//      ListLink
//   AndLink
//   ListLink
Handle PatternMiner::UnifyOneLinkForUnorderedLink(const Handle& link,
                                                  std::map<Handle, Type>& orderedTmpLinkToType)
{
    HandleSeq outputOutgoingLinks;
    bool containNodes = false;

    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
        {
            // it's a node, just add it
            outputOutgoingLinks.push_back(h);
            containNodes = true;
        }
	    else
	    {
            Handle reLink = UnifyOneLinkForUnorderedLink(h, orderedTmpLinkToType);
            outputOutgoingLinks.push_back(reLink);
        }
    }

    Handle returnLink;

    Type originalType = link->get_type();
    if (nameserver().isA(originalType, UNORDERED_LINK))
    {
        unsigned int unifiedLastLinkIndex;
        HandleSeq orderedOutgoings;
        // check if there are only Links in the outgoings
        if (containNodes)
        {
            HandleSeq outgoingLinksTobeUnified;
            HandleSeq nodesInOutgoings;
            // if it also contain Nodes, then only sort the Links, leave all the Nodes in the top of the list
            for (const Handle& h1 : outputOutgoingLinks)
            {
                if (h1->is_link())
                    outgoingLinksTobeUnified.push_back(h1);
                else
                    nodesInOutgoings.push_back(h1);
            }

            orderedOutgoings = _UnifyPatternOrder(outgoingLinksTobeUnified, unifiedLastLinkIndex);
            orderedOutgoings.insert(orderedOutgoings.begin(), nodesInOutgoings.begin(), nodesInOutgoings.end());
        }
        else
            orderedOutgoings = _UnifyPatternOrder(outputOutgoingLinks, unifiedLastLinkIndex);

        // change the original unordered type into a tmp ListLink
        returnLink = as->add_link(LIST_LINK, orderedOutgoings);

        orderedTmpLinkToType.insert({returnLink, originalType});
    }
    else
        returnLink = as->add_link(originalType, outputOutgoingLinks);

    return returnLink;

}

string PatternMiner::unifiedPatternToKeyString(const HandleSeq& inputPattern)
{
    string keyStr = "";
    for (const Handle& h : inputPattern)
    {
        keyStr += Link2keyString(h,"");
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
    for (; i < n_max - 1; ++i)
    {
        if (indexes[i])
        {
            ++trueCount;

            if (! indexes[i+1])
                break;
        }
    }

    indexes[i] = false;
    indexes[i+1] = true;

    for (int j = 0; j < trueCount; ++j)
        indexes[j] = true;

    for (int j = trueCount; j < i; ++j)
        indexes[j] = false;

}

bool PatternMiner::isLastNElementsAllTrue(bool* array, int size, int n)
{
    for (int i = size - 1; i >= size - n; i--)
    {
        if (! array[i])
            return false;
    }

    return true;
}

// TODO replace by stl or boost equivalent
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

Handle PatternMiner::substitute(const Handle& h, const HandleMap& h2h)
{
    // If h in in h2h, its associated handle is returned
    auto it = h2h.find(h);
    if (it != h2h.end())
        return as->add_atom(it->second);

    // Otherwise if h is a node, h itself is returned
    if (h->is_node())
        return h;

    // Recursively substitute its outgoings and reconstruct
    HandleSeq nout;
    for (const Handle& ch : h->getOutgoingSet())
        nout.push_back(substitute(ch, h2h));
    Handle nh = as->add_link(h->get_type(), nout);
    nh->copyValues(h);
    return nh;
}

void PatternMiner::associateNodesToVars(const Handle& link, HandleMap& nodesToVars)
{
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
        {
            if (nodesToVars.find(h) == nodesToVars.end())
            {
                // TODO move variable name creation in its own method
                // add a variable node in Pattern miner Atomspace
                Handle varHandle = as->add_node(opencog::PATTERN_VARIABLENODE_TYPE,
                                                "$var~" + toString(nodesToVars.size()));
                nodesToVars.insert({h, varHandle});
            }

            if ((h->get_type() == opencog::PATTERN_VARIABLENODE_TYPE))
                cout << "Error: instance link contains variables: \n"
                     << h->to_short_string() <<std::endl;
        }
        else
        {
            associateNodesToVars(h, nodesToVars);
        }
    }
}

void PatternMiner::extractAllVariableNodesInAnInstanceLink(const Handle& instanceLink, const Handle& patternLink, HandleSet& allVarNodes)
{
    HandleSeq::const_iterator pit = patternLink->getOutgoingSet().begin();

    for (const Handle& h : instanceLink->getOutgoingSet())
    {
        if (h->is_node())
        {
            if (((*pit)->get_type() == opencog::PATTERN_VARIABLENODE_TYPE))
            {
	            allVarNodes.insert(h);
            }
        }
        else
        {
            extractAllVariableNodesInAnInstanceLink(h, *pit, allVarNodes);
        }

        ++pit;
    }
}

void PatternMiner::extractNodes(const Handle& link, HandleSet& nodes)
{
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
	        nodes.insert(h);
        else
            extractNodes(h, nodes);
    }
}

void PatternMiner::extractVarNodes(const Handle& link, HandleSet& varNodes)
{
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
        {
            if (h->get_type() == opencog::PATTERN_VARIABLENODE_TYPE)
                varNodes.insert(h);
        }
        else
        {
            extractVarNodes(h, varNodes);
        }
    }
}

void PatternMiner::extractConstNodes(const Handle& link, HandleSet& constNodes)
{
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
        {
            if (h->get_type() != opencog::PATTERN_VARIABLENODE_TYPE)
                constNodes.insert(h);
        }
        else
        {
            extractConstNodes(h, constNodes);
        }
    }
}

bool PatternMiner::containOnlyVariables(Handle h)
{
    if (h->is_node())
        return h->get_type() == PATTERN_VARIABLENODE_TYPE;

    return all_of(h->getOutgoingSet(), [&](const Handle& ch) {
            return containOnlyVariables(ch); });
}

bool PatternMiner::containSomeVariables(Handle h)
{
    if (h->is_node())
        return h->get_type() == PATTERN_VARIABLENODE_TYPE;

    return any_of(h->getOutgoingSet(), [&](const Handle& ch) {
            return containSomeVariables(ch); });
}


HandleSeq PatternMiner::copyOutgoings(AtomSpace& to_as, const Handle& link,
                                      HandleSeq& variables)
{
    HandleSeq outgoings;
    for (const Handle& h : link->getOutgoingSet())
        outgoings.push_back(copyAtom(to_as, h, variables));
    return outgoings;
}

HandleSeq PatternMiner::copyLinks(AtomSpace& to_as, const HandleSeq& links,
                                  HandleSeq& variables)
{
    HandleSeq outPutLinks;
    for (const Handle& link : links)
    {
        HandleSeq outgoingLinks = copyOutgoings(to_as, link, variables);
        Handle toLink = to_as.add_link(link->get_type(), outgoingLinks);
        toLink->setTruthValue(link->getTruthValue());
        outPutLinks.push_back(toLink);
    }
    return outPutLinks;
}

Handle PatternMiner::copyAtom(AtomSpace& to_as, const Handle& h,
                              HandleSeq& variables)
{
	if (h->is_node())
	{
		Handle new_node;

		if (h->get_type() == PATTERN_VARIABLENODE_TYPE)
		{
			new_node = to_as.add_node(VARIABLE_NODE, h->get_name());
			if (!isInHandleSeq(new_node, variables)) // avoid duplicated variable
				variables.push_back(new_node);
		}
		else
			new_node = to_as.add_node(h->get_type(), h->get_name());

		return new_node;
	}
	else
	{
		HandleSeq outgoings = copyOutgoings(to_as, h, variables);
		Handle link = to_as.add_link(h->get_type(), outgoings);
		link->setTruthValue(h->getTruthValue());
		return link;
	}
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
//    original_as.get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
//    std::cout <<"Debug: PatternMiner total link number = "
//              << allAtomSpaceLinks.size() << std::endl;

    AtomSpace& _as = original_as;

    HandleSeq  bindLinkOutgoings, variableNodes;

//  HandleSeq patternToMatch = swapLinksBetweenTwoAtomSpaceForBindLink(as, _as, HNode->pattern, variableNodes, linksWillBeDel);
    HandleSeq patternToMatch = copyLinks(_as, HNode->pattern, variableNodes);

    Handle hAndLink = _as.add_link(AND_LINK, patternToMatch);

//    // add variable atoms
//    HandleSet allVariableNodesInPattern;
//    for (const Handle& h : patternToMatch)
//    {
//        extractVarNodes(h, allVariableNodesInPattern, _as);
//    }


//    for (const Handle& varh : allVariableNodesInPattern)
//    {
//        Handle v = _as.add_node(VARIABLE_NODE, varh->get_name());
//        variableNodes.push_back(v);
//    }

    Handle hVariablesListLink = _as.add_link(VARIABLE_LIST, variableNodes);

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hAndLink); // the pattern to match
    bindLinkOutgoings.push_back(hAndLink); // the results to return

    Handle hBindLink = _as.add_link(BIND_LINK, bindLinkOutgoings);

//    std::cout << std::endl << hBindLink->to_short_string() << std::endl;

    // Run pattern matcher
    Handle hResultListLink = opencog::bindlink(&_as, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

    HNode->count = resultSet.size();

    //    //debug
//    std::cout << hResultListLink->to_short_string() << std::endl  << std::endl;

//    if (HNode->pattern.size() == 2)
//    cout << "\nRemoving hResultListLink\n" << hResultListLink->to_short_string() << std::endl;
    _as.remove_atom(hResultListLink);

//    int count = 0;
    for (const Handle& listH  : resultSet)
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
//                if (! containDuplicates(instanceLinks))
//                    HNode->instances.push_back(instanceLinks);
//            }
//        }

//        if (! containVariableNodes(listH, _as))
//            count ++;

//        if (HNode->pattern.size() == 2)
//        cout << "\nRemoving listH \n" << listH->to_short_string() << std::endl;
        _as.remove_atom(listH);
    }

//    HNode->count = count;

//     std::cout << HNode->count << " instances found!" << std::endl ;

//    if (HNode->pattern.size() == 2)
//    cout << "\nRemoving hBindLink\n" << hBindLink->to_short_string() << std::endl;
    _as.remove_atom(hBindLink);

//    if (HNode->pattern.size() == 2)
//    cout << "\nRemoving hAndLink" << hAndLink->to_short_string() << std::endl;
    _as.remove_atom(hAndLink);

//    for (Handle patternLink : linksWillBeDel) // delete the patterns links contains variables
//    {
//        _as.remove_atom(patternLink);
//    }

    for (const Handle& varh : variableNodes)
    {
        _as.remove_atom(varh, true);
    }

//    allAtomSpaceLinks.clear();
//    original_as.get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
//    std::cout <<"After: PatternMiner total link number = "
//              << allAtomSpaceLinks.size() << std::endl;

}

bool PatternMiner::containDuplicates(const HandleSeq &handles)
{
    for (unsigned int i = 0; i < handles.size(); i ++)
        for (unsigned int j = i+1; j < handles.size(); j ++)
            if (handles[j] == handles[i])
                return true;
    return false;
}

bool PatternMiner::isInHandleSeq(const Handle& h, const HandleSeq& hs)
{
    return is_in(h, hs);
}

bool PatternMiner::isInHandleSeqSeq(const Handle& h, const HandleSeqSeq& hss)
{
    return any_of(hss, [&](const HandleSeq& hs) { return is_in(h, hs); });
}

bool PatternMiner::isIgnoredType(Type type)
{
    return is_in(type, param.linktype_black_list);
}

bool PatternMiner::isTypeInList(Type type, const vector<Type> &typeList)
{
    return is_in(type, typeList);
}

bool PatternMiner::isIgnoredContent(const string& keyword)
{
    return is_in(keyword, param.keyword_black_list);
}

bool PatternMiner::doesLinkContainNodesInKeyWordNodes(const Handle& link, const HandleSet& keywordNodes)
{
    for (const Handle& h : link->getOutgoingSet())
    {
        if (h->is_node())
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

bool PatternMiner::containIgnoredContent(Handle link)
{
    string str = link->to_short_string();

    for (const string& ignoreWord : param.keyword_black_list)
    {
        string ignoreStr = "\"" + ignoreWord + "\"";
        if (str.find(ignoreStr) != std::string::npos)
            return true;
    }

    return false;
}

Handle PatternMiner::getFirstNonIgnoredIncomingLink(AtomSpace& atomspace, const Handle& handle)
{
    Handle cur_h = handle;
    while(true)
    {
        IncomingSet incomings = cur_h->getIncomingSet(&atomspace);
        if (incomings.empty())
            return Handle::UNDEFINED;

        Handle incomingHandle = (incomings[0])->get_handle();
        if (isIgnoredType (incomingHandle->get_type()))
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
    return node1->count > node2->count;
}

bool compareHTreeNodeByInteractionInformation(HTreeNode* node1, HTreeNode* node2)
{
    return node1->interactionInformation > node2->interactionInformation;
}

bool compareHTreeNodeBySurprisingness(HTreeNode* node1, HTreeNode* node2)
{
    float s1 = node1->nI_Surprisingness + node1->nII_Surprisingness,
        s2 = node2->nI_Surprisingness + node2->nII_Surprisingness,
        diff = s1 - s2;
    if (FLOAT_MIN_DIFF < diff)
        return true;
    else if (FLOAT_MIN_DIFF < -diff)
        return false;

    return node1->var_num < node2->var_num;
}

bool compareHTreeNodeBySurprisingness_I(HTreeNode* node1, HTreeNode* node2)
{
    float s1 = node1->nI_Surprisingness,
        s2 = node2->nI_Surprisingness;

    if (USE_ABS_SURPRISINGNESS)
    {
        s1 = std::abs(s1);
        s2 = std::abs(s2);
    }

    float diff = s1 - s2;
    if (FLOAT_MIN_DIFF < diff)
        return true;
    else if (FLOAT_MIN_DIFF < -diff)
        return false;

    return node1->var_num < node2->var_num;
}

bool compareHTreeNodeBySurprisingness_II(HTreeNode* node1, HTreeNode* node2)
{

    if (not node1->superPatternRelations.empty() and
        not node2->superPatternRelations.empty())
    {
        float diff = node1->nII_Surprisingness - node2->nII_Surprisingness;
        if (FLOAT_MIN_DIFF < diff)
            return true;
        else if (FLOAT_MIN_DIFF < -diff)
            return false;
    }

    return node1->var_num < node2->var_num;
}

bool compareHTreeNodeBySurprisingness_b(HTreeNode* node1, HTreeNode* node2)
{
    float diff = node1->nII_Surprisingness_b - node2->nII_Surprisingness_b;
    if (FLOAT_MIN_DIFF < diff)
        return true;
    else if (FLOAT_MIN_DIFF < -diff)
        return false;

    return node1->var_num < node2->var_num;
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
    const vector<HTreeNode*>& patternsForThisGram = finalPatternsForGram[n_gram-1];


    resultFile << ";Interesting Pattern Mining final results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;


    for (const HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < param.threshold_frequency)
            continue;

//        if (htreeNode->superPatternRelations.empty())
//            continue;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
        }

//        resultFile << endl << ";Pattern: PatternValues = " << ((htreeNode->quotedPatternLink->getValue(PatternValuesHandle)))->to_short_string() << endl;

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << ", SurprisingnessI = " << toString(htreeNode->nI_Surprisingness);

        resultFile << ", SurprisingnessII = " << toString(htreeNode->nII_Surprisingness);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;
        if (param.if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->to_short_string();
        else
        {
            for (const Handle& link : htreeNode->pattern)
            {
                resultFile << link->to_short_string();
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

    for (const auto& num : allEntityNumMap)
    {
        csvFile << num.first << "," << num.second << std::endl;
    }

    csvFile.close();
}

void PatternMiner::OutPutFrequentPatternsToFile(unsigned int n_gram, const vector<vector<HTreeNode*>>& _patternsForGram, const string& _fileNamebasic)
{

    // out put the n_gram frequent patterns to a file, in the order of frequency
    ofstream resultFile;

    string fileName = _fileNamebasic + "FrequentPatterns_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing  (gram = " + toString(n_gram) + ") frequent patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());
    const vector<HTreeNode*>& patternsForThisGram = _patternsForGram[n_gram-1];

    resultFile << ";Frequent Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;
    num_of_patterns_with_1_frequency[n_gram-1] = 0;

    for (const HTreeNode* htreeNode : patternsForThisGram)
    {

        if (htreeNode->count == 1)
            num_of_patterns_with_1_frequency[n_gram-1] ++;

        if (htreeNode->count < param.threshold_frequency)
            continue;

        // resultFile << endl << ";Pattern: PatternValues = " << ((htreeNode->quotedPatternLink->getValue(PatternValuesHandle)))->to_short_string() << endl;
        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        if (param.if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->to_short_string();
        else
        {
            for (const Handle& link : htreeNode->pattern)
            {
                resultFile << link->to_short_string();
            }
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();
}

void PatternMiner::OutPutInterestingPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, int surprisingness, const string& _fileNamebasic) // surprisingness 1 or 2, it is default 0 which means Interaction_Information
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

    for (const HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < param.threshold_frequency)
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

        if (param.if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->to_short_string();
        else
        {
            for (const Handle& link : htreeNode->pattern)
            {
                resultFile << link->to_short_string();
            }
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();
}

void PatternMiner::OutPutSurpringnessBToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram)
{
    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName = "Surprisingness_b_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") interesting patterns to file " + fileName << std::endl;

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(patternsForThisGram.size()) << endl;

    for (HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < param.threshold_frequency)
            continue;


        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << ", Surprisingnes_II_b = " << toString(htreeNode->nII_Surprisingness_b);

        if (not htreeNode->SubRelation_b_map.empty())
        {
            unsigned int max_sub_num = 0;

            for (std::pair<Handle, vector<SubRelation_b>> sub : htreeNode->SubRelation_b_map)
            {
                if (sub.second.size() > max_sub_num)
                    max_sub_num = sub.second.size();
            }

            htreeNode->max_b_subpattern_num = max_sub_num;
        }

        resultFile << ", max_b_subpattern_num = " << htreeNode->max_b_subpattern_num;

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        if (param.if_quote_output_pattern)
            resultFile << htreeNode->quotedPatternLink->to_short_string();
        else
        {
            for (const Handle& link : htreeNode->pattern)
            {
                resultFile << link->to_short_string();
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

    const vector<HTreeNode*>& patternsForThisGram = patternsForGram[n_gram-1];

    std::cout<<"\nDebug: PatternMiner: writing  (gram = " + toString(n_gram) + ") pattern statics to csv file " + csvfileName << std::endl;

    csvFile.open(csvfileName.c_str());

    csvFile << "Frequency,Surprisingness_I,Surprisingness_II,II_Surprisingness_b, max_b_subpattern_num" << std::endl;

    for (const HTreeNode* htreeNode : patternsForThisGram)
    {
        if (htreeNode->count < param.threshold_frequency)
            continue;

        csvFile << htreeNode->count << "," << htreeNode->nI_Surprisingness << ","

                << htreeNode->nII_Surprisingness << "," << htreeNode->nII_Surprisingness_b << "," << htreeNode->max_b_subpattern_num;
//        if (not htreeNode->superPatternRelations.empty())
//            csvFile << htreeNode->nII_Surprisingness;
//        else
//            csvFile << "unknown";

        csvFile << std::endl;
    }

    csvFile.close();
}


void PatternMiner::OutPutLowFrequencyHighSurprisingnessPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, unsigned int max_frequency_index)
{
    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "LowFrequencyHighSurprisingness_" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") LowFrequencyHighSurprisingness patterns to file " + fileName << std::endl;


    vector<HTreeNode*> resultPatterns;

    for (unsigned int i = patternsForThisGram.size() - 1; i > max_frequency_index; i--)
    {
        HTreeNode* htreeNode = patternsForThisGram[i];
        resultPatterns.push_back(htreeNode);
    }

    boost::sort(resultPatterns, compareHTreeNodeBySurprisingness_I);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency < "<< (patternsForThisGram[max_frequency_index])->count <<", sort by Surprisingness_I"  << std::endl;


    for (const HTreeNode* htreeNode : resultPatterns)
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

        for (const Handle link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
        }

        resultFile << std::endl;

    }

    resultFile << std::endl;
    resultFile.close();
}

void PatternMiner::OutPutHighFrequencyHighSurprisingnessPatternsToFile(const vector<HTreeNode*>& patternsForThisGram, unsigned int n_gram, unsigned int min_frequency_index)
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

    boost::sort(resultPatterns, compareHTreeNodeBySurprisingness_I);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency > " << (patternsForThisGram[min_frequency_index])->count << ", sort by Surprisingness_I"  << std::endl;

    for (const HTreeNode* htreeNode : resultPatterns)
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

        for (const Handle& link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();
}

void PatternMiner::OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(const vector<HTreeNode*>& patternsForThisGram,unsigned int n_gram, float min_surprisingness_I, float max_surprisingness_II)
{
    // out put the n_gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "HighSurprisingILowSurprisingnessII" + toString(n_gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(n_gram) + ") HighSurprisingILowSurprisingnessII patterns to file " + fileName << std::endl;

    vector<const HTreeNode*> resultPatterns;

    for (const HTreeNode* htreeNode : patternsForThisGram)
    {
//        if (htreeNode->superPatternRelations.empty())
//            continue;

        if (htreeNode->count < param.threshold_frequency)
            continue;

        if ( (htreeNode->nI_Surprisingness > min_surprisingness_I) && (htreeNode->nII_Surprisingness < max_surprisingness_II) )
            resultPatterns.push_back(htreeNode);
    }

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(n_gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Surprising_I > " << min_surprisingness_I << ", and Surprisingness_II < "  << max_surprisingness_II << std::endl;

    // boost::sort(resultPatterns, compareHTreeNodeBySurprisingness_I);

    for (const HTreeNode* htreeNode : resultPatterns)
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

        for (const Handle& link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();
}

bool PatternMiner::containLoopVariable(const HandleSeq& pattern)
{
    return any_of(pattern, [&](const Handle& h) {
            return containOnlyVariables(h); });
}

// Each HandleSeq in HandleSeqSeq oneOfEachSeqShouldBeVars is a list of nodes that connect two links in inputLinks,
// at least one node in a HandleSeq should be variable, which means two connected links in inputLinks should be connected by at least a variable
// leaves are those nodes that are not connected to any other links in inputLinks, they should be
// some will be filter out in this phrase, return true to filter out
bool PatternMiner::filters(const HandleSeq& inputLinks, HandleSeqSeq& oneOfEachSeqShouldBeVars, HandleSeq& leaves, HandleSeq& shouldNotBeVars, HandleSeq& shouldBeVars)
{
    HandleSet allNodesInEachLink[inputLinks.size()];

    HandleSet all2ndOutgoingsOfInherlinks;

    // map<predicate, set<value>>
    map<Handle, HandleSet> predicateToValueOfEvalLinks;

    HandleSet all1stOutgoingsOfEvalLinks;


    for (unsigned int i = 0; i < inputLinks.size(); ++i)
    {
        extractNodes(inputLinks[i], allNodesInEachLink[i]);

        if (inputLinks.size() == 1)
            break;

        if (param.enable_filter_links_of_same_type_not_share_second_outgoing)
        {
            for (Type t : param.same_link_types_not_share_second_outgoing)
            {
                // filter: Any two Links of the same type in the should not share their secondary outgoing nodes
                if (((inputLinks[i])->get_type() == t))
                {
                    Handle secondOutgoing = inputLinks[i]->getOutgoingSet()[1];
                    auto rs = all2ndOutgoingsOfInherlinks.insert(secondOutgoing);
                    if (not rs.second)
	                    return true;
                }
            }
        }

        if (param.enable_filter_not_same_var_from_same_predicate ||
            param.enable_filter_not_all_first_outgoing_const ||
            param.enable_filter_first_outgoing_evallink_should_be_var)
        {
            // this filter: Any two EvaluationLinks with the same predicate should not share the same secondary outgoing nodes
            if ((inputLinks[i])->get_type() == EVALUATION_LINK)
            {
                HandleSeq outgoings = inputLinks[i]->getOutgoingSet();
                Handle predicateNode = outgoings[0];

                // value node is the last node of the list link
                HandleSeq outgoings2 = outgoings[1]->getOutgoingSet();
                Handle valueNode = outgoings2[outgoings2.size() - 1];

                if (param.enable_filter_not_same_var_from_same_predicate)
                {
                    map<Handle, HandleSet >::iterator it = predicateToValueOfEvalLinks.find(predicateNode);
                    if (it != predicateToValueOfEvalLinks.end())
                    {
                        HandleSet& values = (it->second);
                        auto rs = values.insert(valueNode);
                        if (not rs.second)
                            return true;
                    }
                    else
                    {
                        HandleSet newValues{valueNode};
                        predicateToValueOfEvalLinks.insert({predicateNode, newValues});
                    }
                }

                // this filter: at least one of all the 1st outgoings of Evaluationlinks should be var
                if (param.enable_filter_first_outgoing_evallink_should_be_var ||
                    param.enable_filter_not_all_first_outgoing_const)
                {
                    if (outgoings2.size() > 1)
                    {
                        all1stOutgoingsOfEvalLinks.insert(outgoings2[0]);
                    }
                }

            }
        }
    }

    if (not all1stOutgoingsOfEvalLinks.empty())
    {
        // this filter: all the first outgoing nodes of all evaluation links should be variables
        if (param.enable_filter_first_outgoing_evallink_should_be_var)
            std::copy(all1stOutgoingsOfEvalLinks.begin(), all1stOutgoingsOfEvalLinks.end(), std::back_inserter(shouldBeVars));
        else if (param.enable_filter_not_all_first_outgoing_const)
        {
            // if enable_filter_first_outgoing_evallink_should_be_var is true, there is no need to enable this filter below
            HandleSeq all1stOutgoingsOfEvalLinksSeq(all1stOutgoingsOfEvalLinks.begin(), all1stOutgoingsOfEvalLinks.end());
            oneOfEachSeqShouldBeVars.push_back(all1stOutgoingsOfEvalLinksSeq);
        }
    }

    if (inputLinks.size() > 1 &&
        param.enable_filter_links_should_connect_by_vars)
    {
        // find the common nodes which are shared among inputLinks
        for (unsigned i = 0; i < inputLinks.size() - 1; i ++)
        {
            for (unsigned j = i+1; j < inputLinks.size(); j ++)
            {
                HandleSeq commonNodes;
                // get the common nodes in allNodesInEachLink[i] and allNodesInEachLink[j]
                boost::set_intersection(allNodesInEachLink[i], allNodesInEachLink[j],
                                        std::back_inserter(commonNodes));

                if (not commonNodes.empty())
                    oneOfEachSeqShouldBeVars.push_back(commonNodes);
            }
        }
    }

    if (param.enable_filter_leaves_should_not_be_vars ||
        param.enable_filter_node_types_should_not_be_vars ||
        param.enable_filter_node_types_should_be_vars)
    {

        for (unsigned i = 0; i < inputLinks.size(); i ++)
        {
            for (const Handle& node : allNodesInEachLink[i])
            {
                // find leaves , do not check this for 1-gram
                if (inputLinks.size() > 1 &&
                    param.enable_filter_leaves_should_not_be_vars)
                {
                    bool is_leaf = true;

                    // try to find if this node exist in other links of inputLinks
                    for (unsigned j = 0; j < inputLinks.size(); j ++)
                    {
                        if (j == i)
                            continue;

                        if (is_in(node, allNodesInEachLink[j]))
                        {
                            is_leaf = false;
                            break;
                        }

                    }

                    if (is_leaf)
                        leaves.push_back(node);
                }

                if (param.enable_filter_node_types_should_not_be_vars ||
                    param.enable_filter_node_types_should_be_vars)
                {
                    Type t = node->get_type();

                    if (is_in(t, param.node_types_should_not_be_vars))
                        shouldNotBeVars.push_back(node);

                    if (is_in(t, param.node_types_should_be_vars))
                        shouldBeVars.push_back(node);
                }
            }
        }
    }

    return false;
}

// return true if the inputLinks are disconnected
// when the inputLinks are connected, the outputConnectedGroups has only one group, which is the same as inputLinks
bool PatternMiner::partitionBySharedVariables(const HandleSeq& links,
                                              HandleSeqSeq& connectedGroups)
{
    if(links.size() < 2)
        return false;

    HandleSet variablesPerLink[links.size()];
    for (unsigned int i = 0; i < links.size(); ++i)
    {
        extractVarNodes(links[i], variablesPerLink[i]);
    }

    int i = -1;
    for (const Handle& link : links)
    {
        i++;

        if (isInHandleSeqSeq(link, connectedGroups))
            continue;

        // This link is not in outputConnectedGroups, which means none
        // of its previous links connect to this link.  So, make a new
        // group.
        HandleSeq newGroup;
        newGroup.push_back(link);
        HandleSet variablesInGroup(variablesPerLink[i]);

        // Only need to scan the links after this link
        for (unsigned int j = i+1; j < links.size(); j++) {
            if (not is_disjoint(variablesInGroup, variablesPerLink[j])) {
                newGroup.push_back(links[j]);
                set_union_modify(variablesInGroup, variablesPerLink[j]);
            }
        }

        connectedGroups.push_back(newGroup);
    }

    return connectedGroups.size() > 1;
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
//        std::cout << link->to_short_string();
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
            HandleMap orderedVarNameMap;
            HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex, orderedVarNameMap);
            string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

//             std::cout<< "Subpattern: " << subPatternKey;

            // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
            HandleSeqSeq splittedSubPattern;
            if (partitionBySharedVariables(unifiedSubPattern, splittedSubPattern))
            {
//                 std::cout<< " is disconnected! splitted it into connected parts: \n" ;
                // The splitted parts are disconnected, so they are independent. So the entroy = the sum of each part.
                // e.g. if ABC is disconneted, and it's splitted into connected subgroups by splitDisconnectedLinksIntoConnectedGroups,
                // for example: AC, B  then H(ABC) = H(AC) + H(B)
                for (HandleSeq aConnectedSubPart : splittedSubPattern)
                {
                    // Unify it again
                    unsigned int _unifiedLastLinkIndex;
                    HandleMap suborderedVarNameMap;
                    HandleSeq unifiedConnectedSubPattern = UnifyPatternOrder(aConnectedSubPart, _unifiedLastLinkIndex, suborderedVarNameMap);
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

unsigned int PatternMiner::getCountOfAConnectedPattern(const string& connectedPatternKey, const HandleSeq& connectedPattern)
{
    uniqueKeyLock.lock();
    // try to find if it has a corresponding HtreeNode
    map<string, HTreeNode*>::iterator patternNodeIter = keyStrToHTreeNodeMap.find(connectedPatternKey);

    if (patternNodeIter != keyStrToHTreeNodeMap.end())
    {
        uniqueKeyLock.unlock();
        HTreeNode* patternNode = patternNodeIter->second;
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
                keyStrToHTreeNodeMap.insert({connectedPatternKey, newHTreeNode});
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

bool PatternMiner::isALinkOneInstanceOfGivenPattern(const Handle& instanceLink, const Handle& patternLink)
{
    if (instanceLink->get_type() != patternLink->get_type())
        return false;

    const HandleSeq& outComingsOfPattern = patternLink->getOutgoingSet();
    const HandleSeq& outComingsOfInstance = instanceLink->getOutgoingSet();

    if (outComingsOfPattern.size() != outComingsOfInstance.size())
        return false;

    for (unsigned int i = 0; i < outComingsOfPattern.size(); i ++)
    {
        bool pis_link = (outComingsOfPattern[i])->is_link();
        bool iis_link = (outComingsOfInstance[i])->is_link();

        if (pis_link && iis_link)
        {
            if (isALinkOneInstanceOfGivenPattern(outComingsOfInstance[i], outComingsOfPattern[i]))
                continue;
            else
                return false;
        }
        else if ( (!pis_link)&&(!iis_link) )
        {
            // they are both nodes

            // If this node is a variable, skip it
            if ((outComingsOfPattern[i])->get_type() == PATTERN_VARIABLENODE_TYPE)
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

void PatternMiner::reNameNodesForALink(const Handle& inputLink, const Handle& nodeToBeRenamed,
                                       Handle& newNamedNode, HandleSeq& renameOutgoingLinks,
                                       AtomSpace& to_as)
{
    HandleSeq outgoingLinks = inputLink->getOutgoingSet();

    for (Handle h : outgoingLinks)
    {
        if (h->is_node())
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
             reNameNodesForALink(h, nodeToBeRenamed, newNamedNode, _renameOutgoingLinks, to_as);
             Handle reLink = to_as.add_link(h->get_type(), _renameOutgoingLinks);
             reLink->setTruthValue(h->getTruthValue());
             renameOutgoingLinks.push_back(reLink);
        }
    }
}

// the return links is in Pattern mining AtomSpace
// toBeExtendedLink is the link contains leaf in the pattern
// varNode is the variableNode of leaf
void PatternMiner::getOneMoreGramExtendedLinksFromGivenLeaf(Handle& toBeExtendedLink, Handle& leaf, Handle& varNode,
                                                            HandleSeq& outPutExtendedPatternLinks, AtomSpace& from_as)
{
    IncomingSet incomings = leaf->getIncomingSet(&from_as);

    for (LinkPtr incomingPtr : incomings)
    {
        Handle incomingHandle  = incomingPtr->get_handle();
        Handle extendedHandle;
        // if this atom is a igonred type, get its first parent that is not in the igonred types
        if (isIgnoredType (incomingHandle->get_type()) )
        {
            extendedHandle = getFirstNonIgnoredIncomingLink(from_as, incomingHandle);
            if (extendedHandle == Handle::UNDEFINED)
                continue;
        }
        else
            extendedHandle = incomingHandle;

        // skip this extendedHandle if it's one instance of toBeExtendedLink
        if (isALinkOneInstanceOfGivenPattern(extendedHandle, toBeExtendedLink))
            continue;

        // Rebind this extendedHandle into Pattern Mining Atomspace
        HandleSeq renameOutgoingLinks;
        reNameNodesForALink(extendedHandle, leaf, varNode, renameOutgoingLinks, *as);
        Handle reLink = as->add_link(extendedHandle->get_type(), renameOutgoingLinks);
        reLink->setTruthValue(extendedHandle->getTruthValue());
        outPutExtendedPatternLinks.push_back(reLink);
    }
}

unsigned int PatternMiner::getAllEntityCountWithSamePredicatesForAPattern(const HandleSeq& pattern)
{
    if (pattern.size() == 1)
    {
        if (pattern[0]->get_type() == EVALUATION_LINK)
        {
            Handle predicate = pattern[0]->getOutgoingAtom(0);

            string predicateName = predicate->get_name();

//            cout << "/npredicate: " << predicateName << std::endl;

            map<string, unsigned int>::iterator eit = allEntityNumMap.find(predicateName);
            if (eit != allEntityNumMap.end())
            {
//                cout << "alredy exists: " << eit->second << std::endl;
                return eit->second;
            }
            else
            {
                IncomingSet allEvals = predicate->getIncomingSet(&original_as);
                allEntityNumMap.insert({predicateName, allEvals.size()});
//                cout << "Found: " << allEvals.size() << " entities." << std::endl;
                return allEvals.size();
            }
        }
        else
        {
            cout << "warning: this pattern contains " << nameserver().getTypeName(pattern[0]->get_type())
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

        for (const Handle& l : pattern)
        {
            if (l->get_type() == EVALUATION_LINK)
            {
                Handle predicate = l->getOutgoingAtom(0);
                allPredicateNodes.push_back(predicate);
            }
            else
            {
                cout << "warning: this pattern contains " << nameserver().getTypeName(l->get_type())
                     << "\nUSE_QUERY_ENTITY_COUNT_FOR_EACH_PREDICATE is for the corpus that only contains EvalutionLinks." << std::endl;
                return 0;
            }
        }

        boost::sort(allPredicateNodes);
        string predicateWords = "";
        for (const Handle& predicate : allPredicateNodes)
        {
            predicateWords += predicate->get_name();
            predicateWords += " ";
        }

//        cout << "/npredicates: " << predicateWords << std::endl;

        map<string,unsigned int>::iterator eit = allEntityNumMap.find(predicateWords);
        if (eit != allEntityNumMap.end())
        {
//            cout << "alredy exists: " << eit->second << std::endl;
            return eit->second;
        }

        HandleSetSeq allEntitiesForEachPredicate;

        for (const Handle& predicate : allPredicateNodes)
        {
            HandleSet allEntitiesForThisPredicate;

            IncomingSet allEvals = predicate->getIncomingSet(&original_as);
            for (LinkPtr incomingPtr : allEvals)
            {
                Handle evalLink = incomingPtr->get_handle();
                Handle listLink = evalLink->getOutgoingAtom(1);
                Handle entityNode = listLink->getOutgoingAtom(0);
                allEntitiesForThisPredicate.insert(entityNode);
            }

            allEntitiesForEachPredicate.push_back(allEntitiesForThisPredicate);

        }

        HandleSeq commonLinks;
        // get the common Links in allEntitiesForEachPredicate
        boost::set_intersection(allEntitiesForEachPredicate[0],
                                allEntitiesForEachPredicate[1],
                                std::back_inserter(commonLinks));

        if (commonLinks.empty())
            return 0;

        for (unsigned int i = 2; i < pattern.size(); ++ i)
        {
            HandleSeq newCommonLinks;
            // get the common Links in allEntitiesForEachPredicate
            boost::set_intersection(allEntitiesForEachPredicate[i], commonLinks,
                                    std::back_inserter(newCommonLinks));

            if (newCommonLinks.empty())
                return 0;

            commonLinks.swap(newCommonLinks);
        }

        allEntityNumMap.insert({predicateWords, commonLinks.size()});
//        cout << "Found: " << commonLinks.size() << " entities." << std::endl;
        return commonLinks.size();
    }

//    HandleSeq allEntityPattern;
//    int var_index_num = 1;
//    Handle entityVar = as->add_node(VARIABLE_NODE, "$var_1");

//    for (Handle l : pattern)
//    {
//        var_index_num ++;

//        if (l->get_type() == EVALUATION_LINK)
//        {
//            Handle conceptNode = l->getOutgoingAtom(0);

//            Handle valueNode = as->add_node(VARIABLE_NODE, "$var_" + toString(var_index_num));
//            Handle newListLink = as->add_link(LIST_LINK, entityVar, valueNode);

//            Handle newEvalLink = as->add_link(EVALUATION_LINK, conceptNode, newListLink);

//            allEntityPattern.push_back(newEvalLink);
//        }
//        else
//        {
//            cout << "warning: this pattern contains " << nameserver().getTypeName(l->get_type())
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

// II_Surprisingness_b can be calculated for all input grams, including 1 gram and max_gram
// only calculate 2~4 gram patterns for nSurprisingness_I and nSurprisingness_II
void PatternMiner::calculateSurprisingness(HTreeNode* HNode)
{
//    // debug
//    if (HNode->nI_Surprisingness != 0 || HNode->nII_Surprisingness != 0)
//        std::cout << "Exception: This pattern has been calculateSurprisingness before!\n";

//    if (HNode->count == 0) // this should not happen
//        HNode->count = 1;

    if (HNode->count < param.threshold_frequency)
    {
        HNode->nII_Surprisingness = 0.0f;
        HNode->nI_Surprisingness = 0.0f;
        HNode->nII_Surprisingness_b = 0.0f;
        return;
    }

    if (param.calculate_type_b_surprisingness)
        calculateTypeBSurprisingness(HNode);

    unsigned int gram = HNode->pattern.size();

    if (gram == 1)
        return;


//    std::cout << "=================Debug: calculate I_Surprisingness for pattern: ====================\n";
//    for (Handle link : HNode->pattern)
//    {
//        std::cout << link->to_short_string();
//    }
//     std::cout << "count of this pattern = " << HNode->count << std::endl;
//     std::cout << std::endl;


    // get the predefined combination:
    // vector<vector<vector<unsigned int>>>
//    int comcount = 0;

    HNode->surprisingnessInfo = "";

    float p;
    unsigned int allNum;
    float allNumFloat; // the divisor

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
            allNum = allEntityCount;
        }
    }
    else if (USE_QUERY_ALL_ENTITY_COUNT)
    {
        allNum = allEntityNumMap.size();
    }
    else
    {
        allNum = actualProcessedLinkNum;
    }

    allNumFloat = (float)allNum;
    p = ((float)HNode->count)/((float)allNumFloat);

    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        HNode->surprisingnessInfo += ";p = " + toString(HNode->count) + "/" + toString(allNumFloat) + " = " + toString(p) + "\n";
    }

    float abs_min_diff = 999999999.9f;
    float min_diff = 999999999.9f;
    // cout << "For this pattern itself: p = " <<  HNode->count << " / " <<  allNum << " = " << p << std::endl;

    for (const vector<vector<unsigned int>>& oneCombin : components_ngram[gram-2])
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

        for (const vector<unsigned int>& oneComponent : oneCombin)
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
            HandleMap orderedVarNameMap;
            HandleSeq unifiedSubPattern = UnifyPatternOrder(subPattern, unifiedLastLinkIndex, orderedVarNameMap);
            string subPatternKey = unifiedPatternToKeyString(unifiedSubPattern);

//            std::cout<< "Subpattern: " << subPatternKey;

            // First check if this subpattern is disconnected. If it is disconnected, it won't exist in the H-Tree anyway.
            HandleSeqSeq splittedSubPattern;
            if (partitionBySharedVariables(unifiedSubPattern, splittedSubPattern))
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
                    p_i = ((float)(component_count)) / allNumFloat;

                    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
                    {
                        HNode->surprisingnessInfo += toString(component_count) + "/" + toString(allNum) + "=" + toString(p_i);
                    }
                }

                // cout << ", p = " << component_count  << " / " << allNum << " = " << p_i << std::endl;
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

        float abs_diff = std::abs(diff);

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

    if (gram == param.MAX_GRAM ) // can't calculate II_Surprisingness for MAX_GRAM patterns, becasue it required gram +1 patterns
        return;


    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile << "=================Debug: calculate II_Surprisingness for pattern: ====================\n";
        surpringnessIICalfile << "Frequency = " << HNode->count << " p = " << HNode->count << "/" << allNum << " = " << p << std::endl;
        for (Handle link : HNode->pattern)
        {
            surpringnessIICalfile << link->to_short_string();
        }

        surpringnessIICalfile << std::endl;
    }

    // II_Surprisingness is to evaluate how easily the frequency of this pattern can be infered from any of  its superpatterns
    // for all its super patterns
    if (HNode->superPatternRelations.empty())
    {
        HNode->nII_Surprisingness  = 999999999.9;
        // debug:
//        cout << "This node has no super patterns, give it Surprisingness_II value: -1.0 \n";
    }
    else
    {
        HNode->nII_Surprisingness  = 999999999.9f;
        float minSurprisingness_II = 999999999.9f;
        unsigned int actualProcessedRelationNum = 0;
        for (const ExtendRelation& curSuperRelation : HNode->superPatternRelations)
        {
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
            HandleMap EorderedVarNameMap;
            HandleSeq unifiedPatternE = UnifyPatternOrder(patternE, unifiedLastLinkIndex, EorderedVarNameMap);

            string patternEKey = unifiedPatternToKeyString(unifiedPatternE);

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
                    surpringnessIICalfile << link->to_short_string();
                }
                //surpringnessIICalfile << unifiedPatternToKeyString(curSuperRelation.extendedHTreeNode->pattern, as);
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

        if (not HNode->superPatternRelations.empty() && actualProcessedRelationNum)
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


void PatternMiner::calculateTypeBSurprisingness(HTreeNode* HNode)
{
    // Currently II_Surprisingness_b only for 1-gram patterns
    // because the tracking of type b super-sub relations for 2-gram or bigger patterns is too costly
    // If really wants to calculate II_Surprisingness_b for 2-gram or biger patterns,
    // need to generate the type b relations when calculating II_Surprisingness_b. But it is very very costly.
    // need to turn on GENERATE_TYPE_B_RELATION_WHEN_CALCULATE_SURPRISINGNESS.
    //    std::cout << "=================Debug: calculate II_Surprisingness_b for 1-gram patterns: ====================\n";
    //    for (Handle link : HNode->pattern)
    //    {
    //        std::cout << link->to_short_string();
    //    }
    //    std::cout << "count of this pattern = " << HNode->count << std::endl;
    //    std::cout << std::endl;
    // Surpringness II also can be calculated via more general patterns of the same gram, e.g.
    // nII_Surprisingness_b(A) = min{Surprisingness_b from all super patterns} = min{Count(A) / Count(S)} = 1.0 - 18 / 98
    // Here we should only consider the number of subpatterns of S, not the exact Frequency of each subpatterns,
    // because even if there is country that occurs 70 times, and other countries only occurs 3 times or less,
    // if there are a lot of countries have the same pattern with A, then S is still a generalized pattern.
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

    // 1-gram patterns already has superRelation_b_list and SubRelation_b_map
    // so first, find this type b relations for 2-gram and bigger patterns
    if (HNode->pattern.size() > 1)
    {
        // generate all the super patterns of same gram of this pattern for 2-gram or bigger patterns
        // By changing one const node into a variable node, if this pattern exist, then it is one super pattern of this pattern
        HandleSet allConstNodes;
        for (Handle link : HNode->pattern)
            extractConstNodes(link, allConstNodes);

        string var_name = "$var_"  + toString(HNode->var_num + 1);
        Handle var_node = as->add_node(opencog::PATTERN_VARIABLENODE_TYPE, var_name);

        for (const Handle& constNode : allConstNodes)
        {
            // replace this const node with a new variable node
            HandleSeq oneSuperPattern = ReplaceConstNodeWithVariableForAPattern(HNode->pattern, constNode, var_node);

            // only try to find it from mined patterns, will not query it by pattern matcher

            unsigned int unifiedLastLinkIndex;
            HandleMap suborderedVarNameMap;
            HandleSeq unifiedSuperPattern = UnifyPatternOrder(oneSuperPattern, unifiedLastLinkIndex, suborderedVarNameMap);

            string superPatternKey = unifiedPatternToKeyString(unifiedSuperPattern);

            // todo: need a lock here?
            map<string, HTreeNode*>::iterator patternNodeIter = keyStrToHTreeNodeMap.find(superPatternKey);
            if (patternNodeIter != keyStrToHTreeNodeMap.end())
            {
                HTreeNode* superPatternNode = patternNodeIter->second;
                SuperRelation_b superb{superPatternNode, constNode};
                HNode->superRelation_b_list.push_back(superb);

                Handle unified_var_node = suborderedVarNameMap[var_node];

                if (superPatternNode->SubRelation_b_map.find(unified_var_node) == superPatternNode->SubRelation_b_map.end())
                {
                    SubRelation_b sub_b{HNode, constNode};
                    vector<SubRelation_b> sub_blist{sub_b};
                    superPatternNode->SubRelation_b_map.insert({unified_var_node, sub_blist});
                }
            }
        }
    }

    if (HNode->superRelation_b_list.empty())
    {
        HNode->nII_Surprisingness_b = 9999999.9;
        return;
    }

    // calculate II_Surprisingness_b
    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile << "=================Debug: calculate II_Surprisingness_b for pattern: ====================\n";
        surpringnessIICalfile << "Count = " << HNode->count << std::endl;
        for (Handle link : HNode->pattern)
        {
            surpringnessIICalfile << link->to_short_string();
        }

        surpringnessIICalfile << std::endl;
    }

    double min_II_Surprisingness_b = 9999999.9;

    for (const SuperRelation_b& superb : HNode->superRelation_b_list)
    {
        // calculate II_Surprisingness_b
        double II_Surprisingness_b = ((float)HNode->count) / ((float)superb.superHTreeNode->count);
        if (II_Surprisingness_b < min_II_Surprisingness_b)
            min_II_Surprisingness_b = II_Surprisingness_b;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
        {
            surpringnessIICalfile << "\nCount(S) = " << superb.superHTreeNode->count << ", II_Surprisingness_b = "
                                  << HNode->count << " / " << superb.superHTreeNode->count << " = " << II_Surprisingness_b << std::endl;

            for (const Handle& link : superb.superHTreeNode->pattern)
            {
                surpringnessIICalfile << link->to_short_string();
            }

            surpringnessIICalfile << "\n-----------end super pattern :---------------\n";
        }

    }

    HNode->nII_Surprisingness_b = min_II_Surprisingness_b;

    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile << "\nmin_II_Surprisingness_b = " << min_II_Surprisingness_b
                              << "\n=================end calculate II_Surprisingness_b  ====================\n";
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
            for (std::string::size_type i = 0; i < oneComponentStr.size(); ++i)
            {
                oneComponent.push_back((unsigned int)(oneComponentStr[i] - '0'));
            }
            oneCombin.push_back(oneComponent);
        }
        componentCombinations.push_back(oneCombin);
    }
}

PatternMiner::PatternMiner(AtomSpace& _original_as)
    : original_as(_original_as)
{
    param.reSetAllSettingsFromConfig();

    initPatternMiner();

    // TODO replace hard coded combinations by generic generator

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

// make sure it is called after reSetAllSettingsFromConfig
void PatternMiner::initPatternMiner()
{
    htree = new HTree();
    as = new AtomSpace(&original_as);

//    FrequencyHandle = as->add_node(CONCEPT_NODE, "Frequency");
//    InteractionInformationHandle = as->add_node(CONCEPT_NODE, "InteractionInformation");
//    SurprisingnessIHandle = as->add_node(CONCEPT_NODE, "SurprisingnessIHandle");
//    SurprisingnessIIHandle = as->add_node(CONCEPT_NODE, "SurprisingnessIIHandle");

    PatternValuesHandle = as->add_node(CONCEPT_NODE, "PatternValues");

    threads = new thread[param.THREAD_NUM];

    is_distributed = false;

    cur_gram = 0;

    htree = nullptr;

    observing_as = nullptr;

    patternsForGram.resize(param.MAX_GRAM);
    finalPatternsForGram.resize(param.MAX_GRAM);
    tmpPatternsForGram.resize(param.MAX_GRAM);
}

// release everything
void PatternMiner::cleanUpPatternMiner()
{

    if (htree != nullptr)
    {
        delete htree;
        htree = nullptr;
    }

    if (as != nullptr)
    {
        delete as;
        as = nullptr;
    }

//    if (threads)
//        delete threads;

    param.linktype_black_list.clear();

    for (std::pair<string, HTreeNode*> OnePattern : keyStrToHTreeNodeMap)
    {
        delete ((HTreeNode*)(OnePattern.second));
    }

    clear_by_swap(keyStrToHTreeNodeMap);

    // clear patternsForGram
    for (auto& p : patternsForGram)
	    clear_by_swap(p);
    clear_by_swap(patternsForGram);

    // clear finalPatternsForGram
    for (auto& p : finalPatternsForGram)
	    clear_by_swap(p);
    clear_by_swap(finalPatternsForGram);

    // clear tmpPatternsForGram
    for (auto& p : tmpPatternsForGram)
	    clear_by_swap(p);
    clear_by_swap(tmpPatternsForGram);
}

void PatternMiner::resetPatternMiner(bool resetAllSettingsFromConfig)
{
    if (resetAllSettingsFromConfig)
        param.reSetAllSettingsFromConfig();

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
//    Handle frequencyValue = as->add_node(NUMBER_NODE, toString(hTreeNode->count));
//    Handle iiValue = as->add_node(NUMBER_NODE, toString(hTreeNode->interactionInformation));
//    Handle SurprisingnessiValue = as->add_node(NUMBER_NODE, toString(hTreeNode->nI_Surprisingness));
//    Handle SurprisingnessiiValue = as->add_node(NUMBER_NODE, toString(hTreeNode->nII_Surprisingness));
//    Handle andLink = as->add_link(AND_LINK,hTreeNode->pattern);

//    quoteOutgoings.push_back(frequencyValue);
//    quoteOutgoings.push_back(iiValue);
//    quoteOutgoings.push_back(SurprisingnessiValue);
//    quoteOutgoings.push_back(SurprisingnessiiValue);
//    quoteOutgoings.push_back(andLink);

    Handle quotedPatternLink = hTreeNode->quotedPatternLink =
	    as->add_link(param.output_pattern_quoted_linktype, hTreeNode->pattern);
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
    cout << "\nQuoting all " << gram << "-gram patterns with " << nameserver().getTypeName(param.output_pattern_quoted_linktype) << std::endl;
    for (HTreeNode* hTreeNode : patternsForGram[gram - 1])
    {
        quoteAPattern(hTreeNode);
    }
}

void PatternMiner::runPatternMiner(bool exit_program_after_finish)
{

    if (not keyStrToHTreeNodeMap.empty())
    {
        cleanUpPatternMiner();
        initPatternMiner();
    }

    std::cout << "\nDebug: PatternMining start! Max gram = "
              << this->param.MAX_GRAM << ", mode = " << param.pattern_mining_mode << std::endl;

    int start_time = time(nullptr);

    allLinks.clear();
    original_as.get_handles_by_type(back_inserter(allLinks), (Type)LINK, true);

    allLinkNumber = (int)allLinks.size();
    atomspaceSizeFloat = (float)allLinkNumber;

    black_keyword_Handles.clear();
    if (param.use_keyword_black_list && (! param.keyword_black_logic_is_contain))
    {
        for (const string& keyword : param.keyword_black_list)
        {
            Handle keywordNode = original_as.get_node(opencog::CONCEPT_NODE, keyword);
            if (keywordNode)
	            black_keyword_Handles.insert(keywordNode);

            keywordNode = original_as.get_node(opencog::PREDICATE_NODE, keyword);
            if (keywordNode)
	            black_keyword_Handles.insert(keywordNode);
        }
    }

    std::cout << "Using " << param.THREAD_NUM << " threads. \n";
    std::cout << "Corpus size: "<< allLinkNumber << " links in total. \n\n";

    if (param.only_mine_patterns_start_from_white_list ||
        param.only_output_patterns_contains_white_keywords)
    {
        allLinksContainWhiteKeywords.clear();
        havenotProcessedWhiteKeywordLinks.clear();

        if (param.only_mine_patterns_start_from_white_list)
        {
            cout << "\nOnly mine patterns start from white list: logic = ";
            if (param.only_mine_patterns_start_from_white_list_contain)
                cout << " Nodes contain keyword." << std::endl;
            else
                cout << " Nodes'label equal to keyword." << std::endl;
        }

        for (string keyword : param.keyword_white_list)
        {
            std::cout << keyword << std::endl;
        }

        if (param.use_keyword_black_list)
        {
            cout << "\nuse_keyword_black_list is also enable, so avoid links that contain any nodes that ";
            if (param.keyword_black_logic_is_contain)
                cout << "contain";
            else
                cout << "equal to";
            cout << " any of the following black keywords:\n";
            for (const string& bkeyword : param.keyword_black_list)
            {
                std::cout << bkeyword << std::endl;
            }
        }

        if (param.use_linktype_black_list)
        {
            cout << "\nuse_linktype_black_list is also enable, so avoid links of these types:\n ";

            for (Type linkTpe : param.linktype_black_list)
            {
                std::cout << nameserver().getTypeName(linkTpe) << std::endl;
            }
        }
        else if (param.use_linktype_white_list)
        {
            cout << "\nuse_linktype_white_list is also enable, so only find links of these types:\n ";

            for (Type linkTpe : param.linktype_white_list)
            {
                std::cout << nameserver().getTypeName(linkTpe) << std::endl;
            }
        }

        cout << "\n\nFinding Links...\n";


        findAllLinksContainKeyWords(param.keyword_white_list, 0, param.only_mine_patterns_start_from_white_list_contain, havenotProcessedWhiteKeywordLinks);

        std::copy(havenotProcessedWhiteKeywordLinks.begin(), havenotProcessedWhiteKeywordLinks.end(), std::back_inserter(allLinksContainWhiteKeywords));
        cout << "Found " << allLinksContainWhiteKeywords.size() << " Links contians the keywords!\n";

    }

    runPatternMinerDepthFirst();

    std::cout<<"PatternMiner:  mining finished!\n";

    if (param.enable_interesting_pattern)
    {
        runInterestingnessEvaluation();
    }
    else
    {
        if (param.if_quote_output_pattern)
        {
            for (unsigned int gram = 1; gram <= param.MAX_GRAM; gram ++)
                quoteAllThePatternSForGram(gram);
        }
    }

    // Out put all patterns with a frequency above the thresthold
    num_of_patterns_with_1_frequency = new unsigned int [param.MAX_GRAM];

    for (unsigned int gram = 1; gram <= param.MAX_GRAM; gram ++)
    {
        // sort by frequency
        boost::sort(patternsForGram[gram-1], compareHTreeNodeByFrequency);

        // Finished mining gram patterns; output to file
        std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGram[gram-1]).size()) + " patterns found! ";

        OutPutFrequentPatternsToFile(gram, patternsForGram);

        if (GENERATE_TMP_PATTERNS && not tmpPatternsForGram[gram-1].empty())
        {
            boost::sort(tmpPatternsForGram[gram-1], compareHTreeNodeByFrequency);

            OutPutFrequentPatternsToFile(gram, tmpPatternsForGram, "tmpPatterns");
        }

        std::cout<< std::endl;
    }


    int end_time = time(nullptr);
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
        original_as.get_handles_by_type(back_inserter(allEvalLinks), (Type) EVALUATION_LINK, false);
        HandleSet allEntityHandles;
        for (const Handle& evalLink : allEvalLinks)
        {
            Handle listLink = evalLink->getOutgoingAtom(1);
            Handle entityHandle = listLink->getOutgoingAtom(0);
            allEntityHandles.insert(entityHandle);
        }

        cout << "All entity number = " << allEntityHandles.size() << std::endl;

    }

    if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
    {
        surpringnessIICalfile.open("surpringnessII_calcualtion_info.scm");
    }

    for (cur_gram = 1; cur_gram <= param.MAX_GRAM; cur_gram ++)
    {

        cout << "\nCalculating";
        if (cur_gram > 1)
        {
            if (param.enable_interaction_information)
                cout << " Interaction_Information ";
            if (param.enable_surprisingness)
                cout << " Surprisingness ";
        }
        else
        {
            if (param.enable_surprisingness)
                cout << " Surprisingness ";
        }


        cout << "for " << cur_gram << " gram patterns." << std::endl;

        cur_index = -1;
        threads = new thread[param.THREAD_NUM];
        num_of_patterns_without_superpattern_cur_gram = 0;

        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
            surpringnessIICalfile << ";*************** surpringnessII calculation process info for " + toString(cur_gram) + " gram patterns.***************" << endl;

        for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
            threads[i] = std::thread([this]{this->evaluateInterestingnessTask();});

        for (unsigned int i = 0; i < param.THREAD_NUM; ++ i)
            threads[i].join();

        delete [] threads;

        std::cout<<"PatternMiner:  done (gram = " + toString(cur_gram) + ") interestingness evaluation!" + toString((patternsForGram[cur_gram-1]).size()) + " patterns found! ";
        std::cout<<"Outputting to file ... ";

        if (param.if_quote_output_pattern)
            quoteAllThePatternSForGram(cur_gram);

        if (param.enable_interaction_information)
        {
            // sort by interaction information
	        boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeByInteractionInformation);
            OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 0);
        }

        if (param.enable_surprisingness)
        {
//            // sort by frequency
//            boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeByFrequency);

//            int max_frequency_threshold_index = FREQUENCY_BOTTOM_THRESHOLD * ((float)(patternsForGram[cur_gram-1].size()));
//            OutPutLowFrequencyHighSurprisingnessPatternsToFile(patternsForGram[cur_gram-1], cur_gram, max_frequency_threshold_index);

//            int min_frequency_threshold_index = FREQUENCY_TOP_THRESHOLD * ((float)(patternsForGram[cur_gram-1].size() - num_of_patterns_with_1_frequency[cur_gram-1]));
//            OutPutHighFrequencyHighSurprisingnessPatternsToFile(patternsForGram[cur_gram-1], cur_gram,  min_frequency_threshold_index);

            if (cur_gram > 1)
            {
                // sort by surprisingness_I first
                boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeBySurprisingness_I);
                OutPutInterestingPatternsToFile(patternsForGram[cur_gram-1], cur_gram,1);
            }

            // output surprisingness b
            if ((cur_gram == 1) || GENERATE_TYPE_B_RELATION_WHEN_CALCULATE_SURPRISINGNESS)
            {
                // sort by surprisingness_I first
                boost::sort(patternsForGram[cur_gram-1], compareHTreeNodeBySurprisingness_b);
                OutPutSurpringnessBToFile(patternsForGram[cur_gram-1], cur_gram);
            }

            OutPutStaticsToCsvFile(cur_gram);

            if (cur_gram == param.MAX_GRAM)
                break;

            vector<HTreeNode*> curGramPatterns = patternsForGram[cur_gram-1];

            if (curGramPatterns.empty())
                break;

            // and then sort by surprisingness_II
            boost::sort(curGramPatterns, compareHTreeNodeBySurprisingness_II);
            OutPutInterestingPatternsToFile(curGramPatterns, cur_gram, 2);

            // Get the min threshold of surprisingness_II
            int threshold_index_II;
            threshold_index_II = SURPRISINGNESS_II_TOP_THRESHOLD * (float)(curGramPatterns.size() - num_of_patterns_without_superpattern_cur_gram);

            int looptimes = 0;
            while (true)
            {
                surprisingness_II_threshold = (curGramPatterns[threshold_index_II])->nII_Surprisingness; // TODO how that comes to be?
                if (surprisingness_II_threshold <= 0.00000f)
                {
                    if (++looptimes > 8)
                    {
                        surprisingness_II_threshold = 0.00000f;
                        break;
                    }

                    threshold_index_II = ((float)threshold_index_II) * SURPRISINGNESS_II_TOP_THRESHOLD;
                }
                else
                    break;
            }

            cout << "surprisingness_II_threshold for " << cur_gram << " gram = "<< surprisingness_II_threshold;

            // go through the top N patterns of surprisingness_I, pick the patterns with surprisingness_II higher than threshold
            int threshold_index_I = SURPRISINGNESS_I_TOP_THRESHOLD * (float)(curGramPatterns.size());
            for (int p = 0; p <= threshold_index_I; p++)
            {
                HTreeNode* pNode = patternsForGram[cur_gram-1][p];

                // for patterns that have no superpatterns, nII_Surprisingness == -1.0, which should be taken into account
                if ( (pNode->nII_Surprisingness < 0) || (pNode->nII_Surprisingness >= surprisingness_II_threshold) )
                    finalPatternsForGram[cur_gram-1].push_back(pNode);
            }

//            OutPutHighSurprisingILowSurprisingnessIIPatternsToFile(patternsForGram[cur_gram-1], cur_gram, 100.0f, 0.51f);

            // sort by frequency
            boost::sort(finalPatternsForGram[cur_gram-1], compareHTreeNodeByFrequency);

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

    boost::sort(resultPatterns, compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] ,"
               << "Surprisingness_I range = [" << toString(min_surprisingness_I) << ", " << toString(max_surprisingness_I) << "]" << std::endl;

    for (const HTreeNode* htreeNode : resultPatterns)
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

        for (const Handle& link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
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

    boost::sort(resultPatterns, compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] ,"
               << "Surprisingness_I range = [" << toString(min_surprisingness_I) << ", " << toString(max_surprisingness_I) << "]"
               << "Surprisingness_II range = [" << toString(min_surprisingness_II) << ", " << toString(max_surprisingness_II) << "]" << std::endl;

    for (const HTreeNode* htreeNode : resultPatterns)
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

        for (const Handle& link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();

    cout <<"\nDone!" << std::endl;
}

void PatternMiner::queryPatternsWithFrequencySurprisingnessBRanges(unsigned int min_frequency, unsigned int max_frequency,
                                                               float min_surprisingness_B, float max_surprisingness_B,
                                                               unsigned int min_subpattern_num, unsigned int max_subpattern_num,int gram)
{
    // out put the gram patterns to a file
    ofstream resultFile;
    string fileName;

    fileName = "GivenFrequencySurprisingnessIAndII_" + toString(gram) + "gram.scm";

    std::cout<<"\nDebug: PatternMiner: writing (gram = " + toString(gram) + ") patterns to file " + fileName << std::endl;
    std::cout<<"Frequency range = [" << min_frequency << ", " << max_frequency << "] "  << std::endl;
    std::cout<<"surprisingness_B range = [" << toString(min_surprisingness_B) << ", " << toString(max_surprisingness_B) << "]"  << std::endl;
    std::cout<<"b_subpattern_num range = [" << toString(min_subpattern_num) << ", " << toString(max_subpattern_num) << "]"  << std::endl;

    vector<HTreeNode*> resultPatterns;

    for (HTreeNode* htreeNode : patternsForGram[gram - 1])
    {
        if ((htreeNode->count >= min_frequency) && (htreeNode->count <= max_frequency) &&
            (htreeNode->nII_Surprisingness_b >= min_surprisingness_B) && (htreeNode->nII_Surprisingness_b <= max_surprisingness_B) &&
            (htreeNode->max_b_subpattern_num >= min_subpattern_num) && (htreeNode->max_b_subpattern_num <= max_subpattern_num)
           )
            resultPatterns.push_back(htreeNode);
    }

    boost::sort(resultPatterns, compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] , "
               <<"surprisingness_B range = [" << toString(min_surprisingness_B) << ", " << toString(max_surprisingness_B) << "] , "
               <<"b_subpattern_num range = [" << toString(min_subpattern_num) << ", " << toString(max_subpattern_num) << "]"  << std::endl;


    for (const HTreeNode* htreeNode : resultPatterns)
    {

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count) << ", "

        << " nII_Surprisingness_b = " << toString(htreeNode->nII_Surprisingness_b) << ", "
        << " max_b_subpattern_num = " << toString(htreeNode->max_b_subpattern_num) << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (const Handle& link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
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

    boost::sort(resultPatterns, compareHTreeNodeByFrequency);

    resultFile.open(fileName.c_str());

    resultFile << ";Interesting Pattern Mining results for " + toString(gram) + " gram patterns. Total pattern number: " + toString(resultPatterns.size()) << endl;

    resultFile << ";This file contains the pattern with Frequency range = [" << min_frequency << ", " << max_frequency << "] ,"
               << "InteractionInformation range = [" << toString(min_ii) << ", " << toString(max_ii) << "]."  << std::endl;

    for (const HTreeNode* htreeNode : resultPatterns)
    {
//        if (OUTPUT_SURPRISINGNESS_CALCULATION_TO_FILE)
//        {
//            resultFile << endl << htreeNode->surprisingnessInfo  << endl;
//        }

        resultFile << endl << ";Pattern: Frequency = " << toString(htreeNode->count);

        resultFile << " InteractionInformation = " << htreeNode->interactionInformation;

        resultFile << endl;

        // resultFile << unifiedPatternToKeyString(htreeNode->pattern)<< endl;

        for (const Handle& link : htreeNode->pattern)
        {
            resultFile << link->to_short_string();
        }

        resultFile << std::endl;
    }

    resultFile << std::endl;
    resultFile.close();

    cout <<"\nDone!" << std::endl;
}

bool PatternMiner::containWhiteKeywords(const string& str, QUERY_LOGIC logic)
{
    return containKeywords(str, param.keyword_white_list, logic);
}

bool PatternMiner::containKeywords(const string& str, const set<string>& keywords, QUERY_LOGIC logic)
{
    auto is_in_str = [&](const string& keyword) {
        return str.find(keyword) != std::string::npos; };

    if (logic == QUERY_LOGIC::OR)
    {
        return any_of(keywords, is_in_str);
    }
    else // QUERY_LOGIC::AND
    {
        return all_of(keywords, is_in_str);
    }
}

void PatternMiner::applyWhiteListKeywordfilterAfterMining()
{
    if (patternsForGram[0].size() < 1)
    {
        std::cout << "\nPatternMiner:  this filter should be applied after mining! Please run pattern miner first!" << std::endl;
        return;
    }

    if (param.keyword_white_list.size() < 1)
    {
        std::cout << "\nPatternMiner:  white key word list is empty! Please set it first!" << std::endl;
        return;
    }

    string logic;
    if (param.keyword_white_list_logic == QUERY_LOGIC::OR)
       logic = "OR";
    else
       logic = "AND";

    string keywordlist = "";
    cout<<"\nPatternMiner:  applying keyword white list (" << logic << ") filter: ";
    for (string keyword : param.keyword_white_list)
    {
        keywordlist += (keyword + "-");
        cout << keyword << " ";
    }
    cout << std::endl;

    string fileNameBasic = "WhiteKeyWord-" + logic + "-" + keywordlist;

    vector<vector<HTreeNode*>> patternsForGramFiltered;

    for (unsigned int gram = 1; gram <= param.MAX_GRAM; gram ++)
    {
        vector<HTreeNode*> patternVector;
        patternsForGramFiltered.push_back(patternVector);

        for (HTreeNode* htreeNode : patternsForGram[gram-1])
        {
            if (htreeNode->count < param.threshold_frequency)
                break;

            string patternStr = unifiedPatternToKeyString(htreeNode->pattern);

            if (param.use_keyword_black_list && containKeywords(patternStr, param.keyword_black_list, QUERY_LOGIC::OR))
                continue;

            if (containWhiteKeywords(patternStr, param.keyword_white_list_logic))
                patternsForGramFiltered[gram-1].push_back(htreeNode);

        }

        // Finished mining gram patterns; output to file
        std::cout<<"gram = " + toString(gram) + ": " + toString((patternsForGramFiltered[gram-1]).size()) + " patterns found after filtering! ";
        boost::sort(patternsForGramFiltered[gram-1], compareHTreeNodeByFrequency);
        OutPutFrequentPatternsToFile(gram, patternsForGramFiltered, fileNameBasic);

        std::cout<< std::endl;
    }

    if (param.enable_interesting_pattern && (param.MAX_GRAM >1))
    {
        for (cur_gram = 2; cur_gram <= param.MAX_GRAM; cur_gram ++)
        {

            if (param.enable_interaction_information)
            {
                // sort by interaction information
                boost::sort(patternsForGramFiltered[cur_gram-1], compareHTreeNodeByInteractionInformation);
                OutPutInterestingPatternsToFile(patternsForGramFiltered[cur_gram-1], cur_gram, 0, fileNameBasic);
            }

            if (param.enable_surprisingness)
            {
                // sort by surprisingness_I first
                boost::sort(patternsForGramFiltered[cur_gram-1], compareHTreeNodeBySurprisingness_I);
                OutPutInterestingPatternsToFile(patternsForGramFiltered[cur_gram-1], cur_gram, 1, fileNameBasic);

                if (cur_gram == param.MAX_GRAM)
                    break;

                // sort by surprisingness_II first
                boost::sort(patternsForGramFiltered[cur_gram-1], compareHTreeNodeBySurprisingness_II);
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
        if (param.THREAD_NUM > 1)
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

            if (param.THREAD_NUM > 1)
                readNextPatternLock.unlock();

            break;
        }

        HTreeNode* htreeNode = patternsForGram[cur_gram - 1][cur_index];

        if (param.THREAD_NUM > 1)
            readNextPatternLock.unlock();

        // evaluate the interestingness
        // Only effective when Enable_Interesting_Pattern is true. The options are "Interaction_Information", "surprisingness"

        if (param.enable_surprisingness)
        {
            calculateSurprisingness(htreeNode);
        }


        if ((cur_gram > 1) && param.enable_interaction_information)
        {
            calculateInteractionInformation(htreeNode);
        }
    }
}

// select a subset for topics from the corpus
void PatternMiner::selectSubsetFromCorpus(const set<string>& topics, unsigned int gram, bool if_contian_logic)
{
    _selectSubsetFromCorpus(topics,gram, if_contian_logic);
}

bool checkIfObjectIsAPerson(const Handle& obj, const HandleSeq& listLinks)
{
    for (const Handle& l : listLinks)
    {
        // only check ListLinks with h as the first outgoing
        if (l->getOutgoingAtom(0) != obj)
            continue;

        HandleSeq evals;
        l->getIncomingSet(back_inserter(evals));

        for (const Handle& eval : evals)
        {
            Handle predicate = eval->getOutgoingAtom(0);
            string predicateStr = predicate->get_name();
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
    original_as.get_handles_by_type(back_inserter(allDBpediaKeyNodes), CONCEPT_NODE);
    cout << allDBpediaKeyNodes.size() << " nodes loaded!" << std::endl;
}

void PatternMiner::selectSubsetForDBpedia()
{
    HandleSet subsetLinks;

    // Handle newPredicate = original_as.add_node(PREDICATE_NODE, "position");

    string titles[] = {"president","chairman","vicePresident","primeMinister","vicePrimeMinister"};

    cout << "\nselecting president related links from DBpedia ... " << std::endl;
    // int x = 0;
    for (const Handle& h : allDBpediaKeyNodes)
    {
//        string objname = h->get_name();
//        if (objname == "Stanley_Crooks")
//        {
//            x ++;
//            cout << x << std::endl;
//        }

        bool is_person = false;
        bool already_check_is_person = false;

        HandleSeq listLinks;
        h->getIncomingSet(back_inserter(listLinks));

        for (const Handle& l : listLinks)
        {
            // only process ListLinks with h as the first outgoing
            if ( l->getOutgoingAtom(0) != h)
                continue;

            HandleSeq evals;
            l->getIncomingSet(back_inserter(evals));

            for (const Handle& eval : evals)
            {

//                // if the valueNode only has one connection, do not keep it
//                Handle valueNode = l->getOutgoingAtom(1);
//                if (valueNode->getIncomingSetSize() < 2)
//                {
//                    cout << valueNode->get_name() << " only has one connection, skip it!" << std::endl;
//
//                    continue;
//                }

                Handle predicate = eval->getOutgoingAtom(0);
                string predicateStr = predicate->get_name();

                if (param.use_keyword_black_list && isIgnoredContent(predicateStr))
                {
                    continue;
                }

                bool skip = false;
                for (const string& title : titles)
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
                            cout << h->get_name() << " is a not a person, keep it!\n";
                        }

                        break;

//                        Handle titlelistLink = original_as.add_link(LIST_LINK, valueNode, predicate);
//                        original_as.add_link(EVALUATION_LINK, newPredicate, titlelistLink);
//                        original_as.remove_atom(eval);
                    }
                }

                if (! skip)
	                subsetLinks.insert(eval);
            }
        }
    }

    cout << "\nDone!" << subsetLinks.size() << " links selected! Writing to file ..." << std::endl;

    ofstream subsetFile;

    string fileName = "DBPediaSubSet.scm";

    subsetFile.open(fileName.c_str());

    // write the first line to enable unicode
    subsetFile <<  "(setlocale LC_CTYPE \"\")" << std::endl;

    for (const Handle& h : subsetLinks)
    {
        subsetFile << h->to_short_string();
    }

    subsetFile.close();

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl;
}

std::string PatternMiner::Link2keyString(const Handle& h, const std::string& indent)
{
    std::stringstream answer;
    std::string more_indent = indent + LINE_INDENTATION;

    answer << indent  << "(" << nameserver().getTypeName(h->get_type()) << " ";

    if (h->is_node())
    {
        answer << h->get_name();
    }

    answer << ")" << "\n";

    if (h->is_link())
    {
        for (const Handle& outgoing : h->getOutgoingSet())
            answer << Link2keyString(outgoing, more_indent);
    }

    return answer.str();
}

void PatternMiner::testPatternMatcher()
{
    HandleSeq allAtomSpaceLinks;
    original_as.get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
    std::cout <<"Debug: PatternMiner total link number = "
              << allAtomSpaceLinks.size() << std::endl;

    HandleSeq variableNodes, bindLinkOutgoings, patternToMatch;

    Handle varHandle1 = original_as.add_node(opencog::VARIABLE_NODE,"$var_1" );
//    Handle varHandle2 = original_as.add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var_2" );
//    Handle varHandle3 = original_as.add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var_3" );
//    Handle varHandle4 = original_as.add_node(opencog::PATTERN_VARIABLENODE_TYPE,"$var_4" );

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
    Handle conceptNode = original_as.add_node(opencog::CONCEPT_NODE, "Male" );
    listlinkOutgoings1.push_back(varHandle1);
    listlinkOutgoings1.push_back(conceptNode);

    Handle listlink1 = original_as.add_link(LIST_LINK, listlinkOutgoings1);
    Handle PredicateNode = original_as.add_node(opencog::PREDICATE_NODE, "has_FC4_0906_Gender" );
    evalLinkOutgoings1.push_back(PredicateNode);
    evalLinkOutgoings1.push_back(listlink1);
    Handle evalLink1 = original_as.add_link(EVALUATION_LINK, evalLinkOutgoings1);

    HandleSeq listlinkOutgoings2, evalLinkOutgoings2;
    Handle conceptNode2 = original_as.add_node(opencog::CONCEPT_NODE, "Larex Personeelsbemiddeling" );
    listlinkOutgoings2.push_back(varHandle1);
    listlinkOutgoings2.push_back(conceptNode2);

    Handle listlink2 = original_as.add_link(LIST_LINK, listlinkOutgoings2);
    Handle PredicateNode2 = original_as.add_node(opencog::PREDICATE_NODE, "has_FC4_0906_EmployerName" );
    evalLinkOutgoings2.push_back(PredicateNode2);
    evalLinkOutgoings2.push_back(listlink2);
    Handle evalLink2 = original_as.add_link(EVALUATION_LINK, evalLinkOutgoings2);


//    // The InheritanceLink
//    HandleSeq inherOutgoings;
//    Handle NotHandicappedNode = original_as.add_node(opencog::CONCEPT_NODE, "soda drinker" );
//    inherOutgoings.push_back(varHandle1);
//    inherOutgoings.push_back(NotHandicappedNode);
//    Handle inherLink = original_as.add_link(INHERITANCE_LINK, inherOutgoings);

    patternToMatch.push_back(evalLink1);
    patternToMatch.push_back(evalLink2);

    Handle hAndLink = original_as.add_link(AND_LINK, patternToMatch);

    //Handle resultList = original_as.add_link(LIST_LINK, variableNodes);

    // add variable atoms
    Handle hVariablesListLink = original_as.add_link(VARIABLE_LIST, variableNodes);

    bindLinkOutgoings.push_back(hVariablesListLink);
    bindLinkOutgoings.push_back(hAndLink);
    bindLinkOutgoings.push_back(hAndLink);
    Handle hBindLink = original_as.add_link(BIND_LINK, bindLinkOutgoings);

    std::cout <<"Debug: PatternMiner::testPatternMatcher for pattern:" << std::endl
              << hBindLink->to_short_string().c_str() << std::endl;

    // Run pattern matcher
    Handle hResultListLink = bindlink(&original_as, hBindLink);

    // Get result
    // Note: Don't forget to remove the hResultListLink and BindLink
    HandleSeq resultSet = hResultListLink->getOutgoingSet();

    std::cout << toString(resultSet.size())  << " instances found:" << std::endl ;

    original_as.remove_atom(hResultListLink);

    for (Handle resultLink : resultSet)
        original_as.remove_atom(resultLink);

    //debug
    std::cout << hResultListLink->to_short_string() << std::endl  << std::endl;

    original_as.remove_atom(hBindLink);
    // original_as.remove_atom(resultList);
    original_as.remove_atom(hVariablesListLink);
    original_as.remove_atom(hAndLink);
    original_as.remove_atom(evalLink1);
    original_as.remove_atom(evalLink2);
    original_as.remove_atom(listlink1);
    original_as.remove_atom(listlink2);

    allAtomSpaceLinks.clear();
    original_as.get_handles_by_type(back_inserter(allAtomSpaceLinks), (Type) LINK, true );
    std::cout <<"After Pattern Matcher: PatternMiner total link number = "
              << allAtomSpaceLinks.size() << std::endl;
}

HandleSet PatternMiner::_getAllNonIgnoredLinksForGivenNode(Handle keywordNode, const HandleSet& allSubsetLinks)
{
    HandleSet newHandles;
    IncomingSet incomings = keywordNode->getIncomingSet(&original_as);

    // cout << "\n " << incomings.size() << " incomings found for keyword: " << keywordNode->to_short_string() << std::endl;
    for (LinkPtr incomingPtr : incomings)
    {
        Handle incomingHandle = incomingPtr->get_handle();
        Handle newh = incomingHandle;

        // if this atom is a igonred type, get its first parent that is not in the igonred types
        if (param.use_linktype_black_list && isIgnoredType (newh->get_type()) )
        {
            newh = getFirstNonIgnoredIncomingLink(original_as, newh);

            if ((newh == Handle::UNDEFINED))
                continue;
        }
        else if (param.use_linktype_white_list && (!is_in(newh->get_type(), param.linktype_white_list)))
        {
            continue;
        }

        if (param.use_keyword_black_list)
        {
            // if the content in this link contains content in the black list,ignore it
            if (param.keyword_black_logic_is_contain)
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

HandleSet PatternMiner::_extendOneLinkForSubsetCorpus(const HandleSet& allNewLinksLastGram, HandleSet& allSubsetLinks, HandleSet& extractedNodes)
{
    HandleSet allNewConnectedLinksThisGram;
    // only extend the links in allNewLinksLastGram. allNewLinksLastGram is a part of allSubsetLinks
    for (const Handle& link : allNewLinksLastGram)
    {
        // find all nodes in this link
        HandleSet allNodes;
        extractNodes(link, allNodes);

        for (const Handle& neighborNode : allNodes)
        {
            if (neighborNode->get_type() == PREDICATE_NODE)
                continue;

            string content = neighborNode->get_name();
            if (isIgnoredContent(content))
                continue;

            if (extractedNodes.find(neighborNode) != extractedNodes.end())
                continue;
            else
                extractedNodes.insert(neighborNode);

            HandleSet newConnectedLinks;
            newConnectedLinks = _getAllNonIgnoredLinksForGivenNode(neighborNode, allSubsetLinks);
            allNewConnectedLinksThisGram.insert(newConnectedLinks.begin(), newConnectedLinks.end());
            allSubsetLinks.insert(newConnectedLinks.begin(), newConnectedLinks.end());
        }
    }

    return allNewConnectedLinksThisGram;
}

// allSubsetLinks is  output
void PatternMiner::findAllLinksContainKeyWords(const set<string>& subsetKeywords, unsigned int max_connection, bool logic_contain, HandleSet& allSubsetLinks)
{
    allSubsetLinks.clear();
    HandleSet extractedNodes;

    if (allLinks.empty())
    {
        original_as.get_handles_by_type(back_inserter(allLinks), (Type) LINK, true );
    }

    if (logic_contain)
    {

        for (const Handle& link : allLinks)
        {
            Handle newh = link;

            // if this atom is a igonred type, get its first parent that is not in the igonred types
            if (param.use_linktype_black_list && isIgnoredType (link->get_type()) )
            {
                newh = getFirstNonIgnoredIncomingLink(original_as, link);

                if ((newh == Handle::UNDEFINED))
                    continue;
            }
            else if (param.use_linktype_white_list && (!is_in(link->get_type(), param.linktype_white_list)))
            {
                continue;
            }


            if (param.use_keyword_black_list)
            {
                // if the content in this link contains content in the black list,ignore it
                if (param.keyword_black_logic_is_contain)
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
                if (containKeywords(newh->to_short_string(), subsetKeywords, QUERY_LOGIC::OR))
                    allSubsetLinks.insert(newh);
//                else
//                {
//                    if (only_mine_patterns_start_from_white_list)
//                    {
//                        // add this Link into the observing_as
//                        HandleSeq outVariableNodes;

//                        HandleSeq outgoingLinks = swapLinkBetweenAtomSpaces(original_as, observing_as, newh, outVariableNodes);
//                        Handle newLink = observing_as->add_link(newh->get_type(), outgoingLinks);
//                        newLink->setTruthValue(newh->getTruthValue());
//                        linkNumLoadedIntoObservingAtomSpace ++;
//                    }
//                }
            }
        }
    }
    else
    {
        for (const string& keyword : subsetKeywords)
        {
            // std::cout << keyword << std::endl;
            Handle keywordNode = original_as.get_node(opencog::CONCEPT_NODE,keyword);
            if (keywordNode == Handle::UNDEFINED)
                keywordNode = original_as.get_node(opencog::PREDICATE_NODE,keyword);

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

void PatternMiner::selectSubsetAllEntityLinksContainsKeywords(set<string>& subsetKeywords)
{
    std::cout << "\nSelecting a subset from loaded corpus in Atomspace for the Entities contain following value keywords." << std::endl ;
    HandleSet allSubsetLinks;
    string topicsStr = "";

    for (const string& keyword : subsetKeywords)
    {
        std::cout << keyword << std::endl;
        topicsStr += "-";
        topicsStr += keyword;
    }

    findAllLinksContainKeyWords(subsetKeywords, 0, false, allSubsetLinks);

    HandleSet allEntityNodes;
    for (const Handle& link : allSubsetLinks)
    {  
        Handle firstOutgoing = link->getOutgoingAtom(1);
        Handle entityNode;
        if (firstOutgoing->is_link())
            entityNode = firstOutgoing->getOutgoingAtom(0);
        else
            entityNode = firstOutgoing;

        allEntityNodes.insert(entityNode);
    }


    int allEntityNum = allEntityNodes.size();
    std::cout << allEntityNum <<" entities has the predicate value found! Now find all the other Links contain these entities ..." << std::endl;

    int processEntityNum = 0;
    for (const Handle& entityNode : allEntityNodes)
    {
        HandleSet allLinks = _getAllNonIgnoredLinksForGivenNode(entityNode, allSubsetLinks);

        allSubsetLinks.insert(allLinks.begin(), allLinks.end());
        processEntityNum++;

        cout<< "\r" << ((float)(processEntityNum ))/((float)allEntityNum)*100.0f << "% completed."; // it's not liner
        std::cout.flush();

    }

    std::cout << "\n " << allSubsetLinks.size() << " Links found! Writing to file ..."  << std::endl ;

    ofstream subsetFile;

    string fileName = "SubSetEntityLinks" + topicsStr + ".scm";

    subsetFile.open(fileName.c_str());

    // write the first line to enable unicode
    subsetFile <<  "(setlocale LC_CTYPE \"\")" << std::endl ;

    for (const Handle& h : allSubsetLinks)
    {
        if (containIgnoredContent(h))
            continue;

        subsetFile << h->to_short_string();
    }

    subsetFile.close();

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl ;
}

// must load the corpus before calling this function
// logic_contain = true will find all the Nodes with a label contains any of the keywords,e.g.
// keyword = Premier , Node "32nd Premier of New South Wales" will be found if logic_contain = true;
// only Node "Premier" will be found if logic_contain = false
void PatternMiner::_selectSubsetFromCorpus(const set<string>& subsetKeywords, unsigned int max_connection, bool logic_contain)
{
    std::cout << "\nSelecting a subset from loaded corpus in Atomspace for the following keywords within " << max_connection << " distance:" << std::endl ;
    HandleSet allSubsetLinks;
    string topicsStr = "";

    for (const string& keyword : subsetKeywords)
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

    for (const Handle& h : allSubsetLinks)
    {
        if (containIgnoredContent(h))
            continue;

        subsetFile << h->to_short_string();
    }

    subsetFile.close();

    std::cout << "\nDone! The subset has been written to file:  " << fileName << std::endl ;
}

// recursively function
bool PatternMiner::loadOutgoingsIntoAtomSpaceFromString(stringstream& outgoingStream, AtomSpace& _as, HandleSeq &outgoings, string parentIndent)
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
        Type atomType = nameserver().getType(atomTypeStr);
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
                Handle node = _as.add_node(atomType, nodeName);
                outgoings.push_back(node);
            }
            else if (linkOrNodeStr == "Link")
            {
                // call this function recursively
                HandleSeq childOutgoings;
                if (! loadOutgoingsIntoAtomSpaceFromString(outgoingStream, _as, childOutgoings, curIndent))
                    return false;

                Handle link = _as.add_link(atomType, childOutgoings);
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
HandleSeq PatternMiner::loadPatternIntoAtomSpaceFromString(string patternStr, AtomSpace& _as)
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

    for (const string& linkStr : strs) // load each link
    {
        if (linkStr == "") continue;

        HandleSeq rootOutgoings;

        std::size_t firstLineEndPos = linkStr.find("\n");
        std::string rootOutgoingStr = linkStr.substr(firstLineEndPos + 1);
        stringstream outgoingStream(rootOutgoingStr);

        if (! loadOutgoingsIntoAtomSpaceFromString(outgoingStream, _as, rootOutgoings))
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

        Type atomType = nameserver().getType(atomTypeStr);
        if (NOTYPE == atomType)
        {

            cout << "Warning: loadPatternIntoAtomSpaceFromString: Not a valid typename: " << atomTypeStr << std::endl;
            HandleSeq emptyPattern;
            return emptyPattern;

        }

        Handle rootLink = _as.add_link(atomType, rootOutgoings);
        pattern.push_back(rootLink);
    }

    // debug:
    // static int pattern_num = 0;
    // string patternToStr = "";

    // for (Handle h : pattern)
    // {
    //    patternToStr += h->to_short_string();
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
bool PatternMiner::loadOutgoingsIntoAtomSpaceFromAtomString(stringstream& outgoingStream, AtomSpace& _as, HandleSeq &outgoings, string parentIndent)
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
        Type atomType = nameserver().getType(atomTypeStr);
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
                Handle node = _as.add_node(atomType, nodeName);
                outgoings.push_back(node);
            }
            else if (linkOrNodeStr == "Link")
            {
                // call this function recursively
                HandleSeq childOutgoings;
                if (! loadOutgoingsIntoAtomSpaceFromAtomString(outgoingStream, _as, childOutgoings, curIndent))
                    return false;

                Handle link = _as.add_link(atomType, childOutgoings);
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
HandleSeq PatternMiner::loadPatternIntoAtomSpaceFromFileString(string patternStr, AtomSpace& _as)
{
    std::vector<std::string> strs;
    boost::algorithm::split_regex( strs, patternStr, boost::regex( "\n\\)\n" ) ) ;

    HandleSeq pattern;

    for (const string& linkStr : strs) // load each link
    {
        if (linkStr == "") continue;

        HandleSeq rootOutgoings;

        std::size_t firstLineEndPos = linkStr.find("\n"); //(EvaluationLink (stv 1.000000 1.000000)\n
        std::string rootOutgoingStr = linkStr.substr(firstLineEndPos + 1);
        stringstream outgoingStream(rootOutgoingStr);

        if (! loadOutgoingsIntoAtomSpaceFromAtomString(outgoingStream, _as, rootOutgoings))
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

        Type atomType = nameserver().getType(atomTypeStr);
        if (NOTYPE == atomType)
        {

            cout << "Warning: loadPatternIntoAtomSpaceFromFileString: Not a valid typename: " << atomTypeStr << std::endl;
            HandleSeq emptyPattern;
            return emptyPattern;

        }

        Handle rootLink = _as.add_link(atomType, rootOutgoings);
        pattern.push_back(rootLink);
    }

    // debug:
//     static int pattern_num = 0;
//     string patternToStr = "";

//     for (Handle h : pattern)
//     {
//        patternToStr += h->to_short_string();
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

    for (std::string line; std::getline(resultFile, line);)
    {
        //cout <<"\nline: " << line << std::endl;
        if (patternStart && (line == "") && (lastLine == "")) // one pattern end, load it
        {
            // add this new found pattern into the Atomspace
            HandleSeq patternHandleSeq = loadPatternIntoAtomSpaceFromFileString(patternStr, *as);

            if (patternHandleSeq.empty())
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

            keyStrToHTreeNodeMap.insert({patternStr, newHTreeNode});
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
