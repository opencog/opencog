/*
 * opencog/learning/PatternMiner/HTree.h
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

#ifndef _OPENCOG_PATTERNMINER_HTREE_H
#define _OPENCOG_PATTERNMINER_HTREE_H

#include <map>
#include <vector>

#include <opencog/util/empty_string.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog { namespace PatternMining {

class HTreeNode;

struct ExtendRelation // to store a super pattern of a pattern
{
	HTreeNode* extendedHTreeNode; // super pattern HTreeNode
	Handle sharedLink; // link in the original pattern that connect to new extended Link
	Handle newExtendedLink; // in super pattern (contains variables, not the instance link), without unifying
	Handle extendedNode; // node that being extended in the original AtomSpace (the value node, not its variable name node)
	bool isExtendedFromVar;
};

struct SuperRelation_b // to store a super pattern of the same gram of a pattern
{
	HTreeNode* superHTreeNode; // the super pattern HTreeNode
	Handle constNode; // the const Node
};

struct SubRelation_b // to store a sub pattern of the same gram of a pattern
{
	HTreeNode* subHTreeNode; // the sub pattern HTreeNode
	Handle constNode; // the const Node
};

class HTreeNode
{
public:
	HandleSeq pattern;
	Handle quotedPatternLink; // only used when if_quote_output_pattern = true
	HandleSeqSeq instances; // the corresponding instances of this pattern in the original AtomSpace, only be used by breadth first mining
	std::set<HTreeNode*> parentLinks;
	std::set<HTreeNode*> childLinks;

	// set<string> instancesUidStrings;// all uid in each instance HandleSeq in all instances, in the form of 5152_815_201584. to prevent the same instance being count multiple times

	std::vector<ExtendRelation> superPatternRelations; // store all the connections to its super patterns

	std::vector<SuperRelation_b> superRelation_b_list; // store all the superRelation_b

	std::map<Handle, std::vector<SubRelation_b>> SubRelation_b_map;// map<VariableNode, vector<SubRelation_b>>

	unsigned int count; // Number of instances grounding this pattern
	unsigned int var_num; // Number of the variables in this pattern
	double interactionInformation;
	double nI_Surprisingness;
	double nII_Surprisingness;
	double nII_Surprisingness_b; // calculated from the other same gram patterns
	unsigned int max_b_subpattern_num;

	std::string surprisingnessInfo; // the middle info record the surpringness calculating process for this pattern

	HandleSeq sharedVarNodeList; // all the shared nodes in these links in the original AtomSpace, each handle is a shared node

	std::string to_string(const std::string& indent=empty_string) const;

	HTreeNode()
		: count(0),
		  var_num(0),
		  interactionInformation(0.0),
		  nI_Surprisingness(0.0),
		  nII_Surprisingness(0.0),
		  nII_Surprisingness_b(1.0),
		  max_b_subpattern_num(0) {}
};

} // ~namespace PatterMining

using namespace PatternMining;

std::string oc_to_string(const std::map<Handle, std::vector<SubRelation_b>>& sm,
                         const std::string& indent=empty_string);
std::string oc_to_string(const std::vector<SuperRelation_b>& srbs,
                         const std::string& indent=empty_string);
std::string oc_to_string(const std::vector<SubRelation_b>& srbs,
                         const std::string& indent=empty_string);
std::string oc_to_string(const SuperRelation_b& srb,
                         const std::string& indent=empty_string);
std::string oc_to_string(const SubRelation_b& srb,
                         const std::string& indent=empty_string);
std::string oc_to_string(const ExtendRelation& extrel,
                         const std::string& indent=empty_string);
std::string oc_to_string(const std::vector<ExtendRelation>& extrel,
                         const std::string& indent=empty_string);
std::string oc_to_string(const std::vector<std::vector<HTreeNode*>>& htrees,
                         const std::string& indent=empty_string);
std::string oc_to_string(const std::vector<HTreeNode*>& htrees,
                         const std::string& indent=empty_string);
std::string oc_to_string(const std::set<HTreeNode*>& htrees,
                         const std::string& indent=empty_string);
std::string oc_to_string(const HTreeNode* htnptr,
                         const std::string& indent=empty_string);
std::string oc_to_string(const HTreeNode& htn,
                         const std::string& indent=empty_string);

/**
 * Template to simplify opencog container string convertion. Name is
 * the name of the container, it will output
 *
 * size = <size of the container>
 * <elname>[0]:
 * <content of first element>
 * ...
 * <elname>[size-1]:
 * <content of last element>
 *
 * The content of each element is obtained using oc_to_string. It
 * assumes that the content string of an element ends by endl.
 *
 * TODO: move this to cogutil
 */
template<typename C>
std::string oc_to_string(const C& c, const std::string& indent, const std::string& elname)
{
	std::stringstream ss;
	ss << indent << "size = " << c.size() << std::endl;
	int i = 0;
	for (const auto& el : c) {
		ss << indent << elname << "[" << i << "]:" << std::endl
		   << oc_to_string(el, indent + OC_TO_STRING_INDENT);
		i++;
	}
	return ss.str();
}

} // ~namespace opencog

#endif //_OPENCOG_PATTERNMINER_HTREE_H
