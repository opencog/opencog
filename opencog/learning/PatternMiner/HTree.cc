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

#include "HTree.h"

namespace opencog {

using namespace std;
using namespace PatternMining;

std::string HTreeNode::to_string() const
{
	stringstream ss;
	if (not pattern.empty())
		ss << "pattern:" << std::endl
		   << oc_to_string(pattern);
	if (quotedPatternLink)
		ss << "quotedPatternLink:" << std::endl
		   << oc_to_string(quotedPatternLink);
	if (not instances.empty())
		ss << "instances:" << std::endl
		   << oc_to_string(instances);
	if (not parentLinks.empty())
		ss << "parentLinks:" << std::endl
		   << oc_to_string(parentLinks);
	if (not childLinks.empty())
		ss << "childLinks:" << std::endl
		   << oc_to_string(childLinks);
	if (not superPatternRelations.empty())
		ss << "superPatternRelations:" << std::endl
		   << oc_to_string(superPatternRelations);
	if (not superRelation_b_list.empty())
		ss << "superRelation_b_list:" << std::endl
		   << oc_to_string(superRelation_b_list);
	if (not SubRelation_b_map.empty())
		ss << "SubRelation_b_map:" << std::endl
		   << oc_to_string(SubRelation_b_map);
	if (count)
		ss << "count: " << count << std::endl;
	if (var_num)
		ss << "var_num: " << var_num << std::endl;
	if (interactionInformation)
		ss << "interactionInformation: " << interactionInformation << std::endl;
	if (nI_Surprisingness)
		ss << "nI_Surprisingness: " << nI_Surprisingness << std::endl;
	if (nII_Surprisingness)
		ss << "nII_Surprisingness: " << nII_Surprisingness << std::endl;
	if (nII_Surprisingness_b)
		ss << "nII_Surprisingness_b: " << nII_Surprisingness_b << std::endl;
	if (max_b_subpattern_num)
		ss << "max_b_subpattern_num: " << max_b_subpattern_num << std::endl;
	if (not surprisingnessInfo.empty())
		ss << "surprisingnessInfo: " << surprisingnessInfo << std::endl;
	if (not sharedVarNodeList.empty())
		ss << "sharedVarNodeList:" << std::endl
		   << oc_to_string(sharedVarNodeList);
	return ss.str();
}

std::string oc_to_string(const std::map<Handle, std::vector<SubRelation_b>>& sm)
{
    std::stringstream ss;
	ss << "size = " << sm.size();
	int i = 0;
	for (const auto& el : sm) {
		ss << "atom[" << i << "]:" << std::endl
		   << oc_to_string(el.first)
		   << "SubRelation_b sequence[" << i << "]:" << std::endl
		   << oc_to_string(el.second);
		i++;
	}
	return ss.str();
}

std::string oc_to_string(const std::vector<SuperRelation_b>& srbs)
{
	return oc_to_string(srbs, "SuperRelation_b");
}

std::string oc_to_string(const std::vector<SubRelation_b>& srbs)
{
	return oc_to_string(srbs, "SubRelation_b");
}

std::string oc_to_string(const SuperRelation_b& srb)
{
	stringstream ss;
	ss << "superHTreeNode:" << std::endl
	   << oc_to_string(srb.superHTreeNode)
	   << "constNode:" << std::endl
	   << oc_to_string(srb.constNode);
	return ss.str();
}

std::string oc_to_string(const SubRelation_b& srb)
{
	stringstream ss;
	ss << "subHTreeNode:" << std::endl
	   << oc_to_string(srb.subHTreeNode)
	   << "constNode:" << std::endl
	   << oc_to_string(srb.constNode);
	return ss.str();
}

std::string oc_to_string(const ExtendRelation& extrel)
{
	stringstream ss;
	ss << "extendedHTreeNode:" << std::endl
	   << oc_to_string(extrel.extendedHTreeNode)
	   << "sharedLink:" << std::endl
	   << oc_to_string(extrel.sharedLink)
	   << "newExtendedLink:" << std::endl
	   << oc_to_string(extrel.newExtendedLink)
	   << "extendedNode:" << std::endl
	   << oc_to_string(extrel.extendedNode)
	   << "isExtendedFromVar: " << extrel.isExtendedFromVar << std::endl;
	return ss.str();
}

std::string oc_to_string(const std::vector<ExtendRelation>& extrels)
{
	return oc_to_string(extrels, "ExtendRelation");
}

std::string oc_to_string(const std::vector<std::vector<HTreeNode*>>& htrees_seq)
{
	return oc_to_string(htrees_seq, "HTreeNodeSeq");
}

std::string oc_to_string(const std::vector<HTreeNode*>& htrees)
{
	return oc_to_string(htrees, "HTreeNode");
}

std::string oc_to_string(const std::set<HTreeNode*>& htrees)
{
	return oc_to_string(htrees, "HTreeNode");
}

std::string oc_to_string(const HTreeNode* htnptr)
{
	return htnptr ? htnptr->to_string() : string("none\n") ;
}

std::string oc_to_string(const HTreeNode& htn)
{
	return htn.to_string();
}

std::string oc_to_string(const HTree& htree)
{
	stringstream ss;
	ss << "rootNode:" << std::endl << htree.rootNode->to_string();
	return ss.str();
}

}
