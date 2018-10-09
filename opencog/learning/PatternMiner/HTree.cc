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

std::string HTreeNode::to_string(const std::string& indent) const
{
	stringstream ss;
	if (not pattern.empty())
		ss << indent << "pattern:" << std::endl
		   << oc_to_string(pattern, indent + OC_TO_STRING_INDENT);
	if (quotedPatternLink)
		ss << indent << "quotedPatternLink:" << std::endl
		   << oc_to_string(quotedPatternLink, indent + OC_TO_STRING_INDENT);
	if (not instances.empty())
		ss << indent << "instances:" << std::endl
		   << oc_to_string(instances, indent + OC_TO_STRING_INDENT);
	if (not parentLinks.empty())
		ss << indent << "parentLinks:" << std::endl
		   << oc_to_string(parentLinks, indent + OC_TO_STRING_INDENT);
	if (not childLinks.empty())
		ss << indent << "childLinks:" << std::endl
		   << oc_to_string(childLinks, indent + OC_TO_STRING_INDENT);
	if (not superPatternRelations.empty())
		ss << indent << "superPatternRelations:" << std::endl
		   << oc_to_string(superPatternRelations, indent + OC_TO_STRING_INDENT);
	if (not superRelation_b_list.empty())
		ss << indent << "superRelation_b_list:" << std::endl
		   << oc_to_string(superRelation_b_list, indent + OC_TO_STRING_INDENT);
	if (not SubRelation_b_map.empty())
		ss << indent << "SubRelation_b_map:" << std::endl
		   << oc_to_string(SubRelation_b_map, indent + OC_TO_STRING_INDENT);
	if (count)
		ss << indent << "count: " << count << std::endl;
	if (var_num)
		ss << indent << "var_num: " << var_num << std::endl;
	if (interactionInformation)
		ss << indent << "interactionInformation: " << interactionInformation << std::endl;
	if (nI_Surprisingness)
		ss << indent << "nI_Surprisingness: " << nI_Surprisingness << std::endl;
	if (nII_Surprisingness)
		ss << indent << "nII_Surprisingness: " << nII_Surprisingness << std::endl;
	if (nII_Surprisingness_b)
		ss << indent << "nII_Surprisingness_b: " << nII_Surprisingness_b << std::endl;
	if (max_b_subpattern_num)
		ss << indent << "max_b_subpattern_num: " << max_b_subpattern_num << std::endl;
	if (not surprisingnessInfo.empty())
		ss << indent << "surprisingnessInfo: " << surprisingnessInfo << std::endl;
	if (not sharedVarNodeList.empty())
		ss << indent << "sharedVarNodeList:" << std::endl
		   << oc_to_string(sharedVarNodeList, indent + OC_TO_STRING_INDENT);
	return ss.str();
}

std::string oc_to_string(const std::map<Handle, std::vector<SubRelation_b>>& sm,
                         const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << sm.size();
	int i = 0;
	for (const auto& el : sm) {
		ss << indent << "atom[" << i << "]:" << std::endl
		   << oc_to_string(el.first, indent + OC_TO_STRING_INDENT)
		   << indent << "SubRelation_b sequence[" << i << "]:" << std::endl
		   << oc_to_string(el.second, indent + OC_TO_STRING_INDENT);
		i++;
	}
	return ss.str();
}

std::string oc_to_string(const std::vector<SuperRelation_b>& srbs,
                         const std::string& indent)
{
	return oc_to_string(srbs, indent, "SuperRelation_b");
}

std::string oc_to_string(const std::vector<SubRelation_b>& srbs,
                         const std::string& indent)
{
	return oc_to_string(srbs, indent, "SubRelation_b");
}

std::string oc_to_string(const SuperRelation_b& srb,
                         const std::string& indent)
{
	stringstream ss;
	ss << indent << "superHTreeNode:" << std::endl
	   << oc_to_string(srb.superHTreeNode, indent + OC_TO_STRING_INDENT)
	   << indent << "constNode:" << std::endl
	   << oc_to_string(srb.constNode, indent + OC_TO_STRING_INDENT);
	return ss.str();
}

std::string oc_to_string(const SubRelation_b& srb,
                         const std::string& indent)
{
	stringstream ss;
	ss << indent << "subHTreeNode:" << std::endl
	   << oc_to_string(srb.subHTreeNode, indent + OC_TO_STRING_INDENT)
	   << indent << "constNode:" << std::endl
	   << oc_to_string(srb.constNode, indent + OC_TO_STRING_INDENT);
	return ss.str();
}

std::string oc_to_string(const ExtendRelation& extrel,
                         const std::string& indent)
{
	stringstream ss;
	ss << indent << "extendedHTreeNode:" << std::endl
	   << oc_to_string(extrel.extendedHTreeNode, indent + OC_TO_STRING_INDENT)
	   << indent << "sharedLink:" << std::endl
	   << oc_to_string(extrel.sharedLink, indent + OC_TO_STRING_INDENT)
	   << indent << "newExtendedLink:" << std::endl
	   << oc_to_string(extrel.newExtendedLink, indent + OC_TO_STRING_INDENT)
	   << indent << "extendedNode:" << std::endl
	   << oc_to_string(extrel.extendedNode, indent + OC_TO_STRING_INDENT)
	   << indent << "isExtendedFromVar: " << extrel.isExtendedFromVar << std::endl;
	return ss.str();
}

std::string oc_to_string(const std::vector<ExtendRelation>& extrels,
                         const std::string& indent)
{
	return oc_to_string(extrels, indent, "ExtendRelation");
}

std::string oc_to_string(const std::vector<std::vector<HTreeNode*>>& htrees_seq,
                         const std::string& indent)
{
	return oc_to_string(htrees_seq, indent, "htrees");
}

std::string oc_to_string(const std::vector<HTreeNode*>& htrees,
                         const std::string& indent)
{
	return oc_to_string(htrees, indent, "htree");
}

std::string oc_to_string(const std::set<HTreeNode*>& htrees,
                         const std::string& indent)
{
	return oc_to_string(htrees, indent, "HTreeNode");
}

std::string oc_to_string(const HTreeNode* htnptr,
                         const std::string& indent)
{
	if (htnptr)
		return htnptr->to_string(indent);
	return indent + string("none\n");
}

std::string oc_to_string(const HTreeNode& htn,
                         const std::string& indent)
{
	return htn.to_string(indent);
}

}
