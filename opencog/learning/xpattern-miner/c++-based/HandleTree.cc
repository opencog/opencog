/*
 * HandleTree.cc
 *
 * Copyright (C) 2017 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include "HandleTree.h"

#include <opencog/util/Logger.h>
#include <opencog/util/dorepeat.h>

#include <sstream>

namespace opencog {

bool content_eq(const HandleTree& htl, const HandleTree& htr)
{
	// Make sure forests are equal
	HandleTree::iterator itl = htl.begin(), itr = htr.begin();
	while (htl.is_valid(itl) and htr.is_valid(itr)) {
		if (not content_eq(itl, itr))
			return false;
		itl = htl.next_sibling(itl);
		itr = htr.next_sibling(itr);
	}
	return htl.is_valid(itl) == htr.is_valid(itr);
}

bool content_eq(HandleTree::iterator itl, HandleTree::iterator itr)
{
	HandleTree::sibling_iterator sibl = itl.begin(), sibr = itr.begin();
	for (; sibl != itl.end() and sibr != itr.end(); ++sibl, ++sibr)
		if (not content_eq(sibl, sibr))
			return false;
	return (sibl == itl.end()) == (sibr == itr.end());
}

bool content_is_in(const Handle& h, const HandleTree& ht)
{
	// TODO: optimize
	for (const Handle& oh : ht)
		if (content_eq(h, oh))
			return true;
	return false;
}

HandleTree merge_patterns(const std::initializer_list<HandleTree>& forests)
{
	// Build forest of forests
	HandleTree forest(forests);

	// TODO: actually the assumption below is not right!!!!! So in
	// order to remove redundants, then must be attached to the other
	// one. I don't like that, because then you loose the trace of how
	// things are being produced.

	// // Remove duplicates
	// HandleSet cash;
	// for (auto it = forest.begin(); it != forest.end();)
	// {
	// 	// Not in the cash? Don't remove it and add it to the cash
	// 	if (cash.find(*it) == cash.end())
	// 	{
	// 		cash.insert(*it);
	// 		++it;
	// 		continue;
	// 	}

	// 	// // In the cash, remove it (and all its children, supposedly in
	// 	// // the cash as well)

	// 	// // TODO: remove the following check when we know we can make
	// 	// // the assumption that all children are in the cash as well
	// 	// // (thus in the forest explored thus far).
	// 	// logger().debug() << "OC_ASSERT all_nodes_in it = " << oc_to_string(HandleTree(it));
	// 	// OC_ASSERT(all_nodes_in(cash, it), "Noooooooooooo!");

	// 	it = forest.erase(it);
	// }

	return forest;
}

bool all_nodes_in(const HandleSet& cash, HandleTree::iterator it)
{
	if (cash.find(*it) == cash.end()) {
		logger().debug() << "all_nodes_in cash = " << oc_to_string(cash)
		                 << ", *it = " << oc_to_string(*it);
		return false;
	}

	for (auto cit = it.begin(); cit != it.end(); ++cit)
		if (not all_nodes_in(cash, cit))
			return false;

	return true;
}

std::string oc_to_string(const HandleTree& ht, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << ht.size() << std::endl;
	unsigned i = 0;
	for (HandleTree::iterator it = ht.begin(); it != ht.end(); ++it)
	{
		int depth = ht.depth(it);
		std::string node_indent = indent;
		dorepeat(depth)
			node_indent += OC_TO_STRING_INDENT;
		ss << node_indent << "atom[" << i << ",depth=" << ht.depth(it) << "]:"
		   << std::endl << oc_to_string(*it, node_indent + OC_TO_STRING_INDENT);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const HandleTree& ht)
{
	return oc_to_string(ht, "");
}

std::string oc_to_string(const HandleMapTree& hmt, const std::string& indent)
{
	// TODO: show the hierarchy
	std::stringstream ss;
	ss << indent << "size = " << hmt.size() << std::endl;
	unsigned i = 0;
	for (HandleMapTree::iterator it = hmt.begin(); it != hmt.end(); ++it) {
		ss << indent << "handle map[" << i << ",depth=" << hmt.depth(it) << "]:"
		   << std::endl << oc_to_string(*it, indent + OC_TO_STRING_INDENT);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const HandleMapTree& hmt)
{
	return oc_to_string(hmt, "");
}

std::string oc_to_string(const HandleHandleTreeMap& hhtm, const std::string& indent)
{
	std::stringstream ss;
	ss << indent << "size = " << hhtm.size() << std::endl;
	unsigned i = 0;
	for (const auto& hht : hhtm) {
		ss << indent << "atom[" << i << "]:" << std::endl
		   << oc_to_string(hht.first, indent + OC_TO_STRING_INDENT);
		ss << indent << "handle tree[" << i << "]:" << std::endl
		   << oc_to_string(hht.second, indent + OC_TO_STRING_INDENT);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const HandleHandleTreeMap& hhtm)
{
	return oc_to_string(hhtm, "");
}

} // ~namespace opencog
