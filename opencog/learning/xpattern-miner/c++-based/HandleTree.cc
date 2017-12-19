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
	for (const Handle& oh : ht)
		if (content_eq(h, oh))
			return true;
	return false;
}

std::string oc_to_string(const HandleTree& ht)
{
	std::stringstream ss;
	ss << "size = " << ht.size() << std::endl;
	unsigned i = 0;
	for (HandleTree::iterator it = ht.begin(); it != ht.end(); ++it) {
		ss << "atom[" << i << ",depth=" << ht.depth(it) << "]:"
		   << std::endl << oc_to_string(*it);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const HandleMapTree& hmt)
{
	// TODO: show the hierarchy
	std::stringstream ss;
	ss << "size = " << hmt.size() << std::endl;
	unsigned i = 0;
	for (HandleMapTree::iterator it = hmt.begin(); it != hmt.end(); ++it) {
		ss << "handle map[" << i << ",depth=" << hmt.depth(it) << "]:"
		   << std::endl << oc_to_string(*it);
		++i;
	}
	return ss.str();
}

std::string oc_to_string(const HandleHandleTreeMap& hhtm)
{
	std::stringstream ss;
	ss << "size = " << hhtm.size() << std::endl;
	unsigned i = 0;
	for (const auto& hht : hhtm) {
		ss << "atom[" << i << "]:" << std::endl << oc_to_string(hht.first);
		ss << "handle tree[" << i << "]:"
		   << std::endl << oc_to_string(hht.second);
		++i;
	}
	return ss.str();
}

} // ~namespace opencog
