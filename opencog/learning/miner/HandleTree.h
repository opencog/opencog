/*
 * HandleTree.h
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

#ifndef OPENCOG_HANDLETREE_H_
#define OPENCOG_HANDLETREE_H_

#include <opencog/atoms/base/Handle.h>
#include <opencog/util/tree.h>

namespace opencog {

typedef tree<Handle> HandleTree;
typedef tree<HandleMap> HandleMapTree;
typedef std::map<Handle, HandleTree> HandleHandleTreeMap;

bool content_eq(const HandleTree& htl, const HandleTree& htr);
bool content_eq(HandleTree::iterator itl, HandleTree::iterator itr);
bool content_is_in(const Handle& h, const HandleTree& ht);

/**
 * Given a list of forests of patterns, merge them into a forest such
 * that duplicates are discarded.
 */
HandleTree merge_patterns(const std::initializer_list<HandleTree>&);

/**
 * Check that it and all its children are in cash.
 */
bool all_nodes_in(const HandleSet& cash, HandleTree::iterator it);

std::string oc_to_string(const HandleTree& ht, const std::string& indent);
std::string oc_to_string(const HandleTree& ht);
std::string oc_to_string(const HandleMapTree& hmt, const std::string& indent);
std::string oc_to_string(const HandleMapTree& hmt);
std::string oc_to_string(const HandleHandleTreeMap& hhtm, const std::string& indent);
std::string oc_to_string(const HandleHandleTreeMap& hhtm);

} // ~namespace opencog

#endif /* OPENCOG_HANDLETREE_H_ */
