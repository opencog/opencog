/**
 * PatternUtils.h
 *
 * Utilities for navigating a tree of outgoing edges.
 *
 * Copyright (C) 2009, 2014 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
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
 *
 * Created by Linas Vepstas February 2008
 */

#ifndef _OPENCOG_PATTERN_UTILS_H
#define _OPENCOG_PATTERN_UTILS_H

#include <set>
#include <vector>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/types.h>

namespace opencog {

///  Find all of the variable nodes occuring in a clause.
class FindVariables
{
	public:
		std::set<Handle> varset;

		/**
		 * Create a set of all of the VariableNodes that lie in the
		 * outgoing set of the handle (recursively).
		 */
		inline void find_vars(Handle h)
		{
			Type t = h->getType();
			if (classserver().isNode(t))
			{
				if (t == VARIABLE_NODE) varset.insert(h);
				return;
			}

			LinkPtr l(LinkCast(h));
			for (Handle oh : l->getOutgoingSet()) find_vars(oh);
		}

		inline void find_vars(std::vector<Handle> hlist)
		{
			for (Handle h : hlist) find_vars(h);
		}
};

/**
 * Return true if the indicated node occurs somewhere in the tree
 * (viz, the tree recursively spanned by the outgoing set of the handle)
 */
static inline bool is_node_in_tree(const Handle& tree, const Handle& node)
{
	if (tree == Handle::UNDEFINED) return false;
	if (tree == node) return true;
	LinkPtr ltree(LinkCast(tree));
	if (NULL == ltree) return false;

	// Recurse downwards...
	const std::vector<Handle> &vh = ltree->getOutgoingSet();
	size_t sz = vh.size();
	for (size_t i = 0; i < sz; i++) {
		if (is_node_in_tree(vh[i], node)) return true;
	}
	return false;
}

/**
 * Return true if the indicated node occurs quoted somewhere in the
 * tree.  That is, it returns true only if the node is inside a
 * QuoteLink.
 */
static inline bool is_quoted_in_tree(const Handle& tree, const Handle& node)
{
	if (tree == Handle::UNDEFINED) return false;
	if (tree == node) return false;  // not quoted, so false.
	LinkPtr ltree(LinkCast(tree));
	if (NULL == ltree) return false;

	if (tree->getType() == QUOTE_LINK)
	{
		if (is_node_in_tree(tree, node)) return true;
		return false;
	}

	// Recurse downwards...
	const std::vector<Handle> &vh = ltree->getOutgoingSet();
	size_t sz = vh.size();
	for (size_t i = 0; i < sz; i++) {
		if (is_quoted_in_tree(vh[i], node)) return true;
	}
	return false;
}

/**
 * Return true if any of the indicated nodes occurs somewhere in
 * the tree (that is, in the tree spanned by the outgoing set.)
 */
static inline bool any_node_in_tree(const Handle& tree, const std::set<Handle>& nodes)
{
	for (Handle n: nodes)
	{
		if (is_node_in_tree(tree, n)) return true;
	}
	return false;
}

/**
 * Return true if the indicated node occurs somewhere in any of the trees.
 */
static inline bool is_node_in_any_tree(const std::vector<Handle>& trees, const Handle& node)
{
	for (Handle tree: trees)
	{
		if (is_node_in_tree(tree, node)) return true;
	}
	return false;
}

/**
 * Returns true if the clause contains an atom of type atom_type.
 * ... but only if it is not quoted.  Quoted terms are constants.
 */
static inline bool contains_atomtype(Handle& clause, Type atom_type)
{
	Type clause_type = clause->getType();
	if (QUOTE_LINK == clause_type) return false;
	if (classserver().isA(clause_type, atom_type)) return true;

	LinkPtr lc(LinkCast(clause));
	if (not lc) return false;

	const std::vector<Handle> &oset = lc->getOutgoingSet();
	std::vector<Handle>::const_iterator i = oset.begin();
	std::vector<Handle>::const_iterator iend = oset.end();
	for (; i != iend; i++)
	{
		Handle subclause(*i);
		if (contains_atomtype(subclause, atom_type)) return true;
	}
	return false;
}

// Make sure that variables can be found in the clauses.
// See C file for description
bool remove_constants(const std::set<Handle> &vars,
                      std::vector<Handle> &clauses);


// See C file for description
void get_connected_components(
                    const std::set<Handle> &vars,
                    const std::vector<Handle> &clauses,
                    std::set<std::vector<Handle>> &compset);

} // namespace opencog

#endif // _OPENCOG_PATTERN_UTILS_H
