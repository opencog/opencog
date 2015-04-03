/**
 * PatternUtils.h
 *
 * Utilities for navigating a tree of outgoing edges.
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas <linasvepstas@gmail.com>
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

/// Find a "target atom", or find all atoms of a given "target type",
/// and all of the links that hold that target, that occur in a clause
/// (an expression tree formed by the outgoing set.) This is typically
/// used to find all variables in an expression, but it can be used for
/// any purpose.
///
/// After find_atoms() is called, the set of target atoms that were found
/// can be retreived from the public member `varset`.  The set of links
/// that had one of these targets occuring somewhere within them is in
/// the public member `holders`.
///
/// A "target" is defined in one of two ways: It is either a specified
/// target type, or it is any one of the given set of specific target
/// atoms.  When only the type is given, it will typically be
/// VARIABLE_NODE, GROUNDED_PREDICATE_NODE or GROUNDED_SCHEMA_NODE,
/// COMPOSE_LINK, or something like that, and so this just finds these
/// atom types if they occur in the clause(s).  Alternately, if a set
/// of specific atoms is given, then this finds the subset of that set
/// that actually occurs in the given clause(s).  That is, it computes
/// the intersection between the set of given atoms, and the set of all
/// atoms that occur in the clauses.
///
/// Note that anything occuring below a QUOTE_LINK is not explored.
/// Thus, a quote acts like a cut, halting recursion.
///
class FindAtoms
{
	public:
		std::set<Handle> varset;
		std::set<Handle> holders;

		inline FindAtoms(Type t, bool recurs_hold=true)
			: _recursive_hold(recurs_hold), _var_type(t) {}
		inline FindAtoms(const std::set<Handle>& selection)
			: _recursive_hold(true), _var_type(NOTYPE),
			 _var_domain(selection) {}

		/**
		 * Create a set of all of the VariableNodes that lie in the
		 * outgoing set of the handle (recursively).
		 */
		inline bool find_atoms(Handle h)
		{
			Type t = h->getType();
			if ((t == _var_type) or _var_domain.count(h) == 1)
			{
				varset.insert(h);
				return true; //! Don't explore link-typed vars!
			}

			if (QUOTE_LINK == t) return false;

			LinkPtr l(LinkCast(h));
			if (l)
			{
				bool held = false;
				for (Handle oh : l->getOutgoingSet())
				{
					if (find_atoms(oh)) held = true;
				}
				if (held) holders.insert(h);
				return _recursive_hold and held;
			}
			return false;
		}

		inline void find_atoms(std::vector<Handle> hlist)
		{
			for (Handle h : hlist) find_atoms(h);
		}
	private:
		bool _recursive_hold;
		Type _var_type;
		std::set<Handle> _var_domain;
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
 * Return true if the indicated node occurs somewhere in the tree
 * (viz, the tree recursively spanned by the outgoing set of the handle)
 * but ONLY if it is not quoted!  This is meant to be be used to search
 * for variables, but only those variables that have not been quoted, as
 * the quoted variables are constants (literals).
 */
static inline bool is_variable_in_tree(const Handle& tree, const Handle& node)
{
	if (tree == Handle::UNDEFINED) return false;
	if (tree == node) return true;
	LinkPtr ltree(LinkCast(tree));
	if (NULL == ltree) return false;

	if (tree->getType() == QUOTE_LINK) return false;

	// Recurse downwards...
	const std::vector<Handle> &vh = ltree->getOutgoingSet();
	size_t sz = vh.size();
	for (size_t i = 0; i < sz; i++) {
		if (is_variable_in_tree(vh[i], node)) return true;
	}
	return false;
}

/**
 * Return true if any of the indicated nodes occurs somewhere in
 * the tree (that is, in the tree spanned by the outgoing set.)
 */
static inline bool any_node_in_tree(const Handle& tree,
                                    const std::set<Handle>& nodes)
{
	for (Handle n: nodes)
	{
		if (is_node_in_tree(tree, n)) return true;
	}
	return false;
}

/**
 * Return true if any of the indicated nodes occurs somewhere in
 * the tree (that is, in the tree spanned by the outgoing set.)
 * But ONLY if they are not quoted!  This is intended to be used to
 * search for variables; but when a variable is quoted, it is no
 * longer a variable.
 */
static inline bool any_variable_in_tree(const Handle& tree,
                                        const std::set<Handle>& nodes)
{
	for (Handle n: nodes)
	{
		if (is_variable_in_tree(tree, n)) return true;
	}
	return false;
}

/**
 * Return true if the indicated node occurs somewhere in any of the trees.
 */
static inline bool is_node_in_any_tree(const std::vector<Handle>& trees,
                                       const Handle& node)
{
	for (Handle tree: trees)
	{
		if (is_node_in_tree(tree, node)) return true;
	}
	return false;
}

/**
 * Return true if the indicated node occurs somewhere in any of the trees,
 * but only if it is not quoted.  This is intended to be used to search
 * for variables, which cease to be variable when they are quoted.
 */
static inline bool is_variable_in_any_tree(const std::vector<Handle>& trees,
                                           const Handle& node)
{
	for (Handle tree: trees)
	{
		if (is_variable_in_tree(tree, node)) return true;
	}
	return false;
}

/**
 * Returns true if the clause contains an atom of type atom_type.
 * ... but only if it is not quoted.  Quoted terms are constants (literals).
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
	for (; i != iend; ++i)
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
void get_connected_components(const std::set<Handle> &vars,
                              const std::vector<Handle> &clauses,
                              std::set<std::vector<Handle>> &compset);

// Split clauses into positive and negative clauses (a negative clause
// starts with NotLink)
void split_clauses_pos_neg(const std::vector<Handle>& clauses,
                           std::vector<Handle>& affirm,
                           std::vector<Handle>& negate);

} // namespace opencog

#endif // _OPENCOG_PATTERN_UTILS_H
