/**
 * FindUtils.h
 *
 * Utilities for finding atoms in trees.
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

/// There are two different types of utilities in this file: those that
/// tell you if some target atom or target atom type occurs in some
/// tree, returning a simple yes/no answer, and another utility, telling
/// you exactly where it occurs.  We start with the more complicated one
/// first.  Otherwise, skip down to get at the simpler ones.

/// Find a "target atom", or find all atoms of a given "target type",
/// and all of the links that hold that target, that occur in a clause
/// (an expression tree formed by the outgoing set.) This is typically
/// used to find all variables in an expression, but it can be used for
/// any purpose.
///
/// After find_atoms() is called, the set of target atoms that were found
/// can be retreived from the public member `varset`.  The set of links
/// that had one of these targets occuring somewhere, anywhere, within
/// them is in the public member `holders`. This includes the holders of
/// holders, etc, all the way up to the very top.  The `least_holders`
/// member will contain only those links that contain a target in thier
/// immediate outgoing set.  That is, the `least_holders` are the
/// smallest links that contain the target.
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
		std::set<Handle> least_holders;

		inline FindAtoms(Type t)
			: _target_types({t}) {}
		inline FindAtoms(Type ta, Type tb)
			: _target_types({ta, tb}) {}
		inline FindAtoms(const std::set<Handle>& selection)
			: _target_types(),
			 _target_atoms(selection) {}

		/**
		 * Create a set of all of the target atoms/types that lie in the
		 * outgoing set of the handle (recursively).
		 */
		inline void find_atoms(const Handle& h)
		{
			find_rec(h);
		}

		inline void find_atoms(const std::vector<Handle>& hlist)
		{
			for (const Handle& h : hlist) find_atoms(h);
		}
	private:
		typedef enum
		{
			NOPE,  // Does not contain.
			YEP,   // Contains, but not immediately so.
			IMM    // Contains immediately below.
		} Loco;

		inline Loco find_rec(const Handle& h)
		{
			Type t = h->getType();
			if (1 == _target_types.count(t) or _target_atoms.count(h) == 1)
			{
				varset.insert(h);
				return IMM; //! Don't explore link-typed vars!
			}

			if (QUOTE_LINK == t) return NOPE;

			LinkPtr l(LinkCast(h));
			if (l)
			{
				bool held = false;
				bool imm = false;
				for (const Handle& oh : l->getOutgoingSet())
				{
					Loco where = find_rec(oh);
					if (NOPE != where) held = true;
					if (IMM == where) imm = true;
				}
				if (held) holders.insert(h);
				if (imm) least_holders.insert(h);
				return YEP;
			}
			return NOPE;
		}

	private:
		std::set<Type> _target_types;
		std::set<Handle> _target_atoms;
};

/**
 * Return true if the indicated atom occurs somewhere in the tree
 * (viz, the tree recursively spanned by the outgoing set of the handle)
 */
static inline bool is_atom_in_tree(const Handle& tree, const Handle& atom)
{
	if (tree == Handle::UNDEFINED) return false;
	if (tree == atom) return true;
	LinkPtr ltree(LinkCast(tree));
	if (NULL == ltree) return false;

	// Recurse downwards...
	for (const Handle h: ltree->getOutgoingSet()) {
		if (is_atom_in_tree(h, atom)) return true;
	}
	return false;
}

/**
 * Return true if the indicated atom occurs quoted somewhere in the
 * tree.  That is, it returns true only if the atom is inside a
 * QuoteLink.
 */
static inline bool is_quoted_in_tree(const Handle& tree, const Handle& atom)
{
	if (tree == Handle::UNDEFINED) return false;
	if (tree == atom) return false;  // not quoted, so false.
	LinkPtr ltree(LinkCast(tree));
	if (NULL == ltree) return false;

	if (tree->getType() == QUOTE_LINK)
	{
		if (is_atom_in_tree(tree, atom)) return true;
		return false;
	}

	// Recurse downwards...
	for (const Handle& h: ltree->getOutgoingSet()) {
		if (is_quoted_in_tree(h, atom)) return true;
	}
	return false;
}

/**
 * Return true if the indicated atom occurs unquoted somewhere in the tree
 * (viz, the tree recursively spanned by the outgoing set of the handle)
 * but ONLY if it is not quoted!  This is meant to be be used to search
 * for variables, but only those variables that have not been quoted, as
 * the quoted variables are constants (literals).
 */
static inline bool is_unquoted_in_tree(const Handle& tree, const Handle& atom)
{
	if (tree == Handle::UNDEFINED) return false;
	if (tree == atom) return true;
	LinkPtr ltree(LinkCast(tree));
	if (NULL == ltree) return false;

	if (tree->getType() == QUOTE_LINK) return false;

	// Recurse downwards...
	for (const Handle& h : ltree->getOutgoingSet()) {
		if (is_unquoted_in_tree(h, atom)) return true;
	}
	return false;
}

/**
 * Return true if any of the indicated atoms occur somewhere in
 * the tree (that is, in the tree spanned by the outgoing set.)
 */
static inline bool any_atom_in_tree(const Handle& tree,
                                    const std::set<Handle>& atoms)
{
	for (const Handle& n: atoms)
	{
		if (is_atom_in_tree(tree, n)) return true;
	}
	return false;
}

/**
 * Return true if any of the indicated atoms occur somewhere in
 * the tree (that is, in the tree spanned by the outgoing set.)
 * But ONLY if they are not quoted!  This is intended to be used to
 * search for variables; but when a variable is quoted, it is no
 * longer a variable.
 */
static inline bool any_unquoted_in_tree(const Handle& tree,
                                        const std::set<Handle>& atoms)
{
	for (const Handle& n: atoms)
	{
		if (is_unquoted_in_tree(tree, n)) return true;
	}
	return false;
}

/**
 * Return true if the indicated atom occurs somewhere in any of the trees.
 */
static inline bool is_atom_in_any_tree(const std::vector<Handle>& trees,
                                       const Handle& atom)
{
	for (const Handle& tree: trees)
	{
		if (is_atom_in_tree(tree, atom)) return true;
	}
	return false;
}

/**
 * Return true if the indicated atom occurs somewhere in any of the trees,
 * but only if it is not quoted.  This is intended to be used to search
 * for variables, which cease to be variable when they are quoted.
 */
static inline bool is_unquoted_in_any_tree(const std::vector<Handle>& trees,
                                           const Handle& atom)
{
	for (const Handle& tree: trees)
	{
		if (is_unquoted_in_tree(tree, atom)) return true;
	}
	return false;
}

/**
 * Returns true if the clause contains an atom of type atom_type.
 * ... but only if it is not quoted.  Quoted terms are constants (literals).
 */
static inline bool contains_atomtype(const Handle& clause, Type atom_type)
{
	Type clause_type = clause->getType();
	if (QUOTE_LINK == clause_type) return false;
	if (classserver().isA(clause_type, atom_type)) return true;

	LinkPtr lc(LinkCast(clause));
	if (not lc) return false;

	for (const Handle& subclause: lc->getOutgoingSet())
	{
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
