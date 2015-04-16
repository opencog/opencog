/*
 * Pattern.h
 *
 * Author: Linas Vepstas April 2015
 *
 * Copyright (C) 2015 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_PATTERN_H
#define _OPENCOG_PATTERN_H

#include <map>
#include <set>
#include <stack>
#include <unordered_map>
#include <vector>

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>  // for typedef Type

namespace opencog {

/** \addtogroup grp_atomspace
 *  @{
 */

typedef std::map<Handle, const std::set<Type> > VariableTypeMap;

/// The Variables struct defines a list of variables in a way that
/// makes it easier and faster to work with in C++.  It implements 
/// the data that is shared between the VariableList link atom
/// and the pattern matcher.
///
struct Variables
{
	/// Unbundled variables and types for them.
	/// _typemap is the (possibly empty) list of restrictions on
	/// the variable types. The _varset contains exactly the same atoms
	/// as the _varseq; it is used for fast lookup; (i.e. is some
	/// some variable a part of this set?) whereas the _varseq list
	/// preserves the original order of the variables.  Yes, the fast
	/// lookup really is needed!  The _index is used to implement the
	/// variable substitution method.
	HandleSeq varseq;
	std::set<Handle> varset;
	VariableTypeMap typemap;
	std::map<Handle, unsigned int> index;
};

/// The Pattern struct defines a search pattern in a way that makes it
/// easier and faster to work with in C++.  It implements the data that
/// is shared between the various pattern-specification atoms and the
/// pattern matcher.
///
struct Pattern
{
	// Private, locally scoped typedefs, not used outside of this class.
	// XX TODO Replace by unordered multimap...
	typedef std::vector<Handle> RootList;
	typedef std::map<Handle, RootList> ConnectMap;
	typedef std::pair<Handle, RootList> ConnectPair;

	// -------------------------------------------
	// The current set of clauses (beta redex context) being grounded.
	std::string redex_name;  // for debugging only!

	/// The actual clauses. Set by validate_clauses()
	HandleSeq        clauses;

	// The cnf_clauses are the clauses, but with the AbsentLink, NotLink
	// removed. This simplifies graph discovery, so that when they are
	// found, they can be rejected (e.g. are not absent)
	HandleSeq        cnf_clauses;  // AbsentLink, NotLink removed!

	// The mandatory clauses must be grounded.
	HandleSeq        mandatory;

	// The optional clauses don't have to be. This is where the
	// negated/absent clauses are held, so e.g. if these get grounded,
	// they might be rejected.
	std::set<Handle> optionals;    // Optional clauses

	std::set<Handle> evaluatable_terms;   // smallest term that is evaluatable
	std::set<Handle> evaluatable_holders; // holds something evaluatable.
	std::set<Handle> executable_terms;    // smallest term that is executable
	std::set<Handle> executable_holders;  // holds something executable.

	// Maps; the value is the largest (evaluatable or executable)
	// term containing the variable. Its a multimap, because
	// a variable may appear in several different evaluatables.
	std::unordered_multimap<Handle,Handle> in_evaluatable;
	std::unordered_multimap<Handle,Handle> in_executable;

	// Any given atom may appear in one or more clauses. Given an atom,
	// the connectivy map tells you what clauses it appears in. It
	// captures how the clauses are connected to one-another, so that,
	// after one clause is solved, we know what parts of the unsolved
	// clauses already have a solution.
	ConnectMap       connectivity_map;     // setup by make_connectivity_map()
};

/** @}*/
} // namespace opencog

#endif // OPENCOG_PATTERN_H
