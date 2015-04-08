/*
 * opencog/atoms/VariableList.h
 *
 * Copyright (C) 2015 Linas Vepstas
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
 */

#ifndef _OPENCOG_VARIABLE_LIST_H
#define _OPENCOG_VARIABLE_LIST_H

#include <map>

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

typedef std::map<Handle, const std::set<Type> > VariableTypeMap;

/// The VariableList class records it's outgoing set in various ways
/// that make it easier and faster to work with.  It implements a
/// substitute method that will replace all variables in a tree by
/// the corresponding atoms that it is given. See the .cc file for
/// more info.
///
/// The constructors make sure that the contents of the variable list
/// are syntactically correct; i.e that it actually contains variables.
/// Otherwise, it throws an error on bad syntax.  Thus, bad
/// VariableLists cannot be inserted into the atomspace.
class VariableList : public Link
{
protected:
	/// Unbundled variables and types for them.
	/// _typemap is the (possibly empty) list of restrictions on
	/// the variable types. The _varset contains exactly the same atoms
	/// as the _varseq; it is used for fast lookup; (i.e. is some
	/// some variable a part of this set?) whereas the _varseq list
	/// preserves the original order of the variables.  Yes, the fast
	/// lookup really is needed!  The _index is used to implement the
	/// variable substitution method.
	HandleSeq _varseq;
	std::set<Handle> _varset;
	VariableTypeMap _typemap;
	std::map<Handle, Arity> _index;

	// See VariableList.cc for comments
	void get_vartype(const Handle&);

	// Validate the variable decls
	void validate_vardecl(const Handle&);
	void validate_vardecl(const HandleSeq&);

	VariableList(Type, const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	void build_index(void);
	Handle substitute_nocheck(const Handle&, const HandleSeq&) const;
public:
	VariableList(const HandleSeq& vardecls,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	VariableList(Link&);

	// Return he list of variables we are holding.
	const HandleSeq& get_variables(void) const { return _varseq; }
	const std::set<Handle>& get_varset(void) const { return _varset; }

	// Return the type restrivtions ffor the variables.
	const VariableTypeMap& get_typemap(void) const { return _typemap; }

	// Return true if we are holding a single variable, and the handle is
	// satisfies any type restrictions. Else return false.
	bool is_type(const Handle&) const;

	// Return true if the sequence is of the same length as the variable
	// declarations we are holding, and if they satisfy all of the type
	// restrictions.
	bool is_type(const HandleSeq&) const;

	// Given the tree `tree` containing variables in it, create and
	// return a new tree with the indicated values `vals` substituted
	// for the variables. The vals must pass the typecheck, else an
	// exception is thrown.
	Handle substitute(const Handle& tree, const HandleSeq& vals) const;
};

typedef std::shared_ptr<VariableList> VariableListPtr;
static inline VariableListPtr VariableListCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<VariableList>(a); }
static inline VariableListPtr VariableListCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<VariableList>(a); }

// XXX temporary hack ...
#define createVariableList std::make_shared<VariableList>

/** @}*/
}

#endif // _OPENCOG_VARIABLE_LIST_H
