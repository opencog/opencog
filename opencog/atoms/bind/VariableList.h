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
 *
 * Experimental VariableList class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */

typedef std::map<Handle, const std::set<Type> > VariableTypeMap;

class PatternMatch;

class VariableList : public Link
{
   friend class PatternMatch;
protected:
	/// Unbundled variables and types for them.
	/// _typemap is the (possibly empty) list of restrictions on
	/// the variable types. The _varset contains exactly the same atoms
	/// as the outgoing set; it is used for fast lookup; (i.e. is some
	/// some variable a part of this set?) whereas the outgoing set
	/// preserves the original order of the variables.  Yes, teh fast
	/// lookup really is needed!
	HandleSeq _varseq;
	std::set<Handle> _varset;
	VariableTypeMap _typemap;

	// See VariableList.cc for comments
	static int get_vartype(const Handle&,
	                       std::set<Handle>&,
	                       VariableTypeMap&);

	// Validate the variable decls
	void validate_vardecl(const Handle&);
	void validate_vardecl(const HandleSeq&);

	VariableList(Type, const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

public:
	VariableList(const HandleSeq& vardecls,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	VariableList(const Handle& varcdecls,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	VariableList(Link &l);

	const VariableTypeMap& get_typemap(void) { return _typemap; }
	bool is_type(const Handle&);
	bool is_type(const HandleSeq&);
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
