/*
 * opencog/atoms/LambdaLink.h
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

#ifndef _OPENCOG_LAMBDA_LINK_H
#define _OPENCOG_LAMBDA_LINK_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental LambdaLink class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */

typedef std::map<Handle, const std::set<Type> > VariableTypeMap;

class PatternMatch;

class LambdaLink : public Link
{
   friend class PatternMatch;
protected:
	/// Handle of the topmost variable declaration.
	Handle _vardecl;

	/// Unbundled variables and types for them.
	/// _typemap is the (possibly empty) list of restrictions on
	/// the variable types. Set by validate_vars()
	std::set<Handle> _varset;
	VariableTypeMap _typemap;

	/// Handle of the body of the expression.
	Handle _body;

	// See LambdaLink.cc for comments
	static int get_vartype(const Handle&,
	                       std::set<Handle>&,
	                       VariableTypeMap&);

	// Extract variable decls and the body.
	void unbundle_body(const Handle&);

	// Validate the variable decls
	void validate_vardecl(const Handle&);

public:
	LambdaLink(Type, const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	LambdaLink(Link &l);
};

typedef std::shared_ptr<LambdaLink> LambdaLinkPtr;
static inline LambdaLinkPtr LambdaLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<LambdaLink>(a); }
static inline LambdaLinkPtr LambdaLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LambdaLink>(a); }

// XXX temporary hack ...
#define createLambdaLink std::make_shared<LambdaLink>

/** @}*/
}

#endif // _OPENCOG_LAMBDA_LINK_H
