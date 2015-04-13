/*
 * opencog/atoms/ScopeLink.h
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

#ifndef _OPENCOG_SCOPE_LINK_H
#define _OPENCOG_SCOPE_LINK_H

#include <map>

#include <opencog/atoms/bind/VariableList.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// The ScopeLink consitsts of two parts: A variable declaration,
/// which must conform to current variable declaration standards: i.e.
/// it must be either a single VariableNode, a single TypedVariableLink,
/// or a VariableLink.  This is then followed by a body, of any
/// arbitrary form.  This class does little other than to check for
/// the above-described format; it will throw an error if an ill-formed
/// ScopeLink is inserted into the atomspace.  In addition to the
/// above, it also unpacks the variable declarations, using the
/// VariableList class as a helper class to do that unpacking.
/// As usual, the unpacked variables act as a memo or cache, speeding
/// up later calculations.
class ScopeLink : public VariableList
{
protected:
	/// Handle of the body of the expression.
	Handle _body;

	ScopeLink(Type, const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	void init(const HandleSeq&);
public:
	ScopeLink(const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ScopeLink(const Handle& varcdecls, const Handle& body,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ScopeLink(Link &l);

	// utility debug print
	void prt(const Handle& h) const
	{
		printf("%s\n", h->toShortString().c_str());
	}

	// Take the list of values `vals`, and substitute them in for the
	// variables in the body of this lambda. The values must satisfy all
	// type restrictions, else an exception will be thrown.
	Handle substitute (const HandleSeq& vals) const
	{
		return VariableList::substitute(_body, vals);
	}
};

typedef std::shared_ptr<ScopeLink> ScopeLinkPtr;
static inline ScopeLinkPtr ScopeLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<ScopeLink>(a); }
static inline ScopeLinkPtr ScopeLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<ScopeLink>(a); }

// XXX temporary hack ...
#define createScopeLink std::make_shared<ScopeLink>

/** @}*/
}

#endif // _OPENCOG_SCOPE_LINK_H
