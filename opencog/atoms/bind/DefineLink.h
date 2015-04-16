/*
 * opencog/atoms/DefineLink.h
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

#ifndef _OPENCOG_DEFINE_LINK_H
#define _OPENCOG_DEFINE_LINK_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atoms/bind/ScopeLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental DefineLink class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */

/// The DefineLink is used to give a name to a pattern, typically to
/// a ScopeLink, a SatisfactionLink or a BindLink.  The DefineLink is
/// unique, in that, if any other atoms exists with this same name, it
/// will throw an error!  Thus, only ONE DefineLink with a given name
/// can exist at a time.
///
/// This class is intended to be used for anything that needs to be
/// accessed by name: for, if there were two things with the same name,
/// it would be ambiguous as to which to access. (It would not make
/// sense to access both: would the result of access have 'and'
/// semantics? 'or' semantics ??)  Thus, this exists to define an atom
/// uniquely.
///
/// The only place where I know of, at the moment, for this beast, is
/// for the construction of recursive patterns, as that is the only
/// place where a simple cut-n-paste is insufficient to specify what
/// comes next.
///
/// It is intended that the DefineLink be used with the ComposeLink,
/// which provides the values for the variables bound by the DefineLink.
/// That is, the ComposeLink acts like function composition. It does not
/// actually call, invoke or ground the resulting composition.
///
/// Currently, the implementation of ComposeLink is half-finished and
/// mostly broken.
class DefineLink : public Link
{
protected:
	// The definition is the named object that this link is defining.
	ScopeLinkPtr _definition;
	void init(const HandleSeq&);
public:
	DefineLink(const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	DefineLink(const Handle& varcdecls, const Handle& body,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	DefineLink(Link &l);
	ScopeLinkPtr get_definition(void) { return _definition; }
};

typedef std::shared_ptr<DefineLink> DefineLinkPtr;
static inline DefineLinkPtr DefineLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<DefineLink>(a); }
static inline DefineLinkPtr DefineLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<DefineLink>(a); }

// XXX temporary hack ...
#define createDefineLink std::make_shared<DefineLink>

/** @}*/
}

#endif // _OPENCOG_DEFINE_LINK_H
