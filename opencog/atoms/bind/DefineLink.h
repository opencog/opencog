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

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental DefineLink class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */

typedef std::map<Handle, const std::set<Type> > VariableTypeMap;

class PatternMatch;

class DefineLink : public Link
{
   friend class PatternMatch;
protected:
	void init(const HandleSeq&);
public:
	DefineLink(const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	DefineLink(const Handle& varcdecls, const Handle& body,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	DefineLink(Link &l);
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
