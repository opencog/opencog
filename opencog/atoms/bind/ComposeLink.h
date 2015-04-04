/*
 * opencog/atoms/ComposeLink.h
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

#ifndef _OPENCOG_COMPOSE_LINK_H
#define _OPENCOG_COMPOSE_LINK_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental ComposeLink class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */

/// The CompseLink is used to specify a set of values that are to be
/// attached to the variables of a function named by a a DefineLink.
/// Thus, a ComposeLink acts much like function composition: it
/// substitutes values for variables.  It does not actually perform any
/// calling, evaluation or grounding; it merely served to identify the
/// named function with which it is to be composed.
///
/// The intended use of this link type is to define recursive patterns
/// the pattern matcher to solve.
class ComposeLink : public Link
{
protected:
	void init(const HandleSeq&);
public:
	ComposeLink(const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ComposeLink(const Handle& varcdecls, const Handle& body,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ComposeLink(Link &l);

	Handle compose(void);
	void satisfy(PatternMatchCallback*);
};

typedef std::shared_ptr<ComposeLink> ComposeLinkPtr;
static inline ComposeLinkPtr ComposeLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<ComposeLink>(a); }
static inline ComposeLinkPtr ComposeLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<ComposeLink>(a); }

// XXX temporary hack ...
#define createComposeLink std::make_shared<ComposeLink>

/** @}*/
}

#endif // _OPENCOG_COMPOSE_LINK_H
