/*
 * opencog/atoms/reduct/TimesLink.h
 *
 * Copyright (C) 2015 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Times Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Times Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_TIMES_LINK_H
#define _OPENCOG_TIMES_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/reduct/FoldLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * The TimesLink implements the mathematical operation of "plus"
 */
class TimesLink : public FoldLink
{
protected:
	void init(void);
	TimesLink(Type, const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	TimesLink(Type, const Handle& a, const Handle& b,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
public:
	TimesLink(const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
	TimesLink(Link& l);
};

typedef std::shared_ptr<TimesLink> TimesLinkPtr;
static inline TimesLinkPtr TimesLinkCast(const Handle& h)
   { AtomPtr a(h); return std::dynamic_pointer_cast<TimesLink>(a); }
static inline TimesLinkPtr TimesLinkCast(AtomPtr a)
   { return std::dynamic_pointer_cast<TimesLink>(a); }

// XXX temporary hack ...
#define createTimesLink std::make_shared<TimesLink>

/** @}*/
}

#endif // _OPENCOG_TIMES_LINK_H
