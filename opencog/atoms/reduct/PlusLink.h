/*
 * opencog/atoms/reduct/PlusLink.h
 *
 * Copyright (C) 2015 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Plus Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Plus Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_PLUS_LINK_H
#define _OPENCOG_PLUS_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/reduct/FoldLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * The PlusLink implements the mathematical operation of "plus"
 */
class PlusLink : public FoldLink
{
protected:
	void init(void);
	PlusLink(Type, const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	PlusLink(Type, const Handle& a, const Handle& b,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
public:
	PlusLink(const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
	PlusLink(Link& l);
};

typedef std::shared_ptr<PlusLink> PlusLinkPtr;
static inline PlusLinkPtr PlusLinkCast(const Handle& h)
   { AtomPtr a(h); return std::dynamic_pointer_cast<PlusLink>(a); }
static inline PlusLinkPtr PlusLinkCast(AtomPtr a)
   { return std::dynamic_pointer_cast<PlusLink>(a); }

// XXX temporary hack ...
#define createPlusLink std::make_shared<PlusLink>

/** @}*/
}

#endif // _OPENCOG_PLUS_LINK_H
