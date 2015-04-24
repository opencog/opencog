/*
 * opencog/atoms/reduct/FoldLink.h
 *
 * Copyright (C) 2015 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Fold Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Fold Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_FOLD_LINK_H
#define _OPENCOG_FOLD_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atoms/reduct/FreeLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * The FoldLink implements the arithmetic operations of plus and times.
 * (Its not currently a general fold; it only works with numbers.)
 */
class FoldLink : public FreeLink
{
protected:
	double knil;
	double (*kons) (double, double);
	void init(void);
	FoldLink(Type, const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	FoldLink(Type, const Handle& a, const Handle& b,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
public:
	FoldLink(const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
	FoldLink(Link& l);

   virtual Handle reduce(void);
};

typedef std::shared_ptr<FoldLink> FoldLinkPtr;
static inline FoldLinkPtr FoldLinkCast(const Handle& h)
   { AtomPtr a(h); return std::dynamic_pointer_cast<FoldLink>(a); }
static inline FoldLinkPtr FoldLinkCast(AtomPtr a)
   { return std::dynamic_pointer_cast<FoldLink>(a); }

// XXX temporary hack ...
#define createFoldLink std::make_shared<FoldLink>

/** @}*/
}

#endif // _OPENCOG_FOLD_LINK_H
