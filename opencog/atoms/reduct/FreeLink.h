/*
 * opencog/atoms/reduct/FreeLink.h
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

#ifndef _OPENCOG_FREE_LINK_H
#define _OPENCOG_FREE_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * The FreeLink records all of the free variables that occur within
 * (underneath) it, in traversal order. The names of those variables
 * are placed in _free_vars.
 */
class FreeLink : public Link
{
private:
	void find_vars(std::set<Handle>&, const HandleSeq&);
protected:
	HandleSeq _free_vars;
	void init(void);
	FreeLink(Type, const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	FreeLink(Type, const Handle& a, const Handle& b,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
public:
	FreeLink(const HandleSeq& oset,
	         TruthValuePtr tv = TruthValue::NULL_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());
	FreeLink(Link& l);
	virtual ~FreeLink() {}

	virtual Handle reduce(void);
};

typedef std::shared_ptr<FreeLink> FreeLinkPtr;
static inline FreeLinkPtr FreeLinkCast(const Handle& h)
   { AtomPtr a(h); return std::dynamic_pointer_cast<FreeLink>(a); }
static inline FreeLinkPtr FreeLinkCast(AtomPtr a)
   { return std::dynamic_pointer_cast<FreeLink>(a); }

// XXX temporary hack ...
#define createFreeLink std::make_shared<FreeLink>

/** @}*/
}

#endif // _OPENCOG_FREE_LINK_H
