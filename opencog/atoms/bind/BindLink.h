/*
 * opencog/atoms/BindLink.h
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

#ifndef _OPENCOG_BIND_LINK_H
#define _OPENCOG_BIND_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atoms/bind/SatisfactionLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental BindLink class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */
class BindLink : public SatisfactionLink
{
protected:
	void init(void);

	/// The actual clauses. Set by validate_body()
	Handle _hclauses;
	Handle _implicand;

	// Validate the structure of the body
	void validate_body(const Handle&);

	BindLink(Type, const HandleSeq&,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

public:
	BindLink(const HandleSeq&,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	BindLink(Link &l);

	void imply(PatternMatchCallback*, bool check_connectivity=true);
	Handle& get_implicand(void) { return _implicand; }
};

typedef std::shared_ptr<BindLink> BindLinkPtr;
static inline BindLinkPtr BindLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<BindLink>(a); }
static inline BindLinkPtr BindLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<BindLink>(a); }

// XXX temporary hack ...
#define createBindLink std::make_shared<BindLink>

/** @}*/
}

#endif // _OPENCOG_BIND_LINK_H
