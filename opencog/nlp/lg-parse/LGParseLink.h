/*
 * LGParseLink.h
 *
 * Copyright (C) 2017 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_LG_PARSE_H
#define _OPENCOG_LG_PARSE_H

#include <opencog/atoms/core/FunctionLink.h>
#include <opencog/nlp/types/atom_types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// Link Grammar parser.
///
/// An atomspace wrapper to the LG parser.

class LGParseLink : public FunctionLink
{
protected:
public:
	LGParseLink(const HandleSeq&, Type=LG_PARSE_LINK);
	LGParseLink(const Link&);

	// Return a pointer to the atom being specified.
	virtual Handle execute(AtomSpace* = NULL) const;

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LGParseLink> LGParseLinkPtr;
static inline LGParseLinkPtr LGParseLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<LGParseLink>(a); }
static inline LGParseLinkPtr LGParseLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LGParseLink>(a); }

// XXX temporary hack ...
#define createLGParseLink std::make_shared<LGParseLink>

/** @}*/
}
#endif // _OPENCOG_LG_PARSE_H
