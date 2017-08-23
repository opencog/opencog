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

#include <link-grammar/link-includes.h>

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
/// The LGParseLink places a full parse into the atomspace, including
/// the disjuncts and the link-instances.
///
/// The LGParseMinimal ony places the words, word-sequence and links
/// into the atomspace.

class LGParseLink : public FunctionLink
{
protected:
	void init();
	Handle cvt_linkage(Linkage, int, const char*, const char*,
	                   bool, AtomSpace*) const;

public:
	LGParseLink(const HandleSeq&, Type=LG_PARSE_LINK);
	LGParseLink(const Link&);

	// Return a pointer to the atom being specified.
	virtual Handle execute(AtomSpace* = nullptr) const;

	static Handle factory(const Handle&);
};

class LGParseMinimal : public LGParseLink
{
public:
	LGParseMinimal(const HandleSeq&, Type=LG_PARSE_MINIMAL);
	LGParseMinimal(const Link&);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LGParseLink> LGParseLinkPtr;
static inline LGParseLinkPtr LGParseLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<LGParseLink>(a); }
static inline LGParseLinkPtr LGParseLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LGParseLink>(a); }

// XXX temporary hack ...
#define createLGParseLink std::make_shared<LGParseLink>

typedef std::shared_ptr<LGParseMinimal> LGParseMinimalPtr;
static inline LGParseMinimalPtr LGParseMinimalCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<LGParseMinimal>(a); }
static inline LGParseMinimalPtr LGParseMinimalCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LGParseMinimal>(a); }

// XXX temporary hack ...
#define createLGParseMinimal std::make_shared<LGParseMinimal>

/** @}*/
}
#endif // _OPENCOG_LG_PARSE_H
