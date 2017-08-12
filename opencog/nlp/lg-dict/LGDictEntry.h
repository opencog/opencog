/*
 * LGDictEntry.h
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

#ifndef _OPENCOG_LG_DICT_ENTRY_H
#define _OPENCOG_LG_DICT_ENTRY_H

#include <link-grammar/link-includes.h>

#include <opencog/atoms/core/FunctionLink.h>
#include <opencog/nlp/types/atom_types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// Link Grammar dictionary lookup.
///
/// An atomspace wrapper to the LG Dictionary.
/// The LGDictEntry places a dictionary entry into the atomspace,.
///
/// Usage:  Just execute the following link:
///
///     LgDictEntry
///          WordNode "foobar"
///          LgDictNode "en"
///
/// Executing the above will look up the word in teh English
/// dictionary, and place the contents into the atomspace.

class LGDictEntry : public FunctionLink
{
protected:
	void init();

public:
	LGDictEntry(const HandleSeq&, Type=LG_DICT_ENTRY);
	LGDictEntry(const Link&);

	// Return a pointer to the atom being specified.
	virtual Handle execute(AtomSpace* = nullptr) const;

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LGDictEntry> LGDictEntryPtr;
static inline LGDictEntryPtr LGDictEntryCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<LGDictEntry>(a); }
static inline LGDictEntryPtr LGDictEntryCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LGDictEntry>(a); }

// XXX temporary hack ...
#define createLGDictEntry std::make_shared<LGDictEntry>

/** @}*/
}
#endif // _OPENCOG_LG_DICT_ENTRY_H
