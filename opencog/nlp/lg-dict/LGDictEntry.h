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
	LGDictEntry(const HandleSeq&&, Type=LG_DICT_ENTRY);
	LGDictEntry(const LGDictEntry&) = delete;
	LGDictEntry& operator=(const LGDictEntry&) = delete;

	// Return a pointer to the atom being specified.
	virtual ValuePtr execute(AtomSpace*, bool);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LGDictEntry> LGDictEntryPtr;
static inline LGDictEntryPtr LGDictEntryCast(const Handle& h)
	{ return std::dynamic_pointer_cast<LGDictEntry>(h); }
static inline LGDictEntryPtr LGDictEntryCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LGDictEntry>(a); }

#define createLGDictEntry std::make_shared<LGDictEntry>

class LGHaveDictEntry : public Link
{
protected:
	void init();

public:
	LGHaveDictEntry(const HandleSeq&&, Type=LG_HAVE_DICT_ENTRY);
	LGHaveDictEntry(const LGHaveDictEntry&) = delete;
	LGHaveDictEntry& operator=(const LGHaveDictEntry&) = delete;

	virtual bool is_evaluatable() const { return true; }
	virtual TruthValuePtr evaluate(AtomSpace*, bool);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LGHaveDictEntry> LGHaveDictEntryPtr;
static inline LGHaveDictEntryPtr LGHaveDictEntryCast(const Handle& h)
	{ return std::dynamic_pointer_cast<LGHaveDictEntry>(h); }
static inline LGHaveDictEntryPtr LGHaveDictEntryCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LGHaveDictEntry>(a); }

#define createLGHaveDictEntry std::make_shared<LGHaveDictEntry>

/** @}*/
}
#endif // _OPENCOG_LG_DICT_ENTRY_H
