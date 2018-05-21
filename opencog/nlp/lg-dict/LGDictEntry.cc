/*
 * LGDictEntry.cc
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

#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include "LGDictNode.h"
#include "LGDictEntry.h"
#include "LGDictReader.h"

using namespace opencog;

/// The expected format of an LgDictEntry is:
///
///     LgDictEntry
///         WordNode "antidisestablishmentarianism"
///         LgDictNode "en"
///
/// When executed, the word will be looked up in the indicated
/// dictionary, and the dictionary entry will be placed into the
/// atomspace.
///
/// The LgDictEntry is a kind of FunctionLink, and can thus be used in
/// any expression that FunctionLinks can be used with.
///
void LGDictEntry::init()
{
	const HandleSeq& oset = _outgoing;

	size_t osz = oset.size();
	if (2 != osz)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry: Expecting two arguments, got %lu", osz);

	Type pht = oset[0]->get_type();
	if (WORD_NODE != pht and VARIABLE_NODE != pht and GLOB_NODE != pht)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry: Expecting WordNode, got %s",
			oset[0]->to_string().c_str());

	Type dit = oset[1]->get_type();
	if (LG_DICT_NODE != dit and VARIABLE_NODE != dit and GLOB_NODE != dit)
		throw InvalidParamException(TRACE_INFO,
			"LGDictEntry: Expecting LgDictNode, got %s",
			oset[1]->to_string().c_str());
}

LGDictEntry::LGDictEntry(const HandleSeq& oset, Type t)
	: FunctionLink(oset, t)
{
	// Type must be as expected
	if (not nameserver().isA(t, LG_DICT_ENTRY))
	{
		const std::string& tname = nameserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgDictEntry, got %s", tname.c_str());
	}
	init();
}

LGDictEntry::LGDictEntry(const Link& l)
	: FunctionLink(l)
{
	// Type must be as expected
	Type tparse = l.get_type();
	if (not nameserver().isA(tparse, LG_DICT_ENTRY))
	{
		const std::string& tname = nameserver().getTypeName(tparse);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgDictEntry, got %s", tname.c_str());
	}
}

// =================================================================

Handle LGDictEntry::execute() const
{
	if (WORD_NODE != _outgoing[0]->get_type()) return Handle();
	if (LG_DICT_NODE != _outgoing[1]->get_type()) return Handle();

	AtomSpace* as = this->getAtomSpace();
	if (nullptr == as)
		as = _outgoing[0]->getAtomSpace();
	if (nullptr == as)
		as = _outgoing[1]->getAtomSpace();
	if (nullptr == as)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry requires an atomspace to work");

	// Get the dictionary
	LgDictNodePtr ldn(LgDictNodeCast(_outgoing[1]));
	Dictionary dict = ldn->get_dictionary();
	if (nullptr == dict)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry requires valid dictionary! %s was given.",
			ldn->get_name().c_str());

	HandleSeq djs = getDictEntry(dict, _outgoing[0]->get_name());
	for (const Handle& dj: djs) as->add_atom(dj);

	return _outgoing[0];
}

DEFINE_LINK_FACTORY(LGDictEntry, LG_DICT_ENTRY)

/* ===================== END OF FILE ===================== */
