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

#include <atomic>
#include <uuid/uuid.h>
#include <link-grammar/link-includes.h>

#include <opencog/atoms/base/Node.h>
#include <opencog/atomspace/AtomSpace.h>
#include "LGDictNode.h"
#include "LGDictEntry.h"
#include "LGDictReader.h"

using namespace opencog;
using namespace opencog::nlp;
void error_handler(lg_errinfo *ei, void *data);

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

	Type pht = oset[0]->getType();
	if (WORD_NODE != pht and VARIABLE_NODE != pht and GLOB_NODE != pht)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry: Expecting WordNode, got %s",
			oset[0]->toString().c_str());

	Type dit = oset[1]->getType();
	if (LG_DICT_NODE != dit and VARIABLE_NODE != dit and GLOB_NODE != dit)
		throw InvalidParamException(TRACE_INFO,
			"LGDictEntry: Expecting LgDictNode, got %s",
			oset[1]->toString().c_str());
}

LGDictEntry::LGDictEntry(const HandleSeq& oset, Type t)
	: FunctionLink(oset, t)
{
	// Type must be as expected
	if (not classserver().isA(t, LG_DICT_ENTRY))
	{
		const std::string& tname = classserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgDictEntry, got %s", tname.c_str());
	}
	init();
}

LGDictEntry::LGDictEntry(const Link& l)
	: FunctionLink(l)
{
	// Type must be as expected
	Type tparse = l.getType();
	if (not classserver().isA(tparse, LG_DICT_ENTRY))
	{
		const std::string& tname = classserver().getTypeName(tparse);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgDictEntry, got %s", tname.c_str());
	}
}

// =================================================================

Handle LGDictEntry::execute(AtomSpace* as) const
{
	if (WORD_NODE != _outgoing[0]->getType()) return Handle();
	if (LG_DICT_NODE != _outgoing[1]->getType()) return Handle();

	if (nullptr == as) as = getAtomSpace();
	if (nullptr == as)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry requires an atomspace to work");

	// Link grammar, for some reason, has a different error handler
	// per thread. Don't know why. So we have to set it every time,
	// because we don't know what thread we are in.
	lg_error_set_handler(error_handler, nullptr);

	// Get the dictionary
	LgDictNodePtr ldn(LgDictNodeCast(_outgoing[1]));
	Dictionary dict = ldn->get_dictionary();
	if (nullptr == dict)
		throw InvalidParamException(TRACE_INFO,
			"LgDictEntry requires valid dictionary! %s was given.",
			ldn->getName().c_str());

	// A convenient name.
	const Handle& h = _outgoing[0];

	// Check if the dictionary entry is already in the atomspace.
	// XXX Is this really correct?  No, not really, but its what the
	// old code did, so we do it too.
	HandleSeq djset;
	h->getIncomingSetByType(std::back_inserter(djset), LG_DISJUNCT);

	// Avoid building the disjuncts if entries exist.
	if (not djset.empty()) return h;

	LGDictReader reader(dict, as);

	reader.getDictEntry(h->getName());

	return h;
}

DEFINE_LINK_FACTORY(LGDictEntry, LG_DICT_ENTRY)

/* ===================== END OF FILE ===================== */
