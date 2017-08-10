/*
 * LGParseLink.cc
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
#include <opencog/atoms/NumberNode.h>
#include <opencog/atomspace/AtomSpace.h>
#include "LGDictNode.h"
#include "LGParseLink.h"

using namespace opencog;

LGParseLink::LGParseLink(const HandleSeq& oset, Type t)
	: FunctionLink(oset, t)
{
	size_t osz = oset.size();
	if (2 != osz)
		throw InvalidParamException(TRACE_INFO,
			"LGParseLink: Expecting two arguments, got %lu", osz);

	Type pht = oset[0]->getType();
	if (PHRASE_NODE != pht and VARIABLE_NODE != pht and GLOB_NODE != pht)
		throw InvalidParamException(TRACE_INFO,
			"LGParseLink: Expecting PhraseNode, got %s",
			oset[0]->toString().c_str());

	Type dit = oset[1]->getType();
	if (LG_DICT_NODE != dit and VARIABLE_NODE != dit and GLOB_NODE != dit)
		throw InvalidParamException(TRACE_INFO,
			"LGParseLink: Expecting LgDictNode, got %s",
			oset[1]->toString().c_str());
}

LGParseLink::LGParseLink(const Link& l)
	: FunctionLink(l)
{
	// Type must be as expected
	Type tparse = l.getType();
	if (not classserver().isA(tparse, LG_PARSE_LINK))
	{
		const std::string& tname = classserver().getTypeName(tparse);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LgParseLink, got %s", tname.c_str());
	}
}

Handle LGParseLink::execute(AtomSpace* as) const
{
	if (PHRASE_NODE != _outgoing[0]->getType()) return Handle();
	if (LG_DICT_NODE != _outgoing[1]->getType()) return Handle();

	if (nullptr == as) as = getAtomSpace();
	if (nullptr == as)
		throw InvalidParamException(TRACE_INFO,
			"LgParseLink requires an atomspace to parse");

	// Get the dictionary
	LgDictNodePtr ldn(LgDictNodeCast(_outgoing[1]));
	Dictionary dict = ldn->get_dictionary();

	// Set up the sentence
	Sentence sent = sentence_create(_outgoing[0]->getName().c_str(), dict);
	if (nullptr == sent) return Handle();

	// Work with the default parse options
	Parse_Options opts = parse_options_create();

	// Count the number of parses
	int num_linkages = sentence_parse(sent, opts);
	if (num_linkages < 0)
	{
		sentence_delete(sent);
		parse_options_delete(opts);
		return Handle();
	}

	// XXX TODO: if num_links is zero, we should try again with null
	// links.
	if (num_linkages == 0)
	{
		sentence_delete(sent);
		parse_options_delete(opts);
		return Handle();
	}

printf("duude hurrah! nlink=%d\n", num_linkages);
	int max_linkages = 4;
	if (max_linkages < num_linkages) num_linkages = max_linkages;

	// Hmm. I hope that uuid_generate() won't block if there is not
	// enough entropy in the entropy pool....
	uuid_t uu;
	uuid_generate(uu);
	char idstr[37];
	uuid_unparse(uu, idstr);

	char sentstr[48] = "sentence@";
	strncat(sentstr, idstr, 48);

	Handle snode(as->add_node(SENTENCE_NODE, sentstr));

	for (int i=0; i<num_linkages; i++)
	{
		Linkage lkg = linkage_create(i, sent, opts);
		Handle pnode = cvt_linkage(lkg, i, sentstr, as);
		as->add_link(PARSE_LINK, pnode, snode);
		linkage_delete(lkg);
	}

	sentence_delete(sent);
	parse_options_delete(opts);
	return snode;
}

static std::atomic<unsigned long> wcnt;

Handle LGParseLink::cvt_linkage(Linkage lkg, int i, const char* idstr,
                              AtomSpace* as) const
{
	char parseid[80];
	snprintf(parseid, 80, "%s_parse_%d", idstr, i);
	Handle pnode(as->add_node(PARSE_NODE, parseid));

	int nwords = linkage_get_num_words(lkg);
	for (int w=0; w<nwords; w++)
	{
		const char* wrd = linkage_get_word(lkg, w);
printf("dduuude uuisd=%d %s %s\n", i, wrd, parseid);
		char buff[800] = "";
		strncat(buff, wrd, 800);
		strncat(buff, "@", 800);
		strncat(buff, parseid, 800);
		Handle winst(as->add_node(WORD_INSTANCE_NODE, buff));
		as->add_link(WORD_INSTANCE_LINK, winst, pnode);
		as->add_link(REFERENCE_LINK, winst,
			as->add_node(WORD_NODE, wrd));
		as->add_link(WORD_SEQUENCE_LINK, winst,
			Handle(createNumberNode(++wcnt)));
	}

	return pnode;
}

DEFINE_LINK_FACTORY(LGParseLink, LG_PARSE_LINK)

/* ===================== END OF FILE ===================== */
