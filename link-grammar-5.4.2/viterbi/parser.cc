/*************************************************************************/
/* Copyright (c) 2012, 2013 Linas Vepstas <linasvepstas@gmail.com>       */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <ctype.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <link-grammar/link-includes.h>
#include "api-types.h"
#include "read-dict.h"
#include "structures.h"

#include "atom.h"
#include "compile.h"
#include "disjoin.h"
#include "garbage.h"
#include "parser.h"
#include "viterbi.h"
#include "word-monad.h"

using namespace std;

#define DBG(X) X;

namespace link_grammar {
namespace viterbi {

Parser::Parser(Dictionary dict)
	: _dict(dict), _alternatives(NULL)
{
	DBG(cout << "=============== Parser ctor ===============" << endl);
	do_init_gc();
	initialize_state();
}

// ===================================================================
/**
 * Convert LG dictionary expression to atomic formula.
 *
 * The returned expression is in the form of an opencog-style
 * prefix-notation boolean expression.  Note that it is not in any
 * particular kind of normal form.  In particular, some AND nodes
 * may have only one child: these can be removed.
 *
 * Note that the order of the connectors is important: while linking,
 * these must be satisfied in left-to-right (nested!?) order.
 *
 * Optional clauses are indicated by OR-ing with null, where "null"
 * is a CONNECTOR Node with string-value "0".  Optional clauses are
 * not necessarily in any sort of normal form; the null connector can
 * appear anywhere.
 */
Atom * Parser::lg_exp_to_atom(Exp* exp)
{
	// log-likelihood is identical to the link-grammar cost.
	float likli = exp->cost;

	if (CONNECTOR_type == exp->type)
	{
		stringstream ss;
		if (exp->multi) ss << "@";
		ss << exp->u.string << exp->dir;

		return new Connector(ss.str().c_str(), likli);
	}

	// Whenever a null appears in an OR-list, it means the
	// entire OR-list is optional.  A null can never appear
	// in an AND-list.
	E_list* el = exp->u.l;
	if (NULL == el)
		return new Connector(OPTIONAL_CLAUSE, likli);

	// The C data structure that link-grammar uses for connector
	// expressions is totally insane, as witnessed by the loop below.
	// Anyway: operators are infixed, i.e. are always binary,
	// with exp->u.l->e being the left hand side, and
	//      exp->u.l->next->e being the right hand side.
	// This means that exp->u.l->next->next is always null.
	OutList alist;
	alist.push_back(lg_exp_to_atom(el->e));
	el = el->next;

	while (el && exp->type == el->e->type)
	{
		el = el->e->u.l;
		alist.push_back(lg_exp_to_atom(el->e));
		el = el->next;
	}

	if (el)
		alist.push_back(lg_exp_to_atom(el->e));

	if (AND_type == exp->type)
		return new And(alist, likli);

	if (OR_type == exp->type)
		return new Or(alist, likli);

	assert(0, "Not reached");
}

// ===================================================================
/**
 * Iterate over a set of connector-sets. If there is a mixture of
 * different costs found in a connector-set, then split it up into
 * several different ones, each with the appropriate cost.
 *
 * In principle, we could split up everything.  Right now, we don't
 * because:
 * 1) the current unit tests would be surprised by this, and some
 *    would fail.
 * 2) the resulting graph would be larger, more verbose.
 * On the other hand, if we did split up everything here, then the
 * parsing algo could become simpler/smaller. Hmmm... what to do ...
 */
static Set* cost_split(Set* raw_csets)
{
	OutList cooked;
	foreach_outgoing(WordCset*, wcs, raw_csets)
	{
		Atom* c = wcs->get_cset();
		Or* orc = dynamic_cast<Or*>(c);
		if (!orc)
		{
			// Promote costs, if any, from the disjunct to the connector set.
			wcs->_tv = c->_tv;
			c->_tv = 0.0f;
			cooked.push_back(wcs);
			continue;
		}

		// If we are here, then we have a set of disjuncts.
		// Split out any costly disjuncts into their own.
		OutList trim;
		foreach_outgoing(Atom*, dj, orc)
		{
			if (dj->_tv == 0.0f)
			{
				trim.push_back(dj);
				continue;
			}
			WordCset* cwcs = new WordCset(wcs->get_word(), dj);
			cwcs->_tv = dj->_tv;
			dj->_tv = 0.0f;
			cooked.push_back(cwcs);
		}
		if (1 == trim.size())
		{
			WordCset* dj = new WordCset(wcs->get_word(), trim[0]);
			cooked.push_back(dj);
		}
		else if (1 < trim.size())
		{
			WordCset* dj = new WordCset(wcs->get_word(), new Or(trim));
			cooked.push_back(dj);
		}
	}
	return new Set(cooked);
}

// ===================================================================

/// Given a disjunct of connectors, propagate a cost
/// on any one of them up to the disjunct as a whole.
static void promote_cost(And* disjunct)
{
	// Promote costs, if any, from each connector, to the disjunct
	foreach_outgoing(Atom*, a, disjunct)
	{
		disjunct->_tv += a->_tv;
		a->_tv = 0.0f;
	}
}

/// Given a list of connector sets, search for any costs pasted onto
/// some individual connector, and push it up onto the disjunct that
/// contains that connector.
static Set* cost_up(Set* raw_csets)
{
	foreach_outgoing(WordCset*, wcs, raw_csets)
	{
		Atom* c = wcs->get_cset();
		And* dj = dynamic_cast<And*>(c);
		if (dj)
		{
			promote_cost(dj);
			continue;
		}

		Or* orc = dynamic_cast<Or*>(c);
		if (orc)
		{
			foreach_outgoing(And*, dj, orc)
			{
				if (dj)
					promote_cost(dj);
			}
		}
	}
	return raw_csets;
}

// ===================================================================
/**
 * Return atomic formula connector expression for the given word.
 *
 * This looks up the word in the link-grammar dictionary, and converts
 * the resulting link-grammar connective expression into an formula
 * composed of atoms.
 */
Set* Parser::word_consets(const string& word)
{
	Set* raw_csets = raw_word_consets(word);
	return cost_split(cost_up(raw_csets));
}

// ===================================================================
/**
 * Return atomic formula connector expression for the given word.
 *
 * This looks up the word in the link-grammar dictionary, and converts
 * the resulting link-grammar connective expression into a formula
 * composed of atoms.
 *
 * The return form is 'raw' in that the costs have not yet been correctly
 * distributed over the words: i.e. this might return a connector set that
 * might be a disjunction over different costs.  For example:
 *
 * SET :
 *   WORD_CSET :
 *     WORD : is.v
 *     OR:
 *       AND :
 *         CONNECTOR : Ss-
 *         CONNECTOR : Wi-
 *       AND :     (2)
 *         CONNECTOR : Ss-
 *         CONNECTOR : Wd-
 *
 * Notice the cost on the second disjunct: this would completely mess
 * things up if it were placed into the atomspace, since that cost would
 * screw things up for any other expressions having this sub-expression.
 */
Set * Parser::raw_word_consets(const string& word)
{
	// See if we know about this word, or not.
	Dict_node* dn_head = dictionary_lookup_list(_dict, word.c_str());
	if (!dn_head) return new Set();

	OutList djset;
	for (Dict_node*dn = dn_head; dn; dn= dn->right)
	{
		Exp* exp = dn->exp;
		DBG({cout << "=============== Parser word: " << dn->string << ": ";
			print_expression(exp); });

		Atom *dj = lg_exp_to_atom(exp);
		dj = disjoin(dj);

		// First atom at the front of the outgoing set is the word itself.
		// Second atom is the first disjuct that must be fulfilled.
		Word* nword = new Word(dn->string);
		djset.push_back(new WordCset(nword, dj));
	}
	free_lookup_list(dn_head);
	return new Set(djset);
}

// ===================================================================
/**
 * Set up initial viterbi state for the parser
 */
void Parser::initialize_state()
{
	_alternatives = new Set(
		new StateTriple(
			new Seq(),
			new Seq(),
			new Set()
		)
	);

	const char * wall_word = "LEFT-WALL";
	stream_word(wall_word);
}

// ===================================================================
/**
 * Add a single word to the parse.
 */
void Parser::stream_word(const string& word)
{
	// Look up the dictionary entries for this word.
	Set *djset = word_consets(word);
	if (!djset)
	{
		cout << "Unhandled error; word not in dict: >>" << word << "<<" << endl;
		assert (0, "word not in dict");
		return;
	}

	// Try to add each dictionary entry to the parse state.
	Set* new_alts = new Set();
	foreach_outgoing(WordCset*, wrd_cset, djset)
	{
		WordMonad cnct(wrd_cset);
		Set* alts = cnct(_alternatives);
		new_alts = new_alts->sum(alts);
	}
	_alternatives = new_alts;
}

// ===================================================================
/** convenience wrapper */
Set* Parser::get_alternatives()
{
	return _alternatives;
}

// ===================================================================
/**
 * Add a stream of text to the input.
 *
 * No particular assumptiions are made about the input, other than
 * that its space-separated words (i.e. no HTML markup or other junk)
 */
void Parser::streamin(const string& text)
{
	// A trivial tokenizer
	size_t pos = 0;
	while(true)
	{
		size_t wend = text.find(' ', pos); // wend == word end
		if (wend != string::npos)
		{
			const string word = text.substr(pos, wend-pos);
			stream_word(word);
			pos = wend + 1; // skip over space
			while (' ' == text[pos])
				pos++;
		}
		else
		{
			const string word = text.substr(pos);
			if (0 < word.size())
				stream_word(word);
			break;
		}
	}
}

// Send in the right wall -- the traditional link-grammar
// design wants this to terminate sentences.
void Parser::stream_end()
{
	const char * right_wall_word = "RIGHT-WALL";
	Set *wall_disj = word_consets(right_wall_word);

	// We are expecting the initial wall to be unique.
	assert(wall_disj->get_arity() == 1, "Unexpected wall structure");
	Atom* wall_cset = wall_disj->get_outgoing_atom(0);
	WordCset* rwcs = dynamic_cast<WordCset*>(wall_cset);

	WordMonad cnct(rwcs);
	_alternatives = cnct(_alternatives);
}

void viterbi_parse(Dictionary dict, const char * sentence)
{
	Parser pars(dict);

	pars.streamin(sentence);

	// The old link-grammar design insists on  having a RIGHT-WALL,
	// so provide one.
	pars.stream_end();

	atombase::Link* alts = pars.get_alternatives();

	/* Print some diagnostic outputs ... for now. Remove when finished. */
	size_t num_alts = alts->get_arity();
	printf("Found %lu alternatives\n", num_alts);
	for (size_t i=0; i<num_alts; i++)
	{
		Atom* a = alts->get_outgoing_atom(i);
		StateTriple* sp = dynamic_cast<StateTriple*>(a);
		Seq* state = sp->get_state();
		size_t state_sz = state->get_arity();
		if (0 == state_sz)
		{
			cout << "\nAlternative =============== " << i << endl;
			cout << sp->get_output() << endl;
		}
		else
		{
			cout << "\nIncomplete parse =============== " << i << endl;
			cout << sp->get_output() << endl;
			cout << "\n---- state for ----" << i << endl;
			cout << sp->get_state() << endl;
		}
	}
}

} // namespace viterbi
} // namespace link-grammar

// ===================================================================

// Wrapper to escape out from C++
void viterbi_parse(const char * sentence, Dictionary dict)
{
	link_grammar::viterbi::viterbi_parse(dict, sentence);
}

