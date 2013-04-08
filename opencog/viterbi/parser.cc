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

// #include <ctype.h>

// #include <algorithm>
// #include <iostream>
#include <string>
// #include <sstream>
// #include <vector>

#include <link-grammar/read-dict.h>

#include <opencog/atomspace/Atom.h>

#if LATER
#include "compile.h"
#include "connect.h"
#include "connector-utils.h"
#include "disjoin.h"
#include "garbage.h"
#include "state.h"
#include "viterbi.h"
#endif

#include "parser.h"

using namespace std;

#define DBG(X) X;

namespace viterbi {

Parser::Parser(Dictionary dict)
	: _dict(dict), _scm_eval(SchemeEval::instance())
// , _alternatives(NULL)
{
	DBG(cout << "=============== Parser ctor ===============" << endl);
	// lg_init_gc();
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
Handle Parser::lg_exp_to_atom(Exp* exp)
{
	_scm_eval.eval_h("(Node \"asdf\")");
#ifdef LATER
	if (CONNECTOR_type == exp->type)
	{
		stringstream ss;
		if (exp->multi) ss << "@";
		ss << exp->u.string << exp->dir;
		return new Connector(ss.str());
	}

	// Whenever a null appears in an OR-list, it means the 
	// entire OR-list is optional.  A null can never appear
	// in an AND-list.
	E_list* el = exp->u.l;
	if (NULL == el)
		return new Connector(OPTIONAL_CLAUSE);

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
		return new And(alist);

	if (OR_type == exp->type)
		return new Or(alist);

	assert(0, "Not reached");
#endif 
	return Handle::UNDEFINED;
}

#ifdef LATER
// ===================================================================
/**
 * Return atomic formula connector expression for the given word.
 *
 * This looks up the word in the link-grammar dictionary, and converts
 * the resulting link-grammar connective expression into an formula
 * composed of atoms.
 */
Set * Parser::word_consets(const string& word)
{
	// See if we know about this word, or not.
	Dict_node* dn_head = dictionary_lookup_list(_dict, word.c_str());
	if (!dn_head) return NULL;

	OutList djset;
	for (Dict_node* dn = dn_head; dn; dn = dn->right)
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
	return new Set(djset);
}

#endif

// ===================================================================
/**
 * Set up initial viterbi state for the parser
 */
void Parser::initialize_state()
{
#if LATER
	const char * wall_word = "LEFT-WALL";

	Set *wall_disj = word_consets(wall_word);

	// We are expecting the initial wall to be unique.
	assert(wall_disj->get_arity() == 1, "Unexpected wall structure");
	OutList state_vec;
	Atom* wall_cset = wall_disj->get_outgoing_atom(0);

	// Initial state: no output, and the wall cset.
	_alternatives = new Set(
		new StatePair(
			new Seq(wall_cset),
			new Seq()
		)
	);
#endif
}

#if LATER
// ===================================================================
/**
 * Add a single word to the parse.
 */
void Parser::stream_word(const string& word)
{
	Set *djset = word_consets(word);
	if (!djset)
	{
		cout << "Unhandled error; word not in dict: " << word << endl;
		return;
	}

	// Try to add each dictionary entry to the parse state.
	Set* new_alts = new Set();
	for (int i = 0; i < djset->get_arity(); i++)
	{
		Atom* cset = djset->get_outgoing_atom(i);
		State stset(_alternatives);
		stset.stream_word_conset(dynamic_cast<WordCset*>(cset));
		new_alts = new_alts->add(stset.get_alternatives());
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
		size_t wend = text.find(' ', pos);
		if (wend != string::npos)
		{
			string word = text.substr(pos, wend-pos);
			stream_word(word);
			pos = wend+1; // skip over space
		}
		else
		{
			string word = text.substr(pos);
			stream_word(word);
			break;
		}
	}
}

void viterbi_parse(Dictionary dict, const char * sentence)
{
	Parser pars(dict);

	pars.streamin(sentence);
	Link* alts = pars.get_alternatives();

	/* Print some diagnostic outputs ... for now. Remove whhen finished. */
	size_t num_alts = alts->get_arity();
	printf("Found %lu alternatives\n", num_alts);
	for (size_t i=0; i<num_alts; i++)
	{
		Atom* a = alts->get_outgoing_atom(i);
		StatePair* sp = dynamic_cast<StatePair*>(a);
		Seq* state = sp->get_state();
		size_t state_sz = state->get_arity();
		if (0 == state_sz)
		{
			cout << "\nAlternative =============== " << i << endl;
			cout << sp->get_output() << endl;
		}
	}
}
#endif

} // namespace viterbi

