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
#include <sstream>
// #include <vector>

#include <link-grammar/read-dict.h>

#include <opencog/util/oc_assert.h>
#include <opencog/atomspace/Atom.h>

#if LATER
#include "compile.h"
#include "connect.h"
#include "connector-utils.h"
#include "disjoin.h"
#include "state.h"
#include "viterbi.h"
#endif

#include "parser.h"

using namespace std;

#define DBG(X) X;

namespace viterbi {

// Print the atomspace contents.
const char* dbg = 
	"(define (prt-atomspace)\n"
	"  (define (prt-atom h)\n"
	"    ; print only the top-level atoms.\n"
	"    (if (null? (cog-incoming-set h))\n"
	"        (display h))\n"
	"  #f)\n"
	"  (define (prt-type type)\n"
	"    (cog-map-type prt-atom type)\n"
	"    ; We have to recurse over sub-types\n"
	"    (for-each prt-type (cog-get-subtypes type))\n"
	"  )\n"
	"  (prt-type 'Atom)\n"
	")\n"
	"(prt-atomspace)\n";


const char * Parser::alternatives_anchor = "(AnchorNode \"# Viterbi Alternatives\")";

Parser::Parser(Dictionary dict)
	: _dict(dict), _scm_eval(SchemeEval::instance())
{
	DBG(cout << "=============== Parser ctor ===============" << endl);
	initialize_state();

	// XXX TODO -- this should load up a bunch of scm files,
	// and should also load the query module, as otherwise
	// cog-bind will fail!
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
	std::string scm = lg_exp_to_scm_string(exp);
	return _scm_eval.eval_h(scm);
}

std::string Parser::lg_exp_to_scm_string(Exp* exp)
{
	if (CONNECTOR_type == exp->type)
	{
		stringstream ss;
		ss << "(LgConnector (LgConnectorNode ";
		ss << "\"" << exp->u.string << "\")";
		ss << "(LgConnDirNode \"" << exp->dir << "\")";
		if (exp->multi) ss << "(LgConnMultiNode \"@\")";
		ss << ")\n";
		return ss.str();
	}

	// Whenever a null appears in an OR-list, it means the 
	// entire OR-list is optional.  A null can never appear
	// in an AND-list.
	E_list* el = exp->u.l;
	if (NULL == el)
		return "(LgConnector (LgConnectorNode \"0\"))\n";

	// The C data structure that link-grammar uses for connector
	// expressions is totally insane, as witnessed by the loop below.
	// Anyway: operators are infixed, i.e. are always binary,
	// with exp->u.l->e being the left hand side, and
	//      exp->u.l->next->e being the right hand side.
	// This means that exp->u.l->next->next is always null.
	std::string alist;

	if (AND_type == exp->type)
		alist = "(LgAnd ";

	if (OR_type == exp->type)
		alist = "(LgOr ";

	alist += lg_exp_to_scm_string(el->e);
	el = el->next;

	while (el && exp->type == el->e->type)
	{
		el = el->e->u.l;
		alist += lg_exp_to_scm_string(el->e);
		el = el->next;
	}

	if (el)
		alist += lg_exp_to_scm_string(el->e);

	alist.append (")\n");
	return alist;
}

// ===================================================================
/**
 * Return atomic formula connector expression for the given word.
 *
 * This looks up the word in the link-grammar dictionary, and converts
 * the resulting link-grammar connective expression into an formula
 * composed of atoms.
 */
Handle Parser::word_consets(const string& word)
{
	// See if we know about this word, or not.
	Dict_node* dn_head = dictionary_lookup_list(_dict, word.c_str());
	if (!dn_head) return Handle::UNDEFINED;

	std::string set = "(SetLink\n";

	for (Dict_node* dn = dn_head; dn; dn = dn->right)
	{
		Exp* exp = dn->exp;
		DBG({cout << "=============== Parser word: " << dn->string << ": ";
			print_expression(exp); });

		// First atom at the front of the outgoing set is the word itself.
		// Second atom is the first disjuct that must be fulfilled.
		std::string word_cset = "  (LgWordCset (WordNode \"";
		word_cset += word;
		word_cset += "\")\n";
		word_cset += lg_exp_to_scm_string(exp);
		word_cset += "  )\n";

		set += word_cset;
	}

	free_lookup_list(dn_head);

	set += ")\n";

#if 0
// XXX TODO Need to disjoin the expression returned above!
const char * dj = 
"(define dj "
"  (ImplicationLink "
"    (LgAnd "
"       (LgOr "
"       )"
"    )"
"  )"
")";
#endif

	return _scm_eval.eval_h(set);
}


// ===================================================================
/**
 * Set up initial viterbi state for the parser
 */
void Parser::initialize_state()
{
	const char * wall_word = "LEFT-WALL";

	Handle wall_conset_h = word_consets(wall_word);

	// We are expecting the initial wall to be unique.
	OC_ASSERT(atomspace().getArity(wall_conset_h) == 1, "Unexpected wall structure");

	// Initial state is anchored where it can be found.
	stringstream pole;
	pole << "(ListLink " << alternatives_anchor;

	// Initial state: no output, and only the wall cset.
	pole << "(LgStatePair (LgSeq (car (cog-outgoing-set (cog-atom "
	     << wall_conset_h << ")))) (LgSeq)))\n";
	
	_scm_eval.eval(pole);

	// Hmmm.  Some cleanup.  We really don't need the left-wall
	// connector set any more, so delete it.  It bugs me that we
	// need to do this .. it should disappear on its own ...
	stringstream clean;
	clean << "(cog-delete (cog-atom " << wall_conset_h << "))";
	_scm_eval.eval(clean);

	// Print out the atomspace contents
	DBG(cout << _scm_eval.eval(dbg) << endl);
}

// ===================================================================
/**
 * Add a single word to the parse.
 */
void Parser::stream_word(const string& word)
{
	Handle djset = word_consets(word);
	if (Handle::UNDEFINED == djset)
	{
		logger().error() << "Unhandled error; word not in dict: " << word;
		return;
	}

	DBG(cout << _scm_eval.eval(dbg) << endl);
	Handle rule = _scm_eval.eval_h("(cog-bind attach)");

	DBG(cout << "---------- post match -------- " << endl);
	DBG(cout << _scm_eval.eval(dbg) << endl);

#if LATER
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
#endif
}

#if LATER
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

