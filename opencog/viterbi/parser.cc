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

#include <link-grammar/dict-api.h>

#include <opencog/util/oc_assert.h>
#include <opencog/util/Logger.h>
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

Parser::Parser(Dictionary dict, AtomSpace *as)
	: _dict(dict), _scm_eval(new SchemeEval(as)), _dict_reader(new LGDictReader(dict, as))
{
	DBG(cout << "=============== Parser ctor ===============" << endl);
	initialize_state();

	// XXX TODO -- this should load up a bunch of scm files,
	// and should also load the query module, as otherwise
	// cog-bind will fail!
}

Parser::~Parser()
{
	delete _scm_eval;
	delete _dict_reader;
}

// ===================================================================
/**
 * Set up initial viterbi state for the parser
 */
void Parser::initialize_state()
{
	const char * wall_word = "LEFT-WALL";

	Handle wall_conset_h = _dict_reader->getAtom(wall_word);

	// We are expecting the initial wall to be unique.
	//OC_ASSERT(atomspace().getArity(wall_conset_h) == 1, "Unexpected wall structure");

	// Initial state is anchored where it can be found.
	stringstream pole;
	pole << "(ListLink " << alternatives_anchor;

	// Initial state: no output, and only the wall cset.
	pole << "(LgStatePair (LgSeq (car (cog-outgoing-set (cog-atom "
	     << wall_conset_h << ")))) (LgSeq)))\n";
	
	_scm_eval->eval(pole);

	// Hmmm.  Some cleanup.  We really don't need the left-wall
	// connector set any more, so delete it.  It bugs me that we
	// need to do this .. it should disappear on its own ...
	stringstream clean;
	clean << "(cog-delete (cog-atom " << wall_conset_h << "))";
	_scm_eval->eval(clean);

	// Print out the atomspace contents
	DBG(cout << _scm_eval->eval(dbg) << endl);
}

// ===================================================================
/**
 * Add a single word to the parse.
 */
void Parser::stream_word(const string& word)
{
	Handle djset = _dict_reader->getAtom(word);
	if (Handle::UNDEFINED == djset)
	{
		logger().error() << "Unhandled error; word not in dict: " << word;
		return;
	}

	DBG(cout << _scm_eval->eval(dbg) << endl);
	Handle rule = _scm_eval->eval_h("(cog-bind attach)");

	DBG(cout << "---------- post match -------- " << endl);
	DBG(cout << _scm_eval->eval(dbg) << endl);

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

