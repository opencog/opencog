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
#include <vector>

#include "utilities.h" // From base link-grammar

#include "atom.h"
#include "compress.h"
#include "connect.h"
#include "word-monad.h"

using namespace std;

// #define DBG(X) X;
#define DBG(X) 

namespace link_grammar {
namespace viterbi {

/**
 * This class is vaguely monad-like.  The constructor takes some
 * arguments.  It returns a "function" called operator() which
 * acts on state, and returns different state.
 *
 * The "core" function wrapped by the monad is reassemble().
 * Everything else is a 'wrapper' around this, handling the various
 * special cases that appear in the state.
 *
 * It is both a C++ functor (because it defines operator()) and it is
 * also a hypergraph functor (because it maps one hypergraph into
 * another).  It has an adjoint functor, which can look at the parse
 * state, and rip out the right-most word, to restore the original
 * state.  (This adjoint is not currently implemented; its not
 * obviously useful.  A different functor, which looks at state, and
 * generates a string of words, left-to-right, might be useful for
 * speech generation.)
 */

/**
 * constructor: argument is a connector set (typically, for a single
 * word) that will be used to try connections. This connector set is
 * assumed to be to the right of the argument to the try_connect()
 * method.
 *
 * To be precise, the right_wconset is presumed to be of the form:
 *	WORD_CSET :
 *    WORD : jabberwoky.n
 *	   OR :
 *       CONNECTOR : Wd-
 *       CONNECTOR : Abc+ etc...
 *
 * In particular, it is assumed to be in DNF (disjunctive normal form).
 */
WordMonad::WordMonad(WordCset* right_wconset)
	: _right_cset(right_wconset)
{
	assert(_right_cset, "Unexpected NULL dictionary entry!");
	DBG(cout << "--------------- WordMonad ctor right=\n" << _right_cset << endl);
}

/// Unite two states into one. 
///
/// This is needed to implement zippering properly: The old set of
/// states is the collection of as-yet unconnected connectors; the 
/// new set of states is the collection that remains after connecting.
/// We have to peel off and discard some certain number of the old
/// states (as these are now connected), and append in their place
/// the new states.  We also typically peel off one new one, as that
/// one will be used for trying new onnections.
static StateTriple* unite(StateTriple* old_sp, size_t old_peel_off,
                          StateTriple* new_sp, size_t new_peel_off)
{
	OutList united_states;
	Seq* old_state = old_sp->get_state();
	Seq* new_state = new_sp->get_state();

	// The no-links-cross rule means that the first state with
	// unconnected right-going links must necessarily be at the
	// head of the state vector. That is, new comes before old.
	const OutList& no = new_state->get_outgoing_set();
	united_states.insert(united_states.end(), 
	                     no.begin() + new_peel_off, no.end());

	const OutList& oo = old_state->get_outgoing_set();
	united_states.insert(united_states.end(), 
	                     oo.begin() + old_peel_off, oo.end());

	// Unite the outputs too ... 
	// This is easy, just concatenate old and append new.
	OutList united_outputs;
	Set* old_output = old_sp->get_output();
	Set* new_output = new_sp->get_output();

	// I don't think the output order matters much, but appending
	// new output to old seems reasonable.
	const OutList& ooo = old_output->get_outgoing_set();
	united_outputs.insert(united_outputs.end(), ooo.begin(), ooo.end());

	const OutList& noo = new_output->get_outgoing_set();
	united_outputs.insert(united_outputs.end(), noo.begin(), noo.end());

// XXX clearly failing to deal with input ...
	return new StateTriple(new Seq(), new Seq(united_states), new Set(united_outputs));
}

/**
 * Try connecting this connector set, from the left, to the right cset.
 * This returns a set of alternative connections, as state pairs: each
 * alternative will consist of new state, and the links that were issued.
 */
static Set* next_connect(WordCset* left_cset, WordCset* right_cset)
{
	assert(left_cset, "State word-connectorset is null");
	Atom* left_a = left_cset->get_cset();

	DBG(cout << "Enter next_connect(), left cset " << left_a << endl);

	// Wrap bare connector with OR; this simplifie the nested loop below.
	Or* left_dnf = NULL;
	AtomType lt = left_a->get_type();
	if (CONNECTOR == lt or AND == lt)
		left_dnf = new Or(left_a);
	else
		left_dnf = upcast<Or*>(left_a);
	assert(left_dnf != NULL, "Left disjuncts not in DNF");


	Atom *right_a = right_cset->get_cset();
	DBG(cout << "in next_connect(), right cset "<< right_a <<endl);
	Or* right_dnf = NULL;
	AtomType rt = right_a->get_type();
	if (CONNECTOR == rt or AND == rt)
		right_dnf = new Or(right_a);
	else
		right_dnf = upcast<Or*>(right_a);
	assert(right_dnf != NULL, "Right disjuncts not in DNF");


	// At this point, both left_dnf and right_dnf should be in
	// disjunctive normal form.  Perform a nested loop over each
	// of them, connecting each to each.

	// "alternatives" records the various different successful ways
	// that connectors can be mated.  Its a list of state pairs.
	OutList alternatives;
	foreach_outgoing(Atom*, ldj, left_dnf)
	{
		foreach_outgoing(Atom*, rdj, right_dnf)
		{
			Connect cnct(left_cset, right_cset);
			StateTriple* sp = cnct.try_alternative(ldj, rdj);
			if (sp)
				alternatives.push_back(sp);
		}
	}

	return compress_alternatives(new Set(alternatives));
}

/**
 * Try connecting this connector set sequence, from the left, to what
 * was passed in ctor.  It is preseumed that left_sp is a single parse
 * state: it should contain no left-pointing connectors whatsoever.  This
 * routine will attempt to attach the right-pointing connectors to the
 * left-pointers passed in the ctor.  A connection is considered to be
 * successful if *all* left-pointers in the ctor were attached (except
 * possibly for optionals).  The returned value is a set of all possible
 * alternative state pairs for which there is a connnection.
 *
 * WordMonadors must be satisfied sequentially: that is, the first connector
 * set in the sequence must be fully satisfied before a connection is made
 * to the second one in the sequence, etc. (counting from zero).
 */
static Set* try_connect_one(StateTriple* left_sp, WordCset* right_cset)
{
	// Zipper up the zipper.
	// The left state can the thought of as a strand of zipper teeth,
	// or half of a strand of DNA, if you wish. Each of the zipper teeth
	// are right-pointing connectors.  These must be mated with the
	// left-pointing connectors from the right cset.  The reason its
	// zipper-like is that the right cset might connect to multiple csets
	// on the left.  The connectins must be made in order, and so we loop
	// through the left state, first trying to satisfy all connectors in
	// the first cset, then the second, and so on, until all of the
	// left-pointing connectors in the right cset have been connected,
	// or they're optional, or there is a failure to connect. 
	Seq* left_state = left_sp->get_state();
	Atom* a = left_state->get_outgoing_atom(0);
	WordCset* lwc = dynamic_cast<WordCset*>(a);
	Set* alternatives = next_connect(lwc, right_cset);

	size_t lsz = left_state->get_arity();
	size_t lnext = 1;

	// OK, so do any of the alternatives include state with
	// left-pointing connectors?  If not, then we are done. If so,
	// then these need to be mated to the next word cset on the left.
	// If they can't be mated, then fail, and we are done.
	OutList filtered_alts;
	foreach_outgoing(StateTriple*, new_sp, alternatives)
	{
		Seq* new_state = new_sp->get_state();

		if (0 < new_state->get_arity())
		{
			Atom* a = new_state->get_outgoing_atom(0);
			WordCset* new_cset = dynamic_cast<WordCset*>(a);
			if (new_cset->has_left_pointers())
			{
				// The left-pointers are either mandatory or optional.
				// If they're mandatory and there is no state to zipper with,
				// then its a parse fail. Otherwise recurse.
// XXX check for optional...
				if (lnext < lsz)
				{
					// Atom* a = left_state->get_outgoing_atom(lnext);
				
// cout <<"rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrecurse"<<endl;
// cout << "old sp, rm "<<lnext<<": " << left_sp<<endl;
// cout << "new sp rm 1:" << new_sp<<endl;
					// Recurse -- zipper up the next unconnected left-pointer.
					StateTriple* united_sp = unite(left_sp, lnext, new_sp, 1);
					// zero out output before continuing... we want only
					// the fresh output just minted by the recurse
					StateTriple* usp = new StateTriple(new Seq(), 
					            united_sp->get_state(), new Set());
					DBG(cout << "United states:" << united_sp << endl);
					Set* new_alts = try_connect_one(usp, new_cset);
// cout << "woot got this:" << new_alts<<endl;

					foreach_outgoing(StateTriple*, asp, new_alts)
					{
						StateTriple* mrg = unite(united_sp, 1, asp, 0);
						filtered_alts.push_back(mrg);
						DBG(cout << "merge result=" << mrg << endl);
					}
					continue;
				}
// XXX we need to increment lnext at some point ...  if there are more
// than one lefty that is undone.
			}
		}
		
		// Append old and new output and state, before pushing back.
// cout<<"mooooooooooooooooooooooooooooooooooooooooooooore"<<endl;
		StateTriple* mrg = unite(left_sp, lnext, new_sp, 0);
//cout<<"left sp was "<<left_sp<<endl;
//cout<<"new sp was "<<new_sp<<endl;
//cout<<"mrg is "<<mrg<<endl;
		filtered_alts.push_back(mrg);
	}

	return new Set(filtered_alts);
}

/**
 * Given a set of state alternatives, try attaching the connector set
 * that was given in the constructor.  Return a set of states that
 * result from successful attachements.
 *
 * For just right now, we've given this the signature of "operator()"
 * because we want to suggest the monad-like nature of this thing:
 * its being given a state, and its transforming it to some other state.
 * its expected that someday, in the future, there will be a unch of
 * these.  In OpenCog terms, this is kind-of-like a "MindAgent", except
 * that it is not an independent thread/process.  The ideas expressed
 * in these last few sentences are subject to change...
 */
Set* WordMonad::operator()(Set* alts)
{
	// The state set consists of a bunch of sequences; each sequence
	// being a single parse state.  Each parse state is a sequence of
	// unsatisfied right-pointing links.
	Set* new_alts = new Set();
	foreach_outgoing(StateTriple*, sp, alts)
	{
		assert(sp, "Missing state");

		Seq* state = sp->get_state();
		if (0 == state->get_arity())
		{
			// If there is nothing to connect to, then we presume that we're
			// dealing with a sentence boundary.  Seentence boundaries cannot
			// have any left-pointers in them.
			if (_right_cset->has_left_pointers())
				continue;
			Set* next_alts = new Set(new StateTriple(new Seq(),
			                         new Seq(_right_cset), new Set()));
			new_alts = new_alts->sum(next_alts);
			continue;
		}
		// Each state sequence consists of a sequence of right-pointing
		// links. These must be sequentially satisfied: This is the
		// viterbi equivalent of "planar graph" or "no-crossing-links"
		// in the classical link-grammar parser.  That is, a new word
		// must link to the first sequence element that has dangling
		// right-pointing connectors.
		Set* next_alts = try_connect_one(sp, _right_cset);
		new_alts = new_alts->sum(next_alts);
	}

	// set_clean_state(new_state_set);
	return new_alts;
}

} // namespace viterbi
} // namespace link-grammar
