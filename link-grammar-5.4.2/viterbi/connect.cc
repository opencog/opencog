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
#include "connect.h"
#include "connector-utils.h"

using namespace std;

// #define DBG(X) X;
#define DBG(X) 

namespace link_grammar {
namespace viterbi {

/**
 * This class is vaguely monad-like.  
 */
Connect::Connect(WordCset* lcs, WordCset* rcs)
	: _left_cset(lcs), _right_cset(rcs)
{}

// =============================================================

/// Try to connect the left and right disjuncts.
///
/// If the connection attempt is successful, then return a
/// StateTriple given the emitted output, and the resulting state.
///
/// The implementation below is just a dispatcher for each of the
/// alternative handlers, depending on whether the arguments are
/// single or multi-connectors.
StateTriple* Connect::try_alternative(Atom* ldj, Atom* rdj)
{
	Connector* lcon = dynamic_cast<Connector*>(ldj);
	Connector* rcon = dynamic_cast<Connector*>(rdj);

	// Left disjunct is a single connector
	if (lcon)
	{
		// Right disjunct is a single connector
		if (rcon)
			return alternative(lcon, rcon);
		else
		{
			// Right disunct better be a multi-connector
			And* rand = upcast<And*>(rdj);
			assert(rand, "Right dj not a disjunct");
			return alternative(lcon, rand);
		}
	}
	else
	{
		// Left disjunct better be a multi-connector.
		And* land = upcast<And*>(ldj);
		assert(land, "Left dj not a disjunct");

		// Right disjunct is a single connector
		if (rcon)
			return alternative(land, rcon);
		else
		{
			// Right disunct better be a multi-connector
			And* rand = upcast<And*>(rdj);
			assert(rand, "Right dj not a disjunct");

			return alternative(land, rand);
		}
	}

	return NULL; // Not reached.
}

// =============================================================
/// Try connecting the left and right disjuncts.
///
/// If a connection was made, return the resulting state pair.
/// If no connection is possible, return NULL.
///
/// The state pair will contain the output generated (if any) and the
/// final state (if any) after the connection is made.
///
/// There are four distinct methods below, depending on whether
/// each disjunct is a single or a multi connector.  Multi-connectors
/// are just a list of conjoind (AND) single-connectors.  A multi-
/// connector is also called a "disjunct" because it is one of the
/// terms in a connector set that has been expanded into dsjunctive
/// normal form. Viz. a single disjunct is a conjoined set of
/// connectors.
//
StateTriple* Connect::alternative(Connector* lcon, Connector* rcon)
{
	Ling* conn = conn_connect_nn(lcon, rcon);
	if (!conn)
	{
		// If we are here, then no connection was possible. It may
		// be the case that rcon was right-pointing, in which case,
		// it can be added to the state.
		char dir = rcon->get_direction();
		if ('-' == dir)
			return NULL;

		Word* lword = _left_cset->get_word();
		Word* rword = _right_cset->get_word();
		WordCset* lcset = new WordCset(lword, lcon);
		WordCset* rcset = new WordCset(rword, rcon);
		Seq* state = new Seq(rcset, lcset);
		StateTriple* sp = new StateTriple(new Seq(), state, new Set());
		DBG(cout<<"------ Empty-output alternative created:\n" << sp << endl;);
		return sp;
	}

	// At this point, conn holds an LG link type, and the
	// two disjuncts that were mated.  Re-assemble these
	// into a pair of word_disjuncts (i.e. stick the word
	// back in there, as that is what later stages need).
	Set* out = new Set(conn);

	// Meanwhile, we exhausted the state, so that's empty.
	StateTriple* sp = new StateTriple(new Seq(), new Seq(), out);
   DBG(cout<<"----- single-connector alternative created:\n" << sp << endl;);
	return sp;
}

// See docs above
StateTriple* Connect::alternative(Connector* lcon, And* rand)
{
	if (0 == rand->get_arity())
		return NULL;

	Atom* rfirst = rand->get_outgoing_atom(0);
	Connector* rfc = dynamic_cast<Connector*>(rfirst);
	assert(rfc, "Exepcting a connector in the right disjunct");

	Ling* conn = conn_connect_nn(lcon, rfc);
// XXX fixme ;;; if all left-pointers are opt, then OK to create state ...
	if (!conn)
		return NULL;

	// At this point, conn holds an LG link type, and the
	// two disjuncts that were mated.  Re-assemble these
	// into a pair of word_disjuncts (i.e. stick the word
	// back in there, as that is what later stages need).
	Set* out = new Set(conn);

	// The state is now everything else left in the disjunct.
	// We need to build this back up into WordCset.
	OutList remaining_cons = rand->get_outgoing_set();
	remaining_cons.erase(remaining_cons.begin());
	And* remaining_cj = new And(remaining_cons);
	Atom* rema = remaining_cj->super_flatten();
	WordCset* rem_cset = new WordCset(_right_cset->get_word(), rema);

// XXX TODO probably need to flatten haere and make sure that
// the new state isn't empty.
Link* rl = dynamic_cast<Link*>(rema);
if (rl and rl->get_arity() == 0)
assert(0, "Need to handle this empty state case like all the others");

	StateTriple* sp = new StateTriple(new Seq(), new Seq(rem_cset), out);
   DBG(cout<<"----- right multi-conn alternative created:\n" << sp << endl;);
	return sp;
}

// See docs above
StateTriple* Connect::alternative(And* land, Connector* rcon)
{
	Atom* lfirst = land->get_outgoing_atom(0);
	Connector* lfc = dynamic_cast<Connector*>(lfirst);
	assert(lfc, "Exepcting a connector in the left disjunct");

	Ling* conn = conn_connect_nn(lfc, rcon);
	if (!conn)
		return NULL;

	// At this point, conn holds an LG link type, and the
	// two disjuncts that were mated.  Re-assemble these
	// into a pair of word_disjuncts (i.e. stick the word
	// back in there, as that is what later stages need).
	Set* out = new Set(conn);

	// The state is now everything left in the disjunct.
	// We need to build this back up into WordCset.
	OutList remaining_cons = land->get_outgoing_set();
	remaining_cons.erase(remaining_cons.begin());
	And* remaining_cj = new And(remaining_cons);
	WordCset* rem_cset = new WordCset(_left_cset->get_word(), remaining_cj);

	// The remaining cset could be empty (e.g. an AND link with
	// nothing left in it.)
	rem_cset = rem_cset->flatten();
	StateTriple* sp;
	if (NULL != rem_cset)
		sp = new StateTriple(new Seq(), new Seq(rem_cset), out);
	else
		sp = new StateTriple(new Seq(), new Seq(), out);

	DBG(cout << "=================> state triple created: " << sp << endl);
	return sp;
}

// See docs above
StateTriple* Connect::alternative(And* land, And* rand)
{
// cout<<"duude land="<<land<<endl;
// cout<<"duude rand="<<rand<<endl;

	OutList outputs;
	size_t m = 0;
	size_t rsz = rand->get_arity();
	size_t lsz = land->get_arity();
	size_t sz = (lsz<rsz) ? lsz : rsz;
	while (m < sz)
	{
		Atom* rfirst = rand->get_outgoing_atom(m);
		Connector* rfc = dynamic_cast<Connector*>(rfirst);
		assert(rfc, "Exepecting a connector in the right disjunct");

		Atom* lfirst = land->get_outgoing_atom(m);
		Connector* lfc = dynamic_cast<Connector*>(lfirst);
		assert(lfc, "Exepecting a connector in the left disjunct");


		Ling* conn = conn_connect_nn(lfc, rfc);
		if (!conn)
			break;
		m++;

		// At this point, conn holds an LG link type, and the
		// two disjuncts that were mated.  Re-assemble these
		// into a pair of word_disjuncts (i.e. stick the word
		// back in there, as that is what later stages need).
		outputs.push_back(conn);
	}
	if (0 == m)
		return NULL;

	// Add the un-connected parts of the left and right csets
	// to the state.  But first, check to make sure that the
	// right cset does not have any (non-optional)
	// left-pointers, because these will never be fulfilled.
	// Lets start with the right cset.
	// We need to build this back up into WordCset.
	OutList remaining_cons = rand->get_outgoing_set();
	for (size_t k = 0; k<m; k++)
		remaining_cons.erase(remaining_cons.begin());

	And* remaining_cj = new And(remaining_cons);
	WordCset* rem_cset = new WordCset(_right_cset->get_word(), remaining_cj);
	rem_cset = cset_trim_left_pointers(rem_cset);
	if (NULL == rem_cset)
		return NULL;

	// If we are here, the remaining right connectors all
	// point right.  Put them into the state.
	OutList statel;
	statel.push_back(rem_cset);

	// And now repeat for the left cset.
	remaining_cons = land->get_outgoing_set();
	for (size_t k = 0; k<m; k++)
		remaining_cons.erase(remaining_cons.begin());

	if (0 < remaining_cons.size())
	{
		remaining_cj = new And(remaining_cons);
		rem_cset = new WordCset(_left_cset->get_word(), remaining_cj);
		statel.push_back(rem_cset);
	}

	Seq* state = new Seq(statel);

	Set* out = new Set(outputs);

	StateTriple* sp = new StateTriple(new Seq(), state, out);
	DBG(cout << "============> multi state pair created: " << sp << endl);
	return sp;
}

// =============================================================

// At this point, conn holds an LG link type, and the
// two disjuncts that were mated.  Re-assemble these
// into a pair of word_disjuncts (i.e. stick the word
// back in there, as that is what later stages need).
//
// The left_cset and right_cset are assumed to be the word-connector
// sets that matched. These are needed, only to extract the words;
// the rest is dicarded.
Ling* Connect::reassemble(Ling* conn, WordCset* left_cset, WordCset* right_cset)
{
	assert(conn, "Bad cast to Ling");

	OutList lwdj;
	lwdj.push_back(left_cset->get_word());    // the word
	lwdj.push_back(conn->get_left());         // the connector
	Link *lwordj = new Link(WORD_DISJ, lwdj);

	OutList rwdj;
	rwdj.push_back(right_cset->get_word());   // the word
	rwdj.push_back(conn->get_right());        // the connector
	Link *rwordj = new Link(WORD_DISJ, rwdj);

	Ling *lg_link = new Ling(conn->get_ling_type(), lwordj, rwordj);

	return lg_link;
}

// =============================================================
/**
 * Try to connect the left and right connectors. If they do connect,
 * then return an LG_LING structure linking them.
 */
Ling* Connect::conn_connect_nn(Connector* lnode, Connector* rnode)
{
// cout<<"try match connectors l="<<lnode->get_name()<<" to r="<< rnode->get_name() << endl;
	if (lnode->is_optional()) return NULL;
	if (rnode->is_optional()) return NULL;
	if (!conn_match(lnode->get_name(), rnode->get_name()))
		return NULL;

// cout << "Yayyyyae connectors match!"<<endl;
	NameString link_name = conn_merge(lnode->get_name(), rnode->get_name());
	Ling* ling = new Ling(link_name, lnode, rnode);

	ling = reassemble(ling, _left_cset, _right_cset);
	return ling;
}

// =============================================================

// Collapse singleton sets, if any.  This is the price we pay
// for otherwise being able to ignore the difference between
// singleton sets, and their elements.
const OutList& Connect::flatten(OutList& alternatives)
{
	size_t asize = alternatives.size();

	// If its a singleton, and its already a set ...
	if (1 == asize)
	{
		Set* set = dynamic_cast<Set*>(alternatives[0]);
		if (set)
			return set->get_outgoing_set();
	}

	for (size_t i = 0; i < asize; i++)
	{
		Set* set = dynamic_cast<Set*>(alternatives[i]);
		if (set and (1 == set->get_arity()))
		{
			alternatives[i] = set->get_outgoing_atom(0);
		}
	}
	return alternatives;
}


} // namespace viterbi
} // namespace link-grammar
