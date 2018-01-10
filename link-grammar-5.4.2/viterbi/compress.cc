/*************************************************************************/
/* Copyright (c) 2013 Linas Vepstas <linasvepstas@gmail.com>             */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include "compress.h"

using namespace std;

namespace link_grammar {
namespace viterbi {

// Merge a pair of word-lists, if they are mergable.
// Internal-use only utility for the function below.
static Seq* merge_wordlists(Seq* wla, Seq* wlb)
{
	size_t sza = wla->get_arity();
	size_t szb = wlb->get_arity();
	if (sza != szb)
		return NULL;

	bool already_merged = false;
	OutList seq;
	for (size_t i = 0; i < sza; i++)
	{
		Atom* a = wla->get_outgoing_atom(i);
		Atom* b = wlb->get_outgoing_atom(i);

		if (a->operator==(b))
		{
			seq.push_back(a);
			continue;
		}

		// If we are here, then the word csets differ.
		if (already_merged)
			return NULL;
		already_merged = true;

		// Check to see if the words are the same.
		WordCset* wcsa = dynamic_cast<WordCset*>(a);
		WordCset* wcsb = dynamic_cast<WordCset*>(b);
		Word* wa = wcsa->get_word();
		Word* wb = wcsb->get_word();

		// Words differ, its not mergable.
		if (not wa->operator==(wb))
			return NULL;

		// OK, all is well, merge the connectors.
		Atom* csa = wcsa->get_cset();
		Atom* csb = wcsb->get_cset();
		Or* dj = new Or(csa, csb);
		dj = dj->flatten();
		dj = dj->uniq();

		WordCset* merge = new WordCset(wa, dj);
		seq.push_back(merge);
	}

	return new Seq(seq);
}

/// Try to shrink down a set of alternatives by collapsing repeated
/// states into disjuncts.  Best explained by example:  Suppose that
/// the input is 
/// SET :
///   STATE_TRIPLE :
///     SEQ :
///       WORD : is
///       WORD : a
///       WORD : test
///     SEQ :
///       WORD_CSET :
///         WORD : this
///         CONNECTOR : Ss*b+
///       WORD_CSET :
///         WORD : LEFT-WALL
///         CONNECTOR : Wd+
///     SEQ :
///   STATE_TRIPLE :
///     SEQ :
///       WORD : is
///       WORD : a
///       WORD : test
///     SEQ :
///       WORD_CSET :
///         WORD : this
///         CONNECTOR : Ss*b+
///       WORD_CSET :
///         WORD : LEFT-WALL
///         CONNECTOR : Wi+
///     SEQ :
/// 
/// Then the compressed output will be:
/// SET :
///   STATE_TRIPLE :
///     SEQ :
///       WORD : is
///       WORD : a
///       WORD : test
///     SEQ :
///       WORD_CSET :
///         WORD : this
///         CONNECTOR : Ss*b+
///       WORD_CSET :
///         WORD : LEFT-WALL
///         OR :
///           CONNECTOR : Wd+
///           CONNECTOR : Wi+
///     SEQ :
///
/// Note how two alternative state triples were collapsed down into a
/// single word cset.  The goal of this compression is to shrink down
/// the state to make the output more tractable, slightly less
/// combinatoric-explosion-y.
//
// XXX TODO: if we created a special atom that held only alternatives,
// then this would be a method on that that atom.  Do we need such an
// atom?  It could help avoid confusion ...

Set* compress_alternatives(Set* state_alternatives)
{
	OutList alts;
	Seq* prev_in = NULL;
	Set* prev_out = NULL;
	Seq* merged = NULL;

	foreach_outgoing(StateTriple*, sp, state_alternatives)
	{
		// If the inputs or the outputs differ, the state triples are
		// fundamentally not mergable.  Move along.
		Seq* in = sp->get_input();
		Set* out = sp->get_output();
		if ((not in->operator==(prev_in)) or
		    (not out->operator==(prev_out)))
		{
			if (merged)
				alts.push_back(new StateTriple(prev_in, merged, prev_out));
			prev_in = in;
			prev_out = out;
			merged = sp->get_state();
			continue;
		}

		Seq* m = merge_wordlists(merged, sp->get_state());
		if (NULL == m)
		{
			alts.push_back(sp);
			continue;
		}
		merged = m;
	}

	if (merged)
		alts.push_back(new StateTriple(prev_in, merged, prev_out));

	return new Set(alts);
}

} // namespace viterbi
} // namespace link-grammar

