/*
 * Mihalcea.cc
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "Mihalcea.h"

#include <stdio.h>

#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/util/platform.h>
#include <opencog/nlp/wsd/MihalceaEdge.h>
#include <opencog/nlp/wsd/MihalceaLabel.h>
#include <opencog/nlp/wsd/ParseRank.h>
#include <opencog/nlp/wsd/SenseRank.h>
#include <opencog/nlp/wsd/ReportRank.h>

using namespace opencog;

#define DEBUG

Mihalcea::Mihalcea(void)
{
	atom_space = NULL;
	previous_parse = Handle::UNDEFINED;
}

Mihalcea::~Mihalcea()
{
	atom_space = NULL;
}

void Mihalcea::set_atom_space(AtomSpace *as)
{
	atom_space = as;
	labeller.set_atom_space(as);
	edger.set_atom_space(as);
	thinner.set_atom_space(as);
}

bool Mihalcea::process_sentence(Handle h)
{
	Handle top_parse = parse_ranker.get_top_ranked_parse(h);
	parse_list.push_back(top_parse);

#ifdef DEBUG
	printf("; Mihalcea::process_sentence parse %lx for sentence %lx\n", top_parse.value(), h.value());
#endif

	// Attach senses to word instances
	labeller.annotate_parse(top_parse);

	// Create edges between sense pairs
	edger.annotate_parse(top_parse);

	// Tweak, based on parser markup
	// nn_adjuster.adjust_parse(top_parse);

	// Link sentences together, since presumably the next
	// sentence deals with topics similar to the previous one.
	if (Handle::UNDEFINED != previous_parse)
	{
		edger.annotate_parse_pair(previous_parse, top_parse);

		// If there are any senses that are not attached to anything,
		// get rid of them now.
		thinner.prune_parse(previous_parse);
	}
	previous_parse = top_parse;

#define WINDOW_SIZE 3
	// Create a short list of the last 3 sentences
	// This is a sliding window of related words.
	short_list.push_back(top_parse);
	if (WINDOW_SIZE < short_list.size())
	{
		Handle earliest = short_list.front();
		short_list.pop_front();
		thinner.thin_parse(earliest, 0);
	}
#define THICKNESS 2
	if (WINDOW_SIZE == short_list.size())
	{
		Handle first = short_list.front();
		thinner.thin_parse(first, THICKNESS);
	}

	// Solve the page-rank equations for the short list.
	sense_ranker.rank_document(short_list);

	return false;
}

bool Mihalcea::process_sentence_list(Handle h)
{
	foreach_outgoing_handle(h, &Mihalcea::process_sentence, this);

	// Solve the page-rank equations for the whole set of sentences.
	// sense_ranker.rank_document(parse_list);
	// No .. don't. Use the sliding-window mechanism, above.

	// Finish up the sliding window processing.
	// Basically, delete all remaining similarity links.
	while (0 < short_list.size())
	{
		Handle earliest = short_list.front();
		short_list.pop_front();
		// double-pass -- first with thickness, to thin senses,
		// then with no thickness, to remove sim links.
		thinner.thin_parse(earliest, THICKNESS);
		thinner.thin_parse(earliest, 0);
	}

	// Report the results.
	reporter.report_document(parse_list);

	return false;
}

void Mihalcea::process_document(Handle h)
{
	foreach_binary_link(h, REFERENCE_LINK, &Mihalcea::process_sentence_list, this);
}
