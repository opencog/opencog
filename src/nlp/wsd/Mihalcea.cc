/*
 * Mihalcea.cc
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include "platform.h"
#include <stdio.h>

#include "ForeachChaseLink.h"
#include "Mihalcea.h"
#include "MihalceaEdge.h"
#include "MihalceaLabel.h"
#include "ParseRank.h"
#include "SenseRank.h"
#include "ReportRank.h"

using namespace opencog;

#define DEBUG

Mihalcea::Mihalcea(void)
{
	atom_space = NULL;
	labeller = new MihalceaLabel();
	edger = new MihalceaEdge();
	nn_adjuster = new NNAdjust();
	parse_ranker = new ParseRank();
	sense_ranker = new SenseRank();
	reporter = new ReportRank();

	previous_parse = UNDEFINED_HANDLE;
}

Mihalcea::~Mihalcea()
{
	atom_space = NULL;
	delete labeller;
	delete edger;
	delete nn_adjuster;
	delete parse_ranker;
	delete sense_ranker;
	delete reporter;
}

void Mihalcea::set_atom_space(AtomSpace *as)
{
	atom_space = as;
	labeller->set_atom_space(as);
	edger->set_atom_space(as);
}

bool Mihalcea::process_sentence(Handle h)
{
	Handle top_parse = parse_ranker->get_top_ranked_parse(h);
	parse_list.push_back(top_parse);

#ifdef DEBUG
	printf("; Handling parse %lx for sentence %lx\n", top_parse, h); 
#endif

	// Attach senses to word instances
	labeller->annotate_parse(top_parse);

	// Create edges between sense pairs
	edger->annotate_parse(top_parse);

	// Tweak, based on parser markup
	// nn_adjuster->adjust_parse(top_parse);

	// Link sentences together, since presumably the next 
	// sentence deals with topics similar to the previous one.
	if (UNDEFINED_HANDLE != previous_parse)
	{
		edger->annotate_parse_pair(previous_parse, top_parse);
	}
	previous_parse = top_parse;

	// Assign initial probabilities to each sense.
	sense_ranker->init_parse(top_parse);
	return false;
}

bool Mihalcea::process_sentence_list(Handle h)
{
	foreach_outgoing_handle(h, &Mihalcea::process_sentence, this);

	// Iterate over parse list
	vector<Handle>::const_iterator i;
	for (i = parse_list.begin(); i != parse_list.end(); i++)
	{
		sense_ranker->rank_parse(*i);
	}
	// reporter->report_parse(top_parse);
	return false;
}

void Mihalcea::process_document(Handle h)
{
	foreach_binary_link(h, REFERENCE_LINK, &Mihalcea::process_sentence_list, this);
}
