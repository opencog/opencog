/*
 * Mihalcea.cc
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "Mihalcea.h"
#include "MihalceaEdge.h"
#include "MihalceaLabel.h"
#include "SenseRank.h"
#include "ReportRank.h"

using namespace opencog;

Mihalcea::Mihalcea(void)
{
	atom_space = NULL;
	labeller = new MihalceaLabel();
	edger = new MihalceaEdge();
	nn_adjuster = new NNAdjust();
	ranker = new SenseRank();
	reporter = new ReportRank();
}

Mihalcea::~Mihalcea()
{
	atom_space = NULL;
	delete labeller;
	delete edger;
	delete nn_adjuster;
	delete ranker;
	delete reporter;
}

void Mihalcea::set_atom_space(AtomSpace *as)
{
	atom_space = as;
	labeller->set_atom_space(as);
	edger->set_atom_space(as);
}

void Mihalcea::process_sentence(Handle h)
{
	// Add handle to sentence to our running list.
	sentence_list.push_back(h);

	labeller->annotate_sentence(h);
	edger->annotate_sentence(h);
	nn_adjuster->adjust_sentence(h);
	ranker->iterate(h);
	reporter->report_rank(h);
}

