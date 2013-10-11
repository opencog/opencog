/*
 * ParseRank.cc
 *
 * Place-holder -- does nothing except return the top-ranked parse,
 * as previously ranked by relex.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "ParseRank.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/nlp/wsd/ForeachWord.h>

using namespace opencog;

// #define DEBUG

ParseRank::ParseRank(void)
{
}

ParseRank::~ParseRank()
{
}

/**
 * For each parse of the sentence, perform the ranking algo.
 * The input handle is assumed to point to a sentence.
 * The returned value is the highest-ranked parse of the
 * bunch.
 */
Handle ParseRank::get_top_ranked_parse(Handle h)
{
	top = Handle::UNDEFINED;
	top_rank = -123456.0;
	foreach_parse(h, &ParseRank::lookat_parse, this);
	return top;
}

/**
 * Get the parse ranking for a given parse.
 */
bool ParseRank::lookat_parse(Handle h)
{
	double rank = h->getTruthValue().getConfidence();
#ifdef DEBUG
	printf("; ParseRank::lookat_parse parse=%lx rank=%f\n", (unsigned long) h, rank);
#endif
	if (top_rank < rank)
	{
		top_rank = rank;
		top = h;
	}
	return false;
}

/* ============================== END OF FILE ====================== */
