/*
 * ReportRank.cc
 *
 * Implements the PageRank graph centrality algorithm for word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include "platform.h"
#include <stdio.h>

#include "ForeachWord.h"
#include "ReportRank.h"
#include "Node.h"
#include "SimpleTruthValue.h"
#include "TruthValue.h"

using namespace opencog;

#define DEBUG

ReportRank::ReportRank(void)
{
	parse_cnt = 0;
}

ReportRank::~ReportRank()
{
}

/**
 * For each parse of the sentence, make a report.
 */
void ReportRank::report_sentence(Handle h)
{
	parse_cnt = 0;
	foreach_parse(h, &ReportRank::report_parse_f, this);
}

/**
 * For each parse, walk over each word.
 */
void ReportRank::report_parse(Handle h)
{
	printf ("Parse %d:\n", parse_cnt);
	foreach_word_instance(h, &ReportRank::report_word, this);
	parse_cnt ++;
}

bool ReportRank::report_parse_f(Handle h)
{
	report_parse(h);
	return false;
}

/**
 * Report the parse rank for this word.
 */
bool ReportRank::report_word(Handle h)
{
	// Only noun-senses and verb-senses get ranked.
	std::string pos = get_part_of_speech(h);
	if (pos.compare("noun") && pos.compare("verb")) return false;

	hi_score = 0.0;
	hi_scorer = UNDEFINED_HANDLE;
	foreach_word_sense_of_inst(h, &ReportRank::choose_sense, this);

	Node *word = dynamic_cast<Node *>(TLB::getAtom(h));
	if (hi_scorer != UNDEFINED_HANDLE)
	{
		Node *sense = dynamic_cast<Node *>(TLB::getAtom(hi_scorer));
		printf ("%s sense %s score=%g\n", 
		        word->getName().c_str(), sense->getName().c_str(), hi_score);
	}
	else
	{
		printf ("No word sense found for %s\n", word->getName().c_str());
	}

	return false;
}

bool ReportRank::choose_sense(Handle word_sense_h,
                              Handle sense_link_h)
{
	Link *l = dynamic_cast<Link *>(TLB::getAtom(sense_link_h));
	double score = l->getTruthValue().getMean();
Node *n = dynamic_cast<Node *>(TLB::getAtom(word_sense_h));
printf ("word sense=%s score=%f hi=%f\n", n->getName().c_str(), score, hi_score);
	if (hi_score < score)
	{
		hi_score = score;
		hi_scorer = word_sense_h;
	}
	return false;
}

/* ============================== END OF FILE ====================== */
