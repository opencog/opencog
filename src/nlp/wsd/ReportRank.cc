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
#ifdef DEBUG
	printf ("; Parse %d:\n", parse_cnt);
#endif
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
	normalization = 0.0;
	foreach_word_sense_of_inst(h, &ReportRank::sum_score, this);
printf("duude norm was %g\n", normalization);

	if (1.0e-6 >= normalization)
	{
		Node *word = dynamic_cast<Node *>(TLB::getAtom(h));
		printf ("; No word sense found for %s\n", word->getName().c_str());
		return false;
	}

	normalization = 1.0 / normalization;
	foreach_word_sense_of_inst(h, &ReportRank::renorm_score, this);
	return false;
}

bool ReportRank::sum_score(Handle word_sense_h,
                              Handle sense_link_h)
{
	Link *l = dynamic_cast<Link *>(TLB::getAtom(sense_link_h));
	normalization += l->getTruthValue().getMean();
	return false;
}

bool ReportRank::renorm_score(Handle word_sense_h,
                              Handle sense_link_h)
{
	Link *l = dynamic_cast<Link *>(TLB::getAtom(sense_link_h));
	double score = l->getTruthValue().getMean();
	score *= normalization;

	// Update the truth value, so that it now functions as a
	// normalized probability
	SimpleTruthValue stv((float) score, 1.0f);
	stv.setConfidence(l->getTruthValue().getConfidence());
	l->setTruthValue(stv);

Node *n = dynamic_cast<Node *>(TLB::getAtom(word_sense_h));
printf ("duu word sense=%s score=%f\n", n->getName().c_str(), score);
	return false;
}

/* ============================== END OF FILE ====================== */
