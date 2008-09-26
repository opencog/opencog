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
	parse_cnt ++;

	normalization = 0.0;
	sense_count = 0.0;
	foreach_word_instance(h, &ReportRank::count_word, this);

printf("duude norm=%g senses=%g\n", normalization, sense_count);

	normalization = 1.0 / normalization;
	foreach_word_instance(h, &ReportRank::renorm_word, this);
}

bool ReportRank::report_parse_f(Handle h)
{
	report_parse(h);
	return false;
}

bool ReportRank::count_word(Handle h)
{
	foreach_word_sense_of_inst(h, &ReportRank::count_sense, this);
	return false;
}

bool ReportRank::renorm_word(Handle h)
{
	foreach_word_sense_of_inst(h, &ReportRank::renorm_sense, this);
	return false;
}

bool ReportRank::count_sense(Handle word_sense_h,
                             Handle sense_link_h)
{
	Link *l = dynamic_cast<Link *>(TLB::getAtom(sense_link_h));
	normalization += l->getTruthValue().getMean();
	sense_count += 1.0;
	return false;
}

bool ReportRank::renorm_sense(Handle word_sense_h,
                              Handle sense_link_h)
{
	Link *l = dynamic_cast<Link *>(TLB::getAtom(sense_link_h));
	double score = l->getTruthValue().getMean();

	score *= normalization * sense_count;
	score -= 1.0;

	// Update the truth value, it will store deviation from average.
	// That is, initially, each word sense of each word instance is
	// assigned a (denormalized) probability of 1.0. Solving the
	// Markov chain/page rank causes the some of this to flow away
	// from less likely to more likely senses. The scoring is 
	// relative to this initial value: thus, unlikely scores will
	// go negative, likely scores will go positive.  "Typical"
	// distributions seem to go from -0.8 to +3.5 or there-abouts.
	//
	SimpleTruthValue stv((float) score, 1.0f);
	stv.setConfidence(l->getTruthValue().getConfidence());
	l->setTruthValue(stv);

Node *n = dynamic_cast<Node *>(TLB::getAtom(word_sense_h));
printf ("duu word sense=%s score=%f\n", n->getName().c_str(), score);
	return false;
}

/* ============================== END OF FILE ====================== */
