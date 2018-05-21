/*
 * ReportRank.cc
 *
 * Report most likely word-senses for each word.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "ReportRank.h"

#include <stdio.h>

#include <opencog/atoms/proto/types.h>
#include <opencog/atoms/base/Node.h>
#include <opencog/truthvalue/CountTruthValue.h>
#include <opencog/truthvalue/TruthValue.h>
#include <opencog/nlp/wsd/ForeachWord.h>
#include <opencog/util/platform.h>

using namespace opencog;

#define DEBUG 1

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
void ReportRank::report_sentence(const Handle& h)
{
	parse_cnt = 0;
	foreach_parse(h, &ReportRank::report_parse_f, this);
}


/**
 * Renormalize the distribution of senses for this document, store result
 * in the truth values.
 *
 * After running the graph algo, scores will have flowed to different
 * senses, with many senses getting a strong boost, others loosing out.
 * This code renormalizes the scores so that they are expressed as a 
 * deviation from a mean.  This is done in two (conceptual) steps:
 *   1) renormalize so that the mean score is 1.0
 *   2) subtract 1.0 from all scores.
 * The result of doing this is that unlikely senses get netgative
 * scores, likely senses get strong positive scores.  It appears that 
 * "typical" distributions seem to go from -0.8 to +3.5 or there-abouts.
 */
void ReportRank::report_document(const std::deque<Handle> &parse_list)
{
	normalization = 0.0;
	sense_count = 0.0;
	choosen_sense_count = 0.0;
	word_count = 0;

	// Iterate over all the parses in the document.
	std::deque<Handle>::const_iterator i;
	for (i = parse_list.begin(); i != parse_list.end(); ++i)
	{
		Handle h = *i;
		foreach_word_instance(h, &ReportRank::count_word, this);
	}

#ifdef DEBUG
	printf("; report_document: norm=%g senses=%g words=%d\n",
		normalization, sense_count, word_count);
#endif

	// Compute the average score per sense. Then renormalize 
	// according to deviations from this average.
	double average = normalization / sense_count;
	normalization = 1.0 / average;

	for (i = parse_list.begin(); i != parse_list.end(); ++i)
	{
		Handle h = *i;
		foreach_word_instance(h, &ReportRank::renorm_word, this);
	}
#ifdef DEBUG
	printf("; report_document: chose=%g senses out of %g (%g percent)\n",
		choosen_sense_count, sense_count, 100.0*choosen_sense_count/sense_count);
	fflush(stdout);
#endif
}

/**
 * Same as report_document, but done only for one parse
 */
void ReportRank::report_parse(const Handle& h)
{
#ifdef DEBUG
	printf ("; ReportRank: Sentence %d:\n", parse_cnt);
#endif
	parse_cnt ++;

	normalization = 0.0;
	sense_count = 0.0;
	choosen_sense_count = 0.0;
	foreach_word_instance(h, &ReportRank::count_word, this);

	// Compute the average score per sense. Then renormalize 
	// according to deviations from this average.
	double average = normalization / sense_count;
	normalization = 1.0 / average;
	foreach_word_instance(h, &ReportRank::renorm_word, this);
}

bool ReportRank::report_parse_f(const Handle& h)
{
	report_parse(h);
	return false;
}

bool ReportRank::count_word(const Handle& h)
{
	word_count ++;
	foreach_word_sense_of_inst(h, &ReportRank::count_sense, this);
	return false;
}

bool ReportRank::renorm_word(const Handle& h)
{
#ifdef HISCORE_DEBUG
	hi_score = -1e10;
	hi_sense = "(none)";
#endif
	foreach_word_sense_of_inst(h, &ReportRank::renorm_sense, this);

#ifdef HISCORE_DEBUG
	Handle wh = get_dict_word_of_word_instance(h);
	const char *wd = wh->get_name().c_str();
	printf("; hi score=%g word = %s sense=%s\n", hi_score, wd, hi_sense);
	fflush (stdout);
#endif
	return false;
}

bool ReportRank::count_sense(const Handle& word_sense_h,
                             const Handle& sense_link_h)
{
	normalization += sense_link_h->getTruthValue()->get_count();
	sense_count += 1.0;
	return false;
}

bool ReportRank::renorm_sense(const Handle& word_sense_h,
                              const Handle& sense_link_h)
{
	double score = sense_link_h->getTruthValue()->get_count();

	score *= normalization;
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
	TruthValuePtr stv(CountTruthValue::createTV(1.0f, 0.0f, (float) score));
	sense_link_h->setTruthValue(stv);

#ifdef DEBUG
	if (hi_score < score) {
		hi_sense = word_sense_h->get_name().c_str();
		hi_score = score;
	}
	if (0.0 < score) {
		choosen_sense_count += 1.0;
	
#if 0
printf ("duu word sense=%s score=%f\n", word_sense_h->get_name().c_str(), score);
fflush (stdout);
#endif
	}
#endif

	return false;
}

/* ============================== END OF FILE ====================== */
