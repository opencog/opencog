/*
 * SenseSimilarityLCH.cc
 *
 * Implements various wordnet-based sense-similarity measures.
 * Currently implements only Leacock-Chodorow, however, Wu-Palmer
 * should be (very) easy to implement in this framework.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "SenseSimilarity.h"

#include <stdio.h>
#include <math.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/wsd/ForeachWord.h>
#include <opencog/nlp/wsd/SenseSimilarityLCH.h>

using namespace opencog;

#define DEBUG

SenseSimilarityLCH::SenseSimilarityLCH()
{
	// Set 'max_follow_holo' to a small number to limit the total number
	// of holonym relations to be followed. Setting this to a large number
	// leads to an exponential explosion of searches performed. Currently,
	// the best value appears to be '1'.  Numbers larger than 4 can lead
	// to hours of cpu time per sentence, since cpu time usage is
	// proportional to 2^(n-factorial). Wow.
	max_follow_holo = 1;
}

SenseSimilarityLCH::~SenseSimilarityLCH()
{
}

/**
 * Compute the Leacock-Chodorow word-sense similarity measure.
 *
 * This similarity measure is given by
 *    sim = -log(len/(2*depth))
 * where len is the distance, in nodes, between the two word-senses,
 * and depth is the total depth (top-to-bottom) of the
 * least-common-subsumer tree containing these two senses.
 *
 * In wordnet 3.0, the total depth (including the root node) of the
 * verb taxonomy is 14, and the total depth of the noun taxonomy is 20.
 * There is no adjective/adverb taxonomy.
 *
 * In wordnet, there are several ways to determine distance:
 * 1) upwards, via hypernyms
 * 2) upwards, via holonym links
 *
 * The current implementation only explores the hypernym tree. This
 * seems to be the intended defintion for Leacock-Chodorow similarity.
 *
 * It is assumed that hypernyms are describe via inheritance links:
 *
 *    InheritanceLink strength=0.8 confidence=0.9
 *       WordSenseNode "bark_sense_23"
 *       WordSenseNode "covering_sense_42"
 *
 * Note that, in this algorithm, two words having different parts-of-speech
 * will have zero similarity (infinite distance).
 *
 * Note that currently, only the "mean" is used to nindicate similarity;
 * whereas teh confidence is set to the arbitrary 0.9
 */
SimpleTruthValuePtr SenseSimilarityLCH::similarity(Handle fs, Handle ss)
{
	first_sense = fs;
	second_sense = ss;

	// If the parts-of-speech don't match, the similarity is zero.
	// If either one is an adjective or adverb, they're unrelated.
	// (Although we are not very confident of that!)
	std::string first_pos = get_part_of_speech(first_sense);
	std::string second_pos = get_part_of_speech(second_sense);
	if ((0 != first_pos.compare(second_pos)) ||
	    (0 == first_pos.compare("adj")) ||
	    (0 == first_pos.compare("adv")))
	{
		return SimpleTruthValue::createSTV(0.0, 0.5);
	}

	// As of wordnet-3.0, the depth of verb taxonomy is 14, noun is 20.
	double depth = 14;
	double norm = -3.3323;
	if (0 == first_pos.compare("noun"))
	{
		depth = 20;
		norm = -3.6889;
	}

	first_cnt = 0;
	min_cnt = 1<<28;

	// Look up the hypernym tree, to see if the seccond sense is
	// immediately above the first sense in either the hypernym (is-a)
	// or the holonym (has-a) hierarchy.
	follow_holo_cnt = 0;
	join_candidate = first_sense;
	second_cnt = 0;
	foreach_binary_link(second_sense, INHERITANCE_LINK,
	                         &SenseSimilarityLCH::up_second, this);
	follow_holo_cnt = 1;
	foreach_binary_link(second_sense, HOLONYM_LINK,
	                         &SenseSimilarityLCH::up_second, this);
	follow_holo_cnt = 0;

	// Now look to see if there is an alternate, shorter, path,
	// where the two senses have a common hypernym or holonym.
	// (This also catches the case where the first sense is
	// immediately above the second sense).
	foreach_binary_link(first_sense, INHERITANCE_LINK,
	                         &SenseSimilarityLCH::up_first, this);
	follow_holo_cnt = 1;
	foreach_binary_link(first_sense, HOLONYM_LINK,
	                         &SenseSimilarityLCH::up_first, this);
	follow_holo_cnt = 0;

	// At this point, min_cnt will contain the shortest distance between
	// the two word senses. Divide by depth, compute the measure.
	double sim = ((double) min_cnt+1) / (2.0*depth);
	sim = log(sim) / norm;
	if (sim < 0.0) sim = 0.0;

#ifdef DEBUG
	printf("(%s, %s) dist=%d sim=%g\n",
	       NodeCast(first_sense)->getName().c_str(),
	       NodeCast(second_sense)->getName().c_str(),
	       min_cnt, sim);
	// printf("----\n");
#endif

	return SimpleTruthValue::createSTV((float) sim, 0.9f);
}

bool SenseSimilarityLCH::up_first(Handle up)
{
	if (up->getType() != WORD_SENSE_NODE) return false;

	first_cnt ++;
	if (up == second_sense)
	{
		if (first_cnt < min_cnt) min_cnt = first_cnt;
		first_cnt --;
		return false;
	}

	// Don't explore paths that are longer than the current shortest path.
	if (first_cnt + 1 >= min_cnt) return false;

	// printf ("first height=%d up=%s\n", first_cnt, n->getName().c_str());

	// Look to see if the join candidate (candidate for the "least common
	// subsumer") appears anywhere on the up chain of the second sense.
	// Explore both the hypernym, and the holonym heirarchies.
	join_candidate = up;
	second_cnt = 0;
	foreach_binary_link(second_sense, INHERITANCE_LINK,
	                         &SenseSimilarityLCH::up_second, this);
	if (follow_holo_cnt < max_follow_holo)
	{
		follow_holo_cnt ++;
		foreach_binary_link(second_sense, HOLONYM_LINK,
		                         &SenseSimilarityLCH::up_second, this);
		follow_holo_cnt --;
	}

	// Go up, see if there are shorter paths
	foreach_binary_link(up, INHERITANCE_LINK,
	                         &SenseSimilarityLCH::up_first, this);
	if (follow_holo_cnt < max_follow_holo)
	{
		follow_holo_cnt ++;
		foreach_binary_link(up, HOLONYM_LINK,
		                         &SenseSimilarityLCH::up_first, this);
		follow_holo_cnt --;
	}
	first_cnt --;

	return false;
}

bool SenseSimilarityLCH::up_second(Handle up)
{
	if (up->getType() != WORD_SENSE_NODE) return false;

	// Don't explore paths that are longer than the current shortest path.
	int dist = first_cnt + second_cnt + 1;
	if (dist >= min_cnt) return false;

	second_cnt ++;
	// printf ("second height=%d up=%s\n", second_cnt, n->getName().c_str());

	// If we found the match to the candidate, compute the distance,
	// and save it. The distance is number of steps from each sense,
	// to their common (indirect) hypernym, i.e. to their "least common
	// subsumer".
	if (up == join_candidate)
	{
#ifdef XDEBUG
		printf("found dist=%d (%d+%d) min=%d\n", dist, first_cnt, second_cnt, min_cnt);
#endif
		if (dist < min_cnt) min_cnt = dist;
		second_cnt --;
		return false;
	}

	// Else, if no match, search upwards in the hypernym tree.
	foreach_binary_link(up, INHERITANCE_LINK,
	                         &SenseSimilarityLCH::up_second, this);

	// Give the holonym heirarchy a spin, too.
	if (follow_holo_cnt < max_follow_holo)
	{
		follow_holo_cnt ++;
		foreach_binary_link(up, HOLONYM_LINK,
		                       &SenseSimilarityLCH::up_second, this);
		follow_holo_cnt --;
	}

	second_cnt --;
	return false;
}

/* ============================== END OF FILE ====================== */
