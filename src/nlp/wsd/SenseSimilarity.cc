/*
 * SenseSimilarity.cc
 *
 * Implements various wordnet-based sense-similarity measures.
 * Currently implements only Leacock-Chodorow, however, Wu-Palmer
 * should be (very) easy to implement in this framework.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>
#include <math.h>

#include "FollowLink.h"
#include "ForeachWord.h"
#include "SenseSimilarity.h"
#include "Node.h"
#include "SimpleTruthValue.h"

using namespace opencog;

SenseSimilarity::SenseSimilarity(void)
{
}

SenseSimilarity::~SenseSimilarity()
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
 *    <InheritanceLink strength=0.8 confidence=0.9 />
 *       <WordSenseNode name="bark_sense_23" />
 *       <WordSenseNode name="covering_sense_42" />
 *    </InheritanceLink>
 *
 * Note that, in this algorithm, two words having different parts-of-speech
 * will have zero similarity (infinite distance).
 */
SimpleTruthValue SenseSimilarity::lch_similarity(Handle fs, Handle ss)
{
	first_sense = fs;
	second_sense = ss;

	// If the parts-of-speech don't match, the similarity is zero.
	// If either one is an adjective or adverb, they're unrelated.
	// (Although we are not very confident of that!)
	std::string first_pos = get_pos_of_word_instance(first_sense);
	std::string second_pos = get_pos_of_word_instance(second_sense);
	if ((0 != first_pos.compare(second_pos)) ||
	    (0 == first_pos.compare("#adj")) ||
	    (0 == first_pos.compare("#adv")))
	{
		SimpleTruthValue stv(0.0, 0.5);
		return stv;
	}

	// As of wordnet-3.0, the depth of verb taxonomy is 14, noun is 20.
	double depth = 14;
	double norm = -3.3323;
	if (0 == first_pos.compare("#noun"))
	{
		depth = 20;
		norm = -3.6889;
	}

	first_cnt = 0;
	min_cnt = 1<<28;

	// Look up the hypernym tree, to see where the two senses have
	// a common hypernym.
	ForeachChaseLink<SenseSimilarity> chase;
	chase.follow_binary_link(first_sense, INHERITANCE_LINK,
	                         &SenseSimilarity::up_first, this);

	// At this point, min_cnt will contain the shortest distance between
	// the two word senses. Divide by depth, compute the measure.
	double sim = ((double) min_cnt+1) / (2.0*depth);
	sim = log(sim) / norm;
	if (sim < 0.0) sim = 0.0;

#define DEBUG
#ifdef DEBUG
	Node *sense = dynamic_cast<Node *>(TLB::getAtom(first_sense));
	printf("first sense %s\n", sense->getName().c_str());
	sense = dynamic_cast<Node *>(TLB::getAtom(second_sense));
	printf("second sense %s\n", sense->getName().c_str());
	printf("distance between senses = %d sim=%g\n", min_cnt, sim);
#endif

	SimpleTruthValue stv(sim,0.9);
	return stv;
}

bool SenseSimilarity::up_first(Handle up)
{
	Node *n = dynamic_cast<Node *>(TLB::getAtom(up));
	if (n == NULL || n->getType() != WORD_SENSE_NODE) return false;

	first_cnt ++;
	// printf ("height=%d up=%s\n", first_cnt, n->getName().c_str());

	// Look to see if the join candidate appears anywhere on the up chain
	// of the second sense.
	join_candidate = up;
	ForeachChaseLink<SenseSimilarity> chase;

	second_cnt = 0;
	chase.follow_binary_link(second_sense, INHERITANCE_LINK,
	                         &SenseSimilarity::up_second, this);

	// Go up, see if there are shorter paths
	chase.follow_binary_link(up, INHERITANCE_LINK,
	                         &SenseSimilarity::up_first, this);
	first_cnt --;

	return false;
}

bool SenseSimilarity::up_second(Handle up)
{
	Node *n = dynamic_cast<Node *>(TLB::getAtom(up));
	if (n == NULL || n->getType() != WORD_SENSE_NODE) return false;

	second_cnt ++;

	// If we found the match to the candidate, compute the distance,
	// and save it. The distance is number of steps from each sense,
	// to thier common (indirect) hypernym.
	if (up == join_candidate)
	{
		int dist = first_cnt + second_cnt;
		// printf("found dist=%d (%d+%d) min=%d\n", dist, first_cnt, second_cnt, min_cnt);
		if (dist < min_cnt) min_cnt = dist;
	}
	else
	{
		// Else, if no match, search upwards.
		ForeachChaseLink<SenseSimilarity> chase;
		chase.follow_binary_link(up, INHERITANCE_LINK,
		                         &SenseSimilarity::up_second, this);
	}

	second_cnt --;
	return false;
}

/* ============================== END OF FILE ====================== */
