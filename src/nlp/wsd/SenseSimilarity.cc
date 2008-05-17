/*
 * SenseSimilarity.cc
 *
 * Implements various wordnet-based sense-similarity measures.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

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
 * and depth is the total depth (top-to-bottom) of the tree containing
 * these two senses.
 *
 * In wordnet, there are several ways to determine distance:
 * 1) upwards, via hypernyms
 * 2) sideways, via similarity links, and then upwards,
 * 3) upwards, via holonym links
 *
 * The current implementation only explores the hypernym tree. It is
 * assumed that hypernyams are describe via ihneritance links:
 *
 *    <InheritanceLink strength=0.8 confidence=0.9 />
 *       <WordSenseNode name="bark_sense_23" />
 *       <WordSenseNode name="covering_sense_42" />
 *    </InheritanceLink>
 *
 * XXX If two words have different parts-of-speech, they willl not have
 * any common senses.
 * 
 */
SimpleTruthValue SenseSimilarity::lch_similarity(Handle fs, Handle ss)
{
	first_sense = fs;
	second_sense = ss;
	first_cnt = 0;
	min_cnt = 1<<28;

	// Look up the hypernym tree, to see where the two senses have 
	// a common hypernym.
	ForeachChaseLink<SenseSimilarity> chase;
	chase.follow_binary_link(first_sense, INHERITANCE_LINK, 
	                         &SenseSimilarity::up_first, this);
	// At this point, min_cnt will contain the shortest distance between 
	// the two word senses.
#define DEBUG
#ifdef DEBUG
	Node *sense = dynamic_cast<Node *>(TLB::getAtom(first_sense));
	printf("first sense %s\n", sense->getName().c_str());
	sense = dynamic_cast<Node *>(TLB::getAtom(second_sense));
	printf("second sense %s\n", sense->getName().c_str());
	printf("distance between senses = %d\n", min_cnt);
#endif

	SimpleTruthValue stv(0.5,1.0);
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
