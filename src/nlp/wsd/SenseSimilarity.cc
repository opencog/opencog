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
 */
SimpleTruthValue SenseSimilarity::lch_similarity(Handle fs, Handle ss)
{
	first_sense = fs;
	second_sense = ss;
	first_cnt = 0;

	ForeachChaseLink<SenseSimilarity> chase;
	// chase.follow_binary_link(first_sense, INHERITANCE_LINK, 
chase.follow_binary_link(ss, INHERITANCE_LINK, 
	                         &SenseSimilarity::up_first, this);

	SimpleTruthValue stv(0.5,1.0);
	return stv;
}

bool SenseSimilarity::up_first(Handle up)
{
	Node *n = dynamic_cast<Node *>(TLB::getAtom(up));
	if (n == NULL || n->getType() != WORD_SENSE_NODE) return false;

	first_cnt ++;
printf ("duuude %d up=%s\n", first_cnt, n->getName().c_str());

	ForeachChaseLink<SenseSimilarity> chase;
	chase.follow_binary_link(up, INHERITANCE_LINK, 
	                         &SenseSimilarity::up_first, this);
	first_cnt --;
	return false;
}

/* ============================== END OF FILE ====================== */
