/*
 * SenseRank.cc
 *
 * Implements the PageRank graph centrality algorithm for word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "FollowLink.h"
#include "ForeachWord.h"
#include "SenseRank.h"
#include "Node.h"
#include "SimpleTruthValue.h"
#include "TruthValue.h"

using namespace opencog;

SenseRank::SenseRank(void)
{
	damping_factor = 0.85;
}

SenseRank::~SenseRank()
{
}

/**
 * For each parse of the sentence, perform the ranking algo.
 */
void SenseRank::iterate(Handle h)
{
	foreach_parse(h, &SenseRank::rank_parse, this);
}

/**
 * For each parse, find some place to start. Since (at this point),
 * noun sense ranking and verb sense ranking are disjoint, start 
 * off on every word. That is, as of right now, the graph consists
 * of multplie connected components.
 */
bool SenseRank::rank_parse(Handle h)
{
	foreach_word_instance(h, &SenseRank::start_word, this);
	return false;
}

/**
 * Pick some random word sense to start at.
 */
bool SenseRank::start_word(Handle h)
{
	// Only noun-senses and verb-senses get ranked.
	std::string pos = get_pos_of_word_instance(h);
	if (pos.compare("#noun") && pos.compare("#verb")) return false;

	foreach_word_sense_of_inst(h, &SenseRank::start_sense, this);
	return false;
}

bool SenseRank::start_sense(Handle word_sense_h,
                            Handle sense_link_h)
{
	rand_walk(sense_link_h);
	return true;
}

/**
 * Compute the page rank for the indicated (word-inst,word-sense) pair.
 * The handle argument points at a (word-inst,word-sense) pair.
 * The page rank is defined as
 *
 * P(a) = (1-d) + d* sum_b w_ba / (sum_c w_bc) P(b)
 */
void SenseRank::rank_sense(Handle h)
{
	rank_sum = 0.0;
	foreach_sense_edge(h, &SenseRank::outer_sum, this);
	rank_sum *= damping_factor;
	rank_sum += 1.0-damping_factor;

	Link *sense = dynamic_cast<Link *>(TLB::getAtom(h));
printf("Hello ranke sense was %g finally %g\n", sense->getTruthValue().getMean(), rank_sum);

	SimpleTruthValue stv(rank_sum, 1.0);
	stv.setConfidence(sense->getTruthValue().getConfidence());
	sense->setTruthValue(stv);
}

/**
 * Perform the outermost sum of the page-rank algorithm.
 */
bool SenseRank::outer_sum(Handle h, Handle hedge)
{
	// Get the weight of the edge
	Link *edge = dynamic_cast<Link *>(TLB::getAtom(hedge));
	double weight_ba = edge->getTruthValue().getMean();

	// Normalize the weight by the sum of competitors.
	edge_sum = 0.0;
	foreach_sense_edge(h, &SenseRank::inner_sum, this);
	double weight = weight_ba / edge_sum; 

	// Get the word-sense probability
	Link *bee = dynamic_cast<Link *>(TLB::getAtom(h));
	double p_b = bee->getTruthValue().getMean();
	weight *= p_b;

	rank_sum += weight;
	// printf("outer sum w=%g sum=%g\n", weight, rank_sum);
	return false;
}

int xxx = 0;
/**
 * Perform the inner, normalization sum of the page-rank algorithm.
 * This sum simply computes the normalization that will be used to
 * adjust an edge weight. 
 */
bool SenseRank::inner_sum(Handle h, Handle hedge)
{
	Link *edge = dynamic_cast<Link *>(TLB::getAtom(hedge));
	double weight_to_b = edge->getTruthValue().getMean();
	edge_sum += weight_to_b;
	// printf("inner sum %g %g\n", weight_to_b, edge_sum);
xxx ++;
	return false;
}

/**
 * Look at each edge in turn, until the sum of edge weights
 * exceeds a random number.
 */
bool SenseRank::random_sum(Handle h, Handle hedge)
{
xxx ++;
	next_sense = h;

	Link *edge = dynamic_cast<Link *>(TLB::getAtom(hedge));
	double weight_to_b = edge->getTruthValue().getMean();
	edge_sum += weight_to_b;
	if (randy < edge_sum)
	{
		return true; // we are done, we found our edge.
	}
	return false;
}

/**
 * Pick a random edge from the set of edges.
 */
Handle SenseRank::pick_random_edge(Handle h)
{
	// get a random number between zero and one.
	randy = ((double) rand()) / ((double) RAND_MAX);

	// Get the total weight of the edges
xxx = 0;
	edge_sum = 0.0;
	foreach_sense_edge(h, &SenseRank::inner_sum, this);
printf("tot edges=%d\n", xxx);

	// randy needs to be exceeeded for an edge to be choosen.
	randy *= edge_sum;
	edge_sum = 0.0;
xxx = 0;
	foreach_sense_edge(h, &SenseRank::random_sum, this);
printf("picked edge =%d\n", xxx);
	return next_sense;
}

/**
 * Walk randomly over a connected component. 
 */
void SenseRank::rand_walk(Handle h)
{
	for (int i=0; i<20; i++)
	{
		rank_sense(h);
		h = pick_random_edge(h);
	}
}

/* ============================== END OF FILE ====================== */
