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

#define DEBUG

SenseRank::SenseRank(void)
{
	// The page-rank damping factor. Normally taken to be quite large.
	damping_factor = 0.85;

	// The convergence damping factor, used to determine when the page
	// rank has converged. This is used to create an exponentially 
	// decaying average of the last N page-rank adjustments, where 
	// N = 1/convergence_damper.  Basically, N should be choosen so
	// that N == total number of word-senses in graph. For now, this
	// is assumed to be 20 (i.e. a single-sentence-worth of senses.)
	// For multi-sentence use, this should probably be pumped up.
	double N = 50;
	convergence_damper = 1.0/N;
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

Node *n = dynamic_cast<Node *>(TLB::getAtom(h));
printf ("duude start word =%s\n", n->getName().c_str());
printf("duude start pos=%s\n", pos.c_str());
	foreach_word_sense_of_inst(h, &SenseRank::start_sense, this);
	return false;
}

bool SenseRank::start_sense(Handle word_sense_h,
                            Handle sense_link_h)
{
Node *n = dynamic_cast<Node *>(TLB::getAtom(word_sense_h));
printf ("duude start sense =%s\n", n->getName().c_str());
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
	double old_rank = sense->getTruthValue().getMean();

#ifdef DEBUG
	std::vector<Handle> oset = sense->getOutgoingSet();
	Node *n = dynamic_cast<Node *>(TLB::getAtom(oset[1]));
	printf ("sense %s was %g nw %g\n", n->getName().c_str(), old_rank, rank_sum);
#endif

	// Compute convergence criterion to determine when the 
	// random walk has settled down/converged.
	converge *= (1.0-convergence_damper);
	converge += convergence_damper * fabs(rank_sum - old_rank);

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
	// printf("outer sum h=%ld w=%g sum=%g\n", h, weight, rank_sum);
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
	// printf("inner sum h=%ld, %g %g\n", h, weight_to_b, edge_sum);
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
int cnt = 0;
	converge = 1.0;
	while (0.01 < converge)
	{
printf("start walk %d conv=%g\n", cnt, converge);
cnt++;
		rank_sense(h);
		h = pick_random_edge(h);
	}
}

/* ============================== END OF FILE ====================== */
