/*
 * SenseRank.cc
 *
 * Implements the PageRank graph centrality algorithm for word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "SenseRank.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/nlp/wsd/ForeachWord.h>
#include <opencog/server/CogServer.h>
#include <opencog/util/Logger.h>

using namespace opencog;

// #define DEBUG

SenseRank::SenseRank(void)
{
	// The page-rank damping factor. Normally taken to be quite large.
	damping_factor = 0.90;

	// The convergence damping factor, used to determine when the page
	// rank has converged. This is used to create an exponentially 
	// decaying average of the last N page-rank adjustments, where 
	// N = 1/convergence_damper.  Basically, N should be choosen so
	// that N == total number of word-senses in graph. For now, this
	// is assumed to be 20 (i.e. a single-sentence-worth of senses.)
	// For multi-sentence use, this should probably be pumped up.
	double N = 10;
	convergence_damper = 1.0/N;

	// The absolute value to which convergence is desired. Should be
	// smaller than the damper, since even large swings in page rank
	// are damped by the damper. 
	convergence_limit = 0.3 * convergence_damper;
}

SenseRank::~SenseRank()
{
}

/**
 * For each parse of the sentence, perform the ranking algo.
 * This routine returns after the graph, as a whole, has converged
 * to a stationary state.
 */
void SenseRank::rank_sentence(Handle h)
{
#ifdef DEBUG
	printf ("; SenseRank rank sentence %x\n", h); 
#endif
	foreach_parse(h, &SenseRank::rank_parse_f, this);
}

void SenseRank::rank_document(const std::deque<Handle> &parse_list)
{
	// Iterate over list of parses making up a "document"
	std::deque<Handle>::const_iterator i;
	for (i = parse_list.begin(); i != parse_list.end(); ++i)
	{
		init_parse(*i);
	}
	for (i = parse_list.begin(); i != parse_list.end(); ++i)
	{
#ifdef DEBUG
	printf ("; SenseRank document sentence parse %x\n", (Handle) *i); 
#endif
		rank_parse(*i);
	}
}

/**
 * Assign equal probability to all senses of all words.
 * Initially, all senses are equi-probabile.
 */
bool SenseRank::init_word(Handle h)
{
	foreach_word_sense_of_inst(h, &SenseRank::init_senses, this);
	return false;
}

bool SenseRank::init_senses(Handle word_sense_h,
                            Handle sense_link_h)
{
	TruthValuePtr ctv(CountTruthValue::createTV(1.0f, 0.0f, 1.0f));
	sense_link_h->setTruthValue(ctv);
	return false;
}

/**
 * Assign initial probabilities to each word sense.
 */ 
void SenseRank::init_parse(Handle h)
{
	converge = 1.0;
	foreach_word_instance(h, &SenseRank::init_word, this);
}

/**
 * For each parse, find some place to start. The graph should not have
 * any disconnected components (this has been taken care of by the 
 * Sweep code).  There might still be some weakly-connected components
 * that fail to mix well. So, just to make sure things are fair, we will
 * iterate for a while, starting at every sense of every word in the parse.
 */
void SenseRank::rank_parse(Handle h)
{
#ifdef DEBUG
	printf ("; SenseRank rank parse %x\n", h); 
#endif
	foreach_word_instance(h, &SenseRank::start_word, this);
}

bool SenseRank::rank_parse_f(Handle h)
{
	rank_parse(h);
	return false;
}

/**
 * For every word sense, try walking the graph from there. 
 */
bool SenseRank::start_word(Handle h)
{
#ifdef DEBUG
	printf ("; SenseRank: start at word %s\n", h->getName().c_str());
#endif
	foreach_word_sense_of_inst(h, &SenseRank::start_sense, this);
	return false;
}

void SenseRank::log_bad_sense(Handle word_sense_h, const std::string& msg,
        bool is_error)
{
    const char *s = "";
    NodePtr n(NodeCast(word_sense_h));
    if (n) s = n->getName().c_str();
    if (is_error) logger().error("SenseRank: %s: %s", msg.c_str(), s);
    else logger().info("SenseRank: %s: %s", msg.c_str(), s);
}


/**
 * Walk randomly over a connected component, applying the 
 * page-rank algo as we walk. Stop iterating when all values
 * are sufficiently converged.
 */
bool SenseRank::start_sense(Handle word_sense_h,
                            Handle sense_link_h)
{
#ifdef DEBUG
	printf ("; SenseRank start at %s\n", NodeCast(word_sense_h)->getName().c_str());
#endif

	if (Handle::UNDEFINED == sense_link_h) {
        // This can't/shouldn't happen -- 
        log_bad_sense(word_sense_h,
                std::string("bad starting sense for word"), true);
        return false;
	}

	// Make sure that this sense is connected to others by sense-pair
	// edges.  Under rare circumstances, it might not be -- for example,
	// the result of thining may have left the sense disconnected.  This
	// sense would not have been culled by the sweep routine, since it
	// was connected before the thining.  Bascially, if we find one of
	// these, we ignore it.
	next_sense = Handle::UNDEFINED;
	foreach_sense_edge(sense_link_h, &SenseRank::inner_sum, this);
	if (Handle::UNDEFINED == next_sense) {
		// This can't/shouldn't happen -- 
        log_bad_sense(word_sense_h,
                std::string("disconnected sense"), false);
		return false;
	}

	// Walk randomly over the connected component 
	do
	{
		rank_sense(sense_link_h);
		sense_link_h = pick_random_edge(sense_link_h);
	} while (convergence_limit < converge);

	return false;
}

/**
 * Compute the page rank for the indicated (word-inst,word-sense) pair.
 * The handle argument points at a (word-inst,word-sense) pair.
 * The page rank is defined as
 *
 * P(a) = (1-d) + d* sum_b (w_ab / (sum_c w_cb)) P(b)
 *
 * where a,b,c are (word-inst-word-sense) pairs,
 * P(a) is the rank of a,
 * w_ab is the weight of edges joining b to a
 * sum_b is a sum over all possible values of b.
 *
 * Note that if we write
 *   t_ab = w_ab / (sum_c w_cb)
 * then we have the identity that
 *   1 = sum_a t_ab
 * so we can interpret t_ab as an entry in the transition matrix of
 * a Markov chain: that is, passing the probability through the Markov
 * chain is (should be) conserved. Any given P(b) will be spread
 * uniformly across the possible P(a)'s. 
 *
 * The value of P(a) can be interpreted as a probability, if it is
 * normalized -- it would be the Markov chain stationary vector
 * (eigenvector). Note that P(a) ranges or *all* senses of all words:
 * the total of senses of any individual word is *not* normalized.
 *
 * The quantity (sum_c w_cb) is called "edge_sum" and is computed 
 * by inner_sum() 
 *
 * The quantity t_ab is computed inside of outer_sum(), where all values
 * of b are summed over.
 */
void SenseRank::rank_sense(Handle sense_link_h)
{
	rank_sum = 0.0;
	foreach_sense_edge(sense_link_h, &SenseRank::outer_sum, this);
	rank_sum *= damping_factor;
	rank_sum += 1.0-damping_factor;

	double old_rank = sense_link_h->getTruthValue()->getCount();

#ifdef DEBUG
	std::vector<Handle> oset = LinkCast(sense_link_h)->getOutgoingSet();
	printf ("; SenseRank: sense %s was %g new %g delta=%g\n",
	        NodePtr(oset[1])->getName().c_str(),
           old_rank, rank_sum, fabs(rank_sum - old_rank));
#endif

	// Compute convergence criterion to determine when the 
	// random walk has settled down/converged.
	converge *= (1.0-convergence_damper);
	converge += convergence_damper * fabs(rank_sum - old_rank);

	// Update the count for this sense.
	TruthValuePtr ctv(CountTruthValue::createTV(1.0, 0.0, (float) rank_sum));
	sense_link_h->setTruthValue(ctv);
}

/**
 * Perform the outermost sum of the page-rank algorithm.
 */
bool SenseRank::outer_sum(Handle sense_b_h, Handle hedge)
{
	// Get the weight of the edge
	double weight_ab = hedge->getTruthValue()->getMean();

	// Normalize the weight_ab summing over all c's weight_cb
	// The sum over 'c' runs over all edges pointing to link "b"
	edge_sum = 0.0;
	foreach_sense_edge(sense_b_h, &SenseRank::inner_sum, this);
	double t_ab = weight_ab / edge_sum; 

	// Get the word-sense probability
	double p_b = sense_b_h->getTruthValue()->getCount();
	double weight = t_ab * p_b;

	rank_sum += weight;
	// printf("outer sum h=%ld w=%g sum=%g\n", h, weight, rank_sum);
	return false;
}

/**
 * Perform the inner, normalization sum of the page-rank algorithm.
 * This sum simply computes the normalization that will be used to
 * adjust an edge weight. 
 */
bool SenseRank::inner_sum(Handle sense_c_h, Handle hedge_bc)
{
	next_sense = sense_c_h;
	double weight_to_b = hedge_bc->getTruthValue()->getMean();
	edge_sum += weight_to_b;
	// printf("inner sum h=%ld, %g %g\n", h, weight_to_b, edge_sum);
	return false;
}

/**
 * Look at each edge in turn, until the sum of edge weights
 * exceeds a random number.
 */
bool SenseRank::random_sum(Handle h, Handle hedge)
{
	next_sense = h;

	double weight_to_b = hedge->getTruthValue()->getMean();
	edge_sum += weight_to_b;
	if (randy < edge_sum) {
		return true; // we are done, we found our edge.
	}
	return false;
}

/**
 * Pick a random edge from the set of edges.
 */
Handle SenseRank::pick_random_edge(Handle h)
{
	// Get the total weight of the edges
	edge_sum = 0.0;
	foreach_sense_edge(h, &SenseRank::inner_sum, this);

	// get a random number between zero and one.
	randy = ((double) rand()) / ((double) RAND_MAX);

	// randy needs to be exceeded for an edge to be choosen.
	next_sense = Handle::UNDEFINED;
	randy *= edge_sum;
	edge_sum = 0.0;
	foreach_sense_edge(h, &SenseRank::random_sum, this);

	// The handle will still be undefined if h is a word-sense
	// without any connected sense-pair-similarity edges. We
	// should not ever see any of these ...
	if (Handle::UNDEFINED == next_sense)
	{
		// This can't/shouldn't happen -- it would mean that we have
		// arrived at a sense node that has no connected edges.
		logger().error("SenseRank: failed to find a random edge!");
		next_sense = h;
	}
	return next_sense;
}

/* ============================== END OF FILE ====================== */
