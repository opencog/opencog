/*
 * SenseRank.cc
 *
 * Implements the PageRank graph centrality algorithm for word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>
#include <math.h>

#include "FollowLink.h"
#include "ForeachWord.h"
#include "SenseRank.h"
#include "Node.h"
#include "SimpleTruthValue.h"

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
 * Follow sense edges. 
 * It is assumed that the incoming handle is a (inst,sense) pair. The
 * incoming set of this pair should be all of the sense-edges.
 */
template <typename T>
class PrivateUseOnlySenseEdge
{
	public:
		Handle near_end;
		bool (T::*user_cb)(Handle, Handle);
		T *user_data;
		bool walk_edge(Handle h)
		{
			// Handle h should be a sense edge.  Verify this, just
			// in case, and reject those that aren't.
			Link *l = dynamic_cast<Link *> (TLB::getAtom(h));
			if ((l == NULL) || (l->getType() != COSENSE_LINK)) return false;

			// The link is assumed binary. Find the far end of the link.

			// return (user_data->*user_cb)(xxx);
			return false;
		}
};

template <typename T>
inline bool
foreach_sense_edge(Handle h,
                   bool (T::*cb)(Handle, Handle), T *data)
{

	PrivateUseOnlySenseEdge<T> se;
	se.user_cb = cb;
	se.user_data = data;
	se.near_end = h;
	return foreach_incoming_handle(h, &PrivateUseOnlySenseEdge<T>::walk_edge, &se);
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
	printf("Hello ranke sense world\n");
	
	foreach_sense_edge(h, &SenseRank::outer_sum, this);
}

bool SenseRank::outer_sum(Handle h, Handle l)
{
	printf("outer sum\n");
	return false;
}

bool SenseRank::inner_sum(Handle h)
{
	printf("outer sum\n");
	return false;
}

/**
 *
 * Walk randomly over a connected component. 
 */
void SenseRank::rand_walk(Handle h)
{
	printf("Hello world\n");
	rank_sense(h);
}

/* ============================== END OF FILE ====================== */
