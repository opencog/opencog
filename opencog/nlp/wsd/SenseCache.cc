/*
 * SenseCache.cc
 *
 * Implements a cache for similarity measures between pairs of word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include "SenseCache.h"

#include <stdio.h>
#include <math.h>

#include <opencog/util/platform.h>
#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>

using namespace opencog;

SenseCache::SenseCache(void)
{
	atom_space = NULL;
}

SenseCache::~SenseCache()
{
	atom_space = NULL;
}

void SenseCache::set_atom_space(AtomSpace *as)
{
	atom_space = as;
}

/**
 * similarity -- return the similarity measure between two word senses.
 *
 * Return the cached word-sense similarity measure between two word
 * senses. If no cached value found, return NULL_TV.
 *
 * Sense similarity is stored via similarity links:
 *
 *    SimilarityLink strength=0.8 confidence=0.9
 *       WordSenseNode "bark_sense_23"
 *       WordSenseNode "covering_sense_42"
 */
TruthValuePtr SenseCache::similarity(Handle sense_a, Handle sense_b)
{
	match_sense = sense_b;

	bool found = foreach_unordered_binary_link(sense_a, SIMILARITY_LINK,
	                         &SenseCache::find_sense, this);

	if (!found) return TruthValue::DEFAULT_TV();
	return found_link->getTruthValue();
}

bool SenseCache::find_sense(Handle sense, Handle link)
{
	if (sense != match_sense) return false;
	found_link = link;
	return true;
}

/**
 * set_similarity -- create link, holding a similarity measure
 *
 * Create a new link, holding the similarity measure between two word
 * senses.  This link is *not* a part of any atom space; it must be
 * aded to an atom space if desired.
 */
void SenseCache::set_similarity(Handle sense_a, Handle sense_b, TruthValuePtr tv)
{
	// Create a link connecting the two senses.
	atom_space->addLink(SIMILARITY_LINK, sense_a, sense_b, tv);
}

/* ============================== END OF FILE ====================== */
