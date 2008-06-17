/*
 * SenseCache.cc
 *
 * Implements a cache for similarity measures between pairs of word-senses.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include "platform.h"
#include <stdio.h>
#include <math.h>

#include "ForeachChaseLink.h"
#include "SenseCache.h"
#include "Node.h"
#include "SimpleTruthValue.h"

using namespace opencog;

SenseCache::SenseCache(void)
{
}

SenseCache::~SenseCache()
{
}

/**
 * similarity -- return the similarity measure between two word senses.
 *
 * Return the cached word-sense similarity measure between two word
 * senses. If no cached value found, return NULL_TV.
 *
 * Sense similarity is stored via similarity links:
 *
 *    <SimilarityLink strength=0.8 confidence=0.9 />
 *       <WordSenseNode name="bark_sense_23" />
 *       <WordSenseNode name="covering_sense_42" />
 *    </SimilarityLink>
 */
const TruthValue& SenseCache::similarity(Handle sense_a, Handle sense_b)
{
	match_sense = sense_b;

	bool found = foreach_unordered_binary_link(sense_a, SIMILARITY_LINK,
	                         &SenseCache::find_sense, this);

	if (!found) return TruthValue::DEFAULT_TV();

	Link *l = dynamic_cast<Link *>(TLB::getAtom(found_link));
	return l->getTruthValue();
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
Link * SenseCache::set_similarity(Handle sense_a, Handle sense_b, const TruthValue &tv)
{
	// Create a link connecting the two senses.
	std::vector<Handle> out;
	out.push_back(sense_a);
	out.push_back(sense_b);
	
	return new Link(SIMILARITY_LINK, out, tv);
}

/* ============================== END OF FILE ====================== */
