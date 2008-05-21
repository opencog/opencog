/*
 * SenseCache.h
 *
 * Implements word-sense similarity cache. Computing the word-sense
 * similarity can take a significant amount of cpu time; caching the
 * results makes sense.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SENSE_CACHE_H
#define OPENCOG_SENSE_CACHE_H

#include "Link.h"
#include "TruthValue.h"

namespace opencog {

class SenseCache
{
	private:
		Handle match_sense;
		Handle found_link;
		bool find_sense(Handle, Handle);

	public:
		SenseCache(void);
		~SenseCache();

		const TruthValue& similarity(Handle, Handle);
		Link * set_similarity(Handle, Handle, const TruthValue&);
};
}

#endif /* OPENCOG_SENSE_CACHE_H */
