/*
 * SenseCache.h
 *
 * Implements word-sense similarity cache. Computing the word-sense
 * similarity can take a significant amount of cpu time; caching the
 * results makes sense.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_SENSE_CACHE_H
#define _OPENCOG_SENSE_CACHE_H

#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/TruthValue.h>

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

} // namespace opencog

#endif // _OPENCOG_SENSE_CACHE_H
