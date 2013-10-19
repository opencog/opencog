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

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/TruthValue.h>

namespace opencog {

class SenseCache
{
	private:
		AtomSpace *atom_space;
		Handle match_sense;
		Handle found_link;
		bool find_sense(Handle, Handle);

	public:
		SenseCache(void);
		~SenseCache();

		void set_atom_space(AtomSpace *);
		TruthValuePtr similarity(Handle, Handle);
		void set_similarity(Handle, Handle, TruthValuePtr);
};

} // namespace opencog

#endif // _OPENCOG_SENSE_CACHE_H
