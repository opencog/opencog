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
#include <opencog/atoms/base/Link.h>
#include <opencog/truthvalue/TruthValue.h>

namespace opencog {

class SenseCache
{
	private:
		AtomSpace *atom_space;
		Handle match_sense;
		Handle found_link;
		bool find_sense(const Handle&, const Handle&);

	public:
		SenseCache(void);
		~SenseCache();

		void set_atom_space(AtomSpace *);
		TruthValuePtr similarity(const Handle&, const Handle&);
		void set_similarity(const Handle&, const Handle&, TruthValuePtr);
};

} // namespace opencog

#endif // _OPENCOG_SENSE_CACHE_H
