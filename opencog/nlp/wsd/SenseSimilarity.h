/*
 * SenseSimilarity.h
 *
 * Interface class for similarity measures.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_SENSE_SIMILARITY_H
#define _OPENCOG_SENSE_SIMILARITY_H

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/SimpleTruthValue.h>

namespace opencog {

class SenseSimilarity
{
	public:
		SenseSimilarity(void) {};
		virtual ~SenseSimilarity() {};

		virtual SimpleTruthValuePtr similarity(const Handle&, const Handle&) = 0;
};

} // namespace opencog

#endif // _OPENCOG_SENSE_SIMILARITY_H
