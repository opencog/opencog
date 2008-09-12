/*
 * SenseSimilarity.h
 *
 * Interface class for similarity measures.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SENSE_SIMILARITY_H
#define OPENCOG_SENSE_SIMILARITY_H

#include "SimpleTruthValue.h"

namespace opencog {

class SenseSimilarity
{
	public:
		SenseSimilarity(void) {};
		virtual ~SenseSimilarity() {};

		virtual SimpleTruthValue similarity(Handle, Handle) = 0;
};
}

#endif /* OPENCOG_SENSE_SIMILARITY_H */
