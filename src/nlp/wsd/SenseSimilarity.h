/*
 * SenseSimilarity.h
 *
 * Implements word-sense similarity measures, starting with
 * the Leacock and Chodorow algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_MIHALCEA_LABEL_H
#define OPENCOG_MIHALCEA_LABEL_H

#include <string>

#include "SimpleTruthValue.h"

namespace opencog {

class SenseSimilarity
{
	private:
		Handle first_sense;
		Handle second_sense;
		int first_cnt;
		bool up_first(Handle);

	public:
		SenseSimilarity(void);
		~SenseSimilarity();

		SimpleTruthValue lch_similarity(Handle, Handle);
};
}

#endif /* OPENCOG_MIHALCEA_LABEL_H */
