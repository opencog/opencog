/*
 * SenseSimilarity.h
 *
 * Implements word-sense similarity measures, starting with
 * the Leacock and Chodorow algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SENSE_SIMILARITY_H
#define OPENCOG_SENSE_SIMILARITY_H

#include "SimpleTruthValue.h"

namespace opencog {

class SenseSimilarity
{
	private:
		Handle first_sense;
		Handle second_sense;
		int first_cnt;
		int second_cnt;
		int min_cnt;

		int follow_holo_cnt;
		int max_follow_holo;
		Handle join_candidate; // aka least common subsumer
		bool up_first(Handle);
		bool up_second(Handle);


	public:
		SenseSimilarity(void);
		~SenseSimilarity();

		SimpleTruthValue lch_similarity(Handle, Handle);
};
}

#endif /* OPENCOG_SENSE_SIMILARITY_H */
