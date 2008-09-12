/*
 * SenseSimilarityLCH.h
 *
 * Implements word-sense similarity measures, starting with
 * the Leacock and Chodorow algorithm. This is computed 
 * directly from a hypergraph copy of the wordnet dataset.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_SENSE_SIMILARITY_LCH_H
#define OPENCOG_SENSE_SIMILARITY_LCH_H

#include "SenseSimilarity.h"
#include "SimpleTruthValue.h"

namespace opencog {

class SenseSimilarityLCH :
	public SenseSimilarity
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
		SenseSimilarityLCH(void);
		virtual ~SenseSimilarityLCH();

		virtual SimpleTruthValue similarity(Handle, Handle);
};
}

#endif /* OPENCOG_SENSE_SIMILARITY_LCH_H */
