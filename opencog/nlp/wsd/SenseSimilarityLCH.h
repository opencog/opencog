/*
 * SenseSimilarityLCH.h
 *
 * Implements word-sense similarity measures, starting with
 * the Leacock and Chodorow algorithm. This is computed 
 * directly from a hypergraph copy of the wordnet dataset.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_SENSE_SIMILARITY_LCH_H
#define _OPENCOG_SENSE_SIMILARITY_LCH_H

#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/nlp/wsd/SenseSimilarity.h>

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
		SenseSimilarityLCH();
		virtual ~SenseSimilarityLCH();

		virtual SimpleTruthValuePtr similarity(Handle, Handle);
};

} // namespace opencog

#endif // _OPENCOG_SENSE_SIMILARITY_LCH_H
