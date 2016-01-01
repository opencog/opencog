/*
 * MihalceaEdge.h
 *
 * Implements the edge-creation part of the Rada Mihalcea word-sense
 * disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_MIHALCEA_EDGE_H
#define _OPENCOG_MIHALCEA_EDGE_H

#include <set>
#include <string>

#include <opencog/atoms/base/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

#include "EdgeUtils.h"
#include "SenseCache.h"
#include "SenseSimilarity.h"

namespace opencog {

class MihalceaEdge : private EdgeUtils
{
	private:
		AtomSpace *atom_space;
		SenseSimilarity *sen_sim;
		SenseCache sense_cache;
		bool annotate_parse_f(const Handle&);

		bool annotate_word_pair(const Handle&, const Handle&);
		int word_pair_count;
		int edge_count;

		Handle second_word_inst;
		Handle first_word_sense;
		Handle first_sense_link;
		bool sense_of_first_inst(const Handle&, const Handle&);
		bool sense_of_second_inst(const Handle&, const Handle&);

	public:
		MihalceaEdge();
		~MihalceaEdge();
		void set_atom_space(AtomSpace *);
		void annotate_sentence(const Handle&);
		void annotate_parse(const Handle&);
		void annotate_parse_pair(const Handle&, const Handle&);
};

} // namespace opencog

#endif // _OPENCOG_MIHALCEA_EDGE_H
