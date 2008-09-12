/*
 * MihalceaEdge.h
 *
 * Implements the edge-creation part of the Rada Mihalcea word-sense
 * disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_MIHALCEA_EDGE_H
#define OPENCOG_MIHALCEA_EDGE_H

#include <set>
#include <string>

#include "Atom.h"
#include "AtomSpace.h"
#include "SenseSimilarity.h"

namespace opencog {

class MihalceaEdge
{
	private:
		AtomSpace *atom_space;
		SenseSimilarity *sen_sim;
		bool annotate_parse_f(Handle);

		std::set<Handle> words;
		bool look_at_word(Handle);
		bool look_at_relation(const std::string &, Handle, Handle);
		bool annotate_word_pair(Handle, Handle);
		int word_pair_count;
		int edge_count;

		Handle second_word_inst;
		Handle first_word_sense;
		Handle first_sense_link;
		bool sense_of_first_inst(Handle, Handle);
		bool sense_of_second_inst(Handle, Handle);

	public:
		MihalceaEdge(void);
		~MihalceaEdge();
		void set_atom_space(AtomSpace *as);
		void annotate_sentence(Handle);
		void annotate_parse(Handle);
		void annotate_parse_pair(Handle, Handle);
};
}

#endif /* OPENCOG_MIHALCEA_EDGE_H */
