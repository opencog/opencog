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

namespace opencog {

class MihalceaEdge
{
	private:
		AtomSpace *atom_space;
		bool annotate_parse(Handle);
		bool annotate_word(Handle);

		std::set<Handle> words;
		bool look_at_relation(const std::string &, Handle, Handle);
		bool annotate_word_pair(Handle, Handle);

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
};
}

#endif /* OPENCOG_MIHALCEA_LABEL_H */
