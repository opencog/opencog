/*
 * Mihalcea.h
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_MIHALCEA_H
#define OPENCOG_MIHALCEA_H

#include <string>

#include "Atom.h"
#include "AtomSpace.h"

namespace opencog {

class Mihalcea
{
	private:
		AtomSpace *atom_space;

		void annotate_sentence(Handle);
		bool annotate_parse(Handle);
		bool annotate_word(Handle);
		bool annotate_word_sense(Handle);

		Atom * word_instance;
		std::string word_inst_pos;

	public:
		Mihalcea(void);
		~Mihalcea();
		void set_atom_space(AtomSpace *as) {atom_space = as;}
		void process_sentence(Handle);
};
}

#endif /* OPENCOG_MIHALCEA_H */
