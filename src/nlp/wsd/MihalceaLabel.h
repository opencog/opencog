/*
 * MihalceaLabel.h
 *
 * Implements the word-instance labelling portion of the Rada Mihalcea
 * word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef OPENCOG_MIHALCEA_LABEL_H
#define OPENCOG_MIHALCEA_LABEL_H

#include <string>

#include "Atom.h"
#include "AtomSpace.h"

namespace opencog {

class MihalceaLabel
{
	private:
		AtomSpace *atom_space;

		bool annotate_parse_f(Handle);
		bool annotate_word(Handle);
		bool annotate_word_sense(Handle);

		Atom * word_instance;

	public:
		MihalceaLabel(void);
		~MihalceaLabel();
		void set_atom_space(AtomSpace *as) {atom_space = as;}

		void annotate_sentence(Handle);
		void annotate_parse(Handle);
};
}

#endif /* OPENCOG_MIHALCEA_LABEL_H */
