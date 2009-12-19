/*
 * MihalceaLabel.h
 *
 * Implements the word-instance labelling portion of the Rada Mihalcea
 * word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_MIHALCEA_LABEL_H
#define _OPENCOG_MIHALCEA_LABEL_H

#include <string>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

class MihalceaLabel
{
	private:
		AtomSpace *atom_space;

		bool annotate_parse_f(Handle);
		bool annotate_word(Handle);
		bool annotate_word_sense(Handle);

		void fetch_senses(Handle);
		bool have_sense(Atom *);
		bool pull_pos(Handle);
		Handle no_sense;

		Atom * word_instance;
		int total_words;
		int total_labels;

	public:
		MihalceaLabel(void);
		~MihalceaLabel();
		void set_atom_space(AtomSpace *);

		void annotate_sentence(Handle);
		void annotate_parse(Handle);
};

} // namespace opencog

#endif // _OPENCOG_MIHALCEA_LABEL_H
