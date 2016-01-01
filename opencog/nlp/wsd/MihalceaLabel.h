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

#include <opencog/atoms/base/Atom.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

class MihalceaLabel
{
	private:
		AtomSpace *atom_space;

		bool annotate_parse_f(const Handle&);
		bool annotate_word(const Handle&);
		bool annotate_word_sense(const Handle&);

		void fetch_senses(const Handle&);
		bool have_sense(const Handle&);
		bool pull_pos(const Handle&);
		Handle no_sense;

		Handle word_instance;
		int total_words;
		int total_labels;

	public:
		MihalceaLabel(void);
		~MihalceaLabel();
		void set_atom_space(AtomSpace *);

		void annotate_sentence(const Handle&);
		void annotate_parse(const Handle&);
};

} // namespace opencog

#endif // _OPENCOG_MIHALCEA_LABEL_H
