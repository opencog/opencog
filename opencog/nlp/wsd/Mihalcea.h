/*
 * Mihalcea.h
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#ifndef _OPENCOG_MIHALCEA_H
#define _OPENCOG_MIHALCEA_H

#include <string>

#include <opencog/atoms/base/Atom.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/nlp/wsd/MihalceaEdge.h>
#include <opencog/nlp/wsd/MihalceaLabel.h>
#include <opencog/nlp/wsd/NNAdjust.h>
#include <opencog/nlp/wsd/ParseRank.h>
#include <opencog/nlp/wsd/SenseRank.h>
#include <opencog/nlp/wsd/ReportRank.h>
#include <opencog/nlp/wsd/Sweep.h>

#include "EdgeThin.h"

namespace opencog {

class Mihalcea
{
	private:
		AtomSpace *atom_space;
		MihalceaLabel labeller;
		MihalceaEdge edger;
		EdgeThin thinner;
		Sweep sweeper;
		NNAdjust nn_adjuster;
		ParseRank parse_ranker;
		SenseRank sense_ranker;
		ReportRank reporter;

		Handle previous_parse;
		std::deque<Handle> parse_list;
		std::deque<Handle> short_list;
		bool process_sentence_list(const Handle&);
		bool process_sentence(const Handle&);

	public:
		Mihalcea(void);
		~Mihalcea();
		void set_atom_space(AtomSpace *as);
		void process_document(const Handle&);
};

} // namespace opencog

#endif // _OPENCOG_MIHALCEA_LABEL_H
