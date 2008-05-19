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
#include "MihalceaEdge.h"
#include "MihalceaLabel.h"
#include "SenseRank.h"
#include "ReportRank.h"

namespace opencog {

class Mihalcea
{
	private:
		AtomSpace *atom_space;
		MihalceaLabel *labeller;
		MihalceaEdge *edger;
		SenseRank *ranker;
		ReportRank *reporter;

	public:
		Mihalcea(void);
		~Mihalcea();
		void set_atom_space(AtomSpace *as);
		void process_sentence(Handle);
};
}

#endif /* OPENCOG_MIHALCEA_LABEL_H */
