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

#include <string>

#include "Atom.h"
#include "AtomSpace.h"

namespace opencog {

class MihalceaEdge
{
	private:
		AtomSpace *atom_space;

	public:
		MihalceaEdge(void);
		~MihalceaEdge();
		void set_atom_space(AtomSpace *as);
		void annotate_sentence(Handle);
};
}

#endif /* OPENCOG_MIHALCEA_LABEL_H */
