/*
 * Mihalcea.cc
 *
 * Implements the Rada Mihalcea word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "Mihalcea.h"
#include "MihalceaEdge.h"
#include "MihalceaLabel.h"

using namespace opencog;

Mihalcea::Mihalcea(void)
{
	atom_space = NULL;
	labeller = new MihalceaLabel();
	edger = new MihalceaEdge();
}

Mihalcea::~Mihalcea()
{
	atom_space = NULL;
	delete labeller;
	delete edger;
}

void Mihalcea::set_atom_space(AtomSpace *as)
{
	atom_space = as;
	labeller->set_atom_space(as);
	edger->set_atom_space(as);
}

void Mihalcea::process_sentence(Handle h)
{
	labeller->annotate_sentence(h);
	edger->annotate_sentence(h);
}

