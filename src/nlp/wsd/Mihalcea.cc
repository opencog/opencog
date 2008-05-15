/*
 * Mihalcea.cc
 *
 * Implements the word-instance labelling portion of the Rada Mihalcea
 * word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "Mihalcea.h"

using namespace opencog;

Mihalcea::Mihalcea(void)
{
	atom_space = NULL;
	labeller = new MihalceaLabel();
}

Mihalcea::~Mihalcea()
{
	atom_space = NULL;
	delete labeller;
}

void Mihalcea::set_atom_space(AtomSpace *as)
{
	atom_space = as;
	labeller->set_atom_space(as);
}

void Mihalcea::process_sentence(Handle h)
{
	labeller->annotate_sentence(h);
}

