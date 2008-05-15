/*
 * MihalceaEdge.cc
 *
 * Implements the edge creation portion of the Rada Mihalcea
 * word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "MihalceaEdge.h"

using namespace opencog;

MihalceaEdge::MihalceaEdge(void)
{
	atom_space = NULL;
}

MihalceaEdge::~MihalceaEdge()
{
	atom_space = NULL;
}

void MihalceaEdge::set_atom_space(AtomSpace *as)
{
	atom_space = as;
}

void MihalceaEdge::annotate_sentence(Handle h)
{
	printf("Hellowwwwwwwwww world\n");
}

