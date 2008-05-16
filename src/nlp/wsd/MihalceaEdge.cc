/*
 * MihalceaEdge.cc
 *
 * Implements the edge creation portion of the Rada Mihalcea
 * word-sense disambiguation algorithm.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */
#include <stdio.h>

#include "ForeachWord.h"
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

/** Loop over all parses for this sentence. */
void MihalceaEdge::annotate_sentence(Handle h)
{
	foreach_parse(h, &MihalceaEdge::annotate_parse, this);
}

/**
 * For each parse, loop over all word-instance syntactic relationships.
 * (i.e. _subj, _obj, _nn, _amod, and so on). For each relationship,
 * create an edge between all corresponding (word-instance, word-sense)
 * pairs.
 */
bool MihalceaEdge::annotate_parse(Handle h)
{
	foreach_word_instance(h, &MihalceaEdge::annotate_word, this);
	return false;
}

/**
 * For each word-instance loop over all syntactic relationships.
 * (i.e. _subj, _obj, _nn, _amod, and so on). For each relationship,
 * create an edge between all corresponding (word-instance, word-sense)
 * pairs.
 */
bool MihalceaEdge::annotate_word(Handle h)
{
	printf("Hellowwwwwwwwww world\n");
	foreach_relex_relation(h, &MihalceaEdge::annotate_relation, this);
	return false;
}

bool MihalceaEdge::annotate_relation(const std::string &relname, Handle h)
{
	printf("dude got rel=%s\n", relname.c_str());
	return false;
}
