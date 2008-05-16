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
 * For each word-instance, loop over all syntactic relationships
 * (i.e. _subj, _obj, _nn, _amod, and so on). For each relationship,
 * call the indicated callback. It is assumed that the relex 
 * relationships are structured as follows:
 *
 *    "The outfielder caught the ball."
 *    <!-- _subj (<<catch>>, <<outfielder>>) -->
 *    <EvaluationLink>
 *       <DefinedLinguisticRelationshipNode name="_subj"/>
 *       <ListLink>
 *          <ConceptNode name="catch_instance_23"/>
 *          <ConceptNode name="outfielder_instance_48"/>
 *       </ListLink>
 *    </EvaluationLink>
 *
 */
inline void foreach_relex_relation(Handle h)
{
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
	foreach_relex_relation(h);
	return false;
}

