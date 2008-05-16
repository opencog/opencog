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
 * It is assumed that the passed handle indicates the first word
 * instance in the relationship.
 */
template <typename T>
class RelexRelationFinder
{
	private:
		Atom *listlink;
		bool look_for_eval_link(Handle h)
		{
			Atom *a = TLB::getAtom(h);
			if (a->getType() != EVALUATION_LINK) return false;

			// If we are here, lets see if the first node is a ling rel.
			Link *l = dynamic_cast<Link *>(a);
			if (l == NULL) return false;

			a = l->getOutgoingAtom(0);
			Node *n = dynamic_cast<Node *>(a);
			if (n == NULL) return false;
			if (n->getType() != DEFINED_LINGUISTIC_RELATIONSHIP_NODE) return false;

			// OK, we've found a relationship. Get the second member of
			// the list link, and call the suer callback with it.
			const std::string &relname = n->getName();

			l = dynamic_cast<Link *>(listlink);
			a = l->getOutgoingAtom(1);
			Handle second = TLB::getHandle(a);

			(user_data->*user_cb)(relname, second);
			return false;
		}
		
	public:
		bool (T::*user_cb)(const std::string &, Handle);
		T *user_data;

		bool look_for_list_link(Handle h)
		{
			Atom *a = TLB::getAtom(h);
			if (a->getType() != LIST_LINK) return false;
			listlink = a;

			// If we are here, lets see if the list link is in eval link.
			foreach_incoming_handle(h, &RelexRelationFinder::look_for_eval_link, this);
			return false;
		}
};

template <typename T>
inline void foreach_relex_relation(Handle h, bool (T::*cb)(const std::string &, Handle), T *data)
{
	RelexRelationFinder<T> rrf;
	rrf.user_cb = cb;
	rrf.user_data = data;
	foreach_incoming_handle(h, &RelexRelationFinder<T>::look_for_list_link, &rrf);
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
