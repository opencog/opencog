/**
 * FrameQuery.cc
 *
 * Implement pattern matching for RelEx semantic frame queries.
 * The pattern matching is performed at the frame level, 
 * and thus should provide at once a looser but more 
 * semnatically correct matching.
 *
 * Experiminetal, incomplete, broken.
 * XXX todo-- should have is_query look for 
 *   <EvaluationLink>
 *       <Element class="DefinedFrameElementNode" name="#Questioning:Message"/>
 *       <ListLink>
 *          <Element class="DefinedLinguisticConceptNode" name="#what"/>
 *          <Element class="ConceptNode" name="_$qVar_be8ca85e"/>
 *       </ListLink>
 *  </EvaluationLink>
 *
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <stdio.h>

#include "Node.h"
#include "FrameQuery.h"

using namespace opencog;


FrameQuery::FrameQuery(void)
{
}

FrameQuery::~FrameQuery()
{
}

/* ======================================================== */
/* Routines to help put the query into normal form. */

/**
 * Return true, if the node is, for example, _subj or _obj
 */
bool FrameQuery::is_frame_elt(Atom *atom)
{
	if (DEFINED_FRAME_ELEMENT_NODE == atom->getType()) return true;
	return false;
}

/**
 * Discard all simple relex relations, keeping only 
 * frame relations.
 *
 * Set do_discard to false to keep the thing.
 */
bool FrameQuery::discard_eval_markup(Atom *atom)
{

	Node *n = dynamic_cast<Node *>(atom);
	if(!n) return false;
	if (DEFINED_FRAME_ELEMENT_NODE != atom->getType()) return false;

	const char *name = n->getName().c_str();

	/* Throw away #Questioning,
	 * as that frame will never occur as a part of the answer.
	 */
	if (!strcmp("#Questioning:Message", name)) do_discard = false;

	return false;
}

bool FrameQuery::discard_extra_markup(Atom *atom)
{

	Node *n = dynamic_cast<Node *>(atom);
	if(!n) return false;

	const char *name = n->getName().c_str();
	if (DEFINED_FRAME_NODE == atom->getType())
	{
		/* Throw away #Questioning,
		 * as that frame will never occur as a part of the answer.
		 */
		if (!strcmp("#Questioning", name)) do_discard = false;
	}

	return false;
}

/**
 * Hack --- not actually applying any rules, except one
 * hard-coded one: if the link involves a
 * DEFINED_LINGUISTIC_RELATIONSHIP_NODE, then its a keeper.
 */
bool FrameQuery::assemble_predicate(Atom *atom)
{
	Handle ah = TLB::getHandle(atom);
	Type atype = atom->getType();
	if (EVALUATION_LINK == atype)
	{
		bool keep = foreach_outgoing_atom(ah, &FrameQuery::is_frame_elt, this);
		if (!keep) return false;

		do_discard = true;
		foreach_outgoing_atom(ah, &FrameQuery::discard_eval_markup, this);
		if (do_discard) return false;
	}
	else if (INHERITANCE_LINK == atype)
	{
		/* Discard strctures that won't appear in the answer,
		 * or that we don't want to match to.  */
		do_discard = true;
		foreach_outgoing_atom(ah, &FrameQuery::discard_extra_markup, this);
		if (do_discard) return false;
	}
	else
	{
		return false;
	}

	// Its a keeper, add this to our list of acceptable predicate terms.
	normed_predicate.push_back(ah);

	return false;
}

/* ======================================================== */
/* rutime matching routines */

/**
 * Are two nodes "equivalent", as far as the opencog representation 
 * of RelEx expressions are concerned? 
 *
 * Return true to signify a mismatch,
 * Return false to signify equivalence.
 */
bool FrameQuery::node_match(Atom *aa, Atom *ab)
{
	// If we are here, then we are comparing nodes.
	// The result of comparing nodes depends on the
	// node types.
	Type ntype = aa->getType();

	// DefinedLinguisticConcept nodes must match exactly;
	// so if we are here, there's already a mismatch.
	if (DEFINED_LINGUISTIC_CONCEPT_NODE == ntype) return true;

#if 0
	// Concept nodes can match if they inherit from the same concept.
	if (CONCEPT_NODE == ntype)
	{
		bool mismatch = concept_match(aa, ab);
		// printf("tree_comp concept mismatch=%d\n", mismatch);
		return mismatch;
	}

	if (DEFINED_LINGUISTIC_CONCEPT_NODE == ntype)
	{
		/* We force agreement for gender, etc.
		 * be have more relaxed agreement for tense...
		 * i.e. match #past to #past_infinitive, etc.
		 */
		Node *na = dynamic_cast<Node *>(aa);
		Node *nb = dynamic_cast<Node *>(ab);
		if (na && nb)
		{
			const char * sa = na->getName().c_str();
			const char * sb = nb->getName().c_str();
			char * ua = strchr(sa, '_');
			if (ua)
			{
				size_t len = ua-sa;
				char * s = (char *) alloca(len+1);
				strncpy(s,sa,len);
				s[len] = 0x0;
				sa = s;
			}
			char * ub = strchr(sb, '_');
			if (ub)
			{
				size_t len = ub-sb;
				char * s = (char *) alloca(len+1);
				strncpy(s,sb,len);
				s[len] = 0x0;
				sb = s;
			}
			if (!strcmp(sa, sb)) return false;
			return true;
		}
		return true;
	}

	fprintf(stderr, "Error: unexpected node type %d %s\n", ntype,
	        ClassServer::getTypeName(ntype));

	std::string sa = aa->toString();
	std::string sb = ab->toString();
	fprintf (stderr, "unexpected comp %s\n"
	                 "             to %s\n", sa.c_str(), sb.c_str());

#endif
	return true;
}

/* ===================== END OF FILE ===================== */
