/**
 * RelexQuery.cc
 *
 * Implement pattern matching for RelEx queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <stdio.h>

#include "RelexQuery.h"

using namespace opencog;

/* ======================================================== */
/**
 * Are two atoms instances of the same concept?
 * Return true if they are are NOT (that is, if they
 * are mismatched). This stops iteration in the standard
 * iterator.
 */
bool RelexQuery::concept_match(Atom *aa, Atom *ab)
{
	// printf ("concept comp "); prt(aa);
	// printf ("          to "); prt(ab);

	// If they're the same atom, then clearly they match.
	if (aa == ab) return false;

	// Look for incoming links that are InheritanceLinks.
	// The "generalized concept" for this should be at the far end.
	Atom *ca = fl.follow_binary_link(aa, INHERITANCE_LINK);
	Atom *cb = fl.follow_binary_link(ab, INHERITANCE_LINK);

	// printf ("gen comp %d ", ca==cb); prt(ca);
	// printf ("        to "); prt(cb);

	if (ca == cb) return false;
	return true;
}


bool RelexQuery::node_match(Atom *aa, Atom *ab)
{
	// If we are here, then we are comparing nodes.
	// The result of comparing nodes depends on the
	// node types.
	Type ntype = aa->getType();

	// DefinedLinguisticRelation nodes must match exactly;
	// so if we are here, there's already a mismatch.
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == ntype) return true;

	// Concept nodes can match if they inherit from the same concept.
	if (CONCEPT_NODE == ntype)
	{
		bool mismatch = concept_match(aa, ab);
		// printf("tree_comp concept mismatch=%d\n", mismatch);
		return mismatch;
	}
	fprintf(stderr, "Error: unexpected node type %d %s\n", ntype,
	        ClassServer::getTypeName(ntype));

	std::string sa = aa->toString();
	std::string sb = ab->toString();
	fprintf (stderr, "unexpected comp %s\n"
	                 "             to %s\n", sa.c_str(), sb.c_str());

	return true;
}

/* ======================================================== */

bool RelexQuery::solution(void)
{
	printf ("duude have soln\n");
	PatternMatch pm(NULL);
	pm.print_solution(*var_solution, *predicate_solution);
	return false;
}

/* ===================== END OF FILE ===================== */
