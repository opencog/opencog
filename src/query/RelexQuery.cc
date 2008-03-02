/**
 * RelexQuery.cc
 *
 * Implement pattern matching for RelEx queries.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 */

#include <stdio.h>

#include "Node.h"
#include "RelexQuery.h"

using namespace opencog;


RelexQuery::RelexQuery(void)
{
	pm = NULL;
}

RelexQuery::~RelexQuery()
{
	if (pm) delete pm;
	pm = NULL;
}

/* ======================================================== */
/* Routines used to determine if an assertion is a query.
 * XXX This algo is flawed, fragile, but simple.
 */

/**
 * Set pointer to Node, if the node name is "match_name".
 */
bool RelexQuery::match_node_name(Atom *atom)
{
	const char *match_name = "_$qVar";
	Node *n = dynamic_cast<Node *>(atom);
	if (n)
	{
		const std::string& name = n->getName();
		if (0 == strcmp(name.c_str(), match_name))
			return true;
	}
	return false;
}

/**
 * Search for queries. 
 * XXX This implementation is kinda-wrong, its very specific
 * to the structure of the relex-to-opencog converstion, and 
 * is fragile, if that structure changes.
 */
bool RelexQuery::check_for_query(Handle rel)
{
	return foreach_outgoing_atom(rel, &RelexQuery::match_node_name, this);
}

/**
 * Return true if assertion is a query.
 * A simple checdk is made: does the assertion have 
 * a _$qVar in it? 
 */
bool RelexQuery::is_query(Handle h)
{
	return foreach_outgoing_handle(h, &RelexQuery::check_for_query, this);
}

/* ======================================================== */
/* Routines to help put the query into normal form. */

/**
 * Return true, for example, if the node is _subj or _obj
 */
bool RelexQuery::is_ling_rel(Atom *atom)
{
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == atom->getType()) return true;
	return false;
}

/**
 * Hack --- not actually applying any rules, except one
 * hard-coded one: if the link involves a
 * DEFINED_LINGUISTIC_RELATIONSHIP_NODE, then its a keeper.
 */
bool RelexQuery::apply_rule(Atom *atom)
{
	if (EVALUATION_LINK != atom->getType()) return false;

	Handle ah = TLB::getHandle(atom);
	bool keep = foreach_outgoing_atom(ah, &RelexQuery::is_ling_rel, this);

	if (!keep) return false;

	// Its a keeper, add this to our list of acceptable predicate terms.
	normed_predicate.push_back(ah);

	return false;
}

/**
 * Check to see if atom is a bound variable.
 * Heuristics are used to determine this: the local atom should
 * be an instance of a concept, whose dictionary word is _$qVar,
 * i.e. one of the bound variable names.
 *
 * XXX this is subject to change, if the relex rep changes.
 */
bool RelexQuery::find_vars(Handle h)
{
	foreach_outgoing_handle(h, &RelexQuery::find_vars, this);

	Atom *atom = TLB::getAtom(h);

	// The local atom will be an instance of a general concept...
	atom = fl.follow_binary_link(atom, INHERITANCE_LINK);
	if(!atom) return false;
	// and we want the "word" associated with this general concept.
	atom = fl.backtrack_binary_link(atom, WR_LINK);
	if(!atom) return false;

	Node *n = dynamic_cast<Node *>(atom);
	if(!n) return false;

	const char * match_name = "_$qVar";
	const std::string& name = n->getName();
	if (strcmp(name.c_str(), match_name)) return false;

	bound_vars.push_back(h);
	return false;
}

/**
 * Put predicate into "normal form".
 * In this case, a cheap hack: remove all relations that
 * are not "defined linguistic relations", e.g. all but
 * _subj(x,y) and _obj(z,w) relations.
 */
void RelexQuery::solve(AtomSpace *atom_space, Handle graph)
{
	if (pm) delete pm;
	pm = new PatternMatch(atom_space);

	// Setup "normed" predicates.
	normed_predicate.clear();
	foreach_outgoing_atom(graph, &RelexQuery::apply_rule, this);

	// Find the variables, so that they can be bound.
	std::vector<Handle>::const_iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); i++)
	{
		Handle h = *i;
		find_vars(h);
	}

	// Solve...
	pm->match(this, &normed_predicate, &bound_vars);
}

/* ======================================================== */
/* rutime matching routines */

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

bool RelexQuery::solution(std::map<Handle, Handle> &pred_soln,
                          std::map<Handle, Handle> &var_soln)
{
	printf ("duude have soln\n");
	PatternMatch pm(NULL);
	pm.print_solution(pred_soln, var_soln);
	return false;
}

/* ===================== END OF FILE ===================== */
