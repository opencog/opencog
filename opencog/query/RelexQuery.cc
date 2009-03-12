/**
 * RelexQuery.cc
 *
 * Implement pattern matching for RelEx queries.
 * The pattern matching is performed for the relex part
 * only, and not for the semantic frame part of a sentence
 * parse.
 *
 * The result of using RelEx-only matching means that
 * queries will be interpreted very literally; the
 * structure of a query sentence must closely resemble
 * the structure of a sentence in the corpus; otherwise,
 * no matching response will be found.
 *
 * Copyright (c) 2008 Linas Vepstas <linas@linas.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "RelexQuery.h"

#include <stdio.h>

#include <opencog/atomspace/Node.h>
#include <opencog/util/platform.h>

using namespace opencog;

RelexQuery::RelexQuery(void)
{
	pme = NULL;
}

RelexQuery::~RelexQuery()
{
	if (pme) delete pme;
	pme = NULL;
}

/* ======================================================== */

#define DEBUG
#ifdef DEBUG
static void prt(Atom *atom)
{
   std::string str = atom->toString();
   printf ("%s\n", str.c_str());
}

static void prt_pred (std::vector<Handle> pred,
                      std::vector<Handle> vars)
{
	printf("\nPredicate:\n");
	std::vector<Handle>::const_iterator i;
	for (i = pred.begin(); i != pred.end(); i++)
	{
		Handle h = *i;
		Atom *a = TLB::getAtom(h);
		prt(a);
	}
	printf("\nVars:\n");
	for (i = vars.begin(); i != vars.end(); i++)
	{
		Handle h = *i;
		Atom *a = TLB::getAtom(h);
		prt(a);
	}
}
#endif /* DEBUG */

/* ======================================================== */
/* Routines used to determine if an assertion is a query.
 * XXX This algo is flawed, fragile, but simple.
 */

/**
 * Return true, if atom is of type Node, and if the node
 * name is "match_name" (currently hard-coded as _$qVar)
 */
bool RelexQuery::is_qVar(Atom *atom)
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
 * to the structure of the relex-to-opencog conversion, and
 * is fragile, if that structure changes.
 */
bool RelexQuery::check_for_query(Handle rel)
{
	return foreach_outgoing_atom(rel, &RelexQuery::is_qVar, this);
}

/**
 * Return true if assertion is a query.
 * A simple check is made: does the assertion have
 * a _$qVar in it?
 *
 * The pattern check here is trivial, in that an assertion
 * that contains <WordNode name="_$qVar"/> will be assumed
 * to be a query.  Perhaps something more sophisticated may
 * be desired eventually.
 */
bool RelexQuery::is_query(Handle h)
{
	return foreach_outgoing_handle(h, &RelexQuery::check_for_query, this);
}

/* ======================================================== */
/* Routines to help put the query into normal form. */

/**
 * Return true, if the node is, for example, _subj or _obj
 */
bool RelexQuery::is_ling_rel(Atom *atom)
{
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == atom->getType()) return true;
	return false;
}

/**
 * Return true, if the node is, for example, #singluar or #masculine.
 */
bool RelexQuery::is_ling_cncpt(Atom *atom)
{
	if (DEFINED_LINGUISTIC_CONCEPT_NODE == atom->getType()) return true;
	return false;
}

bool RelexQuery::is_cncpt(Atom *atom)
{
	if (CONCEPT_NODE == atom->getType()) return true;
	return false;
}

/**
 * Discard
 * QUERY-TYPE(_$qVar,what)
 * HYP(throw, T)
 * from pattern-matching consideration; only
 * questions will have these.
 *
 * Set "do_discard" to false to keep this one.
 */
bool RelexQuery::discard_extra_markup(Atom *atom)
{
	if (DEFINED_LINGUISTIC_CONCEPT_NODE != atom->getType()) return false;

	Node *n = dynamic_cast<Node *>(atom);
	if(!n) return false;

	/* Throw away #past_infinitive and similar forms,
	 * because we haven't implemented tense matching properly,
	 * and so don't do tense matching at all.
	 *
	 * Throw away #copula-question and QUERY-TYPE #what,
	 * #which, etc. and also #truth-query, as these will
	 * never be part of the structure of the answer.
	 *
	 * Keep gender matching, noun_number matching, and
	 * definite-FLAG matching. Everything else is ignored
	 * in the match.
	 */
	const char *name = n->getName().c_str();
	if (!strcmp("#masculine", name)) do_discard = false;
	else if (!strcmp("#feminine", name)) do_discard = false;
	else if (!strcmp("#person", name)) do_discard = false;
	else if (!strcmp("#definite", name)) do_discard = false;
	else if (!strcmp("#singular", name)) do_discard = false;
	else if (!strcmp("#uncountable", name)) do_discard = false;

	return false;
}

/**
 * This method is called once for every top-level link
 * in the candidate query graph.  Out if this, it picks
 * out those relationships that should form a part of a query.
 *
 * For relex-based queries, we try to match up the relex
 * parts of the graph; using the current variant of the
 * relex-to-opencog mapping.
 *
 * These are EvaluationLink's which have a
 * DefinedLinguisticRelationship node in them,
 * and InheritanceLinks which have a
 * DefinedLinguisticConcept node in them.
 */
bool RelexQuery::assemble_predicate(Atom *atom)
{
	Handle ah = TLB::getHandle(atom);
	Type atype = atom->getType();
	if (EVALUATION_LINK == atype)
	{
		bool keep = foreach_outgoing_atom(ah, &RelexQuery::is_ling_rel, this);
		if (!keep) return false;
	}
	else if (INHERITANCE_LINK == atype)
	{
		/* Discard
		 * QUERY-TYPE(_$qVar,what)
		 * HYP(throw, T)
		 * from pattern-matching consideration; only
		 * questions will have these.
		 */
		do_discard = true;
		foreach_outgoing_atom(ah, &RelexQuery::discard_extra_markup, this);
		if (do_discard) return false;

		/* Keep things like "tense (throw, past)" but reject things like
		 * "Temporal_colocation:Time(past,throw)"
		 */
	}
	else
	{
		return false;
	}

	// Its a keeper, add this to our list of acceptable predicate terms.
	add_to_predicate(ah);

	return false;
}

void RelexQuery::add_to_predicate(Handle ah)
{
	/* scan for duplicates, and don't add them */
	std::vector<Handle>::const_iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); i++)
	{
		Handle h = *i;
		if (h == ah) return;
	}
	normed_predicate.push_back(ah);
}

void RelexQuery::add_to_vars(Handle ah)
{
	/* scan for duplicates, and don't add them */
	std::vector<Handle>::const_iterator i;
	for (i = bound_vars.begin();
	     i != bound_vars.end(); i++)
	{
		Handle h = *i;
		if (h == ah) return;
	}
	bound_vars.push_back(ah);
}

/**
 * Check to see if atom is a bound variable.
 * Heuristics are used to determine this: the local atom should
 * be an instance of a concept, whose dictionary word is _$qVar,
 * i.e. one of the bound variable names.
 */
bool RelexQuery::find_vars(Handle h)
{
	foreach_outgoing_handle(h, &RelexQuery::find_vars, this);

	Atom *atom = TLB::getAtom(h);

	if (!is_word_instance (atom, "_$qVar")) return false;

	add_to_vars(h);
	return false;
}

/* non-virtual wrapper to call virtual function */
bool RelexQuery::assemble_wrapper(Atom *atom)
{
	return assemble_predicate(atom);
}

/**
 * Put predicate into "normal form".
 * In this case, a cheap hack: remove all relations that
 * are not "defined linguistic relations", e.g. all but
 * _subj(x,y) and _obj(z,w) relations.
 */
void RelexQuery::solve(AtomSpace *atom_space, Handle graph)
{
	if (pme) delete pme;
	pme = new PatternMatchEngine();
	pme->set_atomspace(atom_space);

	// Setup "normed" predicates.
	normed_predicate.clear();
	foreach_outgoing_atom(graph, &RelexQuery::assemble_wrapper, this);

	// Find the variables, so that they can be bound.
	std::vector<Handle>::const_iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); i++)
	{
		Handle h = *i;
		find_vars(h);
	}

#ifdef DEBUG
	prt_pred(normed_predicate, bound_vars);
#endif

	// Solve...
	pme->match(this, normed_predicate, bound_vars);
}

/* ======================================================== */
/* runtime matching routines */

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

/**
 * Return true if the indicated atom is an instance of
 * the word.
 *
 * XXX
 * The actual determination of whether some concept is
 * represented by some word is fragily dependent on the
 * actual nature of concept representation in the
 * relex-to-opencog mapping. Until this is placed into
 * concrete, its inherently fragile.  This is subject
 * to change, if the relex-to-opencog mapping changes.
 * XXX
 *
 * Current mapping is:
 *   ReferenceLink
 *      WordNode "bark"
 *      ConceptNode "bark_169"
 *
 *  XXX should be "WordInstanceNode" now XXX
 *
 */
bool RelexQuery::is_word_instance(Atom *atom, const char * word)
{
	// We want the word-node associated with this word instance.
	Atom *wrd = fl.backtrack_binary_link(atom, REFERENCE_LINK);
	if (!wrd) return false;

	Node *n = dynamic_cast<Node *>(wrd);
	if(!n) return false;

	// A simple string compare.
	const std::string& name = n->getName();
	if (strcmp(name.c_str(), word)) return false;

	return true;
}

/**
 * Are two nodes "equivalent", as far as the opencog representation
 * of RelEx expressions are concerned?
 *
 * Return true to signify a mismatch,
 * Return false to signify equivalence.
 */
bool RelexQuery::node_match(Node *npat, Node *nsoln)
{
	// If we are here, then we are comparing nodes.
	// The result of comparing nodes depends on the
	// node types.
	Type pattype = npat->getType();
	Type soltype = nsoln->getType();
	if (pattype != soltype) return true;

	// DefinedLinguisticRelation nodes must match exactly;
	// so if we are here, there's already a mismatch.
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == soltype) return true;

	// Concept nodes can match if they inherit from the same concept.
	if (CONCEPT_NODE == soltype)
	{
		bool mismatch = concept_match(npat, nsoln);
		// printf("tree_comp concept mismatch=%d\n", mismatch);
		return mismatch;
	}

	if (DEFINED_LINGUISTIC_CONCEPT_NODE == soltype)
	{
		/* We force agreement for gender, etc.
		 * be have more relaxed agreement for tense...
		 * i.e. match #past to #past_infinitive, etc.
		 */
		const char * sa = npat->getName().c_str();
		const char * sb = nsoln->getName().c_str();
		const char * ua = strchr(sa, '_');
		if (ua)
		{
			size_t len = ua-sa;
			char * s = (char *) alloca(len+1);
			strncpy(s,sa,len);
			s[len] = 0x0;
			sa = s;
		}
		const char * ub = strchr(sb, '_');
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

	fprintf(stderr, "Error: unexpected node type %d %s\n", soltype,
	        ClassServer::getTypeName(soltype).c_str());

	std::string sa = npat->toString();
	std::string sb = nsoln->toString();
	fprintf (stderr, "unexpected comp %s\n"
	                 "             to %s\n", sa.c_str(), sb.c_str());

	return true;
}

/* ======================================================== */

bool RelexQuery::solution(std::map<Handle, Handle> &pred_soln,
                          std::map<Handle, Handle> &var_soln)
{
	// Reject any solution where a variable is solved
	// by another variable (e.g. if there are multiple
	// questions in the corpus, and we just happened to
	// find one of them.)
	std::map<Handle, Handle>::const_iterator j;
	for (j=var_soln.begin(); j != var_soln.end(); j++)
	{
		std::pair<Handle, Handle> pv = *j;
		Handle soln = pv.second;
		Atom *as = TLB::getAtom(soln);
		bool reject_qvar = is_word_instance(as, "_$qVar");
		if (reject_qvar) return false;
	}

	printf ("duude have soln\n");
	PatternMatchEngine::print_solution(pred_soln, var_soln);
	return false;
}

/* ===================== END OF FILE ===================== */
