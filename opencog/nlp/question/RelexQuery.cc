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

#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Node.h>

using namespace opencog;

RelexQuery::RelexQuery(void)
{
	atom_space = NULL;
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
 * XXX It almost surely would be much better to implement this in 
 * scheme instead of C++.
 */

/**
 * Return true, if atom is of type Node, and if the node
 * name is "match_name" (currently hard-coded as _$qVar)
 */
bool RelexQuery::is_qVar(Handle word_prop)
{
	Atom *atom = TLB::getAtom(word_prop);
	if (DEFINED_LINGUISTIC_CONCEPT_NODE != atom->getType()) return false;

	Node *n = static_cast<Node *>(atom);
	const std::string& name = n->getName();
	const char * str = name.c_str();
	if (0 == strcmp(str, "who"))
		return true;
	if (0 == strcmp(str, "what"))
		return true;
	if (0 == strcmp(str, "when"))
		return true;
	if (0 == strcmp(str, "where"))
		return true;
	if (0 == strcmp(str, "why"))
		return true;

	return false;
}

/**
 * Search for queries.
 * XXX This implementation is kinda-wrong, its very specific
 * to the structure of the relex-to-opencog conversion, and
 * is fragile, if that structure changes.
 */
bool RelexQuery::is_word_a_query(Handle word_inst)
{
	return foreach_binary_link(word_inst, INHERITANCE_LINK, &RelexQuery::is_qVar, this);
}

bool RelexQuery::is_wordlist_a_query(Handle wordlist)
{
	return foreach_outgoing_handle(wordlist, &RelexQuery::is_word_a_query, this);
}

bool RelexQuery::is_parse_a_query(Handle parse)
{
	return foreach_binary_link(parse, REFERENCE_LINK, &RelexQuery::is_wordlist_a_query, this);
}

/**
 * Return true if sentence has a parse which is a query.
 * The input argument is a handle to a SentenceNode.
 *
 * A simple check is made: does the sentence have
 * a _$qVar in it?
 *
 * The pattern check here is trivial, in that a sentence
 * that contains (DefinedLinguisticConceptNode "_$qVar") will be assumed
 * to be a query.  Perhaps something more sophisticated may
 * be desired eventually.
 *
 * Current structure of a question is as follows:
 *
 * (ParseLink
 *    (ParseNode "sentence@4509207f-468f-45ec-b8c8-217279dba127_parse_0" (stv 1.0 0.9417))
 *    (SentenceNode "sentence@4509207f-468f-45ec-b8c8-217279dba127")
 * )
 * (ReferenceLink (stv 1.0 1.0)
 *    (ParseNode "sentence@4509207f-468f-45ec-b8c8-217279dba127_parse_0")
 *    (ListLink
 *       (WordInstanceNode "who@15e6eeff-7d2a-4d4c-a29b-9ac2aaa08f7c")
 *       (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *       (WordInstanceNode "the@0d969b75-1f7b-4174-b7c6-40e3fbb87ed9")
 *       (WordInstanceNode "ball@e798a7dc-c8e4-4192-a386-29549c587a2f")
 *       (WordInstanceNode "?@ea5f242b-1ca5-462a-83a3-e6d54da4b23b")
 *    )
 * )
 * ; QUERY-TYPE (_$qVar, who)
 * (InheritanceLink (stv 1.0 1.0)
 *    (WordInstanceNode "who@15e6eeff-7d2a-4d4c-a29b-9ac2aaa08f7c")
 *    (DefinedLinguisticConceptNode "who")
 * )
 *
 * So, given a sentenceNode, we have to follow the parse link to get the
 * list of words, and then see if any of the words correspond to the 
 * DefinedLinguisticConceptNode's of who, what, where, why, when, etc.
 */
bool RelexQuery::is_query(Handle h)
{
	return foreach_reverse_binary_link(h, PARSE_LINK, &RelexQuery::is_parse_a_query, this);
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

#ifdef DEAD_CODE_BUT_DONT_DELETE_JUST_YET
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
 *
 * xxxxxxxxxxx this routine is dead, and no longer used ... 
 * we need it for reference to complete the port.
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
#endif /* DEAD_CODE_BUT_DONT_DELETE_JUST_YET */

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
 * Look to see if word instance is a bound variable,
 * if it is, then add it to the variables list.
 */
bool RelexQuery::find_vars(Handle h)
{
	foreach_outgoing_handle(h, &RelexQuery::find_vars, this);

	bool qvar = is_word_a_query(h);
	if (!qvar) return false;

	add_to_vars(h);
	return false;
}

bool RelexQuery::rel_up(Handle hrelation)
{
	Atom *a = TLB::getAtom(hrelation);
	if (EVALUATION_LINK != a->getType()) return false;

	bool keep = foreach_outgoing_atom(hrelation, &RelexQuery::is_ling_rel, this);
	if (!keep) return false;

	// Its a keeper, add this to our list of acceptable predicate terms.
	add_to_predicate(hrelation);

	return false;
}

bool RelexQuery::word_up(Handle ll)
{
	Atom *a = TLB::getAtom(ll);
	if (LIST_LINK != a->getType()) return false;

	return foreach_incoming_handle(ll,
		&RelexQuery::rel_up, this);
}

bool RelexQuery::word_solve(Handle word_inst)
{
	return foreach_incoming_handle(word_inst,
		&RelexQuery::word_up, this);
}

bool RelexQuery::wordlist_solve(Handle wordlist)
{
	return foreach_outgoing_handle(wordlist, 
		&RelexQuery::word_solve, this);
}

bool RelexQuery::parse_solve(Handle parse_node)
{
	return foreach_binary_link(parse_node, REFERENCE_LINK, 
		&RelexQuery::wordlist_solve, this);
}

/**
 * The input argument is a handle to a SentenceNode.
 *
 * Put predicate into "normal form".
 * In this case, a cheap hack: remove all relations that
 * are not "defined linguistic relations", e.g. all but
 * _subj(x,y) and _obj(z,w) relations.
 *
 * Current structure of a question is:
 *
 * (ParseLink
 *    (ParseNode "sentence@4509207f-468f-45ec-b8c8-217279dba127_parse_0" (stv 1.0 0.9417))
 *    (SentenceNode "sentence@4509207f-468f-45ec-b8c8-217279dba127")
 * )
 *
 * (WordInstanceLink (stv 1.0 1.0)
 *    (WordInstanceNode "ball@e798a7dc-c8e4-4192-a386-29549c587a2f")
 *    (ParseNode "sentence@4509207f-468f-45ec-b8c8-217279dba127_parse_0")
 * )
 *
 * ; _subj (<<throw>>, <<_$qVar>>) 
 * (EvaluationLink (stv 1.0 1.0)
 *    (DefinedLinguisticRelationshipNode "_subj")
 *    (ListLink
 *       (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *       (WordInstanceNode "who@15e6eeff-7d2a-4d4c-a29b-9ac2aaa08f7c")
 *    )
 * )
 * ; _obj (<<throw>>, <<ball>>) 
 * (EvaluationLink (stv 1.0 1.0)
 *    (DefinedLinguisticRelationshipNode "_obj")
 *    (ListLink
 *          (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *          (WordInstanceNode "ball@e798a7dc-c8e4-4192-a386-29549c587a2f")
 *    )
 * )
 *
 * (ReferenceLink (stv 1.0 1.0)
 *    (WordInstanceNode "ball@e798a7dc-c8e4-4192-a386-29549c587a2f")
 *    (WordNode "ball")
 * )
 *
 * So the strategy is:
 * 1) Find all parses that are a part of this sentence.
 * 2) For each wordinstance in the parse, find all relations it 
 *    participates in, add these to the predicate.
 * 3) Avoid duplication in step 3)
 * 4) Find the query var.
 * 5) perform pattern matching.
 *
 */
void RelexQuery::solve(AtomSpace *as, Handle sentence_node)
{
	atom_space = as;
	if (pme) delete pme;
	pme = new PatternMatchEngine();
	pme->set_atomspace(atom_space);

	// Setup "normed" predicates.
	normed_predicate.clear();
	// foreach_outgoing_atom(graph, &RelexQuery::assemble_wrapper, this);
	foreach_reverse_binary_link(sentence_node, PARSE_LINK, 
		&RelexQuery::parse_solve, this);

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
	std::vector<Handle> ign;
	pme->match(this, bound_vars, normed_predicate, ign);
}

/* ======================================================== */
/* runtime matching routines */

/**
 * Do two word instances have the same word lemma (word root form)?
 * Return true if they are are NOT (that is, if they
 * are mismatched). This stops iteration in the standard
 * iterator.
 *
 * Current structure is:
 * (LemmaLink (stv 1.0 1.0)
 *    (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *    (WordNode "throw")
 * )
 * (ReferenceLink (stv 1.0 1.0)
 *    (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *    (WordNode "threw")
 * )
 */
bool RelexQuery::word_instance_match(Atom *aa, Atom *ab)
{
	// printf ("concept comp "); prt(aa);
	// printf ("          to "); prt(ab);

	// If they're the same atom, then clearly they match.
	if (aa == ab) return false;

	// Look for incoming links that are LemmaLinks.
	// The word lemma should be at the far end.
	Atom *ca = fl.follow_binary_link(aa, LEMMA_LINK);
	Atom *cb = fl.follow_binary_link(ab, LEMMA_LINK);

	// printf ("gen comp %d ", ca==cb); prt(ca);
	// printf ("        to "); prt(cb);

	if (ca == cb) return false;
	return true;
}

#ifdef DEAD_CODE_BUT_DONT_DELETE_JUST_YET
/**
 * Return the word string associated with a word instance.
 * xxxxxxxxx this routine is never called!
 *
 * XXX
 * The actual determination of whether some concept is
 * represented by some word is fragily dependent on the
 * actual nature of concept representation in the
 * relex-to-opencog mapping. Until this is placed into
 * concrete, its inherently fragile.  This is subject
 * to change, if the relex-to-opencog mapping changes.
 *
 * Current mapping is:
 *   ReferenceLink
 *      WordInstanceNode "bark@e798a7dc"
 *      WordNode "bark"
 *
 */
const char * RelexQuery::get_word_instance(Atom *atom)
{
	// We want the word-node associated with this word instance.
	Atom *wrd = fl.follow_binary_link(atom, REFERENCE_LINK);
	if (!wrd) return NULL;

	if (WORD_NODE != wrd->getType()) return NULL;

	Node *n = static_cast<Node *>(wrd);
	const std::string& name = n->getName();

	return name.c_str();
}
#endif /* DEAD_CODE_BUT_DONT_DELETE_JUST_YET */

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

	// Word instances match only if they have the same word lemma.
	if (WORD_INSTANCE_NODE == soltype)
	{
		bool mismatch = word_instance_match(npat, nsoln);
		// printf("tree_comp word instance mismatch=%d\n", mismatch);
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
printf("duude compare %s to %s\n", sa, sb);
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
	        classserver().getTypeName(soltype).c_str());

	std::string sa = npat->toString();
	std::string sb = nsoln->toString();
	fprintf (stderr, "unexpected comp %s\n"
	                 "             to %s\n", sa.c_str(), sb.c_str());

	return true;
}

/* ======================================================== */

bool RelexQuery::solution(std::map<Handle, Handle> &pred_grounding,
                          std::map<Handle, Handle> &var_grounding)
{
	// Reject any solution where a variable is solved
	// by another variable (e.g. if there are multiple
	// questions in the corpus, and we just happened to
	// find one of them.)
	std::map<Handle, Handle>::const_iterator j;
	for (j=var_grounding.begin(); j != var_grounding.end(); j++)
	{
		std::pair<Handle, Handle> pv = *j;
		Handle soln = pv.second;
		// Atom *as = TLB::getAtom(soln);
// xxxx
		// bool reject_qvar = is_word_instance(as, "_$qVar");
		// if (reject_qvar) return false;
	}

	printf ("duude Found solution:\n");
	PatternMatchEngine::print_solution(pred_grounding, var_grounding);

	// And now for a cheesy hack to report the solution
	Handle hq = atom_space->addNode(CONCEPT_NODE, "# QUERY SOLUTION");
	Handle hv = bound_vars[0];
	Handle ha = var_grounding[hv];

	Atom *a = TLB::getAtom(ha);
	Atom *wrd = fl.follow_binary_link(a, REFERENCE_LINK);
	Node *n = dynamic_cast<Node *>(wrd);
	if (!n) return false;
	printf("duude answer=%s\n", n->getName().c_str());

	Handle hw = TLB::getHandle(wrd);
	atom_space->addLink(LIST_LINK, hq, hw);
	
	return false;
}

/* ===================== END OF FILE ===================== */
