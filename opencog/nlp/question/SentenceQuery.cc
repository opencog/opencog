/**
 * SentenceQuery.cc
 *
 * Implement pattern matching for Sentence queries. 
 * A "Sentence query" is a sentence such as "What did Bob eat?"
 * RelEx generates a dependency graph for this sentence,  replacing 
 * "What" by "$qVar". Pattern matching is used to find an identical
 * dependency graph, for which $qVar would have a grounding; e.g.
 * "Bob ate cake", so that $qVar is grounded as "cake", thus "solving"
 * the query.
 *
 * The result of matching dependency graphs means that queries will be
 * interpreted very literally; the structure of a query sentence must 
 * closely resemble the structure of a sentence in the corpus; otherwise,
 * no matching response will be found.  Some generality can be obtained
 * by converting dependency graphs into semantic triples; the code below
 * should work for that case as well.
 *
 * Strategy for truth-query questions:
 * 1) Determine if question is of truth-query type.
 * 2) Assume its a hypothetical copula (i.e. Is X a Y?)
 * 3) pattern-match the triple:
 *    is-a(city, Madrid) to hyp-is-a(city, Madrid)
 * 4) Accept only the above pattern, and return yes, no.
 *
 * This code was used to perform the experiments described in the README
 * file. It is also currently used to power the initial stages of the
 * chatbot.  However, work is underway to make this code obsolete, and
 * to replace it purely with ImplicationLinks. The goal is to be able to
 * perform question-answering without requiring any C++ code (except
 * for the pattern matcher) and also without requiring any scheme code
 * either (except for basic i/o interfacing).
 *
 * Once the above is done, this code will be "of historical interest
 * only", as they say.
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
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

#include "SentenceQuery.h"

#include <stdio.h>

#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Node.h>

using namespace opencog;

/* ================================================================= */
/* Determine if the sentence is an ordinary query (e.g. what is X?) */

bool SentenceQuery::is_wordlist_a_query(Handle wordlist)
{
	bool rc = foreach_outgoing_handle(wordlist, 
	              &SentenceQuery::is_word_a_query, (WordRelQuery*) this);
	if (rc) return true;

	rc = foreach_outgoing_handle(wordlist, 
	              &SentenceQuery::is_parse_a_truth_query, this);

	if (rc) its_tq = true;
	return rc;
}

bool SentenceQuery::is_parse_a_query(Handle parse)
{
	return foreach_binary_link(parse, REFERENCE_LINK, 
                             &SentenceQuery::is_wordlist_a_query, this);
}

/**
 * Return true if the atom is a DefinedLinguisticConceptNode
 * whose name is "truth-query".
 */
bool SentenceQuery::is_tq(Handle prop)
{
	AtomSpace *atomspace = cogserver().getAtomSpace();
	if (DEFINED_LINGUISTIC_CONCEPT_NODE != atomspace->getType(prop)) return false;

	const std::string& name = atomspace->getName(prop);
	const char * str = name.c_str();
	if (0 == strcmp(str, "truth-query"))
		return true;
	return false;
}

/**
 * Return true if the indicated parse has the "truth-query" flag set.
 */
bool SentenceQuery::is_parse_a_truth_query(Handle parse)
{
	return foreach_binary_link(parse, INHERITANCE_LINK, 
	                           &SentenceQuery::is_tq, this);
}

/**
 * Return true if sentence has a parse which is a query.
 * The input argument is a handle to a SentenceNode.
 *
 * Two checks are made: for sentences that are simple queries,
 * and for truth queries. Simple queries have the form "Who did X?"
 * while truth queries ask for yes/no responses: "Is X a Y?"
 *
 * Simple queries have a _$qVar in them, specifically, a 
 * DefinedLinguisticConceptNode "_$qVar"
 * Truth queries are tagged by rules in the triples directory.
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
bool SentenceQuery::is_query(Handle h)
{
	its_tq = false;
	bool rc = foreach_reverse_binary_link(h, PARSE_LINK, 
	                         &SentenceQuery::is_parse_a_query, this);

	if (rc) return true;
	return foreach_reverse_binary_link(h, PARSE_LINK,
	                          &SentenceQuery::is_parse_a_truth_query, this);
}

/* ================================================================= */

/**
 * Discard
 * QUERY-TYPE(_$qVar,what)
 * HYP(throw, T)
 * from pattern-matching consideration; only
 * questions will have these.
 *
 * Set "do_discard" to false to keep this one.
 */
bool SentenceQuery::discard_extra_markup(Atom *atom)
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
bool SentenceQuery::assemble_predicate(Atom *atom)
{
	Handle ah = atom->handle;
	Type atype = atom->getType();
	if (EVALUATION_LINK == atype)
	{
		bool keep = foreach_outgoing_atom(ah, &WordRelQuery::is_ling_rel, this);
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
		foreach_outgoing_atom(ah, &WordRelQuery::discard_extra_markup, this);
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

/* ================================================================= */
/**
 * Return true, if the node is, for example, _subj or _obj
 */
bool SentenceQuery::is_ling_rel(Atom *atom)
{
	Type t = atom->getType();
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == t) return true;
	if (PREPOSITIONAL_RELATIONSHIP_NODE == t) return true;
	return false;
}

bool SentenceQuery::rel_up(Handle hrelation)
{
	AtomSpace *atomspace = cogserver().getAtomSpace();
	if (EVALUATION_LINK != atomspace->getType(hrelation)) return false;

	bool keep = foreach_outgoing_atom(hrelation, &SentenceQuery::is_ling_rel, this);
	if (!keep) return false;

	// Its a keeper, add this to our list of acceptable predicate terms.
	add_to_predicate(hrelation);

	return false;
}

bool SentenceQuery::word_up(Handle ll)
{
	AtomSpace *atomspace = cogserver().getAtomSpace();
	if (LIST_LINK != atomspace->getType(ll)) return false;

	return foreach_incoming_handle(ll,
		&SentenceQuery::rel_up, this);
}

/**
 * Find all EvaluationLink- DefinedLinguisticRelationshipNode structures
 * that this word participates in, and add them to the list of predicates.
 */
bool SentenceQuery::word_solve(Handle word_inst)
{
	return foreach_incoming_handle(word_inst,
		&SentenceQuery::word_up, this);
}

bool SentenceQuery::wordlist_solve(Handle wordlist)
{
	return foreach_outgoing_handle(wordlist, 
		&SentenceQuery::word_solve, this);
}

/**
 * Given a ParseNode, add all of the associated RelEx dependencies for
 * that parse, to the predicate to be solved.
 */
bool SentenceQuery::parse_solve(Handle parse_node)
{
	return foreach_binary_link(parse_node, REFERENCE_LINK, 
		&SentenceQuery::wordlist_solve, this);
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
 * 2) For each word-instance in the parse, find all relations it 
 *    participates in, add these to the predicate.
 * 3) Avoid duplication in step 3)
 * 4) Find the query var.
 * 5) perform pattern matching.
 *
 */
void SentenceQuery::solve(AtomSpace *as, Handle sentence_node)
{
	atom_space = as;

	// Setup "normed" predicates.
	normed_predicate.clear();
	// foreach_outgoing_atom(graph, &WordRelQuery::assemble_wrapper, this);
	foreach_reverse_binary_link(sentence_node, PARSE_LINK, 
		&SentenceQuery::parse_solve, this);

	// Find the variables, so that they can be bound.
	bound_vars.clear();
	std::vector<Handle>::const_iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); ++i)
	{
		Handle h = *i;
		find_vars(h);
	}

#define DEBUG
#ifdef DEBUG
	printf("SentenceQuery: basic sentence structure matching:\n");
	PatternMatchEngine::print_predicate(bound_vars, normed_predicate);
#endif

	// Some bad relex parses fail to actually have query variables
	// in them. If there are no variables, don't bother looking for
	// a solution.
	if (0 == bound_vars.size()) return;

	// Solve...
	if (pme) delete pme;
	pme = new PatternMatchEngine();
	pme->set_atomspace(atom_space);

	std::vector<Handle> ign;
	pme->match(this, bound_vars, normed_predicate, ign);
}

/* ===================== END OF FILE ===================== */
