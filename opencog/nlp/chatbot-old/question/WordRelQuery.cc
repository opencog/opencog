/**
 * WordRelQuery.cc
 *
 * Implement pattern matching for RelEx queries. 
 * A "RelEx query" is a sentence such as "What did Bob eat?"
 * RelEx generates a dependency graph for tthis sentence,  replacing 
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
 * show work for that case as well.
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

#include "WordRelQuery.h"

#include <stdio.h>

#include <opencog/atoms/proto/NameServer.h>
#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Node.h>

using namespace opencog;

WordRelQuery::WordRelQuery(void)
{
	atom_space = NULL;
	pme = NULL;
}

WordRelQuery::~WordRelQuery()
{
	if (pme) delete pme;
	pme = NULL;
}

// #define DEBUG
#ifdef DEBUG
static void prt(Atom *atom)
{
   std::string str = atom->to_string();
   printf ("%s\n", str.c_str());
}
#else
static inline void prt(Atom *atom) {}
#endif
#ifdef DEBUG
   #define dbgprt(f, varargs...) printf(f, ##varargs)
#else
   #define dbgprt(f, varargs...)
#endif


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
bool WordRelQuery::is_qVar(Handle word_prop)
{
	AtomSpace *atomspace = cogserver().getAtomSpace();
	if (DEFINED_LINGUISTIC_CONCEPT_NODE != atomspace->getType(word_prop))
        return false;

	const std::string& name = atomspace->get_name(word_prop);
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
bool WordRelQuery::is_word_a_query(Handle word_inst)
{
	return foreach_binary_link(word_inst, INHERITANCE_LINK, &WordRelQuery::is_qVar, this);
}

/* ======================================================== */
/* Routines to help put the query into normal form. */

/**
 * Return true, if the node is, for example, #singluar or #masculine.
 */
bool WordRelQuery::is_ling_cncpt(Atom *atom)
{
	if (DEFINED_LINGUISTIC_CONCEPT_NODE == atom->get_type()) return true;
	return false;
}

bool WordRelQuery::is_cncpt(Atom *atom)
{
	if (CONCEPT_NODE == atom->get_type()) return true;
	return false;
}

void WordRelQuery::add_to_predicate(Handle ah)
{
	/* scan for duplicates, and don't add them */
	HandleSeq::const_iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); ++i)
	{
		Handle h = *i;
		if (h == ah) return;
	}
	normed_predicate.push_back(ah);
}

void WordRelQuery::add_to_vars(Handle ah)
{
	/* scan for duplicates, and don't add them */
	HandleSeq::const_iterator i;
	for (i = bound_vars.begin();
	     i != bound_vars.end(); ++i)
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
bool WordRelQuery::find_vars(Handle word_instance)
{
	foreach_outgoing_handle(word_instance, &WordRelQuery::find_vars, this);

	bool qvar = is_word_a_query(word_instance);
	if (!qvar) return false;

	add_to_vars(word_instance);
	return false;
}

/* ======================================================== */
/* runtime matching routines */

/**
 * Do two word instances have the same word lemma (word root form)?
 * Return true if they are (that is, if they match).
 *
 * Current NLP structure relating word-instances to lemmas is:
 * (LemmaLink (stv 1.0 1.0)
 *    (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *    (WordNode "throw")
 * )
 * (ReferenceLink (stv 1.0 1.0)
 *    (WordInstanceNode "threw@e5649eb8-eac5-48ae-adab-41e351e29e4e")
 *    (WordNode "threw")
 * )
 */
bool WordRelQuery::word_instance_match(Atom *aa, Atom *ab)
{
	dbgprt("comp patt inst "); prt(aa);
	dbgprt("   to wrd inst "); prt(ab);

	// If they're the same atom, then clearly they match.
	if (aa == ab) return true;

	// Look for incoming links that are LemmaLinks.
	// The word lemma should be at the far end.
	Atom *ca = fl.follow_binary_link(aa, LEMMA_LINK);
	Atom *cb = fl.follow_binary_link(ab, LEMMA_LINK);

	dbgprt("gen comp %d ", ca==cb); prt(ca);
	dbgprt("        to "); prt(cb);

	return ca == cb;
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
const char * WordRelQuery::get_word_instance(Atom *atom)
{
	// We want the word-node associated with this word instance.
	Atom *wrd = fl.follow_binary_link(atom, REFERENCE_LINK);
	if (!wrd) return NULL;

	if (WORD_NODE != wrd->get_type()) return NULL;

	Node *n = static_cast<Node *>(wrd);
	const std::string& name = n->get_name();

	return name.c_str();
}
#endif /* DEAD_CODE_BUT_DONT_DELETE_JUST_YET */

/**
 * Are two nodes "equivalent", as far as the opencog representation
 * of RelEx expressions are concerned?
 *
 * Return false to signify a mismatch,
 * Return true to signify equivalence.
 */
bool WordRelQuery::node_match(Node *npat, Node *nsoln)
{
	// If we are here, then we are comparing nodes.
	// The result of comparing nodes depends on the
	// node types.
	Type pattype = npat->get_type();
	Type soltype = nsoln->get_type();
	if ((pattype != soltype) && 
	    (WORD_NODE != soltype) && 
	    (SEME_NODE != soltype) && 
	    (WORD_INSTANCE_NODE != soltype)) return false;

	// DefinedLinguisticRelation nodes must usually match exactly;
	// so if we are here, there's probably already a mismatch.
	if ((DEFINED_LINGUISTIC_RELATIONSHIP_NODE == soltype) ||
	    (PREPOSITIONAL_RELATIONSHIP_NODE == soltype))
	{
		const char *spat = npat->get_name().c_str();
		if (strcmp("isa", spat) && strcmp("hypothetical_isa", spat)) return false;
		const char *ssol = npat->get_name().c_str();
		if (strcmp("isa", ssol) && strcmp("hypothetical_isa", ssol)) return false;

		// If we got to here then we matched isa to isa or hypothetical_isa,
		// and that's all good.  Err.. XXX bad if not a question, but whatever.
		return true;
	}

	// Word instances match only if they have the same word lemma.
	if ((WORD_INSTANCE_NODE == soltype) ||
	    (WORD_NODE == soltype) || // XXX get rid of WordNode here, someday.
	    (SEME_NODE == soltype))
	{
		bool match = word_instance_match(npat, nsoln);
		dbgprt("word instance match=%d\n", mismatch);
		return match;
	}

	// XXX This code is never reached !!!??? 
	// This is a bit of dead code, which may need to be revived for more
	// proper relex matching ... or maybe not ... 
	if (DEFINED_LINGUISTIC_CONCEPT_NODE == soltype)
	{
		/* We force agreement for gender, etc.
		 * be have more relaxed agreement for tense...
		 * i.e. match #past to #past_infinitive, etc.
		 */
		const char * sa = npat->get_name().c_str();
		const char * sb = nsoln->get_name().c_str();
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
		return !strcmp(sa, sb);
	}

	fprintf(stderr, "Error: unexpected ground node type %d %s\n", soltype,
	        nameserver().getTypeName(soltype).c_str());

	std::string sa = npat->to_string();
	std::string sb = nsoln->to_string();
	fprintf (stderr, "unexpected comp %s\n"
	                 "             to %s\n", sa.c_str(), sb.c_str());

	return false;
}

/* ======================================================== */

bool WordRelQuery::solution(HandleMap &pred_grounding,
                          HandleMap &var_grounding)
{
	// Reject any solution where a variable is solved
	// by another variable (e.g. if there are multiple
	// questions in the corpus, and we just happened to
	// find one of them.)
	HandleMap::const_iterator j;
	for (j=var_grounding.begin(); j != var_grounding.end(); ++j)
	{
		HandlePair pv = *j;
		Handle soln = pv.second;

		// Solution is a word instance; is it also a query variable?
		bool qvar = is_word_a_query(soln);
		if (qvar) return false;
	}

	printf ("\nduude Found solution:");
	PatternMatchEngine::print_solution(var_grounding, pred_grounding);

	// And now for a cheesy hack to report the solution
	// XXX this needs to be replaced in the end, for now its just a cheesy
	// hack to pass data back to scheme.
	Handle hq = atom_space->addNode(ANCHOR_NODE, "# QUERY SOLUTION");

	if (0 < bound_vars.size())
	{
		Handle hv = bound_vars[0];
		Handle ha = var_grounding[hv];
	
		// FYI, we expect the grounding to be a SemeNode 
		// at this point, and that's what we return as the
		// answer.
		if (!atom_space->isNode(atom_space->getType(ha))) return false;
		printf("duude answer=%s\n", atom_space->get_name(ha).c_str());

		Handle hw = ha;
		atom_space->addLink(LIST_LINK, hq, hw);
	}
	else
	{
		// Cheesy hack to report "yes" to yes/no questions.
		Handle hw = atom_space->addNode(WORD_NODE, "yes");
		atom_space->addLink(LIST_LINK, hq, hw);
	}
	
	return false;
}

/* ===================== END OF FILE ===================== */
