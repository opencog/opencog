/*
 * PatternMatch.cc
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
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

#include <opencog/atoms/bind/BindLink.h>
#include <opencog/atomutils/PatternUtils.h>
#include <opencog/util/Logger.h>

#include "PatternMatch.h"

using namespace opencog;

/* ================================================================= */
/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Given a BindLink containing variable declarations and an
 * ImplicationLink, this method will "evaluate" the implication,
 * matching
 * the predicate, and creating a grounded implicand, assuming the
 * predicate can be satisfied. Thus, for example, given the structure
 *
 *    BindLink
 *       ListLink
 *          VariableNode "$var0"
 *          VariableNode "$var1"
 *       ImplicationLink
 *          AndList
 *             etc ...
 *
 * Evaluation proceeds as decribed in the "do_imply()" function below.
 * The whole point of the BindLink is to do nothing more than
 * to indicate the bindings of the variables, and (optionally) limit
 * the types of acceptable groundings for the variables.
 */
void BindLink::imply(PatternMatchCallback* pmc, bool check_conn)
{
   if (check_conn) check_connectivity(_components);
   PatternMatch::do_match(pmc, _varset, _virtuals, _components);
}

void SatisfactionLink::satisfy(PatternMatchCallback* pmc)
{
   PatternMatch::do_match(pmc, _varset, _virtuals, _components);
}

/* ================================================================= */

/// See the documentation for do_match() to see what this function does.
/// This is just a convenience wrapper around do_match().
void opencog::match(PatternMatchCallback *cb,
                    const Handle& hvarbles,
                    const Handle& hclauses)
{
	// Empty implicand...
	HandleSeq empty;
	Handle hcand(createLink(LIST_LINK, empty));

	HandleSeq oset;
	oset.push_back(hclauses);
	oset.push_back(hcand);
	Handle himpl(createLink(IMPLICATION_LINK, oset));

	HandleSeq boset;
	boset.push_back(hvarbles);
	boset.push_back(himpl);

	BindLinkPtr bl(createBindLink(BIND_LINK, boset));

	bl->imply(cb);
}

/* ================================================================= */
/**
 * do_imply -- Evaluate an ImplicationLink.
 *
 * Given an ImplicationLink, this method will "evaluate" it, matching
 * the predicate, and creating a grounded implicand, assuming the
 * predicate can be satisfied. Thus, for example, given the structure
 *
 *    ImplicationLink
 *       AndList
 *          EvaluationList
 *             PredicateNode "_obj"
 *             ListLink
 *                ConceptNode "make"
 *                VariableNode "$var0"
 *          EvaluationList
 *             PredicateNode "from"
 *             ListLink
 *                ConceptNode "make"
 *                VariableNode "$var1"
 *       EvaluationList
 *          PredicateNode "make_from"
 *          ListLink
 *             VariableNode "$var0"
 *             VariableNode "$var1"
 *
 * Then, if the atomspace also contains a parsed version of the English
 * sentence "Pottery is made from clay", that is, if it contains the
 * hypergraph
 *
 *    EvaluationList
 *       PredicateNode "_obj"
 *       ListLink
 *          ConceptNode "make"
 *          ConceptNode "pottery"
 *
 * and the hypergraph
 *
 *    EvaluationList
 *       PredicateNode "from"
 *       ListLink
 *          ConceptNode "make"
 *          ConceptNode "clay"
 *
 * Then, by pattern matching, the predicate part of the ImplicationLink
 * can be fulfilled, binding $var0 to "pottery" and $var1 to "clay".
 * These bindings are refered to as the 'groundings' or 'solutions'
 * to the variables. So, e.g. $var0 is 'grounded' by "pottery".
 *
 * Next, a grounded copy of the implicand is then created; that is,
 * the following hypergraph is created and added to the atomspace:
 *
 *    EvaluationList
 *       PredicateNode "make_from"
 *       ListLink
 *          ConceptNode "pottery"
 *          ConceptNode "clay"
 *
 * As the above example illustrates, this function expects that the
 * input handle is an implication link. It expects the implication link
 * to consist entirely of one disjunct (one AndList) and one (ungrounded)
 * implicand.  The variables are explicitly declared in the 'varlist'
 * argument to this function. These variables should be understood as
 * 'bound variables' in the usual sense of lambda-calculus. (It is
 * strongly suggested that variables always be declared as VariableNodes;
 * there are several spots in the code where this is explicitly assumed,
 * and declaring some other node type as a vaiable may lead to
 * unexpected results.)
 *
 * Pattern-matching proceeds by finding groundings for these variables.
 * When a pattern match is found, the variables can be understood as
 * being grounded by some explicit terms in the atomspace. This
 * grounding is then used to create a grounded version of the
 * (ungrounded) implicand. That is, the variables in the implicand are
 * substituted by their grounding values.  This method then returns a
 * list of all of the grounded implicands that were created.
 *
 * The act of pattern-matching to the predicate of the implication has
 * an implicit 'for-all' flavour to it: the pattern is matched to 'all'
 * matches in the atomspace.  However, with a suitably defined
 * PatternMatchCallback, the search can be terminated at any time, and
 * so this method can be used to implement a 'there-exists' predicate,
 * or any quantifier whatsoever.
 *
 * Note that this method can be used to create a simple forward-chainer:
 * One need only to take a set of implication links, and call this
 * method repeatedly on them, until one is exhausted.
 */

/// Deprecated; do not use in new code.
/// This is used only in test cases.
void opencog::do_imply (const Handle& himplication,
                        Implicator &impl)
{
	LinkPtr limp(LinkCast(himplication));
	OC_ASSERT(limp != NULL, "Bad implication link");

	Handle hclauses = limp->getOutgoingAtom(0);

	// Extract the variables; the were not specified.
	FindVariables fv(VARIABLE_NODE);
	fv.find_vars(hclauses);

	HandleSeq vars;
	for (Handle h : fv.varset)
	{
		vars.push_back(h);
	}

	// Stuff the variables into a proper variable list.
	Handle hvars(createLink(VARIABLE_LIST, vars));

	HandleSeq oset;
	oset.push_back(hvars);
	oset.push_back(himplication);

	BindLinkPtr bl(createBindLink(BIND_LINK, oset));

	// Now perform the search.
	impl.implicand = limp->getOutgoingAtom(1);

	bl->imply(&impl);
}

/* ===================== END OF FILE ===================== */
