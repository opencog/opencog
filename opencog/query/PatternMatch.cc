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

#include <opencog/atomutils/PatternUtils.h>
#include <opencog/util/Logger.h>

#include "PatternMatch.h"

using namespace opencog;

PatternMatch::PatternMatch(void)
	: BindLink(BIND_LINK, HandleSeq()), _used(false) {}

/// See the documentation for do_match() to see what this function does.
/// This is just a convenience wrapper around do_match().
void PatternMatch::match(PatternMatchCallback *cb,
                         const Handle& hvarbles,
                         const Handle& hclauses)
{
	// Both must be non-empty.
	LinkPtr lclauses(LinkCast(hclauses));
	LinkPtr lvarbles(LinkCast(hvarbles));
	if (NULL == lclauses or NULL == lvarbles) return;

	// Types must be as expected
	Type tvarbles = hvarbles->getType();
	Type tclauses = hclauses->getType();
	if (VARIABLE_LIST != tvarbles and LIST_LINK != tvarbles)
		throw InvalidParamException(TRACE_INFO,
			"Expected VariableList for the bound variable list.");

	if (AND_LINK != tclauses)
		throw InvalidParamException(TRACE_INFO,
			"Expected AndLink for clause list.");

	std::set<Handle> vars;
	for (Handle v: lvarbles->getOutgoingSet()) vars.insert(v);

	std::vector<Handle> clauses(lclauses->getOutgoingSet());

	do_match(cb, vars, clauses);
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

void PatternMatch::do_imply (const Handle& himplication,
                             Implicator &impl,
                             std::set<Handle>& varset)
{
	// Extract the set of variables, if needed.
	// This is used only by the deprecated imply() function, as the
	// BindLink will include a list of variables up-front.
	if (varset.empty())
	{
		FindVariables fv(VARIABLE_NODE);
		fv.find_vars(_hclauses);
		varset = fv.varset;
	}

	// Now perform the search.
	impl.implicand = _implicand;
	do_match(&impl, varset, _clauses);
}

/* ================================================================= */
/**
 * Validate the body for syntax correctness.
 *
 * Given an ImplicatioLink, this will check to make sure that
 * it is of the appropriate structure: that it consists of two
 * parts: a set of clauses, and an implicand.  That is, it must
 * have the structure:
 *
 *    ImplicationLink
 *       SomeLink
 *       AnotherLink
 *
 * The conetns of "SomeLink" is not validated here, it is
 * validated by validate_clauses()
 *
 * As a side-effect, if SomeLink is an AndLink, the list of clauses
 * is unpacked.
 */
void PatternMatch::validate_body(const Handle& hbody)
{
	// Type must be as expected
	Type tbody = hbody->getType();
	if (IMPLICATION_LINK == tbody)
	{
		LinkPtr lbody(LinkCast(hbody));
		const std::vector<Handle>& oset = lbody->getOutgoingSet();
		if (2 != oset.size())
			throw InvalidParamException(TRACE_INFO,
				"ImplicationLink has wrong size: %d", oset.size());
		_hclauses = oset[0];
		_implicand = oset[1];
		return;
	}

	_hclauses = hbody;
}

void PatternMatch::unbundle_clauses(const Handle& clauses)
{
	// The predicate is either an AndList, or a single clause
	// If its an AndList, then its a list of clauses.
	// XXX FIXME Perhaps, someday, some sort of embedded OrList should
	// be supported, allowing several different patterns to be matched
	// in one go. But not today, this is complex and low priority. See
	// the README for slighly more detail
	Type tclauses = clauses->getType();
	if (AND_LINK == tclauses)
	{
		_clauses = LinkCast(_hclauses)->getOutgoingSet();
	}
	else
	{
		// There's just one single clause!
		_clauses.push_back(clauses);
	}
}

/* ================================================================= */
/**
 * Run the full validation suite.
 */
void PatternMatch::validate(const Handle& hbindlink)
{
	unbundle_body(hbindlink);
	validate_vardecl(_vardecl);
	validate_body(_body);
	unbundle_clauses(_hclauses);
	validate_clauses(_varset, _clauses);
}

/* ================================================================= */
/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Given a BindLink containing variable declarations and an
 * ImplicationLink, this method will "evaluate" the implication, matching
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
 * Evaluation proceeds as decribed in the "do_imply()" function above.
 * The whole point of the BindLink is to do nothing more than
 * to indicate the bindings of the variables, and (optionally) limit
 * the types of acceptable groundings for the varaibles.
 */

void PatternMatch::do_bindlink (const Handle& hbindlink,
                                Implicator& implicator)
{
	unbundle_body(hbindlink);
	validate_vardecl(_vardecl);
	validate_body(_body);
	unbundle_clauses(_hclauses);

	implicator.set_type_restrictions(_typemap);
	do_imply(_body, implicator, _varset);
}

/**
 * Perform a satisfaction check only, no implication is performed.
 *
 */
void PatternMatch::do_satlink (const Handle& hsatlink,
                               Satisfier& sater)
{
	unbundle_body(hsatlink);
	validate_vardecl(_vardecl);
	validate_body(_body);
	unbundle_clauses(_hclauses);

	do_match(&sater, _varset, _clauses);
}

/// Deprecated; do not use in new code.
/// This is used only in test cases.
void PatternMatch::do_imply (const Handle& himplication,
                             Implicator &impl)
{
	std::set<Handle> varset;
	Type timplication = himplication->getType();
	if (IMPLICATION_LINK != timplication)
		throw InvalidParamException(TRACE_INFO,
			"Expecting an ImplicationLink, got: %d", timplication);

	validate_body(himplication);
	unbundle_clauses(_hclauses);
	do_imply(himplication, impl, varset);
}

/* ===================== END OF FILE ===================== */
