/*
 * PatternMatch.cc
 *
 * Copyright (C) 2009 Linas Vepstas
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

#include "PatternMatch.h"
#include "DefaultPatternMatchCB.h"

#include <opencog/util/platform.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/Logger.h>

using namespace opencog;

PatternMatch::PatternMatch(void)
{
}

/* ================================================================= */
/**
 * Solve a predicate by pattern matching.
 * The predicate is defined in terms of two hypergraphs: one is a
 * hypergraph defining a pattern to be matched for, and the other is a
 * list of bound variables in the first.
 *
 * The bound variables are, by definition, nodes. (XXX It might be
 * useful to loosen this restriction someday). The list of bound variables
 * is then assumed to be listed using the ListLink type. So, for
 * example:
 *
 *    ListLink
 *        SomeNode "variable 1"
 *        SomeOtherNode "another variable"
 *
 * The predicate hypergraph is assumed to be a list of "clauses", where
 * each "clause" is a tree. The clauses are assumed to be connected,
 * i.e. share common nodes or links.  The algorithm to find solutions
 * will fail on disconnected hypergraphs.  The list of clauses is
 * specified by means of an AndLink, so, for example:
 *
 *     AndLink
 *        SomeLink ....
 *        SomeOtherLink ...
 *
 * The solution proceeds by requiring each clause to match some part of
 * the atomspace (i.e. of the universe of hypergraphs stored in the
 * atomspace). When a solution is found, PatternMatchCallback::solution
 * method is called, and it is passed two maps: one mapping the bound
 * variables to thier solutions, and the other mapping the pattern
 * clauses to thier corresponding solution clauses.
 *
 * At this time, the list of clauses is understood to be a single
 * disjunct; that is, all of the clauses must be simultaneously
 * satisfied.  Thus, in principle, one could build a layer on top of
 * this that accepts clauses in disjunctive normal form (and so on...)
 * It is not clear at this time how to benefit from Boolean SAT solver
 * technlogy (or at what point this would be needed).
 */
void PatternMatch::match(PatternMatchCallback *cb,
                         Handle hclauses,
                         Handle hvarbles)
{
	Atom * aclauses = TLB::getAtom(hclauses);
	Atom * avarbles = TLB::getAtom(hvarbles);
	Link * lclauses = dynamic_cast<Link *>(aclauses);
	Link * lvarbles = dynamic_cast<Link *>(avarbles);

	// Both must be non-empty.
	if (!lclauses || !lvarbles) return;

	// Types must be as expected
	Type tclauses = lclauses->getType();
	Type tvarbles = lvarbles->getType();
	if (AND_LINK != tclauses)
	{
		logger().warn("%s: expected AndLink for clause list", __FUNCTION__);
		return;
	}
	if (LIST_LINK != tvarbles)
	{
		logger().warn("%s: expected ListLink for bound variable list", __FUNCTION__);
		return;
	}

	PatternMatchEngine::match(cb, lclauses->getOutgoingSet(),
	                              lvarbles->getOutgoingSet());
}

/* ================================================================= */
// Handy dandy utility class.
//
class FindVariables
{
	public:
		std::vector<Handle> varlist;

		/**
		 * Create a list of all of the VariableNodes that lie in the
		 * outgoing set of the handle (recursively).
		 */
		inline bool find_vars(Handle h)
		{
			Atom *a = TLB::getAtom(h);
			Node *n = dynamic_cast<Node *>(a);
			if (n)
			{
				if (n->getType() == VARIABLE_NODE)
				{
					varlist.push_back(h);
				}
				return false;
			}

			return foreach_outgoing_handle(h, &FindVariables::find_vars, this);
		}
};

/* ================================================================= */
// Instantiator.

class Instantiator :
	public DefaultPatternMatchCB
{
	public:
		virtual bool solution(std::map<Handle, Handle> &pred_soln,
		                      std::map<Handle, Handle> &var_soln);
};

bool Instantiator::solution(std::map<Handle, Handle> &pred_soln,
                            std::map<Handle, Handle> &var_soln)
{
	return false;
}

/* ================================================================= */
/**
 * Evaluate an ImplicationLink.
 * Given an ImplicationLink, this method will "evaluate" it, matching
 * the predicate, and creating the implicand, assuming the predicate can
 * be satisfied. Thus, for example, given the structure
 *
 *    ImplicationLink
 *       AndList
 *          EvaluationList
 *             PredicateNode "_obj"
 *             ListLink
 *                ConceptNode "make"
 *                VariableNode $var0
 *          EvaluationList
 *             PredicateNode "from"
 *             ListLink
 *                ConceptNode "make"
 *                VariableNode $var1"
 *       EvaluationList
 *          PredicateNode "make_from"
 *          ListLink
 *             VariableNode $var0
 *             VariableNode $var1
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
 * These bindings are refered to as the 'solutions' to the variables.
 * So, e.g. $var0 is 'solved' by "pottery".
 *
 * Next, the implicand is then created; that is, the following hypergraph
 * is created and added to the atomspace:
 *
 *    EvaluationList
 *       PredicateNode "make_from"
 *       ListLink
 *          ConceptNode "pottery"
 *          ConceptNode "clay"
 *
 * As the above example illustrates, this function expects that the
 * input handle is an implication link. It expects the implication link
 * to consist entirely of one disjunct (one AndList) and one implicand.
 * All variables are implicit, and are identified by VariableNodes (they
 * are not explicitly called out). Variable substitution in the
 * implicand copies over the types of the solutions to the variables.
 */

void PatternMatch::imply (Handle himplication)
{
	Atom * aimpl = TLB::getAtom(himplication);
	Link * limpl = dynamic_cast<Link *>(aimpl);

	// Must be non-empty.
	if (!limpl) return;

	// Type must be as expected
	Type timpl = limpl->getType();
	if (IMPLICATION_LINK != timpl)
	{
		logger().warn("%s: expected ImplicationLink", __FUNCTION__);
		return;
	}

	const std::vector<Handle>& oset = limpl->getOutgoingSet();
	if (2 != oset.size())
	{
		logger().warn("%s: ImplicationLink has wrong size", __FUNCTION__);
		return;
	}

	Handle hclauses = oset[0];
	Handle implicand = oset[1];

	Atom * aclauses = TLB::getAtom(hclauses);
	Link * lclauses = dynamic_cast<Link *>(aclauses);

	// Must be non-empty.
	if (!lclauses) return;

	// Types must be as expected
	Type tclauses = lclauses->getType();
	if (AND_LINK != tclauses)
	{
		logger().warn("%s: expected AndLink for clause list", __FUNCTION__);
		return;
	}

	// Extract a list of variables.
	FindVariables fv;
	fv.find_vars(hclauses);

	// Now perform the search.
	Instantiator inst;
	PatternMatchEngine::match(&inst, lclauses->getOutgoingSet(),
	                                 fv.varlist);
}

/* ===================== END OF FILE ===================== */
