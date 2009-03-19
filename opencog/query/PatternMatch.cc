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
#include "SimpleLogicPMCB.h"

#include <opencog/util/platform.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/util/Logger.h>

using namespace opencog;

PatternMatch::PatternMatch(void)
{
	atom_space = NULL;
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

	pme.match(cb, lclauses->getOutgoingSet(), lvarbles->getOutgoingSet());
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
/**
 * class Instantiator -- create grounded expressions from ungrounded ones.
 * Given an ungrounded expression (i.e. an expression containing variables)
 * and a map between variables and ground terms, it will create a new
 * expression, with the ground terms substituted for the variables.
 */
class Instantiator
{
	private:
		std::map<Handle, Handle> *vmap;

		std::vector<Handle> oset;
		bool walk_tree(Handle tree);

	public:
		AtomSpace *as;
		Handle instantiate(Handle expr, std::map<Handle, Handle> &vars);
};

bool Instantiator::walk_tree(Handle expr)
{
	Atom *a = TLB::getAtom(expr);
	Type t = a->getType();
	Node *n = dynamic_cast<Node *>(a);
	if (n)
	{
		if (VARIABLE_NODE != t)
		{
			oset.push_back(expr);
			return false;
		}

		// If we are here, we found a variable. Look it up.
		std::map<Handle,Handle>::const_iterator it = vmap->find(expr);
		if (vmap->end() != it)
		{
			Handle soln = it->second;
			oset.push_back(soln);
		}
		else
		{
			oset.push_back(expr);
		}
		return false;
	}

	// If we are here, then we have a link. Walk it.
	std::vector<Handle> save_oset = oset;
	oset.clear();

	// Walk the subtree, substituting values for variables.
	foreach_outgoing_handle(expr, &Instantiator::walk_tree, this);

	// Now create a duplicate link, but with an outgoing set where
	// the variables have been substituted by thier values.
	Handle sh = as->addLink(t, oset);

	oset = save_oset;
	oset.push_back(sh);

	return false;
}

/**
 * Given a handle to an ungrounded expression, and a set of groundings,
 * this will create a grounded expression.
 *
 * The set of groundings is to be passed in with the map 'vars', which
 * maps variable names to thier groundings -- it maps variable names to
 * atoms that already exist in the atomspace.  This method will then go
 * through all of the variables in the expression, and substitute them
 * with thier values, creating a new expression. The new expression is
 * added to the atomspace, and its handle is returned.
 */
Handle Instantiator::instantiate(Handle expr, std::map<Handle, Handle> &vars)
{
	vmap = &vars;
	oset.clear();
	walk_tree(expr);
	if (oset.size() != 1)
	{
		logger().warn("%s: outgoing set size %d is insane!",
			__FUNCTION__, oset.size());
	}
	return oset[0];
}

/* ================================================================= */

/**
 * class Implicator -- pattern matching callback for grounding implicands.
 *
 * This class is meant to be used with the pattern matcher. When the
 * pattern matcher calls the callback, it will do so with a particular
 * grounding of the search pattern. This class then holds an ungrounded
 * implicand, and will create a grounded version of the implicand.
 *
 * The 'var_soln' argument in the callback contains the map from variables
 * to ground terms. 'class Instantiator' is used to perform the actual
 * grounding.  A list of grounded expressions is created in 'result_list'.
 */
class Implicator :
	public virtual PatternMatchCallback
{
	private:
		Instantiator inst;
	public:
		AtomSpace *as;
		Handle implicand;
		std::vector<Handle> result_list;
		virtual bool solution(std::map<Handle, Handle> &pred_soln,
		                      std::map<Handle, Handle> &var_soln);
};

bool Implicator::solution(std::map<Handle, Handle> &pred_soln,
                          std::map<Handle, Handle> &var_soln)
{
	// PatternMatchEngine::print_solution(pred_soln,var_soln);
	inst.as = as;
	Handle h = inst.instantiate(implicand, var_soln);
	result_list.push_back(h);
	return false;
}

/* ================================================================= */
/**
 * Evaluate an ImplicationLink.
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
 * implicand.  All variables are implicit, and are identified by being
 * VariableNodes.  These variables are interpreted as 'free variables'
 * having no binding.  The act of pattern-matching to the predicate of
 * the implication has an implicit 'for-all' flavour to it: the pattern
 * is matched to 'all' matches in the atomspace.
 *
 * When a pattern match is found, the variables can be understood as
 * being grounded by some explicit ground terms in the atomspace. This
 * grounding is then used to create a grounded version of the
 * (ungrounded) implicand. That is, the variables in the implicand are
 * substituted by thier grounding values.  This method then returns a
 * list of all of the grounded implicands that were created.
 *
 * Note that this method can be used to create a simple forward-chainer:
 * One need only to take a set of implication links, and call this
 * method repeatedly on them, until one is exhausted.
 */

Handle PatternMatch::do_imply (Handle himplication, PatternMatchCallback *pmc)
{
	Atom * aimpl = TLB::getAtom(himplication);
	Link * limpl = dynamic_cast<Link *>(aimpl);

	// Must be non-empty.
	if (!limpl) return Handle::UNDEFINED;

	// Type must be as expected
	Type timpl = limpl->getType();
	if (IMPLICATION_LINK != timpl)
	{
		logger().warn("%s: expected ImplicationLink", __FUNCTION__);
		return Handle::UNDEFINED;
	}

	const std::vector<Handle>& oset = limpl->getOutgoingSet();
	if (2 != oset.size())
	{
		logger().warn("%s: ImplicationLink has wrong size", __FUNCTION__);
		return Handle::UNDEFINED;
	}

	Handle hclauses = oset[0];
	Handle implicand = oset[1];

	Atom * aclauses = TLB::getAtom(hclauses);
	Link * lclauses = dynamic_cast<Link *>(aclauses);

	// Must be non-empty.
	if (!lclauses) return Handle::UNDEFINED;

	// Types must be as expected
	Type tclauses = lclauses->getType();
	if (AND_LINK != tclauses)
	{
		logger().warn("%s: expected AndLink for clause list", __FUNCTION__);
		return Handle::UNDEFINED;
	}

	// Inpute is in conjunctive normal form, consisting of clasues,
	// or thier negations. Split these into two distinct lists.
	// Any clause that is a NotLink is "negated"; strip off the 
	// negation and put it into its own list.
	const std::vector<Handle>& cset = lclauses->getOutgoingSet();
	std::vector<Handle> affirm, negate;
	size_t clen = cset.size();
	for (size_t i=0; i<clen; i++)
	{
		Handle h = cset[i];
		Atom *a = TLB::getAtom(h);
		Type t = a->getType();
		if (NOT_LINK == t)
		{
			Link *l = static_cast<Link *>(a);
			h = l->getOutgoingHandle(0);
			negate.push_back(h);
		}
		else
		{
			affirm.push_back(h);
		}
	}

	// Extract a list of variables.
	FindVariables fv;
	fv.find_vars(hclauses);

	// Now perform the search.
	Implicator *impl = dynamic_cast<Implicator *>(pmc);
	impl->implicand = implicand;
	pme.match(pmc, affirm, fv.varlist);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = atom_space->addLink(LIST_LINK, impl->result_list);

	return gl;
}

class DefaultImplicator:
	public virtual Implicator,
	public virtual DefaultPatternMatchCB
{};

Handle PatternMatch::imply (Handle himplication)
{
	// Now perform the search.
	DefaultImplicator impl;
	impl.as = atom_space;
	return do_imply(himplication, &impl);
}

class LogicImplicator:
	public virtual Implicator,
	public virtual SimpleLogicPMCB
{};

Handle PatternMatch::logic_imply (Handle himplication)
{
	// Now perform the search.
	LogicImplicator impl;
	impl.as = atom_space;
	return do_imply(himplication, &impl);
}

/* ===================== END OF FILE ===================== */
