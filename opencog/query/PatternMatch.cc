/*
 * PatternMatch.cc
 *
 * Copyright (C) 2009, 2014 Linas Vepstas
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

#include <opencog/atomspace/ClassServer.h>
#include <opencog/util/Logger.h>

#include "PatternMatch.h"
#include "PatternUtils.h"

using namespace opencog;

PatternMatch::PatternMatch(void) {}

/// See the documentation for do_match() to see what this function does.
/// This is just a convenience wrapper around do_match().
void PatternMatch::match(PatternMatchCallback *cb,
                         Handle hvarbles,
                         Handle hclauses,
                         Handle hnegates)
	throw (InvalidParamException)
{
	// Both must be non-empty.
	LinkPtr lclauses(LinkCast(hclauses));
	LinkPtr lvarbles(LinkCast(hvarbles));
	if (NULL == lclauses or NULL == lvarbles) return;

	// Types must be as expected
	Type tvarbles = hvarbles->getType();
	Type tclauses = hclauses->getType();
	if (LIST_LINK != tvarbles)
		throw InvalidParamException(TRACE_INFO,
			"Expected ListLink for bound variable list.");

	if (AND_LINK != tclauses)
		throw InvalidParamException(TRACE_INFO,
			"Expected AndLink for clause list.");

	// negation clauses are optionally present
	std::vector<Handle> negs;
	LinkPtr lnegates(LinkCast(hnegates));
	if (lnegates)
	{
		if (AND_LINK != lnegates->getType())
			throw InvalidParamException(TRACE_INFO,
				"Expected AndLink holding negated/otional clauses.");
		negs = lnegates->getOutgoingSet();
	}

	std::set<Handle> vars;
	for (Handle v: lvarbles->getOutgoingSet()) vars.insert(v);

	std::vector<Handle> clauses(lclauses->getOutgoingSet());

	do_match(cb, vars, clauses, negs);
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

void PatternMatch::do_imply (Handle himplication,
                             Implicator &impl,
                             std::set<Handle>& varset)
	throw (InvalidParamException)
{
	// Must be non-empty.
	LinkPtr limplication(LinkCast(himplication));
	if (NULL == limplication)
		throw InvalidParamException(TRACE_INFO,
			"Expected ImplicationLink");

	// Type must be as expected
	Type timpl = himplication->getType();
	if (IMPLICATION_LINK != timpl)
		throw InvalidParamException(TRACE_INFO,
			"Expected ImplicationLink");

	const std::vector<Handle>& oset = limplication->getOutgoingSet();
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"ImplicationLink has wrong size: %d", oset.size());

	Handle hclauses(oset[0]);
	Handle implicand(oset[1]);

	// Must be non-empty.
	LinkPtr lclauses(LinkCast(hclauses));
	if (NULL == lclauses)
		throw InvalidParamException(TRACE_INFO,
			"Expected non-empty set of clauses in the ImplicationLink");

	// The predicate is either an AndList, or a single clause
	// If its an AndList, then its a list of clauses.
	// XXX FIXME Perhaps, someday, some sort of embedded OrList should
	// be supported, allowing several different patterns to be matched
	// in one go. But not today, this is complex and low priority. See
	// the README for slighly more detail
	std::vector<Handle> affirm, negate;
	Type tclauses = hclauses->getType();
	if (AND_LINK == tclauses)
	{
		// Input is in conjunctive normal form, consisting of clauses,
		// or their negations. Split these into two distinct lists.
		// Any clause that is a NotLink is "negated"; strip off the
		// negation and put it into its own list.
		const std::vector<Handle>& cset = lclauses->getOutgoingSet();
		size_t clen = cset.size();
		for (size_t i=0; i<clen; i++)
		{
			Handle h(cset[i]);
			Type t = h->getType();
			if (NOT_LINK == t)
			{
				negate.push_back(LinkCast(h)->getOutgoingAtom(0));
			}
			else
			{
				affirm.push_back(h);
			}
		}
	}
	else
	{
		// There's just one single clause!
		affirm.push_back(hclauses);
	}

	// Extract the set of variables, if needed.
	// This is used only by the deprecated imply() function, as the
	// BindLink will include a list of variables up-front.
	if (0 == varset.size())
	{
		FindVariables fv;
		fv.find_vars(hclauses);
		varset = fv.varset;
	}

	// Now perform the search.
	impl.implicand = implicand;
	do_match(&impl, varset, affirm, negate);
}

/* ================================================================= */
typedef std::pair<Handle, const std::set<Type> > ATPair;

/**
 * Extract the variable type(s) from a TypedVariableLink
 *
 * The call is expecting htypelink to point to one of the two
 * following structures:
 *
 *    TypedVariableLink
 *       VariableNode "$some_var_name"
 *       VariableTypeNode  "ConceptNode"
 *
 * or
 *
 *    TypedVariableLink
 *       VariableNode "$some_var_name"
 *       ListLink
 *          VariableTypeNode  "ConceptNode"
 *          VariableTypeNode  "NumberNode"
 *          VariableTypeNode  "WordNode"
 *
 * In either case, the variable itself is appended to "vset",
 * and the list of allowed types are associated with the variable
 * via the map "typemap".
 */
int PatternMatch::get_vartype(Handle htypelink,
                              std::set<Handle> &vset,
                              VariableTypeMap &typemap)
{
	const std::vector<Handle>& oset = LinkCast(htypelink)->getOutgoingSet();
	if (2 != oset.size())
	{
		logger().warn("%s: TypedVariableLink has wrong size",
		       __FUNCTION__);
		return 1;
	}

	Handle varname = oset[0];
	Handle vartype = oset[1];

	// The vartype is either a single type name, or a list of typenames.
	Type t = vartype->getType();
	if (VARIABLE_TYPE_NODE == t)
	{
		const std::string &tn = NodeCast(vartype)->getName();
		Type vt = classserver().getType(tn);

		if (NOTYPE == vt)
		{
			logger().warn("%s: VariableTypeNode specifies unknown type: %s\n",
			               __FUNCTION__, tn.c_str());
			return 4;
		}

		std::set<Type> ts;
		ts.insert(vt);
		typemap.insert(ATPair(varname,ts));
		vset.insert(varname);
	}
	else if (LIST_LINK == t)
	{
		std::set<Type> ts;

		const std::vector<Handle>& tset = LinkCast(vartype)->getOutgoingSet();
		size_t tss = tset.size();
		for (size_t i=0; i<tss; i++)
		{
			Handle h(tset[i]);
			if (VARIABLE_TYPE_NODE != h->getType())
			{
				logger().warn("%s: TypedVariableLink has unexpected content:\n"
				              "Expected VariableTypeNode, got %s",
				              __FUNCTION__,
				              classserver().getTypeName(h->getType()).c_str());
				return 3;
			}
			const std::string &tn = NodeCast(h)->getName();
			Type vt = classserver().getType(tn);
			if (NOTYPE == vt)
			{
				logger().warn("%s: VariableTypeNode specifies unknown type: %s\n",
				               __FUNCTION__, tn.c_str());
				return 5;
			}
			ts.insert(vt);
		}

		typemap.insert(ATPair(varname,ts));
		vset.insert(varname);
	}
	else
	{
		logger().warn("%s: Unexpected contents in TypedVariableLink\n"
				        "Expected VariableTypeNode or ListLink, got %s",
		              __FUNCTION__,
		              classserver().getTypeName(t).c_str());
		return 2;
	}

	return 0;
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

void PatternMatch::do_bindlink (Handle hbindlink,
                                Implicator& implicator)
	throw (InvalidParamException)
{
	// Must be non-empty.
	LinkPtr lbl(LinkCast(hbindlink));
	if (NULL == lbl)
		throw InvalidParamException(TRACE_INFO,
			"Expecting a BindLink");

	// Type must be as expected
	Type tscope = hbindlink->getType();
	if (BIND_LINK != tscope)
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a BindLink, got %s", tname.c_str());
	}

	const std::vector<Handle>& oset = lbl->getOutgoingSet();
	if (2 != oset.size())
		throw InvalidParamException(TRACE_INFO,
			"BindLink has wrong size %d", oset.size());

	Handle hdecls(oset[0]);  // VariableNode declarations
	Handle himpl(oset[1]);   // ImplicationLink

	// vset is the vector of variables.
	// typemap is the (possibly empty) list of restrictions on atom types.
	std::set<Handle> vset;
	VariableTypeMap typemap;

	// Expecting the declaration list to be either a single
	// variable, or a list of variable declarations
	Type tdecls = hdecls->getType();
	if ((VARIABLE_NODE == tdecls) or
	    NodeCast(hdecls)) // allow *any* node as a variable
	{
		vset.insert(hdecls);
	}
	else if (TYPED_VARIABLE_LINK == tdecls)
	{
		if (get_vartype(hdecls, vset, typemap))
			throw InvalidParamException(TRACE_INFO,
				"Cannot understand the typed variable definition");
	}
	else if (LIST_LINK == tdecls)
	{
		// The list of variable declarations should be .. a list of
		// variables! Make sure its as expected.
		const std::vector<Handle>& dset = LinkCast(hdecls)->getOutgoingSet();
		size_t dlen = dset.size();
		for (size_t i=0; i<dlen; i++)
		{
			Handle h(dset[i]);
			Type t = h->getType();
			if (VARIABLE_NODE == t)
			{
				vset.insert(h);
			}
			else if (TYPED_VARIABLE_LINK == t)
			{
				if (get_vartype(h, vset, typemap))
					throw InvalidParamException(TRACE_INFO,
						"Don't understand the TypedVariableLink");
			}
			else
				throw InvalidParamException(TRACE_INFO,
					"Expected a VariableNode or a TypedVariableLink");
		}
	}
  	else
	{
		throw InvalidParamException(TRACE_INFO,
			"Expected a ListLink holding variable declarations");
	}

	implicator.set_type_restrictions(typemap);
	do_imply(himpl, implicator, vset);
}

void PatternMatch::do_imply (Handle himplication,
                             Implicator &impl)
	throw (InvalidParamException)
{
	std::set<Handle> varset;
	do_imply(himplication, impl, varset);
}

/* ===================== END OF FILE ===================== */
