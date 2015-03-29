/*
 * SatisfactionLink.cc
 *
 * Copyright (C) 2009, 2014, 2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomutils/PatternUtils.h>

#include "SatisfactionLink.h"

using namespace opencog;

SatisfactionLink::SatisfactionLink(Type t, const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(t, hseq, tv, av)
{
	if (not classserver().isA(t, SATISFACTION_LINK))
	{
		const std::string& tname = classserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a SatsifactionLink, got %s", tname.c_str());
	}

	// BindLink has a different clause sequence
	if (SATISFACTION_LINK != t) return;

	// The LambdaLink contructor sets up _body and _varset
	_hclauses = _body;
	unbundle_clauses(_hclauses);
	validate_clauses(_varset, _clauses);
}

SatisfactionLink::SatisfactionLink(Link &l)
	: LambdaLink(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, SATISFACTION_LINK))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a SatsifactionLink, got %s", tname.c_str());
	}

	// BindLink has a different clause sequence
	if (SATISFACTION_LINK != tscope) return;

	// The LambdaLink contructor sets up _body and _varset
	_hclauses = _body;
	unbundle_clauses(_hclauses);
	validate_clauses(_varset, _clauses);
}


/* ================================================================= */

void SatisfactionLink::unbundle_clauses(const Handle& clauses)
{
	// The predicate is either an AndList, or a single clause
	// If its an AndList, then its a list of clauses.
	// XXX FIXME Perhaps, someday, some sort of embedded OrList should
	// be supported, allowing several different patterns to be matched
	// in one go. But not today, this is complex and low priority. See
	// the README for slighly more detail
	if (AND_LINK == clauses->getType())
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
 * Validate a collection of clauses and negations for correctness.
 *
 * Every clause should contain at least one variable in it; clauses
 * that are constants and can be trivially discarded.
 * Furthermore, all clauses should be connected. Two clauses are
 * connected if they contain a common variable.
 *
 * As a side effect, this looks for 'virtual links' and separates
 * them out into a distinct list, _virtuals and _nonvirts.
 *
 * It also partition the clauses into a set of connected components,
 * _components.
 */
void SatisfactionLink::validate_clauses(std::set<Handle>& vars,
                                        std::vector<Handle>& clauses)

{
	// Make sure that the user did not pass in bogus clauses.
	// Make sure that every clause contains at least one variable.
	// The presence of constant clauses will mess up the current
	// pattern matcher.  Constant clauses are "trivial" to match,
	// and so its pointless to even send them through the system.
	bool bogus = remove_constants(vars, clauses);
	if (bogus)
	{
		logger().warn("%s: Constant clauses removed from pattern matching",
		              __FUNCTION__);
	}

	// Make sure that each declared variable appears in some clause.
	// We can't ground variables that aren't attached to something.
	// Quoted variables are constants, and so don't count.
	for (Handle v : vars)
	{
		if (not is_variable_in_any_tree(clauses, v))
			throw InvalidParamException(TRACE_INFO,
				"The variable %s does not appear (unquoted) in any clause!",
				v->toString().c_str());
	}

	// Are there any virtual links in the clauses? If so, then we need
	// to do some special handling.  BTW: a clause is virtual only if
	// it contains a GroundedPredicate, and the GroundedPredicate takes
	// an argument that contains a variable. Otherwise, its not really
	// virtual.
	//
	// The GreaterThanLink is a link type that implicitly contains
	// a GroundedPredicate for numeric greater-than relations. So
	// we search for that too.
	//
	// XXX FIXME, the check below is not quite correct; for example,
	// it would tag the following as virtual, although it is not:
	// (BlahLink (VariableNode "$var") (EvaluationLink (GPN "scm:duh")
	// (ListLink (ConceptNode "Stuff"))))  -- the var is there but not
	// in the GPN.
	for (Handle clause: clauses)
	{
		if ((contains_atomtype(clause, GROUNDED_PREDICATE_NODE)
		    or contains_atomtype(clause, GREATER_THAN_LINK))
		    and any_variable_in_tree(clause, vars))
			_virtuals.push_back(clause);
		else
			_nonvirts.push_back(clause);
	}

#ifdef I_DONT_THINK_THIS_CHECK_IS_NEEDED
	// For now, the virtual links must be at the top. Not sure
	// what code breaks if this isn't the case.  Why are we checking
	// this???
	for (Handle v : _virtuals)
	{
		Type vt = v->getType();
		if ((not classserver().isA(vt, EVALUATION_LINK))
		    and (not classserver().isA(vt, GREATER_THAN_LINK)))
		{
			throw InvalidParamException(TRACE_INFO,
				"Expecting EvaluationLink at the top level!");
		}
	}
#endif
}
/* ===================== END OF FILE ===================== */
