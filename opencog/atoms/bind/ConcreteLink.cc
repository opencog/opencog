/*
 * ConcreteLink.cc
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
#include <opencog/atomutils/FindUtils.h>

#include "ConcreteLink.h"

using namespace opencog;

void ConcreteLink::init(void)
{
	// The LambdaLink constructor sets up _body and _varset
	_hclauses = _body;
	unbundle_clauses(_hclauses);
	validate_clauses(_varset, _clauses);
}

ConcreteLink::ConcreteLink(const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(CONCRETE_LINK, hseq, tv, av)
{
	init();
}

ConcreteLink::ConcreteLink(const Handle& vars, const Handle& body,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(CONCRETE_LINK, HandleSeq({vars, body}), tv, av)
{
	init();
}

ConcreteLink::ConcreteLink(Type t, const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(t, hseq, tv, av)
{
	init();
}

ConcreteLink::ConcreteLink(Link &l)
	: LambdaLink(l)
{
	// Type must be as expected
	Type tscope = l.getType();
	if (not classserver().isA(tscope, CONCRETE_LINK))
	{
		const std::string& tname = classserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting a ConcreteLink, got %s", tname.c_str());
	}
	init();
}


/* ================================================================= */

void ConcreteLink::unbundle_clauses(const Handle& clauses)
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

/// Are there any virtual links, or any evaluatable links in the
/// clause?  Set the two booleans as appropriate.
///
/// A clause is "evalutable" if it contains a GroundedPredicate,
/// or if it inherits from VirtualLink (such as the GreaterThanLink).
/// Such clauses need evaluation at grounding time, to determine
/// thier truth values.
///
/// A clause is "virtual" if it is evaluatable, and also takes an
/// argument that is a variable. Such virtual clauses not only require
/// evaluation, but an evaluation must be performed for each different
/// variable grounding.  Virtual clauses get a very different (and more
/// complex) treatment from the pattern matcher.
///
void ConcreteLink::holds_virtual(const std::set<Handle>& vars,
                                 const Handle& clause,
                                 bool& is_evaluatable,
                                 bool& is_virtual)
{
	is_evaluatable = false;
	is_virtual = false;

	FindAtoms fgpn(GROUNDED_PREDICATE_NODE, true);
	fgpn.find_atoms(clause);
	for (const Handle& sh : fgpn.least_holders)
	{
		is_evaluatable = true;
		if (any_unquoted_in_tree(sh, vars))
		{
			is_virtual = true;
			return;
		}
	}

	// Subclasses of VirtualLink, e.g. GreaterThanLink, which
	// are essentially a kind of EvaluationLink holding a GPN
	FindAtoms fgtl(VIRTUAL_LINK, true);
	fgtl.find_atoms(clause);
	// Unlike the above, its varset, not least_holders...
	// because its a link...
	for (const Handle& sh : fgtl.varset)
	{
		is_evaluatable = true;
		if (any_unquoted_in_tree(sh, vars))
		{
			is_virtual = true;
			return;
		}
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
 * them out into a distinct list, _virtual; the rest go in _fixed.
 *
 * It also partition the clauses into a set of connected components,
 * _components.
 */
void ConcreteLink::validate_clauses(std::set<Handle>& vars,
                                        HandleSeq& clauses)

{
	// Make sure that the user did not pass in bogus clauses.
	// Make sure that every clause contains at least one variable.
	// The presence of constant clauses will mess up the current
	// pattern matcher.  Constant clauses are "trivial" to match,
	// and so its pointless to even send them through the system.
	bool bogus = remove_constants(vars, clauses);
	if (bogus)
	{
		logger().warn("%s: Constant clauses removed from pattern",
		              __FUNCTION__);
	}

	// Make sure that each declared variable appears in some clause.
	// We can't ground variables that aren't attached to something.
	// Quoted variables are constants, and so don't count.
	for (const Handle& v : vars)
	{
		if (not is_unquoted_in_any_tree(clauses, v))
			throw InvalidParamException(TRACE_INFO,
				"The variable %s does not appear (unquoted) in any clause!",
				v->toShortString().c_str());
	}
}

/* ===================== END OF FILE ===================== */
