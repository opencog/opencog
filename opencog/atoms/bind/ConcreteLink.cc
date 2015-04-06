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
	unbundle_clauses();
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
///
/// Unpack the clauses.
///
/// The predicate is either an AndLink, a SequentialAndLink,
/// or a single clause. If its an AndLink, then its a list of
/// clauses; a SequentialAnd is also a list, and must be specifically
/// satisfied in sequential order.  Currently, an OrLink is treated
/// as a single clause, and is handled separately.
void ConcreteLink::unbundle_clauses(void)
{
	Type t = _body->getType();
	if (AND_LINK == t or SEQUANTIAL_AND_LINK == t)
	{
		_clauses = LinkCast(_body)->getOutgoingSet();
	}
	else
	{
		// There's just one single clause!
		_clauses.push_back(_body);
	}
}

/* ================================================================= */
/**
 * A simple validatation a collection of clauses for correctness.
 *
 * Every clause should contain at least one variable in it; clauses
 * that are constants and can be trivially discarded.
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
 * Given the initial list of variables and clauses, extract the optional
 * clauses and the dynamically-evaluatable clauses. Also make note of
 * the connectivity diagram of the clauses.
 *
 * It is assumed that the set of clauses form a single, connected
 * component; i.e. that the clauses are pair-wise connected by common,
 * shared variables, and that this pair-wise connection extends over
 * the entire set of clauses. There is no other restriction on the
 * connection topology; they can form any graph whatsoever (as long as
 * it is connected).
 */
void ConcreteLink::setup_pattern(const std::set<Handle> &vars,
                                 const std::vector<Handle> &component)
{
	// Split in positive and negative clauses
	for (const Handle& h : component)
	{
		Type t = h->getType();
		if (NOT_LINK == t or ABSENT_LINK == t)
		{
			Handle inv(LinkCast(h)->getOutgoingAtom(0));
			_optionals.insert(inv);
			_cnf_clauses.push_back(inv);
		}
		else
		{
			_mandatory.push_back(h);
			_cnf_clauses.push_back(h);
		}
	}

	if (_cnf_clauses.empty()) return;

	// Preparation prior to search.
	// Find everything that contains GPN's or the like.
	FindAtoms fgpn(GROUNDED_PREDICATE_NODE, VIRTUAL_LINK, true);
	fgpn.find_atoms(_cnf_clauses);
	_evaluatable = fgpn.holders;

	// Create a table of the nodes that appear in the clauses, and
	// a list of the clauses that each node participates in.
	for (const Handle& h : _cnf_clauses)
	{
		make_connectivity_map(h, h);
	}

	// Save some minor amount of space by erasing those atoms that
	// participate in only one clause. These atoms cannot be used
	// to determine connectivity between clauses, and so are un-needed.
	auto it = _connectivity_map.begin();
	auto end = _connectivity_map.end();
	while (it != end)
	{
		if (it->second.size() == 1)
			it = _connectivity_map.erase(it);
		else
			it++;
	}

#ifdef DEBUG
	// Print out the predicate ...
	printf("\nRedex '%s' has following clauses:\n", _redex_name.c_str());
	int cl = 0;
	for (Handle h : _mandatory)
	{
		printf("Mandatory %d: ", cl);
		prt(h);
		cl++;
	}

	if (0 < _optionals.size())
	{
		printf("Predicate includes the following optional clauses:\n");
		cl = 0;
		for (Handle h : _optionals)
		{
			printf("Optional clause %d: ", cl);
			prt(h);
			cl++;
		}
	}
	else
	{
		printf("No optional clauses\n");
	}

	// Print out the bound variables in the predicate.
	for (Handle h : _bound_vars)
	{
		if (NodeCast(h))
		{
			printf(" Bound var: "); prt(h);
		}
	}

	if (_bound_vars.empty())
	{
		printf("There are no bound vars in this pattern\n");
	}
	printf("\n");
	fflush(stdout);
#endif
}

/* ===================== END OF FILE ===================== */
