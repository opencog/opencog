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
#include <opencog/atomutils/FindUtils.h>

#include "SatisfactionLink.h"

using namespace opencog;

void SatisfactionLink::init(void)
{
	// The LambdaLink constructor sets up _body and _varset
	_hclauses = _body;
	unbundle_clauses(_hclauses);
	validate_clauses(_varset, _clauses);
}

SatisfactionLink::SatisfactionLink(const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(SATISFACTION_LINK, hseq, tv, av)
{
	init();
}

SatisfactionLink::SatisfactionLink(const Handle& vars, const Handle& body,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(SATISFACTION_LINK, HandleSeq({vars, body}), tv, av)
{
	init();
}

SatisfactionLink::SatisfactionLink(Type t, const HandleSeq& hseq,
                   TruthValuePtr tv, AttentionValuePtr av)
	: LambdaLink(t, hseq, tv, av)
{
	// BindLink has a different clause initialization sequence
	if (SATISFACTION_LINK != t) return;
	init();
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

	init();
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
 * them out into a distinct list, _virtual; the rest go in _fixed.
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
	for (const Handle& v : vars)
	{
		if (not is_unquoted_in_any_tree(clauses, v))
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
	for (const Handle& clause: clauses)
	{
		bool have_virt = false;
		FindAtoms fgpn(GROUNDED_PREDICATE_NODE);
		fgpn.find_atoms(clause);
		for (const Handle& sh : fgpn.least_holders)
			if (any_unquoted_in_tree(sh, vars))
			{
				have_virt = true;
				break;
			}

		if (not have_virt)
		{
			// Subclasses of VirtualLink, e.g. GreaterThanLink, which
			// are essentially a kind of EvaluationLink holding a GPN
			FindAtoms fgtl(VIRTUAL_LINK, true);
			fgtl.find_atoms(clause);
			// Unlike the above, its varset, not least_holders...
			// because its a link...
			for (const Handle& sh : fgtl.varset)
				if (any_unquoted_in_tree(sh, vars))
				{
					have_virt = true;
					break;
				}
		}

		if (have_virt)
			_virtual.push_back(clause);
		else
			_fixed.push_back(clause);
	}

	// Split the non virtual clauses into connected components
	get_connected_components(vars, _fixed, _components);

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

/* ================================================================= */
/**
 * Check that all clauses are connected
 */
void SatisfactionLink::check_connectivity(
	const std::set<std::vector<Handle>>& components)
{
	if (1 == components.size()) return;

	// Users are going to be stumped by this one, so print
	// out a verbose, user-freindly debug message to help
	// them out.
	std::stringstream ss;
	ss << "Pattern is not connected! Found "
	   << components.size() << " components:\n";
	int cnt = 0;
	for (const auto& comp : components)
	{
		ss << "Connected component " << cnt
		   << " consists of ----------------: \n";
		for (Handle h : comp) ss << h->toString();
		cnt++;
	}
	throw InvalidParamException(TRACE_INFO, ss.str().c_str());
}

/* ===================== END OF FILE ===================== */
