/*
 * Implicator.cc
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

#include <opencog/atomspace/SimpleTruthValue.h>

#include "DefaultImplicator.h"
#include "PatternMatch.h"

using namespace opencog;

bool Implicator::grounding(const std::map<Handle, Handle> &var_soln,
                           const std::map<Handle, Handle> &pred_soln)
{
	// PatternMatchEngine::print_solution(pred_soln,var_soln);
	Handle h = inst.instantiate(implicand, var_soln);
	if (Handle::UNDEFINED != h)
	{
		result_list.push_back(h);
	}
	return false;
}

/**
 * The crisp implicator needs to tweak the truth value of the
 * resulting implicand. In most cases, this is not (strictly) needed,
 * for example, if the implicand has ungrounded variables, then
 * a truth value can be assigned to it, and the implicand will obtain
 * that truth value upon grounding.
 *
 * HOWEVER, if the implicand is fully grounded, then it will be given
 * a truth value of (false, uncertain) to start out with, and, if a
 * solution is found, then the goal here is to change its truth value
 * to (true, certain).  That is the whole point of this function:
 * to tweak (affirm) the truth value of existing clauses!
 */
bool CrispImplicator::grounding(const std::map<Handle, Handle> &var_soln,
                                const std::map<Handle, Handle> &pred_soln)
{
	// PatternMatchEngine::print_solution(pred_soln,var_soln);
	Handle h = inst.instantiate(implicand, var_soln);

	if (h != Handle::UNDEFINED)
	{
		result_list.push_back(h);

		// Set truth value to true+confident
		TruthValuePtr stv(SimpleTruthValue::createTV(1, SimpleTruthValue::confidenceToCount(1)));
		h->setTruthValue(stv);
	}
	return false;
}

/**
 * The single implicator behaves like the default implicator, except that
 * it terminates after the first solution is found.
 */
bool SingleImplicator::grounding(const std::map<Handle, Handle> &var_soln,
                                 const std::map<Handle, Handle> &pred_soln)
{
	Handle h = inst.instantiate(implicand, var_soln);

	if (h != Handle::UNDEFINED)
	{
		result_list.push_back(h);
	}
	return true;
}

/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Use the default implicator to find pattern-matches. Associated truth
 * values are completely ignored during pattern matching; if a set of
 * atoms that could be a ground are found in the atomspace, then they
 * will be reported.
 *
 * See the do_bindlink function documentation for details.
 */
Handle PatternMatch::bindlink (Handle himplication)
{
	// Now perform the search.
	DefaultImplicator impl(_atom_space);
	do_bindlink(himplication, impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Returns the first match only. Otherwise, the behavior is identical to
 * PatternMatch::bindlink above.
 *
 * See the do_bindlink function documentation for details.
 */
Handle PatternMatch::single_bindlink (Handle himplication)
{
	// Now perform the search.
	SingleImplicator impl(_atom_space);
	do_bindlink(himplication, impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * Evaluate an ImplicationLink embedded in a BindLink
 *
 * Use the crisp-logic callback to evaluate boolean implication
 * statements; i.e. statements that have truth values assigned
 * their clauses, and statements that start with NotLink's.
 * These are evaluated using "crisp" logic: if a matched clause
 * is true, its accepted, if its false, its rejected. If the
 * clause begins with a NotLink, true and false are reversed.
 *
 * The NotLink is also interpreted as an "absence of a clause";
 * if the atomspace does NOT contain a NotLink clause, then the
 * match is considered postive, and the clause is accepted (and
 * it has a null or "invalid" grounding).
 *
 * See the do_bindlink function documentation for details.
 */
Handle PatternMatch::crisp_logic_bindlink (Handle himplication)
{
	// Now perform the search.
	CrispImplicator impl(_atom_space);
	do_bindlink(himplication, impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * PLN specific PatternMatchCallback implementation
 */
Handle PatternMatch::pln_bindlink(Handle himplication)
{
	// Now perform the search.
	PLNImplicator impl(_atom_space);
	do_bindlink(himplication, impl);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/* ================================================================= */
/**
 * DEPRECATED: USE BIND_LINK INSTEAD!
 * Right now, this method is used only in the unit test cases;
 * and it should stay that way.
 *
 * Default evaluator of implication statements.  Does not consider
 * the truth value of any of the matched clauses; instead, looks
 * purely for a structural match.
 *
 * See the do_imply function for details.
 */
Handle PatternMatch::imply (Handle himplication)
{
	// Now perform the search.
	DefaultImplicator impl(_atom_space);
	std::set<Handle> varset;

	do_imply(himplication, impl, varset);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/**
 * DEPRECATED: USE CRISP_LOGIC_BINDLINK INSTEAD!
 * At this time, this method is used only by the unit test cases.
 * It should stay that way, too; no one else should use this.
 *
 * Use the crisp-logic callback to evaluate boolean implication
 * statements; i.e. statements that have truth values assigned
 * their clauses, and statements that start with NotLink's.
 * These are evaluated using "crisp" logic: if a matched clause
 * is true, its accepted, if its false, its rejected. If the
 * clause begins with a NotLink, true and false are reversed.
 *
 * The NotLink is also interpreted as an "absence of a clause";
 * if the atomspace does NOT contain a NotLink clause, then the
 * match is considered postive, and the clause is accepted (and
 * it has a null or "invalid" grounding).
 *
 * See the do_imply function for details.
 */
Handle PatternMatch::crisp_logic_imply (Handle himplication)
{
	// Now perform the search.
	CrispImplicator impl(_atom_space);
	std::set<Handle> varset;

	do_imply(himplication, impl, varset);

	// The result_list contains a list of the grounded expressions.
	// Turn it into a true list, and return it.
	Handle gl = _atom_space->addLink(LIST_LINK, impl.result_list);
	return gl;
}

/* ===================== END OF FILE ===================== */
