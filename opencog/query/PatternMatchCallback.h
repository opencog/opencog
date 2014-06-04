/*
 * PatternMatchCallback.h
 *
 * Author: Linas Vepstas February 2008
 *
 * Copyright (C) 2008,2009,2014 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_PATTERN_MATCH_CALLBACK_H
#define _OPENCOG_PATTERN_MATCH_CALLBACK_H

#include <map>
#include <set>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/Link.h>

namespace opencog {
class PatternMatchEngine;
typedef std::map<Handle, const std::set<Type> > VariableTypeMap;

/**
 * Callback interface, used to implement specifics of hypergraph
 * matching, and also, to report solutions when found.
 */
class PatternMatchCallback
{
	public:
		virtual ~PatternMatchCallback() {};

		/**
		 * Called when a node in the template pattern
		 * needs to be compared to a possibly matching
		 * node in the atomspace. The first argument
		 * is a node from the pattern, and the second
		 * is a possible solution node from the atomspace.
		 * Return false if the nodes match, else return
		 * true. (i.e. return true if mis-match).
		 */
		virtual bool node_match(Handle& node1, Handle& node2) = 0;

		/**
		 * Called when a variable in the template pattern
		 * needs to be compared to a possible grounding
		 * node in the atomspace. The first argument
		 * is a variable from the pattern, and the second
		 * is a possible solution node from the atomspace.
		 * Return false if the grouding is acceptable, else
		 * return true. (i.e. return true if mis-match).
		 */
		virtual bool variable_match(Handle& node1, Handle& node2) = 0;

		/**
		 * Called when a link in the template pattern
		 * needs to be compared to a possibly matching
		 * link in the atomspace. The first argument
		 * is a link from the pattern, and the second
		 * is a possible solution link from the atomspace.
		 * Return false if the links match, else return
		 * true. (i.e. return true if mis-match).
		 *
		 * The link_match() callback should not compare the
		 * link contents; the pattern matcher will do that
		 * next, if this callback accepts the match.
		 * After the pattern matcher has found a grounding
		 * for a link, the post_link_match() callback is
		 * called, offering a second chance to reject a link.
		 */
		virtual bool link_match(LinkPtr& link1, LinkPtr& link2) = 0;

		/** 
		 * Called after a candidate grounding has been found
		 * for a link.  This callback offers a final chance
		 * to reject the link match based on the actual
		 * grounding, or to perform post-match processing.
		 * For example, the post_link_match() callback is 
		 * used by the GreaterThanLink to perform a comparison
		 * of the grounded values; if the comparison fails,
		 * the match is rejected.
		 */
		virtual bool post_link_match(LinkPtr& link1, LinkPtr& link2) = 0;

		/**
		 * Called when a solution is found. Should
		 * return false to search for more solutions;
		 * or return true to terminate search.
		 */
		virtual bool solution(std::map<Handle, Handle> &pred_soln,
		                      std::map<Handle, Handle> &var_soln) = 0;

		/**
		 * Called when a top-level clause has been fully grounded.
		 * This is meant to be used for evaluating the truth value
		 * of the clause, as an intermediate stage for evaluating
		 * the overall truth value of a solution (grounding).
		 *
		 * A clause match has occured if all calls to node_match()
		 * and link_match() in that clause have returned false.
		 *
		 * Return true to discard the use of this clause as a possible
		 * grounding, return false to use this grounding.
		 */
		virtual bool clause_match(Handle& pattrn_link_h, Handle& grnd_link_h)
		{
			return false;
		}

		/**
		 * Called when the search for a top-level optional clause
		 * has been completed. The clause may or may not have been
		 * grounded as a result of the search. If it has been grounded,
		 * then grnd will be non-null.
		 *
		 * Return true to terminate further searches from this point
		 * on; the result of termination will be backtracking to search
		 * for other possible groundings of the required clauses.
		 * Return false to examine the next optional clause (if any).
		 *
		 * Note that all required clauses will have been grounded before
		 * any optional clauses are examined.
		 */
		virtual bool optional_clause_match(Handle& pattrn, Handle& grnd)
		{
			return false;
		}

		/**
		 * Called whenever the incoming set of an atom is to be explored.
		 * The search space can be prioritized by returning a list in
		 * some sorted order: the first ones will be searched first.
		 * The search space can also be limited, by returning a set that
		 * is smaller than the full incoming set (for example, by
		 * returning only those atoms with a high av sti).
		 */
		virtual IncomingSet get_incoming_set(Handle h)
		{
			return h->getIncomingSet();
		}

		/**
		 * Called after a top-level predicate (tree) has been fully
		 * grounded. This gives the callee the opportunity to save
		 * state onto a stack, if needed.
		 */
		virtual void push(void) {}

		/**
		 * Called prior to starting a back-track, retreating from the
		 * most recently grounded top-level predicate (tree). This
		 * gives the callee the opportunity to save state onto a stack,
		 * if needed.
		 */
		virtual void pop(void) {}

		/**
		 * Called very early, before pattern-matching has begun. This
		 * conveys how the variable declarations in a BindLink were
		 * decoded.  The argument is nothing more than the variable
		 * declarations, as given in the atomspace, but re-expressed
		 * in a slightly more convenient C++ form, is all.
		 */
		virtual void set_type_restrictions(VariableTypeMap &tm) {}

		/**
		 * Called to initiaite the search. This callback is responsbile
		 * for performing the top-most, outer loop of the search. That is,
		 * it gets to pick the starting points for the search, thereby
		 * possibly limiting the breadth of the search.  It may also cull
		 * the variables, clauses, or negated clauses to remove those that
		 * will not alter the final semantics of the search.
		 */
		virtual void perform_search(PatternMatchEngine *,
		                            std::set<Handle> &vars,
		                            std::vector<Handle> &clauses,
		                            std::vector<Handle> &negations) = 0;
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_CALLBACK_H
