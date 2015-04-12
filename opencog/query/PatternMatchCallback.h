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
#include <opencog/atoms/bind/VariableList.h> // for VariableTypeMap

#define DEBUG 1

namespace opencog {
class PatternMatchEngine;

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
		 * Return true if the nodes match, else return
		 * false. (i.e. return false if mis-match).
		 */
		virtual bool node_match(const Handle& patt_node,
		                        const Handle& grnd_node) = 0;

		/**
		 * Called when a variable in the template pattern
		 * needs to be compared to a possible grounding
		 * node in the atomspace. The first argument
		 * is a variable from the pattern, and the second
		 * is a possible solution node from the atomspace.
		 * Return true if the grouding is acceptable, else
		 * return false. (i.e. return false if mis-match).
		 */
		virtual bool variable_match(const Handle& patt_node,
		                            const Handle& grnd_node) = 0;

		/**
		 * Called right before link in the template pattern
		 * is to be compared to a possibly matching link in
		 * the atomspace. The first argument is a link from
		 * the pattern, and the second is a possible
		 * grounding link from the atomspace. Return true
		 * if the link contents should be compared, else
		 * return false. (i.e. return false if mis-match).
		 *
		 * If true is returned, then the pattern matcher
		 * will proceed, and will compare the outgoing sets
		 * of the two links.  Thus, this callback should not
		 * bother with looking at the outgoing sets.  Indeed,
		 * it is very possible that the outgoing sets will
		 * fail to match; but this is not yet known, at the
		 * time that this callback is made.  By contrast,
		 * the post_link_match() callback will be called
		 * after a full grounding has been established;
		 * that is, after the outgoing sets have been compared.
		 *
		 * This callback offers a good time to check the
		 * truth value, the attention value, and the link
		 * type, and to proceed with the search, or cut it
		 * off, based on these values.
		 */
		virtual bool link_match(const LinkPtr& patt_link,
		                        const LinkPtr& grnd_link) = 0;

		/**
		 * Called after a candidate grounding has been found
		 * for a link.  This callback offers a final chance
		 * to reject the link match based on the actual
		 * grounding, or to perform post-match processing.
		 * Return true to reject the match.
		 *
		 * That is, this callback is called after the two
		 * links have been fully compared, and have been
		 * found to match.  It offers a chance to record
		 * the candidate grounding, or to reject it for some
		 * reason.
		 *
		 * The first link is from the pattern, the second is
		 * from the proposed grounding.
		 */
		virtual bool post_link_match(const LinkPtr& patt_link,
		                             const LinkPtr& grnd_link)
		{
			return true; // Accept the match, by default.
		}

		/**
		 * Invoked to perform the matching of a virtual link.
		 * A virtual link is one that does not (might not) exist as a
		 * real link in the AtomSpace, but might still exist in a
		 * 'virtual' sense, in that it is instead considered to exist if
		 * a GroundedPredicateNode evaluates to true or not.  When such
		 * a virtual link is encountered, this callback is called to make
		 * this decision. This should return false to reject the match.
		 * That is, a return value of "false" denotes that the virtual
		 * atom does not exist; while "true" implies that it does exist.
		 * This is the same convention as link_match() and post_link_match().
		 *
		 * Unlike the other callbacks, this takes arguments in s slightly
		 * different form.  Here, 'virt' is the virtual link specification,
		 * as it appears in the pattern.  At this time, it is assumed that
		 * these are always of the form
		 *
		 *       EvaluationLink
		 *          GroundedPredicateNode "scm:some-function"
		 *          ListLink
		 *             SomeAtom arg1       ;; could be a VariableNode
		 *             VariableNode $arg2  ;; could be some other node, too.
		 *             EtcAtom ...
		 *
		 * The 'args' handle is a candidate grounding for the ListLink.
		 * Note that the candidate grounding is NOT instantiated in the
		 * main AtomSpace (it is held in a temporary AtomSpace that is
		 * deleted upon return from this callback).
		 */
		virtual bool virtual_link_match(const Handle& virt,
		                                const Handle& args) = 0;

		/**
		 * Like the above, except that the free variables occuring
		 * in the virtual link have not been grounded.  Instead, it
		 * gets a map of variables to candidate grounds, and must
		 * then return either 'true' to accept the grounding, or
		 * 'false' to reject it.
		 */
		virtual bool evaluate_link(const Handle& virt,
		                           const std::map<Handle,Handle>& gnds) = 0; 

		/**
		 * Called when a complete grounding to all clauses is found.
		 * Should return false to search for more solutions; or return
		 * true to terminate search.  (Just as in all the other callbacks,
		 * a return value of `true` means that the proposed grounding is
		 * acceptable. The engine is designed to halt once an acceptable
		 * solution has been found; thus, in order to force it to search
		 * for more, a return value of false is needed.)
		 */
		virtual bool grounding(const std::map<Handle, Handle> &var_soln,
		                       const std::map<Handle, Handle> &term_soln) = 0;

		/**
		 * Called when a top-level clause has been fully grounded.
		 * This is meant to be used for evaluating the truth value
		 * of the clause, as an intermediate stage for evaluating
		 * the overall truth value of a solution (grounding).
		 *
		 * A clause match has occured if all calls to node_match()
		 * and link_match() in that clause have returned true.
		 *
		 * Return false to discard the use of this clause as a possible
		 * grounding, return true to use this grounding.
		 */
		virtual bool clause_match(const Handle& pattrn_link_h,
		                          const Handle& grnd_link_h)
		{
			// By default, reject a clause that is grounded by itself.
			// XXX someone commented this out... why??? Do you really
			// want self-grounding???
			//	if (pattrn_link_h == grnd_link_h) return false;
			return true;
		}

		/**
		 * Called when the search for a top-level optional clause
		 * has been completed. The clause may or may not have been
		 * grounded as a result of the search. If it has been grounded,
		 * then grnd will be non-null.
		 *
		 * Return false to terminate further searches from this point
		 * on; the result of termination will be backtracking to search
		 * for other possible groundings of the required clauses.
		 * Return true to examine the next optional clause (if any).
		 *
		 * Note that all required clauses will have been grounded before
		 * any optional clauses are examined.
		 *
		 * The default semantics here is to reject a match if the optional
		 * clauses are detected.  This is in keeping with the semantics of
		 * AbsentLink: a match is possible only if the indicated clauses
		 * are absent!
		 */
		virtual bool optional_clause_match(const Handle& pattrn,
		                                   const Handle& grnd)
		{
			if (Handle::UNDEFINED == grnd) return true;
			return false;
		}

		/**
		 * Called whenever the incoming set of an atom is to be explored.
		 * This callback allows the search space to be prioritized, by
		 * returning (all or some of) the incoming set in some sorted
		 * order: the first in the list will be searched first.
		 * The search space can also be limited, by returning a set that
		 * is smaller than the full incoming set (for example, by
		 * returning only those atoms with a high av-sti).
		 */
		virtual IncomingSet get_incoming_set(const Handle& h)
		{
			return h->getIncomingSet();
		}

		/**
		 * Called after a top-level clause (tree) has been fully
		 * grounded. This gives the callee the opportunity to save
		 * state onto a stack, if needed.
		 */
		virtual void push(void) {}

		/**
		 * Called prior to starting a back-track, retreating from the
		 * most recently grounded top-level clause (tree). This
		 * gives the callee the opportunity to maintain state with a
		 * stack, if needed.
		 */
		virtual void pop(void) {}

		/**
		 * Called very early, before pattern-matching has begun. This
		 * conveys how the variable declarations in a BindLink were
		 * decoded.  The argument contains nothing more than a map
		 * holding the type restrictions, if any, on each variable that
		 * was declared in the VariableList bound to the pattern. The
		 * map allows a fast lookup by variable name, to find any of
		 * it's type restrictions.
		 */
		virtual void set_type_restrictions(const VariableTypeMap& tm) {}

		/**
		 * Called very early, before pattern-matching has begun. This
		 * conveys a list of all of the evaluatable terms in the pattern.
		 * By "evaluatable", it is meant any term that does not have a
		 * fixed TruthValue, but rather has a truth value computed
		 * dynamically, at runtime. Currently, such terms are any
		 * EvaluationLink that contains a GroundedPredicateNode or any
		 * link that inherits from a VirtualLink.  If the callbacks make
		 * match decisions based on TruthValues, then these terms will
		 * typically need to be evaluated during the search.
		 */
		virtual void set_evaluatable_terms(const std::set<Handle>&) {}

		/**
		 * Called very early, before pattern-matching has begun. This
		 * conveys a list of all of the links in the search pattern
		 * that contain ("hold") evaluatable terms in them. See above
		 * for the definition of an "evaluatable term".
		 */
		virtual void set_evaluatable_holders(const std::set<Handle>&) {}

		/**
		 * Called to initiate the search. This callback is responsible
		 * for performing the top-most, outer loop of the search. That is,
		 * it gets to pick the starting points for the search, thereby
		 * possibly limiting the breadth of the search.  It may also cull
		 * the variables, clauses, or negated clauses to remove those that
		 * will not alter the final semantics of the search.
		 *
		 * The return value is used to indicate if the search pattern was
		 * satisfied (grounded) or not.  This is just like the return
		 * values on all the other callbacks; it summarizes (passes
		 * through) the return values of all the others.
		 */
		virtual bool initiate_search(PatternMatchEngine *,
		                             const std::set<Handle> &vars,
		                             const std::vector<Handle> &clauses) = 0;
};

} // namespace opencog

#endif // _OPENCOG_PATTERN_MATCH_CALLBACK_H
