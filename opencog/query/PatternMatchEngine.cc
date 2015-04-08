/*
 * PatternMatchEngine.cc
 *
 * Copyright (C) 2008,2009,2011,2014,2015 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  February 2008
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

#include <opencog/util/oc_assert.h>
#include <opencog/atomutils/FindUtils.h>
#include <opencog/atoms/bind/PatternUtils.h>
#include <opencog/atoms/bind/SatisfactionLink.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/Node.h>

#include "PatternMatchEngine.h"

using namespace opencog;

// Uncomment below to enable debug print
// #define DEBUG
#ifdef WIN32
#ifdef DEBUG
	#define dbgprt printf
#else
	// something better?
	#define dbgprt
#endif
#else
#ifdef DEBUG
	#define dbgprt(f, varargs...) printf(f, ##varargs)
#else
	#define dbgprt(f, varargs...)
#endif
#endif

static inline bool prt(const Handle& h)
{
	std::string str = h->toShortString();
	printf ("%s\n", str.c_str());
	return false;
}

static inline void prtmsg(const char * msg, const Handle& h)
{
#ifdef DEBUG
	if (h == Handle::UNDEFINED) {
		printf ("%s (invalid handle)\n", msg);
		return;
	}
	std::string str = h->toShortString();
	printf ("%s %s\n", msg, str.c_str());
#endif
}

/* ======================================================== */

/**
 * Pattern Match Engine Overview
 * -----------------------------
 *
 * The pattern match engine finds groundings (interpretations) for a
 * set of mandatory and optional clauses. It can be thought of in seveal
 * ways: it performs a variable unification across multiple clauses;
 * it discovers groundings for variables; it solves the subgraph
 * isomorphism problem; it queries a graphical relational database
 * for data that satisfies the query statement; it solves a typed
 * satisfiability problem; it finds an "answer set".  Which of these
 * things it does depends on your point of view; it does all of these
 * things, although, in each case, the langauge is slightly different.
 *
 * It works by being given a list ov variables ("bound variables") and
 * a set of mandatory and optional clauses containing those variables.
 * Each "clause" is nothing more than an ordinary OpenCog Atom:
 * specifically, a typed link.  It should be thought of as a tree, the
 * tree being formed by the outgoing set of the Atom.  We could call
 * these things "Atoms" or "Links", but it is more convenient to call
 * them "clauses", in part to avoid confusion, and in part because they
 * resemble terms (from "term algebra") or the "sentences" (of
 * first-order logic). They, in fact, are neither, but there is a
 * general resemblance that is useful to keep in mind.
 *
 * Any given clause is "solved" or "grounded", if there exists a
 * substitution for the variables such that the resulting clause exists
 * in the atomspace.  Thus, a grounding can be thought of as an
 * "interpretation" of the clause; the pattern matcher searches for
 * interpretations.  It can be thought of as a "unifier", in that ALL
 * of the mandatory clauses must be grounded, and they must grounded in
 * a self-consistent way (i.e. the clauses must be grounded in
 * conjunction with one-another.)
 *
 * The list of "bound vars" are to be solved for ("grounded", or
 * "interpreted") during pattern matching. That is, if the subgraph
 * defined by the clauses is located, then the vars are given the
 * corresponding values associated to that match. Because these
 * variables can be shared across multiple clauses, this can be
 * understood to be a unification problem; the pattern matcher is thus
 * a unifier.
 *
 * The optionals are a set of clauses which are also searched for,
 * but whose presence is not mandatory. The optional clauses can be
 * used to implement negation or pattern-rejection (among other things).
 * Thier use for negation/rejection is determined by the "optional
 * clause" callback.  Thus, if an optional clause is found, the callback
 * can then signal that the pattern as a whole should be rejected.
 * Alternately, the callback could base its considerations on the
 * truth-value of the optionall clause: if the optional clause has
 * a TV of FALSE, then it is accepted; otherwise it is rejected.
 * Other strategies for handling optional clauses in the callback
 * are also possible. Thus, optional clauses themselves can be thought
 * as implementing a bridge between "intuitionistic logic" and
 * "classical logic", depending on ow they are handled in the callback.
 *
 * At every step of the process, the PatternMatchCallback is consulted
 * to determine whether a candidate match should be accepted or not.
 * In general, the callbacks have almost total control over the search;
 * the pattern match engine only provides the general framework for
 * navigating and backtracking and maintaining the needed state for
 * such traversal.  It is up to the callbacks to perform the actual
 * grounding or interpreation, and to record the search results.
 */

/* ======================================================== */

// At this time, we don't want groundings where variables ground
// themselves.   However, there is a semi-plausible use-case for
// this, see https://github.com/opencog/opencog/issues/1092
// Undefine this to experiment. See also the unit tests.
#define NO_SELF_GROUNDING 1

/* Reset the current variable grounding to the last grounding pushed
 * onto the stack. */
#define POPGND(soln,stack) {         \
   OC_ASSERT(not stack.empty(), "Unbalanced grounding stack"); \
   soln = stack.top();               \
   stack.pop();                      \
}

/* ======================================================== */

/**
 * tree_compare compares two trees, side-by-side.
 *
 * Compare two incidence trees, side-by-side.  The incidence tree is
 * given by following the "outgoing set" of the links appearing in the
 * tree.  The incidence tree is the so-called "Levi graph" of the
 * hypergraph.  The first arg should be a handle to a clause in the
 * pattern, while the second arg is a handle to a candidate grounding.
 * The pattern (predicate) clause is compared to the candidate grounding,
 * returning true if there is a mis-match.
 *
 * The comparison is recursive, so this method calls itself on each
 * subtree of the predicate clause, performing comparisons until a
 * match is found (or not found).
 *
 * Return false if there's a mis-match. The goal here is to walk over
 * the entire tree, without mismatches.  Since a return value of false
 * stops the iteration, false is used to signal a mismatch.
 *
 * The pattern clause may contain quotes (QuoteLinks), which signify
 * that what follows must be treated as a literal (constant), rather
 * than being interpreted.  Thus, quotes can be used to search for
 * expressions containing variables (since a quoted variable is no
 * longer a variable, but a constant).  Quotes can also be used to
 * search for GroundedPredicateNodes (since a quoted GPN will be
 * treated as a constant, and not as a function).  Quotes can be nested,
 * only the first quote is used to escape into the literal context,
 * and so quotes can be used to search for expressions containing
 * quotes.  It is assumed that the QuoteLink has an arity of one, as
 * its quite unclear what an arity of more than one could ever mean.
 *
 * That method have side effects. The main one is to insert variable
 * groundings (and in fact sub-clauses grounding as well) in
 * var_grounding when encountering variables (and sub-clauses) in the
 * pattern.
 */
bool PatternMatchEngine::tree_compare(const Handle& hp,
                                      const Handle& hg,
                                      Caller caller)
{
	// If the pattern link is a quote, then we compare the quoted
	// contents. This is done recursively, of course.  The QuoteLink
	// must have only one child; anything else beyond that is ignored
	// (as its not clear what else could possibly be done).
	Type tp = hp->getType();
	if (not in_quote and QUOTE_LINK == tp)
	{
		in_quote = true;
		LinkPtr lp(LinkCast(hp));
		if (1 != lp->getArity())
			throw InvalidParamException(TRACE_INFO, "QuoteLink has unexpected arity!");
		more_depth ++;
		bool ma = tree_compare(lp->getOutgoingAtom(0), hg, CALL_QUOTE);
		more_depth --;
		in_quote = false;
		return ma;
	}

	// Handle hp is from the pattern clause, and it might be one
	// of the bound variables. If so, then declare a match.
	if (not in_quote and _bound_vars.end() != _bound_vars.find(hp))
	{
#ifdef NO_SELF_GROUNDING
		// But... if handle hg happens to also be a bound var,
		// then its a mismatch.
		if (_bound_vars.end() != _bound_vars.find(hg)) return false;
#endif

		// If we already have a grounding for this variable, the new
		// proposed grounding must match the existing one. Such multiple
		// groundings can occur when traversing graphs with loops in them.
		Handle gnd(var_grounding[hp]);
		if (Handle::UNDEFINED != gnd) {
			return (gnd == hg);
		}

		// VariableNode had better be an actual node!
		// If it's not then we are very very confused ...
		NodePtr np(NodeCast(hp));
		OC_ASSERT (NULL != np,
		           "ERROR: expected variable to be a node, got this: %s\n",
		           hp->toShortString().c_str());

#ifdef NO_SELF_GROUNDING
		// Disallow matches that contain a bound variable in the
		// grounding. However, a bound variable can be legitimately
		// grounded by a free variable (because free variables are
		// effectively constant literals, during the pattern match.
		if (VARIABLE_NODE == hg->getType() and
		    _bound_vars.end() != _bound_vars.find(hg))
		{
			return false;
		}
#endif

		// Else, we have a candidate grounding for this variable.
		// The node_match may implement some tighter variable check,
		// e.g. making sure that grounding is of some certain type.
		if (not _pmc->variable_match (hp,hg)) return false;

		// Make a record of it.
		dbgprt("Found grounding of variable:\n");
		prtmsg("$$ variable:    ", hp);
		prtmsg("$$ ground term: ", hg);
		if (hp != hg) var_grounding[hp] = hg;
		return true;
	}

	// If they're the same atom, then clearly they match.
	// ... but only if hp is a constant i.e. contains no bound variables)
	if (hp == hg)
	{
		prtmsg("Compare atom to itself:\n", hp);

#ifdef NO_SELF_GROUNDING
		if (hg == curr_pred_handle)
		{
			// Mismatch, if hg contains bound vars in it.
			if (any_unquoted_in_tree(hg, _bound_vars)) return false;
		}
		else
		{
#if THIS_CANT_BE_RIGHT
			// Bound but quoted variables cannot be solutions to themselves.
			// huh? whaaaat?
			if (not in_quote or
			    (in_quote and
			     (VARIABLE_NODE != tp or
			       _bound_vars.end() == _bound_vars.find(hp))))
			{
				if (hp != hg) var_grounding[hp] = hg;
			}
#endif // THIS_CANT_BE_RIGHT

		}
#endif
		// If the pattern contains atoms that are evaluatable i.e. GPN's
		// then we must fall through, and let the tree comp mechanism
		// find and evaluate them.
		if (_evaluatable.find(hp) == _evaluatable.end()) return true;
		else
		{ dbgprt("Its evaluatable, continuing.\n"); }
	}

	// OR_LINK's are multiple-choice links. As long as we can
	// can match one of the sub-expressions of the OrLink, then
	// the OrLink as a whole can be considered to be grounded.
	//
	// After finding a grounding for a subexpression, we need to save
	// state, so that, later one, we can resume, and try a different
	// subexpression.  The state that needs to be pushed is this:
	// * The current var and pred groundings,
	// * The current OR-clause (so that we can resume exploring the next
	//   one)
	// This means that the loop over the outgoing set of an OR-link
	// cannot be an actual loop, but must be a recursive call.
	if (classserver().isA(tp, OR_LINK))
	{
		// XXX TODO: the below finds the first possible match,
		// and then calls it quits. In fact, we need to find all
		// of them.
		LinkPtr lp(LinkCast(hp));
		bool match = false;
		const std::vector<Handle> &osp = lp->getOutgoingSet();
		for (Handle hop : osp)
		{
			prtmsg("tree_comp or_link choice: ", hop);
			var_solutn_stack.push(var_grounding);

			match = tree_compare(hop, hg, CALL_CHOICE);
			// If no match, then try the next one.
			if (not match)
			{
				// Get rid of any grounding that might have been proposed
				// during the tree-match.
				POPGND(var_grounding, var_solutn_stack);
			}
			else
			{
				// Keep the grounding that was found. Even up the stack.
				var_solutn_stack.pop();

				// If we've found a grounding, record it.
				// Note we record the grounding for both the OrLink,
				// and for the specific clause that was picked.
				var_grounding[hop] = hg;
				var_grounding[hp] = hg;
				break;
			}
		}
		if (not match) return false;

		return true;
	}

	// If both are links, compare them as such.
	// Unless pattern link is a QuoteLink, in which case, the quoted
	// contents is compared. (well, that was done up above...)
	LinkPtr lp(LinkCast(hp));
	LinkPtr lg(LinkCast(hg));
	if (lp and lg)
	{
#ifdef NO_SELF_GROUNDING
		// The proposed grounding must NOT contain any bound variables!
		// .. unless they are quoted in the pattern, in which case they
		// are allowed... well, not just allowed, but give the right
		// answer. The below checks for this case. The check is not
		// entirely correct though; there are some weird corner cases
		// where a variable may appear quoted in the pattern, but then
		// be in the wrong spot entirely in the proposed grounding, and
		// so should not have been allowed.
		// XXX FIXME For now, we punt on this... a proper fix would be
		// ... hard, as we would have to line up the location of the
		// quoted and the unquoted parts.
		if (any_unquoted_in_tree(hg, _bound_vars))
		{
			for (Handle vh: _bound_vars)
			{
				// OK, which tree is it in? And is it quoted in the pattern?
				if (is_unquoted_in_tree(hg, vh))
				{
					prtmsg("found bound variable in grounding tree:", vh);
					prtmsg("matching  pattern  is:", hp);
					prtmsg("proposed grounding is:", hg);

					if (is_quoted_in_tree(hp, vh)) continue;
					dbgprt("miscompare; var is not in pattern, or its not quoted\n");
					return false;
				}
			}
		}
#endif
		// If the pattern is defined elsewhere, not here, then we have
		// to go to where it is defined, and pattern match things there.
		if (BETA_REDEX == tp)
		{
			return redex_compare(lp, lg);
		}

		// Let the callback perform basic checking.
		bool match = _pmc->link_match(lp, lg);
		if (not match) return false;

		dbgprt("depth=%d\n", depth);
		prtmsg("> tree_compare", hp);
		prtmsg(">           to", hg);

		// If the two links are both ordered, its enough to compare
		// them "side-by-side"; the foreach_atom_pair iterator does
		// this. If they are un-ordered, then we have to compare every
		// possible permutation.  This can be a bit of a combinatoric
		// explosion.
		if (classserver().isA(tp, ORDERED_LINK))
		{
			LinkPtr lp(LinkCast(hp));
			LinkPtr lg(LinkCast(hg));
			const HandleSeq &osp = lp->getOutgoingSet();
			const HandleSeq &osg = lg->getOutgoingSet();

			size_t oset_sz = osp.size();
			if (oset_sz != osg.size()) return false;

			// The recursion step: traverse down the tree.
			// In principle, we could/should push the current groundings
			// onto the stack before recursing, and then pop them off on
			// return.  Failure to do so could leave some bogus groundings,
			// sitting around, i.e. groundings that were found during
			// recursion but then discarded due to a later mis-match.
			//
			// In practice, I was unable to come up with any test case
			// where this mattered; any bogus groundings eventually get
			// replaced by valid ones.  Thus, we save some unknown amount
			// of cpu time by simply skipping the push & pop here.
			//
			// var_solutn_stack.push(var_grounding);
			depth ++;
			more_depth ++;

			match = true;
			for (size_t i=0; i<oset_sz; i++)
			{
				if (not tree_compare(osp[i], osg[i], CALL_ORDER))
				{
					match = false;
					break;
				}
			}

			more_depth --;
			depth --;
			dbgprt("tree_comp down link match=%d\n", match);

			if (not match) return false;

			// If we've found a grounding, lets see if the
			// post-match callback likes this grounding.
			match = _pmc->post_link_match(lp, lg);
			if (not match) return false;

			// If we've found a grounding, record it.
			if (hp != hg) var_grounding[hp] = hg;

			return true;
		}
		else
		{
			// If we are here, we are dealing with an unordered link.
			// Enumerate all the permutations of the outgoing set of
			// the predicate.  We have to try all possible permuations
			// here, as different variable assignments could lead to
			// very differrent problem solutions.  We have to push/pop
			// the stack as we try each permutation, so that each
			// permutation has a chance of finding a grounding.
			//
			// The problem with exhaustive enumeration is that we can't
			// do it here, by ourselves: this can only be done at the
			// clause level. So if we do find a grounding, we have to
			// report it, and return.  We might get called again, and,
			// if so, we have to pick up where we left off, and so there
			// is yet more stack trickery to save and restore that state.
			//
			const std::vector<Handle> &osg = lg->getOutgoingSet();
			std::vector<Handle> mutation;

			if (more_stack.size() <= more_depth) more_stack.push_back(false);
			if (not more_stack[more_depth])
			{
				dbgprt("tree_comp fresh unordered link at depth %zd\n",
				       more_depth);
				mutation = lp->getOutgoingSet();
				sort(mutation.begin(), mutation.end());
			}
			else
			{
				dbgprt("tree_comp resume unordered link at depth %zd\n",
				       more_depth);
				mutation = mute_stack.top();
				mute_stack.pop();
			}

			do {
				size_t oset_sz = mutation.size();
				if (oset_sz != osg.size()) return false;

				// The recursion step: traverse down the tree.
				dbgprt("tree_comp start downwards unordered link at depth=%lu\n",
				       more_depth);
				var_solutn_stack.push(var_grounding);
				depth ++;

				have_more = false;
				more_depth ++;

				match = true;
				for (size_t i=0; i<oset_sz; i++)
				{
					if (not tree_compare(mutation[i], osg[i], CALL_UNORDER))
					{
						match = false;
						break;
					}
				}

				more_depth --;
				depth --;
				dbgprt("tree_comp down unordered link depth=%lu mismatch=%d\n",
				       more_depth, match);

				// If we've found a grounding, lets see if the
				// post-match callback likes this grounding.
				if (match)
				{
					match = _pmc->post_link_match(lp, lg);
				}

				// If we've found a grounding, record it.
				if (match)
				{
					// Since we leave the loop, we better go back
					// to where we found things.
					var_solutn_stack.pop();
					if (hp != hg) var_grounding[hp] = hg;
					dbgprt("tree_comp unordered link have gnd at depth=%lu\n",
					      more_depth);

					// If a lower part of a tree has more to do, it will have
					// set the have_more flag.  If it is done, then the
					// have_more flag is clear.  If its clear, we, have to
					// advance, so we don't restart in the same place.
					// Ugh. Seems like there should be a more elegant way.
					if (have_more)
					{
						mute_stack.push(mutation);
						more_stack[more_depth] = true;
						have_more = true;
					}
					else
					{
						// Lower loop is done. Advance.
						bool more_perms = std::next_permutation(mutation.begin(), mutation.end());
						if (more_perms)
						{
							mute_stack.push(mutation);
							more_stack[more_depth] = true;
							have_more = true;
						}
						else
						{
							more_stack[more_depth] = false;
							have_more = false;
						}
					}

					return true;
				}
				POPGND(var_grounding, var_solutn_stack);
			} while (std::next_permutation(mutation.begin(), mutation.end()));

			dbgprt("tree_comp down unordered exhausted all permuations\n");
			more_stack[more_depth] = false;
			have_more = false;
			return false;
		}

		OC_ASSERT(false, "statement should not be reached");
		return match;
	}

	// If both are nodes, compare them as such.
	NodePtr np(NodeCast(hp));
	NodePtr ng(NodeCast(hg));
	if (np and ng)
	{
		// Call the callback to make the final determination.
		bool match = _pmc->node_match(hp, hg);
		if (match)
		{
			dbgprt("Found matching nodes\n");
			prtmsg("# pattern: ", hp);
			prtmsg("# match:   ", hg);
			if (hp != hg) var_grounding[hp] = hg;
		}
		return match;
	}

	// If we got to here, there is a clear mismatch, probably because
	// one is a node, and the other a link.
	return false;
}

/* ======================================================== */

/// Return true if a grounding was found.
bool PatternMatchEngine::soln_up(const Handle& hsoln)
{
	// Let's not stare at our own navel. ... Unless the current
	// clause has GroundedPredicateNodes in it. In that case, we
	// have to make sure that they get evaluated.
	if ((hsoln == curr_root)
	    and _evaluatable.find(curr_root) == _evaluatable.end())
		return false;

	have_stack.push(have_more);
	have_more = false;

	do {
		var_solutn_stack.push(var_grounding);

		bool match = tree_compare(curr_pred_handle, hsoln, CALL_SOLN);
		// If no match, then try the next one.
		if (not match)
		{
			// Get rid of any grounding that might have been proposed
			// during the tree-match.
			POPGND(var_grounding, var_solutn_stack);
			have_more = have_stack.top();
			have_stack.pop();
			return false;
		}

		bool found = do_soln_up(hsoln);
		if (found)
		{
			// Keep the grounding that was found. Even up the stack.
			var_solutn_stack.pop();
			have_more = have_stack.top();
			have_stack.pop();
			return true;
		}

		// Get rid of any grounding that might have been proposed
		// during the tree-compare or soln_up.
		POPGND(var_grounding, var_solutn_stack);

		if (have_more) { dbgprt("Wait ----- there's more!\n"); }
		else { dbgprt("No more unordered, more_depth=%zd\n", more_depth); }
	} while (have_more);

	have_more = have_stack.top();
	have_stack.pop();
	return false;
}

/**
 * Push all stacks related to the grounding of a clause. This push is
 * meant to be done only when a grounding for a clause has been found,
 * and the next clause is about the be attempted. It saves all of the
 * traversal data associated with the current clause, so that, later
 * on, traversal can be resumed where it was left off.
 *
 * This does NOT push and of the redex stacks because (with the current
 * redex design), all redex substitutions should have terminatated by
 * now, and returned to the main clause. i.e. the redex stack is assumed
 * to be empty, at this point.  (Its possible this design may change in
 * in the future if multi-clause redexes are allowed, whatever the heck
 * that may be!?)
 */
void PatternMatchEngine::clause_stacks_push(void)
{
	_clause_stack_depth++;
	dbgprt("--- That's it, now push to stack depth=%d\n\n", _clause_stack_depth);

	OC_ASSERT(not in_quote, "Can't posssibly happen!");

	root_handle_stack.push(curr_root);
	pred_handle_stack.push(curr_pred_handle);
	soln_handle_stack.push(curr_soln_handle);

	var_solutn_stack.push(var_grounding);
	pred_solutn_stack.push(clause_grounding);

	issued_stack.push(issued);

	// Reset the unordered-set stacks with each new clause.
	// XXX this cannot possible be right: we may need to pop,
	// and try a different ordering.
	have_stack.push(have_more);
	have_more = false;
	depth_stack.push(more_depth);
	more_depth = 0;
	unordered_stack.push(more_stack);
	more_stack.resize(1);
	more_stack[0] = false;
	permutation_stack.push(mute_stack);

	_pmc->push();
}

/**
 * Pop all clause-traversal-related stacks. This restores state
 * so that the traversal of a single clause can resume where it left
 * off. These do NOT affect any of the redex stacks (which are assumed
 * to be empty at this point.)
 */
void PatternMatchEngine::clause_stacks_pop(void)
{
	_pmc->pop();
	curr_root = root_handle_stack.top();
	root_handle_stack.pop();

	curr_pred_handle = pred_handle_stack.top();
	pred_handle_stack.pop();

	curr_soln_handle = soln_handle_stack.top();
	soln_handle_stack.pop();

	// The grounding stacks are handled differently.
	POPGND(clause_grounding, pred_solutn_stack);
	POPGND(var_grounding, var_solutn_stack);

	issued = issued_stack.top();
	issued_stack.pop();

	// Handle different unordered links that live in different
	// clauses. The mute_stack deals with different unordered
	// links that live in the *same* clause.
	have_more = have_stack.top();
	have_stack.pop();

	more_depth = depth_stack.top();
	depth_stack.pop();

	more_stack = unordered_stack.top();
	unordered_stack.pop();

	mute_stack = permutation_stack.top();
	permutation_stack.pop();

	_clause_stack_depth --;

	dbgprt("pop to depth %d\n", _clause_stack_depth);
	prtmsg("pop to joiner", curr_pred_handle);
	prtmsg("pop to clause", curr_root);
}

/// Return true if a grounding was found.  It also has the side effect
/// of updating clause_grounding map when the current clause is being
/// grounded.
bool PatternMatchEngine::do_soln_up(const Handle& hsoln)
{
	depth = 1;

	// If we are here, then everything below us matches.  If we are
	// not yet at the top of a clause, i.e. we are in the middle of
	// a clause, then we need to move up.
	if (curr_pred_handle != curr_root)
	{
		soln_handle_stack.push(curr_soln_handle);
		curr_soln_handle = hsoln;

		// Move up the predicate, and hunt for a match, again.
		dbgprt("Term has ground, move up.\n");
		// Do not use the callback get_incoming_set on the pattern!
		IncomingSet iset = curr_pred_handle->getIncomingSet();
		size_t sz = iset.size();
		bool found = false;
		for (size_t i = 0; i < sz; i++) {
			Handle hi(iset[i]);

			// Is this link even a part of the predicate we
			// are considering?   If not, try the next atom.
			bool valid = is_atom_in_tree(curr_root, hi);
			if (not valid) continue;

			// Ugh. If the next step up the predicate is an OrLink,
			// we consider it to be solved already.  So re-enter, and
			// try again.
			if (OR_LINK == hi->getType())
			{
				curr_pred_handle = hi;
				found = do_soln_up(hsoln);
				break;
			}

			found = pred_up(hi);
			if (found) break;
		}
		dbgprt("After moving up the clause, found = %d\n", found);

		curr_soln_handle = soln_handle_stack.top();
		soln_handle_stack.pop();

		return found;
	}

	// If we are here, we've navigated to the top of the clause, and
	// it is matched, then it is fully grounded, and we're done with it.
	// Start work on the next unsovled predicate. But first, see what
	// the callbacks have to say.

	// Is this clause a required clause? If so, then let the callback
	// make the final decision; if callback rejects, then it's the
	// same as a mismatch; try the next one.
	bool match;
	if (_optionals.count(curr_root))
	{
		clause_accepted = true;
		match = _pmc->optional_clause_match(curr_pred_handle, hsoln);
	}
	else
	{
		match = _pmc->clause_match(curr_pred_handle, hsoln);
	}
	dbgprt("clause match callback match=%d\n", match);
	if (not match) return false;

	curr_soln_handle = hsoln;
	clause_grounding[curr_root] = curr_soln_handle;
	prtmsg("---------------------\nclause:", curr_root);
	prtmsg("ground:", curr_soln_handle);

	clause_stacks_push();
	get_next_untried_clause();

	// If there are no further predicates to solve,
	// we are really done! Report the solution via callback.
	bool found = false;
	if (Handle::UNDEFINED == curr_root)
	{
#ifdef DEBUG
		dbgprt ("==================== FINITO!\n");
		print_solution(var_grounding, clause_grounding);
#endif
		found = _pmc->grounding(var_grounding, clause_grounding);
	}
	else
	{
		prtmsg("Next clause is", curr_root);
		dbgprt("This clause is %s\n",
			_optionals.count(curr_root)? "optional" : "required");
		dbgprt("This clause is %s\n",
			_evaluatable.count(curr_root)?
			"dynamically evaluatable" : "non-dynamic");
		prtmsg("Joining variable  is", curr_pred_handle);
		prtmsg("Joining grounding is", var_grounding[curr_pred_handle]);

		// Else, start solving the next unsolved clause. Note: this is
		// a recursive call, and not a loop. Recursion is halted when
		// the next unsolved clause has no grounding.
		//
		// We continue our search at the atom that "joins" (is shared
		// in common) between the previous (solved) clause, and this
		// clause. If the "join" was a variable, look up its grounding;
		// else the join is a 'real' atom.

		clause_accepted = false;
		curr_soln_handle = var_grounding[curr_pred_handle];
		OC_ASSERT(curr_soln_handle != Handle::UNDEFINED,
			"Error: joining handle has not been grounded yet!");
		found = soln_up(curr_soln_handle);

		// If we are here, and found is false, then we've exhausted all
		// of the search possibilities for the current clause. If this
		// is an optional clause, and no solutions were reported for it,
		// then report the failure of finding a solution now. If this was
		// also the final optional clause, then in fact, we've got a
		// grounding for the whole thing ... report that!
		//
		// Note that lack of a match halts recursion; thus, we can't
		// depend on recursion to find additional unmatched optional
		// clauses; thus we have to explicitly loop over all optional
		// clauses that don't have matches.
		while ((false == found) and
		       (false == clause_accepted) and
		       (_optionals.count(curr_root)))
		{
			Handle undef(Handle::UNDEFINED);
			match = _pmc->optional_clause_match(curr_pred_handle, undef);
			dbgprt ("Exhausted search for optional clause, cb=%d\n", match);
			if (not match) return false;

			// XXX Maybe should push n pop here? No, maybe not ...
			clause_grounding[curr_root] = Handle::UNDEFINED;
			get_next_untried_clause();
			prtmsg("Next optional clause is", curr_root);
			if (Handle::UNDEFINED == curr_root)
			{
				dbgprt ("==================== FINITO BANDITO!\n");
#ifdef DEBUG
				print_solution(var_grounding, clause_grounding);
#endif
				found = _pmc->grounding(var_grounding, clause_grounding);
			}
			else
			{
				// Now see if this optional clause has any solutions,
				// or not. If it does, we'll recurse. If it does not,
				// we'll loop around back to here again.
				clause_accepted = false;
				curr_soln_handle = var_grounding[curr_pred_handle];
				found = soln_up(curr_soln_handle);
			}
		}
	}

	// If we failed to find anything at this level, we need to
	// backtrack, i.e. pop the stack, and begin a search for
	// other possible matches and groundings.
	clause_stacks_pop();

	return found;
}

bool PatternMatchEngine::pred_up(const Handle& h)
{
	// Move up the solution outgoing set, looking for a match.
	Handle curr_pred_save(curr_pred_handle);
	curr_pred_handle = h;

	IncomingSet iset = _pmc->get_incoming_set(curr_soln_handle);
	size_t sz = iset.size();
	bool found = false;
	for (size_t i = 0; i < sz; i++) {
		found = soln_up(Handle(iset[i]));
		if (found) break;
	}

	curr_pred_handle = curr_pred_save;
	dbgprt("Found upward soln = %d\n", found);
	return found;
}

/**
 * Search for the next untried, (thus ungrounded, unsolved) clause.
 *
 * The "issued" set contains those clauses which are currently in play,
 * i.e. those for which a grounding is currently being explored. Both
 * grounded, and as-yet-ungrounded clauses may be in this set.  The
 * sole reason of this set is to avoid infinite resursion, i.e. of
 * re-identifying the same clause over and over as unsolved.
 *
 * The words "solved" and "grounded" are used as synonyms throught the
 * code.
 */
void PatternMatchEngine::get_next_untried_clause(void)
{
	// First, try to ground all the mandatory clauses, only.
	// Optional clauses are grounded only after all the mandatory
	// ones are done.
	if (get_next_untried_helper(false)) return;

	// If there are no optional clauses, we are done.
	if (_optionals.empty())
	{
		// There are no more ungrounded clauses to consider. We are done.
		curr_root = Handle::UNDEFINED;
		curr_pred_handle = Handle::UNDEFINED;
		return;
	}

	// Try again, this time, considering the optional clauses.
	if (get_next_untried_helper(true)) return;

	// If we are here, there are no more unsolved clauses to consider.
	curr_root = Handle::UNDEFINED;
	curr_pred_handle = Handle::UNDEFINED;
}

/// Same as above, but with a boolean flag:  if not set, then only the
/// list of mandatory clauses is considered, else all clauses (mandatory
/// and optional) are considered.
///
/// Return true if we found the next ungrounded clause.
bool PatternMatchEngine::get_next_untried_helper(bool search_optionals)
{
	// Search for an as-yet ungrounded clause. Search for required
	// clauses first; then, only if none of those are left, move on
	// to the optional clauses.  We can find ungrounded clauses by
	// looking at the grounded vars, looking up the root, to see if
	// the root is grounded.  If its not, start working on that.
	Handle pursue(Handle::UNDEFINED);
	Handle unsolved_clause(Handle::UNDEFINED);
	bool unsolved = false;
	bool solved = false;

	// We are looking for a joining atom, one that is shared in common
	// with the a fully grounded clause, and an as-yet ungrounded clause.
	// The joint is called "pursue", and the unsolved clause that it
	// joins will become our next untried clause.  Note that the join
	// is not necessarily a variable; it could be a constant atom.
	// (wouldn't things be faster if they were variables only, that
	// joined? the problem is that var_grounding stores both grounded
	// variables and "grounded" constants.
#define OLD_LOOP
#ifdef OLD_LOOP
	for (const ConnectPair& vk : _connectivity_map)
	{
		const RootList& rl(vk.second);
		pursue = vk.first;
		if (Handle::UNDEFINED == var_grounding[pursue]) continue;
#else // OLD_LOOP
	// Newloop might be slightly faster than old loop.
	// ... but maybe not...
	for (auto gndpair : var_grounding)
	{
		pursue = gndpair.first;
		try { _connectivity_map.at(pursue); }
		catch(...) { continue; }
		const RootList& rl(_connectivity_map.at(pursue));
#endif

		unsolved = false;
		solved = false;
		for (const Handle& root : rl)
		{
			if (Handle::UNDEFINED != clause_grounding[root])
			{
				solved = true;
			}
			else if ((issued.end() == issued.find(root)) and
			         (search_optionals or
			          (_optionals.end() == _optionals.find(root))))
			{
				unsolved_clause = root;
				unsolved = true;
			}
			if (solved and unsolved) break;
		}

		// XXX TODO ... Rather than settling for the first one that we find,
		// we should instead look for the "thinnest" one, the one with the
		// smallest incoming set.  That is because the very next thing that
		// we do will be to iterate over the incoming set of "pursue" ... so
		// it could be a huge pay-off to minimize this.
		//
		// In particular, the "thinnest" one is probably the one with the
		// fewest ungrounded variables in it. Thus, if there is just one
		// variable that needs to be grounded, then this can be done in
		// direct fashion; it resembles the concept of "unit propagation"
		// in the DPLL algorithm.
		//
		// If there are two ungrounded variables in a clause, then the
		// "thickness" is the *product* of the sizes of the two incoming
		// sets. Thus, the fewer ungrounded variables, the better.
		if (solved and unsolved) break;
	}

	if (solved and unsolved)
	{
		// Pursue is a pointer to a (variable) node that's shared between
		// several clauses. One of the predicates has been grounded,
		// another has not.  We want to now traverse upwards from this node,
		// to find the top of the ungrounded clause.
		curr_root = unsolved_clause;
		curr_pred_handle = pursue;

		if (Handle::UNDEFINED != unsolved_clause)
		{
			issued.insert(unsolved_clause);
			return true;
		}
	}

	return false;
}

/* ======================================================== */

/**
 * explore_neighborhood - explore the local (connected) neighborhood
 * of the starter clause, looking for a match.  The idea here is that
 * it is much easier to traverse a connected graph looking for the
 * appropriate subgraph (pattern) than it is to try to explore the
 * whole atomspace, at random.  The user callback `initiate_search()`
 * should call this method, suggesting a clause to start with, and
 * where in the clause the search should begin.
 *
 * Inputs:
 * do_clause: must be one of the clauses previously specified in the
 *            clause list of the match() method.
 * starter:   must be a sub-clause of do_clause; that is, must be a link
 *            that appears in do_clause.
 * ah:        must be a (non-variable) node in the "starter" clause.
 *            That is, this must be one of the outgoing atoms of the
 *            "starter" link, it must be a node, and it must not be
 *            a variable node.
 *
 * Returns true if one (or more) matches are found
 *
 * This routine is meant to be invoked on every candidate atom taken
 * from the atom space. That atom is assumed to anchor some part of
 * a graph that hopefully will match the pattern.
 */
bool PatternMatchEngine::explore_neighborhood(const Handle& do_clause,
                                      const Handle& starter,
                                      const Handle& ah)
{
	clause_stacks_clear();
	return explore_redex(do_clause, starter, ah);
}

/**
 * Same as above, obviously; we just pick up the graph context
 * where we last left it.
 */
bool PatternMatchEngine::explore_redex(const Handle& do_clause,
                                      const Handle& starter,
                                      const Handle& ah)
{
	// Cleanup
	clear_current_state();

	// Match the required clauses.
	curr_root = do_clause;
	curr_pred_handle = starter;
	issued.insert(curr_root);
	bool found = soln_up(ah);

	// If found is false, then there's no solution here.
	// Bail out, return false to try again with the next candidate.
	return found;
}

/**
 * Clear current traversal state. This gets us into a state where we
 * can start traversing a set of clauses.
 */
void PatternMatchEngine::clear_current_state(void)
{
	// Clear all state.
	var_grounding.clear();
	clause_grounding.clear();
	issued.clear();
	in_quote = false;

	curr_root = Handle::UNDEFINED;
	curr_soln_handle = Handle::UNDEFINED;
	curr_pred_handle = Handle::UNDEFINED;
	depth = 0;
}

/**
 * Unconditionally clear all graph traversal stacks
 */
void PatternMatchEngine::clause_stacks_clear(void)
{
	_clause_stack_depth = 0;
	while (!pred_handle_stack.empty()) pred_handle_stack.pop();
	while (!soln_handle_stack.empty()) soln_handle_stack.pop();
	while (!root_handle_stack.empty()) root_handle_stack.pop();
	while (!pred_solutn_stack.empty()) pred_solutn_stack.pop();
	while (!var_solutn_stack.empty()) var_solutn_stack.pop();
	while (!issued_stack.empty()) issued_stack.pop();

	have_more = false;
	more_depth = 0;
	more_stack.clear();
	while (!mute_stack.empty()) mute_stack.pop();
	while (!have_stack.empty()) have_stack.pop();
	while (!depth_stack.empty()) depth_stack.pop();
	while (!unordered_stack.empty()) unordered_stack.pop();
	while (!permutation_stack.empty()) permutation_stack.pop();
}

PatternMatchEngine::PatternMatchEngine(void)
{
	// current state
	in_quote = false;
	curr_root = Handle::UNDEFINED;
	curr_soln_handle = Handle::UNDEFINED;
	curr_pred_handle = Handle::UNDEFINED;
	depth = 0;

	// graph state
	_clause_stack_depth = 0;

	// unordered link state
	have_more = false;
	more_depth = 0;
}

/**
 * Clear only the internal clause declarations
 */
void PatternMatchEngine::clear_redex(const std::string& name)
{
	_redex_name = name;
	// Clear all pattern-related state.
	_bound_vars.clear();
	_cnf_clauses.clear();
	_mandatory.clear();
	_optionals.clear();
	_evaluatable.clear();
	_connectivity_map.clear();
}

/* ======================================================== */

void PatternMatchEngine::print_solution(
	const std::map<Handle, Handle> &vars,
	const std::map<Handle, Handle> &clauses)
{
	printf("\nNode groundings:\n");

	// Print out the bindings of solutions to variables.
	std::map<Handle, Handle>::const_iterator j = vars.begin();
	std::map<Handle, Handle>::const_iterator jend = vars.end();
	for (; j != jend; ++j)
	{
		Handle var(j->first);
		Handle soln(j->second);

		// Only print grounding for variables.
		if (VARIABLE_NODE != var->getType()) continue;

		if (soln == Handle::UNDEFINED)
		{
			printf("ERROR: ungrounded variable %s\n",
			       var->toShortString().c_str());
			continue;
		}

		printf("\t%s maps to %s\n",
		       var->toShortString().c_str(),
		       soln->toShortString().c_str());
	}

	// Print out the full binding to all of the clauses.
	printf("\nGrounded clauses:\n");
	std::map<Handle, Handle>::const_iterator m;
	int i = 0;
	for (m = clauses.begin(); m != clauses.end(); ++m, ++i)
	{
		if (m->second == Handle::UNDEFINED)
		{
			Handle mf(m->first);
			prtmsg("ERROR: ungrounded clause: ", mf);
			continue;
		}
		std::string str = m->second->toShortString();
		printf ("%d.   %s\n", i, str.c_str());
	}
	printf ("\n");
}

/**
 * For debug printing only
 */
void PatternMatchEngine::print_predicate(
                  const std::set<Handle> &vars,
                  const std::vector<Handle> &clauses)
{
	printf("\nClauses:\n");
	for (Handle h : clauses) prt(h);

	printf("\nVars:\n");
	for (Handle h : vars) prt(h);
}

/* ===================== END OF FILE ===================== */
