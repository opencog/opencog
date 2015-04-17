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
#include <opencog/util/Logger.h>
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
#ifdef DEBUG
   #define POPSTK(stack,soln) {         \
      OC_ASSERT(not stack.empty(),      \
           "Unbalanced stack " #stack); \
      soln = stack.top();               \
      stack.pop();                      \
   }
#else
   #define POPSTK(stack,soln) {         \
      soln = stack.top();               \
      stack.pop();                      \
   }
#endif

/* ======================================================== */

/**
 * tree_compare compares two trees, side-by-side.
 *
 * Compare two incidence trees, side-by-side.  The incidence tree is
 * given by following the "outgoing set" of the links appearing in the
 * tree.  The incidence tree is the so-called "Levi graph" of the
 * hypergraph.  The first arg should be a handle to a clause in the
 * pattern, while the second arg is a handle to a candidate grounding.
 * The pattern (template) clause is compared to the candidate grounding,
 * returning true if there is a mis-match.
 *
 * The comparison is recursive, so this method calls itself on each
 * subtree (term) of the template clause, performing comparisons until a
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
			throw InvalidParamException(TRACE_INFO,
			            "QuoteLink has unexpected arity!");
		more_depth ++;
		bool ma = tree_compare(lp->getOutgoingAtom(0), hg, CALL_QUOTE);
		more_depth --;
		in_quote = false;
		return ma;
	}

	// Handle hp is from the pattern clause, and it might be one
	// of the bound variables. If so, then declare a match.
	if (not in_quote and _varlist->varset.end() != _varlist->varset.find(hp))
	{
#ifdef NO_SELF_GROUNDING
		// But... if handle hg happens to also be a bound var,
		// then its a mismatch.
		if (_varlist->varset.end() != _varlist->varset.find(hg)) return false;
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
		    _varlist->varset.end() != _varlist->varset.find(hg))
		{
			return false;
		}
#endif

		// Else, we have a candidate grounding for this variable.
		// The node_match may implement some tighter variable check,
		// e.g. making sure that grounding is of some certain type.
		if (not _pmc.variable_match (hp,hg)) return false;

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
		if (hg == curr_term_handle)
		{
			// Mismatch, if hg contains bound vars in it.
			if (any_unquoted_in_tree(hg, _varlist->varset)) return false;
		}
		else
		{
#if THIS_CANT_BE_RIGHT
			// Bound but quoted variables cannot be solutions to themselves.
			// huh? whaaaat?
			if (not in_quote or
			    (in_quote and
			     (VARIABLE_NODE != tp or
			       _varlist->varset.end() == _varlist->varset.find(hp))))
			{
				if (hp != hg) var_grounding[hp] = hg;
			}
#endif // THIS_CANT_BE_RIGHT

		}
#endif
		// If the pattern contains atoms that are evaluatable i.e. GPN's
		// then we must fall through, and let the tree comp mechanism
		// find and evaluate them. That's for two reasons: (1) because
		// evaluation may have side-effects (e.g. send a message) and
		// (2) evaluation may depend on external state. These are
		// typically used to implement behavior trees, e.g SequenceUTest
		if (not is_evaluatable(hp)) return true;
		else
		{ dbgprt("Its evaluatable, continuing.\n"); }
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
		if (any_unquoted_in_tree(hg, _varlist->varset))
		{
			for (Handle vh: _varlist->varset)
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
		bool match = _pmc.link_match(lp, lg);
		if (not match) return false;

		dbgprt("depth=%d\n", depth);
		prtmsg("> tree_compare", hp);
		prtmsg(">           to", hg);

		// CHOICE_LINK's are multiple-choice links. As long as we can
		// can match one of the sub-expressions of the ChoiceLink, then
		// the ChoiceLink as a whole can be considered to be grounded.
		//
		if (CHOICE_LINK == tp)
		{
			const std::vector<Handle> &osp = lp->getOutgoingSet();

			// _choice_state lets use resume wher we last left off.
			size_t iend = osp.size();
			size_t istart = next_choice(hp, hg);
			for (size_t i=istart; i<iend; i++)
			{
				const Handle& hop = osp[i];

				dbgprt("tree_comp or_link choice %zu of %zu\n", i, iend);

				match = tree_recurse(hop, hg, CALL_CHOICE);
				if (match)
				{
					// If we've found a grounding, lets see if the
					// post-match callback likes this grounding.
					match = _pmc.post_link_match(lp, lg);
					if (not match) continue;

					// If we've found a grounding, record it.
					if (hp != hg) var_grounding[hp] = hg;
					_choice_state[Choice(hp, hg)] = i+1;
					return true;
				}
			}
			// If we are here, we've explored all the possibilities already
			_choice_state.erase(Choice(hp, hg));
			return false;
		}

		// If the two links are both ordered, its enough to compare
		// them "side-by-side"; the foreach_atom_pair iterator does
		// this. If they are un-ordered, then we have to compare every
		// possible permutation.  This can be a bit of a combinatoric
		// explosion.
		if (_classserver.isA(tp, ORDERED_LINK))
		{
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
			depth ++;
			more_depth ++;

			match = true;
			for (size_t i=0; i<oset_sz; i++)
			{
				if (not tree_recurse(osp[i], osg[i], CALL_ORDER))
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
			match = _pmc.post_link_match(lp, lg);
			if (not match) return false;

			// If we've found a grounding, record it.
			if (hp != hg) var_grounding[hp] = hg;

			return true;
		}
		else
		{
			// If we are here, we are dealing with an unordered link.
			// Enumerate all the permutations of the outgoing set of
			// the term.  We have to try all possible permuations
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
				POPSTK(mute_stack, mutation);
			}

			do {
				size_t oset_sz = mutation.size();
				if (oset_sz != osg.size()) return false;

				// The recursion step: traverse down the tree.
				dbgprt("tree_comp start downwards unordered link at depth=%lu\n",
				       more_depth);
				var_solutn_stack.push(var_grounding);
				// XXX why are we not pushing the claus soln stack??
				depth ++;

				have_more = false;
				more_depth ++;

				match = true;
				for (size_t i=0; i<oset_sz; i++)
				{
					if (not tree_recurse(mutation[i], osg[i], CALL_UNORDER))
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
					match = _pmc.post_link_match(lp, lg);
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
				POPSTK(var_solutn_stack, var_grounding);
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
		bool match = _pmc.node_match(hp, hg);
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

bool PatternMatchEngine::tree_recurse(const Handle& hp,
                                      const Handle& hg,
                                      Caller caller)
{
	return tree_compare(hp, hg, caller);
}

size_t PatternMatchEngine::next_choice(const Handle& hp,
                                       const Handle& hg)
{
	size_t istart;
	try { istart = _choice_state.at(Choice(hp, hg)); }
	catch(...) { istart = 0;}
	return istart;
}

/* ======================================================== */

/// start_sol_up -- look for a grounding for the gieven term.
///
/// The argument passed to this function is a term that needs to be
/// grounded. One of this term's children has already been grounded:
/// the term's child is in curr_term_handle, and the corresponding
/// grounding is in curr_soln_handle.  Thus, if the argument is going
/// to be grounded, it will be grounded by some atom in the incoming set
/// of cur_soln_handle. Viz, we are walking upwards in these trees,
/// in lockstep.
///
/// Returns true if a grounding for the term was found.
///
bool PatternMatchEngine::start_sol_up(const Handle& h)
{
	Handle curr_term_save(curr_term_handle);
	curr_term_handle = h;

	// Move up the solution graph, looking for a match.
	IncomingSet iset = _pmc.get_incoming_set(curr_soln_handle);
	size_t sz = iset.size();
	bool found = false;
	for (size_t i = 0; i < sz; i++) {
		found = xsoln_up(Handle(iset[i]));
		if (found) break;
	}

	curr_term_handle = curr_term_save;
	dbgprt("Found upward soln = %d\n", found);
	return found;
}

/// Return true if a grounding was found.
bool PatternMatchEngine::xsoln_up(const Handle& hsoln)
{
	// Let's not stare at our own navel. ... Unless the current
	// clause has GroundedPredicateNodes in it. In that case, we
	// have to make sure that they get evaluated.
	if ((hsoln == curr_root)
	    and not is_evaluatable(curr_root))
		return false;

	have_stack.push(have_more);
	have_more = false;

	do {
		bool found = false;

		do {
			solution_push();

			if (_need_choice_push) choice_stack.push(_choice_state);
			bool match = tree_recurse(curr_term_handle, hsoln, CALL_SOLN);
			if (_need_choice_push) POPSTK(choice_stack, _choice_state);
			_need_choice_push = false;

			// If no match, then try the next one.
			if (not match)
			{
				// Get rid of any grounding that might have been proposed
				// during the tree-match.
				solution_pop();
				POPSTK(have_stack, have_more);
				return false;
			}

			if (do_term_up(hsoln)) found = true;

			// Get rid of any grounding that might have been proposed
			// during the tree-compare or do_term_up.
			solution_pop();
		} while (0 < next_choice(curr_term_handle, hsoln));

		if (found)
		{
			POPSTK(have_stack, have_more);
			return true;
		}

		if (have_more) { dbgprt("Wait ----- there's more!\n"); }
		else { dbgprt("No more unordered, more_depth=%zd\n", more_depth); }
	} while (have_more);

	POPSTK(have_stack, have_more);
	return false;
}

/// do_term_up() -- move upwards from the current term.
///
/// Given the current term, in curr_term_handle, find its parent in the
/// clause, and then call start_sol_up() to see if the term's parent
/// has corresponding match in the solution graph.
///
/// Note that, in the "normal" case, a given term has only one, unique
/// parent in the given root_clause, and so its easy to find; one just
/// looks at the path from the root clause down to the term, and the
/// parent is the link immediately above it.
///
/// There are five exceptions to this "unique parent" case:
///  * The term is already the root clause; it has no parent. In this
///    case, we send it off to the machinery that explores the next
///    clause.
///  * Exactly the same term may appear twice, 3 times, etc. in the
///    clause, all at different locations.  This is very rare, but
///    can happen. In essence, it has multiple parents; each needs
///    to be checked. We loop over these.
///  * The term is a part of a larger, evaluatable term. In this case,
///    we don't want to go to the immediate parent, we want to go to
///    the larger evaluatable term, and offer that up as the thing to
///    match (i.e. to evaluate, to invoke callbacks, etc.)
///  * The parent is an ChoiceLink. In this case, the ChoiceLink
///    itself cannot be directly matched, as is; only its children can
///    be. So in this case, we fetch the ChoiceLink's parent, instead.
///  * Some crazy combination of the above.
///
/// If it weren't for these complications, this method would be small
/// and simple: it would send the parent to start_sol_up(), and start_sol_up()
/// would respond as to whether it is staisfiable (solvable) or not.
///
/// Takes as an argument the atom that is curently matched up to
/// curr_term_handle. Thus, curr_term_handle's parent will need to be
/// matched to hsoln's parent.
///
/// Returns true if a grounding for the term's parent was found.
///
bool PatternMatchEngine::do_term_up(const Handle& hsoln)
{
	depth = 1;

	// If we are here, then everything below us matches.  If we are
	// at the top of the clause, move on to the next clause. Else,
	// we are working on a term somewhere in the middle of a clause
	// and need to walk upwards.
	if (curr_term_handle == curr_root)
		return clause_accept(hsoln);

	// Move upwards in the term, and hunt for a match, again.
	// There are two ways to move upwards: for a normal term, we just
	// find its parent in the clause. For an evaluatable term, we find
	// the parent evaluatable in the clause, which may be many steps
	// higher.
	dbgprt("Term UUID = %lu of clause UUID = %lu has ground, move upwards.\n",
	       curr_term_handle.value(), curr_root.value());

	if (0 < _pat->in_evaluatable.count(curr_term_handle))
	{
		// If we are here, there are four possibilities:
		// 1) curr_term_handle is not in any evaluatable that lies
		//    between it and the clause root.  In this case, we need to
		//    fall through to the bottom.
		// 2) The evaluatable is the clause root. We evaluate it, and
		//    consider the clause satisfied if the evaluation returns
		//    true. In that case, we continue to te next clause, else we
		//    backtrack.
		// 3) The evaluatable is in the middle of a clause, in which case,
		//    it's parent must be a logical connective: an AndLink, an
		//    OrLink or a NotLink. In this case, we have to loop over
		//    all of the evaluatables within this clause, and connect
		//    them as appropriate. The structure may be non-trivial, so
		//    that presents a challange.  However, it must be logical
		//    connectives all the way up to the root of the clause, so the
		//    easiest thing to do is simply to start at the top, and
		//    recurse downwards.  Ergo, this is much like case 2): the
		//    evaluation either suceeds or fails; we proceed or backtrack.
		// 4) The evaluatable is in the middle of something else. We don't
		//    know what that means, so we throw an error. Actually, this
		//    is too harsh. It may be in the middle of some function that
		//    expects a boolean value as an argument. But I don't know of
		//    any, just right now.
		//
		// Anyway, all of this talk abbout booleans is emphasizing the
		// point that, someday, we need to replace this crisp logic with
		// probabalistic logic of some sort. XXX TODO. The fuzzy matcher
		// tries to do this, but I'm not sure its correct. We eventually
		// need to do this here, not there.
		//
		// By the way, if we are here, then curr_term_handle is surely
		// a variable, at least it is, if we are working in the canonical
		// interpretation.

		auto evra = _pat->in_evaluatable.equal_range(curr_term_handle);
		for (auto evit = evra.first; evit != evra.second; evit++)
		{
			if (not is_unquoted_in_tree(curr_root, evit->second))
				continue;

			prtmsg("Term inside evaluatable, move up to it's top:\n",
			        evit->second);

			// All of the variables occurring in the term should have
			// grounded by now. If not, then its virtual term, and we
			// shouldn't even be here (we can't just backtrack, and
			// try again later).  So validate the grounding, but leave
			// the evaluation for the callback.
// XXX TODO count the number of ungrounded vars !!! (make sure its zero)
// XXX TODO make sure that all links from the curr_root to the term are
// connectives (i.e. are in the _connectives set).  Else throw an error.
// why bother with this extra overhead, though?? Do we really need to do
// this?

			bool found = _pmc.evaluate_sentence(curr_root, var_grounding);
			dbgprt("After evaluating clause, found = %d\n", found);
			if (found)
			{
				curr_term_handle = evit->second;
				return clause_accept(hsoln);
			}
			return false;
		}
	}

	FindAtoms fa(curr_term_handle);
	fa.search_set(curr_root);

	// It is almost always the case, but not necessarily, that
	// least_holders contains one atom. (i.e. the one atom that
	// is the parent of curr_term_handle that is within curr_root.
	// If curr_term_handle appears twice (or N times) in a curr_root,
	// then it will show up twice (or N times) in least_holders.
	// As far as I can tell, it is sufficient to examine only the
	// first appearance in almost all cases, unless the holders
	// lie below an ChoiceLink.  For ChoiceLinks, we really have to
	// examine all the different holders, they correspond to the
	// different choices.
	OC_ASSERT(0 < fa.least_holders.size(), "Impossible situation");

	bool found = false;
	for (const Handle& hi : fa.least_holders)
	{
		// Do the simple case first, ChoiceLinks are harder.
		if (CHOICE_LINK != hi->getType())
		{
			dbgprt("Exploring one possible embedding out of %zu\n",
			       fa.least_holders.size());
			soln_handle_stack.push(curr_soln_handle);
			curr_soln_handle = hsoln;

			if (start_sol_up(hi)) found = true;

			POPSTK(soln_handle_stack, curr_soln_handle);

			dbgprt("After moving up the clause, found = %d\n", found);
		}
		else
		if (hi == curr_root)
		{
			dbgprt("Exploring one possible ChoiceLink at root out of %zu\n",
			       fa.least_holders.size());
			curr_term_handle = hi;
			if (clause_accept(hsoln)) found = true;
		}
		else
		{
			// If we are here, we have an embedded ChoiceLink, i.e. a
			// ChoiceLink that is not at the clause root. It's contained
			// in some other link, and we have to get that link and
			// perform comparisons on it. i.e. we have to "hop over"
			// (hop up) past the ChoiceLink, before resuming the search.
			// The easiest way to hop is to do it recursively... i.e.
			// call ourselves again.
			dbgprt("Exploring one possible ChoiceLink in clause out of %zu\n",
			       fa.least_holders.size());

			OC_ASSERT(0 == next_choice(hi, hsoln),
			          "Something is wrong with the ChoiceLink code");

			soln_handle_stack.push(curr_soln_handle);
			curr_soln_handle = hsoln;
			solution_push();

			_need_choice_push = true;
			curr_term_handle = hi;
			if (do_term_up(hsoln)) found = true;

			solution_pop();
			POPSTK(soln_handle_stack, curr_soln_handle);
		}
	}
	dbgprt("Done exploring %zu choices, found %d\n",
	       fa.least_holders.size(), found);
	return found;
}

/// This is called when we've navigated to the top of a clause,
/// and so it is fully grounded, and we're essentially done.
/// However, let the callbacks have the final say on whether to
/// proceed onwards, or to backtrack.
///
bool PatternMatchEngine::clause_accept(const Handle& hsoln)
{
	// Is this clause a required clause? If so, then let the callback
	// make the final decision; if callback rejects, then it's the
	// same as a mismatch; try the next one.
	bool match;
	if (is_optional(curr_root))
	{
		clause_accepted = true;
		match = _pmc.optional_clause_match(curr_term_handle, hsoln);
		dbgprt("optional clause match callback match=%d\n", match);
	}
	else
	{
		match = _pmc.clause_match(curr_term_handle, hsoln);
		dbgprt("clause match callback match=%d\n", match);
	}
	if (not match) return false;

	curr_soln_handle = hsoln;
	clause_grounding[curr_root] = curr_soln_handle;
	prtmsg("---------------------\nclause:", curr_root);
	prtmsg("ground:", curr_soln_handle);

	// Now go and do more clauses.
	return do_next_clause();
}

// This is called when all previous clauses have been grounded; so
// we search for the next one, and try to round that.
bool PatternMatchEngine::do_next_clause(void)
{
	clause_stacks_push();
	get_next_untried_clause();

	// If there are no further clauses to solve,
	// we are really done! Report the solution via callback.
	bool found = false;
	if (Handle::UNDEFINED == curr_root)
	{
#ifdef DEBUG
		dbgprt ("==================== FINITO!\n");
		print_solution(var_grounding, clause_grounding);
#endif
		found = _pmc.grounding(var_grounding, clause_grounding);
	}
	else
	{
		prtmsg("Next clause is", curr_root);
		dbgprt("This clause is %s\n",
			is_optional(curr_root)? "optional" : "required");
		dbgprt("This clause is %s\n",
			is_evaluatable(curr_root)?
			"dynamically evaluatable" : "non-dynamic");
		prtmsg("Joining variable  is", curr_term_handle);
		prtmsg("Joining grounding is", var_grounding[curr_term_handle]);

		// Else, start solving the next unsolved clause. Note: this is
		// a recursive call, and not a loop. Recursion is halted when
		// the next unsolved clause has no grounding.
		//
		// We continue our search at the atom that "joins" (is shared
		// in common) between the previous (solved) clause, and this
		// clause. If the "join" was a variable, look up its grounding;
		// else the join is a 'real' atom.

		clause_accepted = false;
		curr_soln_handle = var_grounding[curr_term_handle];
		OC_ASSERT(curr_soln_handle != Handle::UNDEFINED,
			"Error: joining handle has not been grounded yet!");
		found = xsoln_up(curr_soln_handle);

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
		       (is_optional(curr_root)))
		{
			Handle undef(Handle::UNDEFINED);
			bool match = _pmc.optional_clause_match(curr_term_handle, undef);
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
				found = _pmc.grounding(var_grounding, clause_grounding);
			}
			else
			{
				// Now see if this optional clause has any solutions,
				// or not. If it does, we'll recurse. If it does not,
				// we'll loop around back to here again.
				clause_accepted = false;
				curr_soln_handle = var_grounding[curr_term_handle];
				found = xsoln_up(curr_soln_handle);
			}
		}
	}

	// If we failed to find anything at this level, we need to
	// backtrack, i.e. pop the stack, and begin a search for
	// other possible matches and groundings.
	clause_stacks_pop();

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
	if (_pat->optionals.empty())
	{
		// There are no more ungrounded clauses to consider. We are done.
		curr_root = Handle::UNDEFINED;
		curr_term_handle = Handle::UNDEFINED;
		return;
	}

	// Try again, this time, considering the optional clauses.
	if (get_next_untried_helper(true)) return;

	// If we are here, there are no more unsolved clauses to consider.
	curr_root = Handle::UNDEFINED;
	curr_term_handle = Handle::UNDEFINED;
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
	for (const Pattern::ConnectPair& vk : _pat->connectivity_map)
	{
		const Pattern::RootList& rl(vk.second);
		pursue = vk.first;
		if (Handle::UNDEFINED == var_grounding[pursue]) continue;
#else // OLD_LOOP
	// Newloop might be slightly faster than old loop.
	// ... but maybe not...
	for (auto gndpair : var_grounding)
	{
		pursue = gndpair.first;
		try { _pat->connectivity_map.at(pursue); }
		catch(...) { continue; }
		const RootList& rl(_pat->connectivity_map.at(pursue));
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
			          (not is_optional(root))))
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
		// several clauses. One of the clauses has been grounded,
		// another has not.  We want to now traverse upwards from this node,
		// to find the top of the ungrounded clause.
		curr_root = unsolved_clause;
		curr_term_handle = pursue;

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
	term_handle_stack.push(curr_term_handle);
	soln_handle_stack.push(curr_soln_handle);

	var_solutn_stack.push(var_grounding);
	term_solutn_stack.push(clause_grounding);

	issued_stack.push(issued);
	choice_stack.push(_choice_state);

	// Reset the unordered-set stacks with each new clause.
	// XXX this cannot possibly be right: we may need to pop,
	// and try a different ordering.
	have_stack.push(have_more);
	have_more = false;
	depth_stack.push(more_depth);
	more_depth = 0;
	unordered_stack.push(more_stack);
	more_stack.resize(1);
	more_stack[0] = false;
	permutation_stack.push(mute_stack);

	_pmc.push();
}

/**
 * Pop all clause-traversal-related stacks. This restores state
 * so that the traversal of a single clause can resume where it left
 * off. These do NOT affect any of the redex stacks (which are assumed
 * to be empty at this point.)
 */
void PatternMatchEngine::clause_stacks_pop(void)
{
	_pmc.pop();
	POPSTK(root_handle_stack, curr_root);
	POPSTK(term_handle_stack, curr_term_handle);
	POPSTK(soln_handle_stack, curr_soln_handle);

	// The grounding stacks are handled differently.
	POPSTK(term_solutn_stack, clause_grounding);
	POPSTK(var_solutn_stack, var_grounding);
	POPSTK(issued_stack, issued);

	POPSTK(choice_stack, _choice_state);

	// Handle different unordered links that live in different
	// clauses. The mute_stack deals with different unordered
	// links that live in the *same* clause.
	POPSTK(have_stack, have_more);
	POPSTK(depth_stack, more_depth);
	POPSTK(unordered_stack, more_stack);
	POPSTK(permutation_stack, mute_stack);

	_clause_stack_depth --;

	dbgprt("pop to depth %d\n", _clause_stack_depth);
	prtmsg("pop to joiner", curr_term_handle);
	prtmsg("pop to clause", curr_root);
}

void PatternMatchEngine::solution_push(void)
{
	var_solutn_stack.push(var_grounding);
	term_solutn_stack.push(clause_grounding);
}

void PatternMatchEngine::solution_pop(void)
{
	POPSTK(var_solutn_stack, var_grounding);
	POPSTK(term_solutn_stack, clause_grounding);
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
	curr_term_handle = starter;
	issued.insert(curr_root);
	bool found = xsoln_up(ah);

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
	curr_term_handle = Handle::UNDEFINED;
	depth = 0;

	_choice_state.clear();
}

/**
 * Unconditionally clear all graph traversal stacks
 */
void PatternMatchEngine::clause_stacks_clear(void)
{
	_clause_stack_depth = 0;
	while (!term_handle_stack.empty()) term_handle_stack.pop();
	while (!soln_handle_stack.empty()) soln_handle_stack.pop();
	while (!root_handle_stack.empty()) root_handle_stack.pop();
	while (!term_solutn_stack.empty()) term_solutn_stack.pop();
	while (!var_solutn_stack.empty()) var_solutn_stack.pop();
	while (!issued_stack.empty()) issued_stack.pop();
	while (!choice_stack.empty()) choice_stack.pop();

	have_more = false;
	more_depth = 0;
	more_stack.clear();
	while (!mute_stack.empty()) mute_stack.pop();
	while (!have_stack.empty()) have_stack.pop();
	while (!depth_stack.empty()) depth_stack.pop();
	while (!unordered_stack.empty()) unordered_stack.pop();
	while (!permutation_stack.empty()) permutation_stack.pop();
}

PatternMatchEngine::PatternMatchEngine(PatternMatchCallback& pmcb,
                                       const Variables& v,
                                       const Pattern& p)
	: _pmc(pmcb),
	_classserver(classserver()),
	_varlist(&v),
	_pat(&p)
{
	// current state
	in_quote = false;
	curr_root = Handle::UNDEFINED;
	curr_soln_handle = Handle::UNDEFINED;
	curr_term_handle = Handle::UNDEFINED;
	depth = 0;
	_need_choice_push = false;

	// graph state
	_clause_stack_depth = 0;

	// unordered link state
	have_more = false;
	more_depth = 0;
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
void PatternMatchEngine::print_term(
                  const std::set<Handle> &vars,
                  const std::vector<Handle> &clauses)
{
	printf("\nClauses:\n");
	for (Handle h : clauses) prt(h);

	printf("\nVars:\n");
	for (Handle h : vars) prt(h);
}

/* ===================== END OF FILE ===================== */
