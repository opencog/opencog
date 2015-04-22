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
/* ======================================================== */
/**
 * Pattern Match Engine Overview
 * -----------------------------
 * The explanation of how this code *actually* qorks has been
 * moved to README-Algorithm. That README provies the "big-picture"
 * explanation of the rather complex interwoven code below. Read it
 * first, and refert to it when examining the main methods below.
 */
/* ======================================================== */

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

/// If the pattern link is a quote, then we compare the quoted
/// contents. This is done recursively, of course.  The QuoteLink
/// must have only one child; anything else beyond that is ignored
/// (as its not clear what else could possibly be done).
bool PatternMatchEngine::quote_compare(const Handle& hp,
                                       const Handle& hg)
{
	in_quote = true;
	LinkPtr lp(LinkCast(hp));
	if (1 != lp->getArity())
		throw InvalidParamException(TRACE_INFO,
		            "QuoteLink has unexpected arity!");
	bool ma = tree_compare(lp->getOutgoingAtom(0), hg, CALL_QUOTE);
	in_quote = false;
	return ma;
}

/* ======================================================== */

/// Compare a VariableNode in the pattern to the proposed grounding.
///
/// Handle hp is from the pattern clause.
bool PatternMatchEngine::variable_compare(const Handle& hp,
                                          const Handle& hg)
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

/* ======================================================== */

/// Compare an atom to itself. Amazingly, this is more complicated
/// that what it might seem to be ...
///
/// If they're the same atom, then clearly they match.
/// ... but only if hp is a constant i.e. contains no bound variables)
bool PatternMatchEngine::self_compare(const Handle& hp)
{
	prtmsg("Compare atom to itself:\n", hp);

#ifdef NO_SELF_GROUNDING
	if (hp == curr_term_handle)
	{
		// Mismatch, if hg contains bound vars in it.
		if (any_unquoted_in_tree(hp, _varlist->varset)) return false;
		return true;
	}
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
#endif
	return true;
}

/* ======================================================== */

/// Compare two nodes, one in the pattern, one proposed grounding.
bool PatternMatchEngine::node_compare(const Handle& hp,
                                      const Handle& hg)
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

/* ======================================================== */

/// If the two links are both ordered, its enough to compare
/// them "side-by-side".
bool PatternMatchEngine::ordered_compare(const Handle& hp,
                                         const Handle& hg,
                                         const LinkPtr& lp,
                                         const LinkPtr& lg)
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

	bool match = true;
	for (size_t i=0; i<oset_sz; i++)
	{
		if (not tree_compare(osp[i], osg[i], CALL_ORDER))
		{
			match = false;
			break;
		}
	}

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

/* ======================================================== */

/// Compare a ChoiceLink in the pattern to the proposed grounding.
/// hp points at the ChoiceLink.
///
/// CHOICE_LINK's are multiple-choice links. As long as we can
/// can match one of the sub-expressions of the ChoiceLink, then
/// the ChoiceLink as a whole can be considered to be grounded.
///
bool PatternMatchEngine::choice_compare(const Handle& hp,
                                        const Handle& hg,
                                        const LinkPtr& lp,
                                        const LinkPtr& lg)
{
	const std::vector<Handle> &osp = lp->getOutgoingSet();

	// _choice_state lets use resume where we last left off.
	size_t iend = osp.size();
	bool fresh = false;
	size_t icurr = curr_choice(hp, hg, fresh);
	if (fresh) choose_next = false; // took a step, clear the flag

	dbgprt("tree_comp resume choice search at %zu of %zu of UUID=%lu "
          "choose_next=%d\n",
	       icurr, iend, hp.value(), choose_next);

	// XXX This is almost surely wrong... if there are two
	// nested choice links, then this will hog the steps,
	// and the deeper choice will fail.
	if (choose_next)
	{
		icurr++;
		choose_next = false; // we are taking a step, so clear the flag.
	}

	while (icurr<iend)
	{
		solution_push();
		const Handle& hop = osp[icurr];

		dbgprt("tree_comp or_link choice %zu of %zu\n", icurr, iend);

		bool match = tree_compare(hop, hg, CALL_CHOICE);
		if (match)
		{
			// If we've found a grounding, lets see if the
			// post-match callback likes this grounding.
			match = _pmc.post_link_match(lp, lg);
			if (match)
			{
				// Even the stack, *without* erasing the discovered grounding.
				var_solutn_stack.pop();

				// If the grounding is accepted, record it.
				if (hp != hg) var_grounding[hp] = hg;

				_choice_state[Choice(hp, hg)] = icurr;
				return true;
			}
		}
		solution_pop();
		choose_next = false; // we are taking a step, so clear the flag.
		icurr++;
	}

	// If we are here, we've explored all the possibilities already
	_choice_state.erase(Choice(hp, hg));
	return false;
}

/// Return the current choice state for the given pattern & ground
/// combination.
size_t PatternMatchEngine::curr_choice(const Handle& hp,
                                       const Handle& hg,
                                       bool& fresh)
{
	size_t istart;
	try { istart = _choice_state.at(Choice(hp, hg)); }
	catch(...) { istart = 0; fresh = true; }
	return istart;
}

bool PatternMatchEngine::have_choice(const Handle& hp,
                                     const Handle& hg)
{
#if USE_AT
	bool have = true;
	try { _choice_state.at(Choice(hp, hg)); }
	catch(...) { have = false;}
	return have;
#else
	return 0 < _choice_state.count(Choice(hp, hg));
#endif
}

/* ======================================================== */
#ifdef DEBUG
static int facto (int n) { return (n==1)? 1 : n * facto(n-1); };
#endif

/// Unordered link comparison
///
/// Compare two unordered links, side by side. In some ways, this is
/// similar to the ordered link compare: for a given, fixed permuation
/// of the unordered link, the compare is side by side.  However, if
/// that compare fails, the next permuation must then be tried, until
/// a match is found or all permuations are exhausted.  But there's a
/// problem: if there are multiple, nested unordered links, or if they
/// are peers (siblings) in the tree, then if one takes a step, the
/// other must not. Coordinating this is difficult, and requires a long
/// explanation. So here goes:
///
/*****************************************************************

How do unordered links work?
----------------------------
This is complicted, so we write it out.  When ascending from below (i.e.
from do_term_up()), unordered links may be found in two different
places: The parent term may be unordered, or the parent link may hold
another link (a sibling to us) that is unordered. Traversal needs to
handle both cases.  Thus, the upwards-movement methods (do_term_sup(),
explore_up_branches(), etc.) are incapable of discovering unordered links,
as they cannot "see" the siblings.  Siblings can only be found during
tree-compare, moving downwards.  Thus, tree_compare must do a lot of
heavy lifting.

When comparing trees downwards, we have two situations we may be in:
we may be asked to compare things exactly like last time, or we may be
asked to try out the next permutation. We need to report back two bits
of state: whether or not we found a match, and whether or not we have
more possibilities to explore. Our behavior is thus controlled by this
third bit, as well as what happened below us.

The correct actions to take are best explored by truth tables: the
settings of the take_step and have_more flags on entry, and what to
do after running tree_compare downwards. These are handled by two
truth tables.

The topmost routine to call tree_compare must *always* set have_more=F
and take_step=T before calling tree compare.  This will cause
tree_compare to advance to the next matching permuation, or to run until
all permuations are exhausted, and no match was found.


Flag settings upon entry
------------------------

     take  have
case step  more    Comments / Action to be Taken
---- ----  ----    -----------------------------
  A    T    T     Impossible, Who set have_more? Who set take_step?
  B    T    F     Normal entry: we are first unordered link to be hit
                  during downward travesal. Proceed using the truth
                  table below.
  C    F    T     We are not the first unorder. Someone ran before us.
                  If we are in the very first permuation, (i.e. we are
                    entering for the first time) we must call downwards.
                    If this returns a mismatch, we must step until we
                    find a match, or until we exhaust all permuatations.
                    See next table for what to return: we return cases
                    5, 7 or 8.
                  If we are not the first permutation, we could just
                    return T, because that is what we did last time.
                    i.e. we are told to not take a step, so we don't,
                    and we know a-priori last time were were here, we
                    must have returned a match.  Thus, we can return
                    case 5 below.  We cannot return case 7 because we
                    can't know what std::next_perm returns.
                    (See however, footnote below).
                  If we hold an evaluatable, we must call down.
  D    F    F     Perform same as C above.

Footnote: case C: Well, thr reasoning there is almost riht, but not
quite. If the unordered link contains a variable, and it is also not in
the direct line of exploration (i.e. its grounding is NOT recorded)
then its truthiness holds only for a grounding that no longer exists.
Thus, for case C, it is safer to always check.

However, by the above reasoning: if the grounding wasn't recorded
(because the link is not in te recursion path) then the permuation
should not be recorded either. It should start with a permutation from
nothing.  XXX FIXME ... except we have no test case that illustrates
this failure mode.  It would require a peer unordered link that takes
a different order when the parent changes. Perhaps unordered links
nested inside a ChoiceLink would trigger this?

If case B was encountered on entry, we call downwards ourselves, and
then report back, according to the truth table below.

     returned result flags
     take  have   got
case step  more  match    Comments / Action to tbe Taken
---- ----  ----  ----     ------------------------------
 1     T    T      T    Impossible, error: who set have_more w/o
                          taking step??
 2     T    T      F    ditto
 3     T    F      T    We have no unordered links below us; we are at
                        the bottom.  If there had been unordered links,
                        they would have cleared the take_step flag. The
                        match we detected is the same match the last
                        time we were here. So we take a step, call
                        down again, and keep stepping until there is a
                        match or until all permuations are exhausted.
                          If match, we return: take_step=F,
                            have_more = result of std::next_perm
                            (we return case 5 or 7)
                          If exhaust, we return take_step=F, have_more=F
                            (we return case 8)

 4     T    F      F    If we are not holding any evaluatable links,
                        then this is impossible, as last time we were
                        here, we returned T.  If we are holding
                        evaluatable links, then one of them changed its
                        mind. Oh well. Take a step, proceed as in case 3.

 5     F    T      T    Someone below us took a step. Do nothing.
                        Return case 5 flags.
 6     F    T      F    Impossible; link that took the step should have
                        stepped until match or exhastion.
 7     F    F      T    Unusual; its the last match for a lower unordered
                        link. We report the match. We do not take a
                        step; we do set the have_more flag to indicate
                        that we ourselves still have more.  i.e We return
                        case 5 flags.
 8     F    F      F    Typical; a lower unord link ran to exhaustion,
                        and got nada.  So *we* take a step, and call
                        downwards again. We keep  going till match or till
                        exhaustion. If there's a match, we expect to see
                        the case 5 flags, so we halt and return.  If we
                        exhaust, we return case 8.

The above assumes that the curr_perm() method always returns either
the current permuation, or it returns a fresh permuation. If it returned
a fresh permuation, this counts as "taking a step", so we need to know
this.

Notice that these rules never pushed of popped the have-more stack.
The have-more stack is only pushed/popped by other branchpoints, before
they call compare_tree.

******************************************************************/

bool PatternMatchEngine::unorder_compare(const Handle& hp,
                                         const Handle& hg,
                                         const LinkPtr& lp,
                                         const LinkPtr& lg)
{
	const HandleSeq& osg = lg->getOutgoingSet();
	const HandleSeq& osp = lp->getOutgoingSet();
	size_t arity = osp.size();

	// They've got to be the same size, at the least!
	if (osg.size() != arity) return false;

	// Test for case A, described above.
	OC_ASSERT (not (take_step and have_more),
	           "Impossible situation! BUG!");

	// _perm_state lets use resume where we last left off.
	bool fresh = false;
	Permutation mutation = curr_perm(hp, hg, fresh);
	if (fresh) take_step = false; // took a step, clear the flag.

	// Cases C and D fall through.
	// If we are here, we've got possibilities to explore.
#ifdef DEBUG
	int num_perms = facto(mutation.size());
	dbgprt("tree_comp resume unordered search at %d of %d of UUID=%lu "
	       "take_step=%d have_more=%d\n",
	       perm_count[Unorder(hp, hg)], num_perms, hp.value(),
	       take_step, have_more);
#endif
	do
	{
		dbgprt("tree_comp explore unordered perm %d of %d of UUID=%lu\n",
		       perm_count[Unorder(hp, hg)], num_perms, hp.value());

		solution_push();
		bool match = true;
		for (size_t i=0; i<arity; i++)
		{
			if (not tree_compare(mutation[i], osg[i], CALL_UNORDER))
			{
				match = false;
				break;
			}
		}

		// Check for cases 1&2 of description above.
		// The step-next may have been taken by someone else, in the
		// tree_compare immediate above.
		OC_ASSERT(not (take_step and have_more),
		          "This shouldn't happen. Impossible situation! BUG!");

		// Handle cases 3&4 of the description above. Seems wisest
		// to do this before post_link_match() !??
		if (take_step and not have_more)
		{
			OC_ASSERT(match or (0 < _pat->evaluatable_holders.count(hp)),
			          "Impossible: should have matched!");
			goto take_next_step;
		}

		// If we are here, then take_step is false, because
		// cases 1,2,3,4 already handled above.
		// Well, actually, this can happen, if we are not careful
		// to manage the have_more flag on a stack.
//		OC_ASSERT(match or not have_more or 1==num_perms,
//		          "Impossible: case 6 happened!");

		if (match)
		{
			// If we've found a grounding, lets see if the
			// post-match callback likes this grounding.
			match = _pmc.post_link_match(lp, lg);
			if (match)
			{
				// Even the stack, *without* erasing the discovered grounding.
				var_solutn_stack.pop();

				// If the grounding is accepted, record it.
				if (hp != hg) var_grounding[hp] = hg;

				// Handle case 5&7 of description above.
				have_more = true;
				dbgprt("Good permutation %d for UUID=%lu have_more=%d\n",
				       perm_count[Unorder(hp, hg)], hp.value(), have_more);
				_perm_state[Unorder(hp, hg)] = mutation;
				return true;
			}
		}
		// If we are here, we are handling case 8.
		dbgprt("Above permuation %d failed UUID=%lu\n",
		       perm_count[Unorder(hp, hg)], hp.value());

take_next_step:
		take_step = false; // we are taking a step, so clear the flag.
		have_more = false; // start with a clean slate...
		solution_pop();
#ifdef DEBUG
		perm_count[Unorder(hp, hg)] ++;
#endif
	} while (std::next_permutation(mutation.begin(), mutation.end()));

	// If we are here, we've explored all the possibilities already
	dbgprt("Exhausted all permuations of UUID=%lu\n", hp.value());
	_perm_state.erase(Unorder(hp, hg));
	have_more = false;
	return false;
}

/// Return the saved unordered-link permutation for this
/// particular point in the tree comparison (i.e. for the
/// particular unordered link hp in the pattern.)
PatternMatchEngine::Permutation
PatternMatchEngine::curr_perm(const Handle& hp,
                              const Handle& hg,
                              bool& fresh)
{
	Permutation perm;
	try { perm = _perm_state.at(Unorder(hp, hg)); }
	catch(...)
	{
#ifdef DEBUG
		dbgprt("tree_comp fresh start unordered link UUID=%lu\n", hp.value());
		perm_count[Unorder(hp, hg)] = 0;
#endif
		LinkPtr lp(LinkCast(hp));
		perm = lp->getOutgoingSet();
		sort(perm.begin(), perm.end());
		fresh = true;
	}
	return perm;
}

/// Return true if there are more permutations to explore.
/// Else return false.
bool PatternMatchEngine::have_perm(const Handle& hp,
                                   const Handle& hg)
{
	try { _perm_state.at(Unorder(hp, hg)); }
	catch(...) { return false; }
	return true;
}

void PatternMatchEngine::perm_push(void)
{
	perm_stack.push(_perm_state);
#ifdef DEBUG
	perm_count_stack.push(perm_count);
#endif
}

void PatternMatchEngine::perm_pop(void)
{
	POPSTK(perm_stack, _perm_state);
#ifdef DEBUG
	POPSTK(perm_count_stack, perm_count);
#endif
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
		return quote_compare(hp, hg);

	// Handle hp is from the pattern clause, and it might be one
	// of the bound variables. If so, then declare a match.
	if (not in_quote and _varlist->varset.end() != _varlist->varset.find(hp))
		return variable_compare(hp, hg);

	// If they're the same atom, then clearly they match.
	// ... but only if hp is a constant i.e. contains no bound variables)
	//
	// If the pattern contains atoms that are evaluatable i.e. GPN's
	// then we must fall through, and let the tree comp mechanism
	// find and evaluate them. That's for two reasons: (1) because
	// evaluation may have side-effects (e.g. send a message) and
	// (2) evaluation may depend on external state. These are
	// typically used to implement behavior trees, e.g SequenceUTest
	if ((hp == hg) and not is_evaluatable(hp))
		return self_compare(hp);

	// If both are nodes, compare them as such.
	NodePtr np(NodeCast(hp));
	NodePtr ng(NodeCast(hg));
	if (np and ng)
		return node_compare(hp, hg);

	// If they're not both are links, then it is clearly a mismatch.
	LinkPtr lp(LinkCast(hp));
	LinkPtr lg(LinkCast(hg));
	if (not (lp and lg)) return false;

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
		return redex_compare(lp, lg);

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
		return choice_compare(hp, hg, lp, lg);

	// If the two links are both ordered, its enough to compare
	// them "side-by-side".
	if (_classserver.isA(tp, ORDERED_LINK))
		return ordered_compare(hp, hg, lp, lg);

	// If we are here, we are dealing with an unordered link.
	return unorder_compare(hp, hg, lp, lg);
}

/* ======================================================== */

/// explore_up_branches -- look for groundings for the given term.
///
/// The argument passed to this function is a term that needs to be
/// grounded. One of this term's children has already been grounded:
/// the term's child is in curr_term_handle, and the corresponding
/// grounding is in curr_soln_handle.  Thus, if the argument is going
/// to be grounded, it will be grounded by some atom in the incoming set
/// of cur_soln_handle. Viz, we are walking upwards in these trees,
/// in lockstep.
///
/// This method wraps the major branch-point of the entire pattern
/// matching process. Each element of the incoming set is the start of
/// a different possible branch to be explored; each one might yeild
/// a grounding. Thus, when backtracking, after a failed grounding in
/// one branch, we backtrack to here, and try another branch. When
/// backtracking, all state must be popped and pushed again, to enter
/// the new branch. We don't pushd & pop here, we push-n-pop in the
/// explore_link_branches() method.
///
/// This method is part of a recursive chain that only terminates
/// when a grounding for *the entire pattern* was found (and the
/// grounding was accepted) or if all possibilities were exhaustively
/// explored.  Thus, this returns true only if entire pattern was
/// grounded.
///
bool PatternMatchEngine::explore_up_branches(const Handle& hp,
                                             const Handle& hg)
{
	// Move up the solution graph, looking for a match.
	IncomingSet iset = _pmc.get_incoming_set(hg);
	size_t sz = iset.size();
	dbgprt("Looking upward for pat-UUID=%lu have %zu branches\n",
	        hp.value(), sz);
	bool found = false;
	for (size_t i = 0; i < sz; i++) {
		dbgprt("Try upward branch %zu of %zu for pat-UUID=%lu propose=%lu\n",
		       i, sz, hp.value(), Handle(iset[i]).value());
		found = explore_link_branches(hp, Handle(iset[i]));
		if (found) break;
	}

	dbgprt("Found upward soln = %d\n", found);
	return found;
}

/// explore_link_branches -- verify the suggested grounding.
///
/// There are two ways to understand this method. In the "simple" case,
/// where there are no unordered links, and no ChoiceLinks, this becomes
/// a simple wrapper around tree_compare(), and it just returns true of
/// false to indicate if the suggested grounding `hsoln` actually is a
/// match for the current term being grounded. Before calling
/// tree_compare(), it pushes all current state, and then pops it upon
/// return. In other words, this encapsulates a single up-branch
/// (incoming-set branch): grounding of that single branch succeeds or
/// fails. Failure backtracks to the caller of this method; upon return,
/// the current state has been restored; this routine leaves the current
/// state as it found it. For the simple case, this method is mis-named:
/// it should be called "explore_one_branch".
///
/// The non-simple case is a pattern that includes ChoiceLinks or
/// unordered links. These links represent branch-points themselves.
/// A ChoiceLink of arity N wraps N different possible branches to be
/// explored. An unordered link of arity N wraps N-factorial different
/// possible permuations, each of which must be explored. This method
/// controls the exploration of these different branches. For each
/// possible branch, it saves state, explores the branch, and pops the
/// state. If the exploration yielded nothing, then the next bracnh is
/// explored, until exhaustion of the possibilities.  Upon exhaustion, it
/// returns to the caller.
///
/// This method is part of a recursive chain that only terminates
/// when a grounding for *the entire pattern* was found (and the
/// grounding was accepted) or if all possibilities were exhaustively
/// explored.  Thus, this returns true only if entire pattern was
/// grounded.
///
bool PatternMatchEngine::explore_link_branches(const Handle& hp,
                                               const Handle& hg)
{
	// Let's not stare at our own navel. ... Unless the current
	// clause has GroundedPredicateNodes in it. In that case, we
	// have to make sure that they get evaluated.
	if ((hg == curr_root)
	    and not is_evaluatable(curr_root))
		return false;

	// If its not an unordered link, then don't try to iterate.
	curr_term_type = hp->getType();
	if (not _classserver.isA(curr_term_type, UNORDERED_LINK))
		return explore_choice_branches(hp, hg);

	do {
		// If the pattern was satisfied, then we are done for good.
		if (explore_choice_branches(hp, hg))
			return true;

		dbgprt("Step to next permuation\n");
		// If we are here, there was no match.
		// On the next go-around, take a step.
		take_step = true;
		have_more = false;
	} while (have_perm(hp, hg));

	dbgprt("No more unordered permutations\n");

	return false;
}

/// See explore_link_branches() for a general explanation. This method
/// handles the ChoiceLink branch alternatives only.  It assumes
/// that the caller had handles the unordered-link alternative branches.
bool PatternMatchEngine::explore_choice_branches(const Handle& hp,
                                                 const Handle& hg)
{
	// If its not an choice link, then don't try to iterate.
	if (CHOICE_LINK != curr_term_type)
		return explore_single_branch(hp, hg);

	dbgprt("Begin choice branchpoint iteration loop\n");
	do {
		// XXX this "need_choice_push thing is probably wrong; it probably
		// should resemble the perm_push() used for unordered links.
		// However, currently, no test case trips this up. so .. OK.
		// whatever. This still probably needs fixing.
		if (_need_choice_push) choice_stack.push(_choice_state);
		bool match = explore_single_branch(hp, hg);
		if (_need_choice_push) POPSTK(choice_stack, _choice_state);
		_need_choice_push = false;

		// If the pattern was satisfied, then we are done for good.
		if (match)
			return true;

		dbgprt("Step to next choice\n");
		// If we are here, there was no match.
		// On the next go-around, take a step.
		choose_next = true;
	} while (have_choice(hp, hg));

	dbgprt("Exhausted all choice possibilities\n");

	return false;
}

/// Check the proposed grounding hg for pattern term hp.
///
/// As the name implies, this will explore only one single potential
/// (proposed) grounding for the current pattern term. This is meant
/// to be called after a viable branch has been identified for
/// exploration.
///
/// This is wrapper around tree compare; if tree_compare
/// returns false, then this returns immediately.
///
/// However, this method is part of the upwards-recursion chain,
/// so if tree_compare approves the proposed grounding, this will
/// recurse upwards, calling do_term_up to get the next pattern
/// term. Thus, this method will return true ONLY if ALL OF the terms
/// and clauses in the pattern are satisfiable (are accepted matches).
///
bool PatternMatchEngine::explore_single_branch(const Handle& hp,
                                               const Handle& hg)
{
	solution_push();

	dbgprt("Checking pattern UUID=%lu for soln by %lu\n",
	       hp.value(), hg.value());

	bool match = tree_compare(hp, hg, CALL_SOLN);

	if (not match)
	{
		dbgprt("Pattern UUID=%lu NOT solved by %lu\n",
		       hp.value(), hg.value());
		solution_pop();
		return false;
	}

	dbgprt("UUID=%lu solved by %lu move up\n",
          hp.value(), hg.value());

	// XXX should not do perm_push every time... only selectively.
	// But when? This is very confusing ...
	perm_push();
	bool found = do_term_up(hg);
	perm_pop();

	solution_pop();
	return found;
}

/// do_term_up() -- move upwards from the current term.
///
/// Given the current term, in curr_term_handle, find its parent in the
/// clause, and then call explore_up_branches() to see if the term's
/// parent has corresponding match in the solution graph.
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
/// and simple: it would send the parent to explore_up_branches(), and
/// then explore_up_branches() would respond as to whether it is
/// satisfiable (solvable) or not.
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
		return clause_accept(curr_term_handle, hsoln);

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
				return clause_accept(curr_term_handle, hsoln);
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
			Handle curr_term_save(curr_term_handle);
			curr_term_handle = hi;

			if (explore_up_branches(hi, hsoln)) found = true;

			curr_term_handle = curr_term_save;
			POPSTK(soln_handle_stack, curr_soln_handle);

			dbgprt("After moving up the clause, found = %d\n", found);
		}
		else
		if (hi == curr_root)
		{
			dbgprt("Exploring one possible ChoiceLink at root out of %zu\n",
			       fa.least_holders.size());
			curr_term_handle = hi;
			if (clause_accept(curr_term_handle, hsoln)) found = true;
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

			OC_ASSERT(not have_choice(hi, hsoln),
			          "Something is wrong with the ChoiceLink code");

			soln_handle_stack.push(curr_soln_handle);
			curr_soln_handle = hsoln;

			_need_choice_push = true;
			curr_term_handle = hi;
			if (do_term_up(hsoln)) found = true;

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
bool PatternMatchEngine::clause_accept(const Handle& hp,
                                       const Handle& hg)
{
	// Is this clause a required clause? If so, then let the callback
	// make the final decision; if callback rejects, then it's the
	// same as a mismatch; try the next one.
	bool match;
	if (is_optional(curr_root))
	{
		clause_accepted = true;
		match = _pmc.optional_clause_match(hp, hg);
		dbgprt("optional clause match callback match=%d\n", match);
	}
	else
	{
		match = _pmc.clause_match(hp, hg);
		dbgprt("clause match callback match=%d\n", match);
	}
	if (not match) return false;

	curr_soln_handle = hg;
	clause_grounding[curr_root] = hg;
	prtmsg("---------------------\nclause:", curr_root);
	prtmsg("ground:", hg);

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
		prtmsg("Next clause is\n", curr_root);
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
		found = explore_link_branches(curr_term_handle, curr_soln_handle);

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
				found = explore_link_branches(curr_term_handle, curr_soln_handle);
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
 *
 * Additional complications are introduced by the presence of
 * evaluatable terms, black-box terms, and optional clauses. An
 * evaluatable term is any term that needs to be evaluated to determine
 * if it matches: such terms typically do not exist in the atomspace;
 * they are "virtual", and "exist" only when the evaluation returns
 * "true". Thus, these can only be grounded after all other possible
 * clauses are grounded; thus these are saved for last.  It is always
 * possible to save these for last, because earlier stages have
 * guaranteed that all of he non-virtual clauses are connected.
 * Anyway, evaluatables come in two forms: those that can be evaluated
 * quickly, and those that require a "black-box" evaluation of some
 * scheme or python code. Of the two, we save "black-box" for last.
 *
 * Then afer grounding all of the mandatory clauses (virtual or not),
 * we look for optional clauses, if any. Again, these might be virtual,
 * and they might be black...
 *
 * Thus, we use a helper function to broaden the search in each case.
 */
void PatternMatchEngine::get_next_untried_clause(void)
{
	// First, try to ground all the mandatory clauses, only.
	// no virtuals, no black boxes, no optionals.
	if (get_next_untried_helper(false, false, false)) return;

	// Don't bother looking for evaluatables if they are not there.
	if (not _pat->evaluatable_holders.empty())
	{
		if (get_next_untried_helper(true, false, false)) return;
		if (not _pat->black.empty())
		{
			if (get_next_untried_helper(true, true, false)) return;
		}
	}

	// If there are no optional clauses, we are done.
	if (_pat->optionals.empty())
	{
		// There are no more ungrounded clauses to consider. We are done.
		curr_root = Handle::UNDEFINED;
		curr_term_handle = Handle::UNDEFINED;
		return;
	}

	// Try again, this time, considering the optional clauses.
	if (get_next_untried_helper(false, false, true)) return;
	if (not _pat->evaluatable_holders.empty())
	{
		if (get_next_untried_helper(true, false, true)) return;
		if (not _pat->black.empty())
		{
			if (get_next_untried_helper(true, true, true)) return;
		}
	}

	// If we are here, there are no more unsolved clauses to consider.
	curr_root = Handle::UNDEFINED;
	curr_term_handle = Handle::UNDEFINED;
}

// Count teh number of ungrounded variables in a clause.
//
// This is used to search for the "thinnest" ungrounded clause:
// the one with the fewest ungrounded variables in it. Thus, if
// there is just one variable that needs to be grounded, then this
// can be done in a direct fashion; it resembles the concept of
// "unit propagation" in the DPLL algorithm.
//
// XXX TODO ... Rather than counting the number of variables, we
// should instead look for one with the smallest incoming set.
// That is because the very next thing that we do will be to
// iterate over the incoming set of "pursue" ... so it could be
// a huge pay-off to minimize this.
//
// If there are two ungrounded variables in a clause, then the
// "thickness" is the *product* of the sizes of the two incoming
// sets. Thus, the fewer ungrounded variables, the better.
//
// Danger: this assumes a suitable dataset, as otherwise, the cost
// of this "optimization" can add un-necessarily to the overhead.
//
unsigned int PatternMatchEngine::thickness(const Handle& clause,
                                           const std::set<Handle>& live)
{
	// If there are only zero or one ungrounded vars, then any clause
	// will do. Blow this pop stand.
	if (live.size() < 2) return 1;

	unsigned int count = 0;
	for (const Handle& v : live)
	{
		if (is_unquoted_in_tree(clause, v)) count++;
	}
	return count;
}

/// Same as above, but with three boolean flags:  if not set, then only
/// those clauses satsifying the criterion are considered, else all
/// clauses are considered.
///
/// Return true if we found the next ungrounded clause.
bool PatternMatchEngine::get_next_untried_helper(bool search_virtual,
                                                 bool search_black,
                                                 bool search_optionals)
{
	// Search for an as-yet ungrounded clause. Search for required
	// clauses first; then, only if none of those are left, move on
	// to the optional clauses.  We can find ungrounded clauses by
	// looking at the grounded vars, looking up the root, to see if
	// the root is grounded.  If its not, start working on that.
	Handle joint(Handle::UNDEFINED);
	Handle unsolved_clause(Handle::UNDEFINED);
	bool unsolved = false;
	unsigned int thinnest = UINT_MAX;

	// Make a list of the as-yet ungrounded variables.
	std::set<Handle> undead;
	for (const Handle &v : _varlist->varseq)
	{
		try { var_grounding.at(v); }
		catch(...) { undead.insert(v); }
	}

	// We are looking for a joining atom, one that is shared in common
	// with the a fully grounded clause, and an as-yet ungrounded clause.
	// The joint is called "pursue", and the unsolved clause that it
	// joins will become our next untried clause.  Note that the join
	// is not necessarily a variable; it could be a constant atom.
	// (wouldn't things be faster if they were variables only, that
	// joined?) the problem is that var_grounding stores both grounded
	// variables and "grounded" constants.
#define OLD_LOOP
#ifdef OLD_LOOP
	for (const Pattern::ConnectPair& vk : _pat->connectivity_map)
	{
		const Pattern::RootList& rl(vk.second);
		const Handle& pursue = vk.first;
		if (Handle::UNDEFINED == var_grounding[pursue]) continue;
#else // OLD_LOOP
	// Newloop might be slightly faster than old loop.
	// ... but maybe not...
	for (auto gndpair : var_grounding)
	{
		const Handle& pursue = gndpair.first;
		try { _pat->connectivity_map.at(pursue); }
		catch(...) { continue; }
		const RootList& rl(_pat->connectivity_map.at(pursue));
#endif

		for (const Handle& root : rl)
		{
			if ((issued.end() == issued.find(root))
			        and (search_virtual or not is_evaluatable(root))
			        and (search_black or not is_black(root))
			        and (search_optionals or not is_optional(root)))
			{
				unsigned int th = thickness(root, undead);
				if (th < thinnest)
				{
					thinnest = th;
					unsolved_clause = root;
					joint = pursue;
					unsolved = true;
				}
			}
			if (unsolved and thinnest < 2) break;
		}

		if (unsolved and thinnest < 2) break;
	}

	if (unsolved)
	{
		// Joint is a (variable) node that's shared between several
		// clauses. One of the clauses has been grounded, another
		// has not.  We want to now traverse upwards from this node,
		// to find the top of the ungrounded clause.
		curr_root = unsolved_clause;
		curr_term_handle = joint;

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

	perm_push();

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

	perm_pop();

	_clause_stack_depth --;

	dbgprt("pop to depth %d\n", _clause_stack_depth);
	prtmsg("pop to joiner", curr_term_handle);
	prtmsg("pop to clause", curr_root);
}

/**
 * Unconditionally clear all graph traversal stacks
 * XXX TODO -- if the algo is working correctly, then all
 * of these should already be empty, when this method is
 * called. So really, we should check the stack size, and
 * assert if it is not zero ...
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

	while (!perm_stack.empty()) perm_stack.pop();
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
	bool found = explore_link_branches(curr_term_handle, ah);

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

	// choice link state
	_choice_state.clear();
	_need_choice_push = false;
	choose_next = true;

	// unordered link state
	have_more = false;
	take_step = true;
	_perm_state.clear();
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

	// graph state
	_clause_stack_depth = 0;

	// choice link state
	_need_choice_push = false;
	choose_next = true;

	// unordered link state
	have_more = false;
	take_step = true;
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
