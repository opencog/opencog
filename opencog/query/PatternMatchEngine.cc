/*
 * PatternMatchEngine.cc
 *
 * Copyright (C) 2008,2009,2011,2014 Linas Vepstas
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

#include "PatternMatchEngine.h"

#include <opencog/util/oc_assert.h>
#include <opencog/atomspace/Foreach.h>
#include <opencog/atomspace/ForeachTwo.h>

using namespace opencog;

// #define DEBUG 1
#ifdef WIN32
#if DEBUG
	#define dbgprt printf
#else
	// something better?
	#define dbgprt
#endif
#else
#if DEBUG
	#define dbgprt(f, varargs...) printf(f, ##varargs)
#else
	#define dbgprt(f, varargs...)
#endif
#endif

PatternMatchEngine::PatternMatchEngine(void)
{
	atom_space = NULL;
}

void PatternMatchEngine::set_atomspace(AtomSpace *as)
{
	atom_space = as;

	HandleSeq oset;
	invalid_grounding = as->addLink(LINK, oset);
}

bool PatternMatchEngine::prt(Handle& h)
{
	std::string str = h->toShortString();
	printf ("%s\n", str.c_str());
	return false;
}

inline void PatternMatchEngine::prtmsg(const char * msg, Handle& h)
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

/* Reset the current variable grounding to the last grounding pushed
 * onto the stack. */
#define POPGND(soln,stack) {         \
   if (stack.empty()) {              \
      printf("ERROR: Unbalanced stack pop!\n"); \
      soln.clear();                  \
   }                                 \
   else soln = stack.top();          \
   stack.pop();                      \
}

/* ======================================================== */

/**
 * tree_compare compares two trees, side-by-side.
 *
 * Compare two incidence trees, side-by-side. It is assumed that the
 * first of these is a clause in the predicate, and so the comparison
 * is between the clause, and a candidate grounding.
 *
 * The graph/tree refered to here is the incidence graph/tree (aka
 * Levi graph) of the hypergraph (and not the hypergraph itself).
 * The incidence graph is given by the "outgoing set" of the atom.
 *
 * This routine is recursive, calling itself on each subtree of the
 * predicate clause, performing comparisions until a match is found
 * (or not found).
 *
 * Return true if there's a mis-match. The goal here is to walk over
 * the entire tree, without mismatches.  Since a return value of true
 * stops the iteration, true is used to signal a mismatch.
 */
bool PatternMatchEngine::tree_compare(Handle hp, Handle hg)
{
	// Handle hp is from the pattern clause, and it might be one
	// of the bound variables. If so, then declare a match.
	if (bound_vars.end() != bound_vars.find(hp))
	{
		// But... if handle hg happens to also be a bound var,
		// then its a mismatch.
		if (bound_vars.end() != bound_vars.find(hg)) return true;

		// If we already have a grounding for this variable, the new
		// proposed grounding must match the existing one. Such multiple
		// groundings can occur when traversing graphs with loops in them.
		Handle gnd(var_grounding[hp]);
		if (Handle::UNDEFINED != gnd) {
			return (gnd != hg);
		}

		// VariableNode had better be an actual node!
		// If it's not then we are very very confused ...
		NodePtr np(NodeCast(hp));
		OC_ASSERT (NULL != np,
		      "ERROR: expected variable to be a node, got this: %s\n",
				hp->toShortString().c_str());

		// Else, we have a candidate grounding for this variable.
		// The node_match may implement some tighter variable check,
		// e.g. making sure that grounding is of some certain type.
		if (pmc->variable_match (hp,hg)) return true;

		// Make a record of it.
		dbgprt("Found grounding of variable:\n");
		prtmsg("$$ variable:    ", hp);
		prtmsg("$$ ground term: ", hg);
		var_grounding[hp] = hg;
		return false;
	}

	// If they're the same atom, then clearly they match.
	// ... but only if hg is not a subclause of the current clause.
	if ((hp == hg) && (hg != curr_pred_handle))
	{
		var_grounding[hp] = hg;
		return false;
	}

	// If both are links, compare them as such.
	LinkPtr lp(LinkCast(hp));
	LinkPtr lg(LinkCast(hg));
	if (lp and lg)
	{
		// Let the callback perform basic checking.
		bool mismatch = pmc->link_match(lp, lg);
		if (mismatch) return true;

		dbgprt("depth=%d\n", depth);
		prtmsg("> tree_compare", hp);
		prtmsg(">           to", hg);

		// If the two links are both ordered, its enough to compare
		// them "side-by-side"; the foreach_atom_pair iterator does
		// this. If they are un-ordered, then we have to compare (at
		// most) every possible permutation.
		//
		Type tp = hp->getType();
		if (classserver().isA(tp, ORDERED_LINK))
		{
			LinkPtr lp(LinkCast(hp));
			LinkPtr lg(LinkCast(hg));
			const std::vector<Handle> &osp = lp->getOutgoingSet();
			const std::vector<Handle> &osg = lg->getOutgoingSet();

			// The recursion step: traverse down the tree.
			// In principle, we could/should push the current groundings
			// onto the stack before recursing, and then pop them off on
			// return.  Failure to do so could leave some bogus groundings,
			// sitting around, i.e. groundings that were found during
			// recursion but then discarded due to a later mistmatch.
			//
			// In practice, I was unable to come up with any test case
			// where this mattered; any bogus groundings eventually get
			// replaced by valid ones.  Thus, we save some unknown amount
			// of cpu time by simply skipping the push & pop here.
			//
			// var_solutn_stack.push(var_grounding);
			depth ++;

			mismatch = foreach_atom_pair(osp, osg,
			                   &PatternMatchEngine::tree_compare, this);
			depth --;
			dbgprt("tree_comp down link mismatch=%d\n", mismatch);

			// If we've found a grounding, record it.
			if (false == mismatch)
				var_grounding[hp] = hg;

			// if (false == mismatch)
			//	   var_solutn_stack.pop();
			// else
			//	   POPGND(var_grounding, var_solutn_stack);

			return mismatch;
		}
		else
		{
			// Enumerate all the permutations of the outgoing set of
			// the predicate.  We keep trying different permuations
			// until we find one that works, and just return that, and
			// ignore the rest.  We have to push/pop the stack as we try
			// each permutation, so that each permutation has a chance
			// of finding a grounding.
			//
			// We don't try all possible groundings; this is not the place
			// to do this; this is done elsewhere. Here, its enough to find
			// any grounding that works (i.e. is consistent with all
			// groundings up till now).
			LinkPtr lp(LinkCast(hp));
			LinkPtr lg(LinkCast(hg));
			std::vector<Handle> osp = lp->getOutgoingSet();
			const std::vector<Handle> &osg = lg->getOutgoingSet();
			sort(osp.begin(), osp.end());

			do {
				// The recursion step: traverse down the tree.
				dbgprt("tree_comp begin down unordered link\n");
				var_solutn_stack.push(var_grounding);
				depth ++;

				mismatch = foreach_atom_pair(osp, osg,
				                   &PatternMatchEngine::tree_compare, this);
				depth --;
				dbgprt("tree_comp down unordered link mismatch=%d\n", mismatch);

				if (false == mismatch)
				{
					var_solutn_stack.pop();
					var_grounding[hp] = hg;
					return false;
				}

				POPGND(var_grounding, var_solutn_stack);
			} while (next_permutation(osp.begin(), osp.end()));

			dbgprt("tree_comp down unordered exhausted all permuations\n");
			return mismatch;
		}

		return mismatch;
	}

	// If both are nodes, compare them as such.
	NodePtr np(NodeCast(hp));
	NodePtr ng(NodeCast(hg));
	if (np and ng)
	{
		// Call the callback to make the final determination.
		bool mismatch = pmc->node_match(hp, hg);
		if (false == mismatch)
		{
			dbgprt("Found matching nodes\n");
			prtmsg("# pattern: ", hp);
			prtmsg("# match:   ", hg);
			var_grounding[hp] = hg;
		}
		return mismatch;
	}

	// If we got to here, there is a clear mismatch, probably because
	// one is a node, and the other a link.
	return true;
}

/* ======================================================== */

bool PatternMatchEngine::soln_up(Handle hsoln)
{
	var_solutn_stack.push(var_grounding);
	bool found = do_soln_up(hsoln);
	if (found)
	{
		var_solutn_stack.pop();  // pop entry created, but keep current.
	}
	else
	{
		POPGND(var_grounding, var_solutn_stack);
	}

	return found;
}

bool PatternMatchEngine::do_soln_up(Handle& hsoln)
{
	// Let's not look at our own navel
	if (hsoln == curr_root) return false;
	depth = 1;

	bool no_match = tree_compare(curr_pred_handle, hsoln);

	// If no match, then try the next one.
	if (no_match) return false;

	// If we've navigated to the top of the clause, and its matched,
	// then it is fully grounded, and we're done with it.
	// Start work on the next unsovled predicate. But do all of this
	// only if the callback allows it.
	if (curr_pred_handle == curr_root)
	{
		// Is this required to match? If so, then let the callback
		// make the final decision; if callback rejects, then it's
		// the same as a mismatch; try the next one.
		if (optionals.count(curr_root))
		{
			clause_accepted = true;
			no_match = pmc->optional_clause_match(curr_pred_handle, hsoln);
		}
		else
		{
			no_match = pmc->clause_match(curr_pred_handle, hsoln);
		}
		dbgprt("clause match callback no_match=%d\n", no_match);
		if (no_match) return false;

		curr_soln_handle = hsoln;
		clause_grounding[curr_root] = curr_soln_handle;
		prtmsg("---------------------\nclause:", curr_root);
		prtmsg("ground:", curr_soln_handle);
		dbgprt("--- That's it, now push and do the next one.\n\n");

		root_handle_stack.push(curr_root);
		pred_handle_stack.push(curr_pred_handle);
		soln_handle_stack.push(curr_soln_handle);
		pred_solutn_stack.push(clause_grounding);
		var_solutn_stack.push(var_grounding);
		issued_stack.push(issued);
		pmc->push();

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
			found = pmc->solution(clause_grounding, var_grounding);
		}
		else
		{
			prtmsg("next clause is", curr_root);
			dbgprt("This clause is %s\n", optionals.count(curr_root)? "optional" : "required");
			prtmsg("joining handle is", curr_pred_handle);

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
			while ((false == found) &&
			       (false == clause_accepted) &&
			       (optionals.count(curr_root)))
			{
				Handle undef(Handle::UNDEFINED);
				no_match = pmc->optional_clause_match(curr_pred_handle, undef);
				dbgprt ("Exhausted search for optional clause, cb=%d\n", no_match);
				if (no_match) return false;

				// XXX Maybe should push n pop here? No, maybe not ...
				clause_grounding[curr_root] = invalid_grounding;
				get_next_untried_clause();
				prtmsg("Next optional clause is", curr_root);
				if (Handle::UNDEFINED == curr_root)
				{
					dbgprt ("==================== FINITO BANDITO!\n");
#ifdef DEBUG
					print_solution(var_grounding, clause_grounding);
#endif
					found = pmc->solution(clause_grounding, var_grounding);
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
		pmc->pop();
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

		prtmsg("pop to joiner", curr_pred_handle);
		prtmsg("pop to clause", curr_root);

		return found;
	}

	// If we are here, then we are somewhere in the middle of a clause,
	// and everything below us matches. So need to move up.
	soln_handle_stack.push(curr_soln_handle);
	curr_soln_handle = hsoln;

	// Move up the predicate, and hunt for a match, again.
	prtmsg("node has grnd, move up:", hsoln);
	// IncomingSet iset = get_incoming_set(curr_pred_handle);
	IncomingSet iset = curr_pred_handle->getIncomingSet();
	size_t sz = iset.size();
	bool found = false;
	for (size_t i = 0; i < sz; i++) {
		found = pred_up(Handle(iset[i]));
		if (found) break;
	}
	dbgprt("after moving up the clause, found = %d\n", found);

	curr_soln_handle = soln_handle_stack.top();
	soln_handle_stack.pop();

	return found;
}

bool PatternMatchEngine::pred_up(Handle h)
{
	// Is this link even a part of the predicate we are considering?
	// If not, try the next atom.
	bool valid = ot.is_node_in_tree(curr_root, h);
	if (!valid) return false;

	// Move up the solution outgoing set, looking for a match.
	Handle curr_pred_save(curr_pred_handle);
	curr_pred_handle = h;

	IncomingSet iset = pmc->get_incoming_set(curr_soln_handle);
	size_t sz = iset.size();
	bool found = false;
	for (size_t i = 0; i < sz; i++) {
		found = soln_up(Handle(iset[i]));
		if (found) break;
	}

	curr_pred_handle = curr_pred_save;
	dbgprt("found upward soln = %d\n", found);
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
 */
void PatternMatchEngine::get_next_untried_clause(void)
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

	RootMap::iterator k;
	for (k = root_map.begin(); k != root_map.end(); k++)
	{
		RootPair vk = *k;
		RootList *rl = vk.second;
		pursue = vk.first;

		unsolved = false;
		solved = false;

		std::vector<Handle>::iterator i = rl->begin();
		std::vector<Handle>::iterator iend = rl->end();
		for (; i != iend; i++)
		{
			Handle root(*i);
			if (Handle::UNDEFINED != clause_grounding[root])
			{
				solved = true;
			}
			else if ((issued.end() == issued.find(root)) &&
			         (optionals.end() == optionals.find(root)))
			{
				unsolved_clause = root;
				unsolved = true;
			}
		}

		// XXX TODO ... Rather than settling for the first one that we find,
		// we should instead look for the "thinnest" one, the one with the
		// smallest incoming set.  That is because the very next thing that
		// we do will be to iterate over the incoming set of "pursue" ... so
		// it could be a huge pay-off to minimize this.
		if (solved && unsolved) break;
	}

	if (solved && unsolved)
	{
		// Pursue is a pointer to a node that's shared between
		// several clauses. One of the predicates has been
		// solved, another has not.  We want to now traverse
		// upwards from this node, to find the top of the
		// unsolved clause.
		curr_root = unsolved_clause;
		curr_pred_handle = pursue;

		if (Handle::UNDEFINED != unsolved_clause)
		{
			issued.insert(unsolved_clause);
			return;
		}
	}

	unsolved = false;
	solved = false;

	// Try again, this time, considering the optional clauses.
	for (k=root_map.begin(); k != root_map.end(); k++)
	{
		RootPair vk = *k;
		RootList *rl = vk.second;
		pursue = vk.first;

		unsolved = false;
		solved = false;

		std::vector<Handle>::iterator i = rl->begin();
		std::vector<Handle>::iterator iend = rl->end();
		for (; i != iend; i++)
		{
			Handle root(*i);
			Handle root_gnd(clause_grounding[root]);
			if (Handle::UNDEFINED != root_gnd and root_gnd != invalid_grounding)
			{
				solved = true;
			}
			else if (issued.end() == issued.find(root))
			{
				unsolved_clause = root;
				unsolved = true;
			}
		}
		if (solved && unsolved) break;
	}

	if (solved && unsolved)
	{
		// Pursue is a pointer to a node that's shared between
		// several clauses. One of the predicates has been
		// solved, another has not.  We want to now traverse
		// upwards from this node, to find the top of the
		// unsolved clause.
		curr_root = unsolved_clause;
		curr_pred_handle = pursue;

		if (Handle::UNDEFINED != unsolved_clause)
		{
			issued.insert(unsolved_clause);
			return;
		}
	}

	// If we are here, there are no more unsolved clauses to consider.
	curr_root = Handle::UNDEFINED;
	curr_pred_handle = Handle::UNDEFINED;
}

/* ======================================================== */

/**
 * do_candidate - examine candidates, looking for matches.
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
 * This routine is meant to be invoked on every candidate atom taken
 * from the atom space. That atom is assumed to anchor some part of
 * a graph that hopefully will match the predicate.
 */
bool PatternMatchEngine::do_candidate(Handle& do_clause, Handle& starter, Handle& ah)
{
	// Cleanup
	var_grounding.clear();
	clause_grounding.clear();
	issued.clear();
	while(!pred_handle_stack.empty()) pred_handle_stack.pop();
	while(!soln_handle_stack.empty()) soln_handle_stack.pop();
	while(!root_handle_stack.empty()) root_handle_stack.pop();
	while(!pred_solutn_stack.empty()) pred_solutn_stack.pop();
	while(!var_solutn_stack.empty()) var_solutn_stack.pop();
	while(!issued_stack.empty()) issued_stack.pop();

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
 * Create an associative array that gives a list of all of the
 * clauses that a given node participates in.
 */
bool PatternMatchEngine::note_root(Handle h)
{
	RootList *rl = root_map[h];
	if (NULL == rl)
	{
		rl = new RootList();
		root_map[h] = rl;
	}
	rl->push_back(curr_root);

	LinkPtr l(LinkCast(h));
	if (l) foreach_outgoing_handle(l, &PatternMatchEngine::note_root, this);
	return false;
}

/**
 * Clear all internal state.
 * This allows a given instance of this class to be used again.
 */
void PatternMatchEngine::clear(void)
{
	// Clear all state.
	bound_vars.clear();
	cnf_clauses.clear();
	optionals.clear();
	var_grounding.clear();
	clause_grounding.clear();
	root_map.clear();
	issued.clear();

	curr_root = Handle::UNDEFINED;
	curr_soln_handle = Handle::UNDEFINED;
	curr_pred_handle = Handle::UNDEFINED;
	depth = 0;

	while (!pred_handle_stack.empty()) pred_handle_stack.pop();
	while (!soln_handle_stack.empty()) soln_handle_stack.pop();
	while (!root_handle_stack.empty()) root_handle_stack.pop();
	while (!pred_solutn_stack.empty()) pred_solutn_stack.pop();
	while (!var_solutn_stack.empty()) var_solutn_stack.pop();
	while (!issued_stack.empty()) issued_stack.pop();
}


/**
 * Find groundings for a sequence of clauses in conjunctive normal form.
 * That is, perform a variable unification across multiple clauses.
 *
 * The list of clauses, and the list of negations, are both OpenCog
 * hypergraphs.  Both can be (should be) envisioned as model-theoretic
 * predicates: i.e. statements with are "true" only if they exist in
 * the atomspace (which is the "universe" of all statements).  That is,
 * the clauses define a subgraph which may or may not exist in the
 * atomspace.
 *
 * The list of "bound vars" are to be solved for ("grounded", or
 * "evaluated") during pattern matching. That is, if the subgraph
 * defined by the clauses is located, then the vars are given the
 * corresponding values associated to that match. Becuase these
 * variables can be shared across multiple clauses, this can be
 * understood to be a unification problem; the pattern matcher is thus
 * a unifier.
 *
 * The negations are a set of clauses whose truth values are to be
 * inverted.  That is, while the clauses define a subgraph that
 * *must* be found, the negations define a subgraph that should
 * not be found, or, if found, should have a truth value of 'false'.
 * The precise meaning of 'false' in the sentence above is determined
 * by the callback, which can use arbitrary criteria for this.
 * In particular, the search engine itself will happily proclaim
 * a match whether or not it finds any of the negated clauses. So,
 * in this sense, the negated clauses can be understood to be
 * "optional" matches: they will be matched, if possible, but are not
 * required to be matched. It is up to the callback to explictly
 * reject these clauses, if it so wishes to, thus implementing the
 * concept of negation.
 *
 * The PatternMatchCallback is consulted to determine whether a
 * veritable match has been found, or not. The callback is given
 * individual nodes and links to compare for a match.
 *
 * The callback may alter the sequence of the clauses, in order to
 * otpimze the search. It may also remove some clauses or variables,
 * if it determines that these are irrelevant to the search.
 */
void PatternMatchEngine::match(PatternMatchCallback *cb,
                         std::vector<Handle> &vars,
                         std::vector<Handle> &clauses,
                         std::vector<Handle> &negations)
{
	// Clear all state.
	clear();

	// Copy the variables from vector to set; this makes it easier to
	// determine set membership.
	std::vector<Handle>::const_iterator i = vars.begin();
	std::vector<Handle>::const_iterator iend = vars.end();
	for (; i != iend; i++)
	{
		bound_vars.insert(*i);
	}

	cnf_clauses = clauses;

	// Copy the negates into the clause list
	// Copy the negates into a set.
	iend = negations.end();
	for (i = negations.begin(); i != iend; i++)
	{
		cnf_clauses.push_back(*i);
		optionals.insert(*i);
	}

	if (cnf_clauses.size() == 0) return;

	// Preparation prior to search.
	// Create a table of the nodes that appear in the clauses, and
	// a list of the clauses that each node participates in.
	iend = cnf_clauses.end();
	for (i = cnf_clauses.begin(); i != iend; i++)
	{
		Handle h(*i);
		curr_root = h;
		note_root(h);
	}
	pmc = cb;

#ifdef DEBUG
	// Print out the predicate ...
	printf("\nPredicate consists of the following clauses:\n");
	int cl = 0;
	iend = cnf_clauses.end();
	for (i = cnf_clauses.begin(); i != iend; i++)
	{
		Handle h(*i);
		printf("Clause %d: ", cl);
		prt(h);
		cl++;
	}

	// Print out the bound variables in the predicate.
	std::set<Handle>::const_iterator j;
	for (j=bound_vars.begin(); j != bound_vars.end(); j++)
	{
		Handle h(*j);
		if (NodeCast(h))
		{
			printf(" Bound var: "); prt(h);
		}
	}

	if (0 == bound_vars.size())
	{
		printf("There are no bound vars in this pattern\n");
	}
	printf("\n");
#endif

	// Perform the actual search!
	cb->perform_search(this, vars, clauses, negations);

	dbgprt ("==================== Done Matching ==================\n");
#ifdef DEBUG
	fflush(stdout);
#endif
}

/**
 * Validate -- every clause must contain at least one variable.
 *
 * Make sure that every clause contains at least one variable;
 * if not, remove the clause from the list of clauses.
 *
 * The core idea is that pattern matching against a constant expression
 * "doesn't make sense" -- the constant expression will always match to
 * itself and is thus "trivial".  In principle, the programmer should
 * never include constants in the list of clauses ... but, due to
 * programmer error, this can happen, and will lead to failures during
 * pattern matching. Thus, the routine below can be used to validate
 * the input.
 *
 * Returns true if the list of clauses was modified, else returns false.
 */
bool PatternMatchEngine::validate(
                         const std::vector<Handle> &vars,
                         std::vector<Handle> &clauses)
{
	bool modified = false;

	std::vector<Handle>::iterator i;
	for (i = clauses.begin();
	     i != clauses.end();)
	{
		Handle clause(*i);
		if (validate (vars, clause))
		{
			i++;
		}
		else
		{
			i = clauses.erase(i);
			modified = true;
		}
	}

	return modified;
}

/**
 * Return true if clause contains one of the vars, else return false.
 *
 * If the clause, (or any of its subclauses) contain any variable in
 * the list of variables, then return true; else return false.
 */
bool PatternMatchEngine::validate(
                         const std::vector<Handle> &vars,
                         Handle& clause)
{
	// If its a node, then it must be one of the vars.
	Type clause_type = clause->getType();
	if (classserver().isNode(clause_type))
	{
		std::vector<Handle>::const_iterator i = vars.begin();
		std::vector<Handle>::const_iterator iend = vars.end();
		for (; i != iend; i++)
		{
			if (*i == clause) return true;
		}
		return false;
	}

	// If its a link, then recurse to subclauses.
	LinkPtr lc(LinkCast(clause));
	if (lc)
	{
		const std::vector<Handle> &oset = lc->getOutgoingSet();
		std::vector<Handle>::const_iterator i = oset.begin();
		std::vector<Handle>::const_iterator iend = oset.end();
		for (; i != iend; i++)
		{
			Handle subclause(*i);
			if (validate(vars, subclause)) return true;
		}
		return false;
	}

	return false;
}

void PatternMatchEngine::print_solution(
	const std::map<Handle, Handle> &vars,
	const std::map<Handle, Handle> &clauses)
{
	printf("\nNode groundings:\n");

	// Print out the bindings of solutions to variables.
	std::map<Handle, Handle>::const_iterator j = vars.begin();
	std::map<Handle, Handle>::const_iterator jend = vars.end();
	for (; j != jend; j++)
	{
		Handle var(j->first);
		Handle soln(j->second);

		Type tyv = var->getType();
		Type tys = soln->getType();

		// Only print grounding for variables.
		if (VARIABLE_NODE != tyv) continue;

		const std::string &vart = classserver().getTypeName(tyv);
		const std::string &slnt = classserver().getTypeName(tys);
		std::string solstr;
		if (classserver().isNode(tyv))
		{
			NodePtr n(NodeCast(soln));
			solstr = n->getName().c_str();
		}
		else
		{
			solstr = soln->toShortString().c_str();
		}

		if (soln == Handle::UNDEFINED)
		{
			NodePtr nv(NodeCast(var));
			printf("ERROR: ungrounded variable %s %s\n",
				vart.c_str(), nv->getName().c_str());
			continue;
		}

		NodePtr nv(NodeCast(var));
		printf("\t%s %s maps to %s %s\n", vart.c_str(),
			nv->getName().c_str(),
			slnt.c_str(), solstr.c_str());
	}

	// Print out the full binding to all of the clauses.
	printf("\nGrounded clauses:\n");
	std::map<Handle, Handle>::const_iterator m;
	int i = 0;
	for (m = clauses.begin(); m != clauses.end(); m++, i++)
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
                  const std::vector<Handle> &vars,
                  const std::vector<Handle> &clauses)
{
	printf("\nClauses:\n");
	std::vector<Handle>::const_iterator i;
	for (i = clauses.begin(); i != clauses.end(); i++)
	{
		Handle h(*i);
		prt(h);
	}
	printf("\nVars:\n");
	for (i = vars.begin(); i != vars.end(); i++)
	{
		Handle h(*i);
		prt(h);
	}
}

/* ===================== END OF FILE ===================== */
