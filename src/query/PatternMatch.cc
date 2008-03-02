/*
 * PatternMatch.cc
 *
 * Linas Vepstas February 2008
 */

#include "Foreach.h"
#include "ForeachTwo.h"
#include "Link.h"
#include "Node.h"
#include "PatternMatch.h"
#include "TLB.h"

using namespace opencog;

PatternMatch::PatternMatch(AtomSpace *as)
{
	atom_space = as;
}

bool PatternMatch::prt(Atom *atom)
{
	if (!atom) return false;
	std::string str = atom->toString();
	printf ("%s\n", str.c_str());
	return false;
}

bool PatternMatch::prt(Handle h)
{
	return prt(TLB::getAtom(h));
}

/* ======================================================== */

/**
 * tree_compare compares two trees, side-by-side.
 *
 * Compare two incidence trees, side-by-side. It is assumed
 * that the first of these is the predicate, and so the
 * comparison is between the predicate, and a candidate
 * graph. 
 *
 * The graph/tree refered to here is the incidence 
 * graph/tree (aka Levi graph) of the hypergraph.
 * (and not the hypergraph itself).
 * The incidence graph is given by the "outgoing set"
 * of the atom.
 *
 * This routine is recursive, calling itself on each
 * subtree of the predicate, performing comparisions until
 * a match is found (or not found).
 *
 * Return true if there's a mis-match. The goal here
 * is to iterate the entire tree, without mismatches.
 * Since a return value of true stops the iteration,
 * true is used to signal a mistmatch.
 */
bool PatternMatch::tree_compare(Atom *aa, Atom *ab)
{
	Handle ha = TLB::getHandle(aa);

	// Atom aa is from the predicate, and it might be one
	// of the bound variables. If so, then declare a match.
	if (bound_vars.count(ha))
	{
		// If ab is the very same var, then its a mismatch.
		if (aa == ab) return true;

		// Else, we have a candidate solution.
		// Make a record of it.
		var_solution[ha] = ab;
		return false;
	}

	// If they're the same atom, then clearly they match.
	// ... but only if ab is not one of the predicates itself.
	if ((aa == ab) && (ab != curr_root))
	{
		var_solution[ha] = ab;
		return false;
	}

	// If one is null, but the other is not, there's clearly no match.
	if (aa && !ab) return true;
	if (ab && !aa) return true;

	// If types differ, then no match.
	if (aa->getType() != ab->getType()) return true;

	// printf ("tree_compare depth=%d comp "); prt(aa);
	// printf ("                       to "); prt(ab);

	// The recursion step: traverse down the tree.
	// Only links should have non-empty outgoing sets.
	if (dynamic_cast<Link *>(aa))
	{
		Handle hb = TLB::getHandle(ab);

		depth ++;
		bool mismatch = foreach_outgoing_atom_pair(ha, hb,
		              	      &PatternMatch::tree_compare, this);
		depth --;
		if (false == mismatch) var_solution[ha] = ab;
		// printf("tree_comp down link mismatch=%d\n", mismatch);
		return mismatch;
	}

	// Call the callback to make the final determination.
	bool mismatch = pmc->node_match(aa, ab);
	if (false == mismatch) var_solution[ha] = ab;
	return mismatch;
}

/* ======================================================== */

bool PatternMatch::soln_up(Handle hsoln)
{
	Atom *as = TLB::getAtom(hsoln);
	Atom *ap = TLB::getAtom(curr_pred_handle);
	depth = 1;
	bool no_match = tree_compare(ap, as);

	// If no match, try the next one.
	if (no_match) return false;

	curr_soln_handle = TLB::getHandle(as);

	// Ahh ! found a match!  If we've navigated to the top of the 
	// predicate, then we're done with it.  Look for the next 
	// unsovled predicate.
	if (curr_pred_handle == curr_root)
	{

		root_handle_stack.push(curr_root);
		pred_handle_stack.push(curr_pred_handle);
		soln_handle_stack.push(curr_soln_handle);
		pred_solutn_stack.push(predicate_solution);

		predicate_solution[curr_root] = curr_soln_handle;
		// printf("--------------------- \npred: "); prt(curr_root);
		// printf("soln: "); prt(curr_soln_handle);
		
		get_next_unsolved_pred();

		// printf("next handle is "); prt(curr_pred_handle);
		// printf("next pred is "); prt(curr_root);

		// If there are no further predicates to solve,
		// we are really done! Return true to terminate the search.
		if (UNDEFINED_HANDLE == curr_root)
		{
			// printf ("==================== FINITO!\n");
			return pmc->solution(predicate_solution, var_solution);
		}

		curr_soln_handle = var_solution[curr_pred_handle];
		bool found = soln_up(curr_soln_handle);

		// If we failed to find anything at this level,
		// we need to pop, and try other possible matches.
		if (!found)
		{
			curr_root = root_handle_stack.top();
			root_handle_stack.pop();

			curr_pred_handle = pred_handle_stack.top();
			pred_handle_stack.pop();

			curr_soln_handle = soln_handle_stack.top();
			soln_handle_stack.pop();

			predicate_solution = pred_solutn_stack.top();
			pred_solutn_stack.pop();
		}

		return found;
	}

	// printf("node has soln, move up: "); prt(as);

	// Move up the predicate, and hunt for a match, again.
	bool found = foreach_incoming_handle(curr_pred_handle,
	                &PatternMatch::pred_up, this);
	// printf("up pred find =%d\n", found);
	return found;
}

bool PatternMatch::pred_up(Handle h)
{
	// Is this atom even a part of the predicate we are considering?
	// If not, try the next atom.
	bool valid = ot.is_node_in_tree(curr_root, h);
	if (!valid) return false;

	// Now, move up the solution outgoing set, looking for a match.
	curr_pred_handle = h;

	bool found = foreach_incoming_handle(curr_soln_handle,
	                     &PatternMatch::soln_up, this);

	// printf("upward soln find =%d\n", found);
	return found;
}

void PatternMatch::get_next_unsolved_pred(void)
{
	// Search for an as-yet unsolved/unmatched predicate.
	// For each solved node, look up root to see if root is solved.
	// If not, start working on that.
	Handle pursue = UNDEFINED_HANDLE;
	Handle unsolved_pred = UNDEFINED_HANDLE;
	RootMap::iterator k;
	for (k=root_map.begin(); k != root_map.end(); k++)
	{
		RootPair vk = *k;
		RootList *rl = vk.second;
		pursue = vk.first;

		bool unsolved = false;
		bool solved = false;

		std::vector<Handle>::iterator i;
		for (i=rl->begin(); i != rl->end(); i++)
		{
			Handle root = *i;
			if(predicate_solution[root] != NULL)
			{
				solved = true;
			}
			else
			{
				unsolved_pred = root;
				unsolved = true;
			}
		}
		if (solved && unsolved) break;
	}

	// pursue is a pointer to a node that's shared between
	// several predicates. One of the predicates has been
	// solved, another has not.  We want to now traverse 
	// upwards from this node, to find the top of the 
	// unsolved predicate.
	curr_root = unsolved_pred;
	curr_pred_handle = pursue;
}

/* ======================================================== */
/**
 * do_candidate - examine candidates, looking for matches.
 *
 * This routine is invoked on every candidate atom taken from
 * the atom space. That atom is assumed to anchor some part of
 * a graph that hopefully will match the predicate.
 */
bool PatternMatch::do_candidate(Handle ah)
{
	// Don't stare at our navel.
	std::vector<Handle>::iterator i;
	for (i = normed_predicate.begin(); i != normed_predicate.end(); i++)
	{
		if (ah == *i) return false;
	}

	// Cleanup
	predicate_solution.clear();
	var_solution.clear();
	while(!pred_handle_stack.empty()) pred_handle_stack.pop();
	while(!soln_handle_stack.empty()) soln_handle_stack.pop();
	while(!root_handle_stack.empty()) root_handle_stack.pop();
	while(!pred_solutn_stack.empty()) pred_solutn_stack.pop();

	curr_root = normed_predicate[0];
	curr_pred_handle = curr_root;
	bool found = soln_up(ah);

	// If found is false, then there's no solution here.
	// Bail out, return false to try again with the next candidate.
	return found;
}

/**
 * Create an associative array that gives a list of all of the
 * predicatees that a given node participates in.
 */
bool PatternMatch::note_root(Handle h)
{
	RootList *rl = root_map[h];
	if (NULL == rl)
	{
		rl = new RootList();
		root_map[h] = rl;
	}
	rl->push_back(curr_root);

	foreach_outgoing_handle(h, &PatternMatch::note_root, this);
	return false;
}

/**
 * Solve a predicate.
 * Its understood that the input "graph" is a predicate, of sorts,
 * with the list of "bound vars" are to be solved for (or "evaluated")
 * bound vars must be, by definition, Nodes.
 */
void PatternMatch::match(PatternMatchCallback *cb,
                         std::vector<Handle> *preds,
                         std::vector<Handle> *vars)
{
	std::vector<Handle>::const_iterator i;

	normed_predicate = *preds;

	for (i = vars->begin();
	     i != vars->end(); i++)
	{
		Handle h = *i;
		bound_vars.insert(h);
	}

	var_solution.clear();
	predicate_solution.clear();

	if (normed_predicate.size() == 0) return;

	// Preparation prior to search.
	// Create a table of nodes in the predicates, with
	// a list of the predicates that each node participates in.
	root_map.clear();
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); i++)
	{
		Handle h = *i;
		curr_root = h;
		note_root(h);
	}
	pmc = cb;

#if 0
	// Print out the predicate ...
	printf("\nPredicate is\n");
	std::vector<Handle>::iterator i;
	for (i = normed_predicate.begin();
	     i != normed_predicate.end(); i++)
	{
		foreach_outgoing_atom(*i, &PatternMatch::prt, this);
	}

	// Print out the bound variables in the predicate.
	std::set<Handle>::const_iterator j;
	for (j=bound_vars.begin(); j != bound_vars.end(); j++)
	{
		Handle h = *j;
		Atom *a = TLB::getAtom(h);
		Node *n = dynamic_cast<Node *>(a);
		if (n)
		{
			printf(" bound var: %s\n", n->getName().c_str());
		}
	}
#endif

	// Get type of the first item in the predicate list.
	Handle h = normed_predicate[0];
	Atom *a = TLB::getAtom(h);
	Type ptype = a->getType();

	// Plunge into the deep end - start looking at all viable
	// candidates in the AtomSpace.
	foreach_handle_of_type(atom_space, ptype,
	      &PatternMatch::do_candidate, this);

}

void PatternMatch::print_solution(std::map<Handle, Handle> &preds,
                                  std::map<Handle, Handle> &vars)
{
#if 0
	printf("\nSolution variable bindings:\n");

	// Print out the bindings of solutions to variables.
	std::set<Handle>::const_iterator j;
	for (j=bound_vars.begin(); j != bound_vars.end(); j++)
	{
		Handle var = *j;
		Handle soln = vars[var];
		Atom *av = TLB::getAtom(var);
		Atom *as = TLB::getAtom(soln);
		Node *nv = dynamic_cast<Node *>(av);
		Node *ns = dynamic_cast<Node *>(as);
		if (ns && nv)
		{
			printf("var %s solved by %s\n", 
			       nv->getName().c_str(), ns->getName().c_str());
		}
	}
#endif

	// Print out the full binding to all of the preds.
	printf("\nFull solution:\n");
	std::map<Handle, Handle>::const_iterator m;
	for (m = preds.begin(); m != preds.end(); m++) 
	{
		std::pair<Handle, Handle> pm = *m;
		prt(pm.second);
	}
	fflush(stdout);
}

/* ===================== END OF FILE ===================== */
