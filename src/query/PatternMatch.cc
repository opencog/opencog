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

/* ======================================================== */

/**
 * Return true, for example, if the node is _subj or _obj
 */
bool PatternMatch::is_ling_rel(Atom *atom)
{
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == atom->getType()) return true;
	return false;
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
 * Hack --- not actually applying any rules, except one
 * hard-coded one: if the link involves a
 * DEFINED_LINGUISTIC_RELATIONSHIP_NODE, then its a keeper.
 */
bool PatternMatch::apply_rule(Atom *atom)
{
	if (EVALUATION_LINK != atom->getType()) return false;

	Handle ah = TLB::getHandle(atom);
	bool keep = foreach_outgoing_atom(ah, &PatternMatch::is_ling_rel, this);

	if (!keep) return false;

	// Its a keeper, add this to our list of acceptable predicate terms.
	normed_predicate.push_back(ah);

	// Create a table of nodes in the predicates, with
	// a list of the predicates that each node participates in.
	curr_root = ah;
	foreach_outgoing_handle(ah, &PatternMatch::note_root, this);
	return false;
}

/**
 * Put predicate into "normal form".
 * In this case, a cheap hack: remove all relations that
 * are not "defined linguistic relations", e.g. all but
 * _subj(x,y) and _obj(z,w) relations.
 */
void PatternMatch::filter(Handle graph, const std::vector<Handle> &bvars)
{
	std::vector<Handle>::const_iterator i;
	for (i = bvars.begin();
	     i != bvars.end(); i++)
	{
		Handle h = *i;
		bound_vars.insert(h);
	}

	var_solution.clear();
	predicate_solution.clear();
	normed_predicate.clear();
	root_map.clear();
	foreach_outgoing_atom(graph, &PatternMatch::apply_rule, this);
}

/* ======================================================== */

/**
 * Are two atoms instances of the same concept?
 * Return true if they are are NOT (that is, if they
 * are mismatched). This stops iteration in the standard
 * iterator.
 */
bool PatternMatch::concept_match(Atom *aa, Atom *ab)
{
	std::string sa = aa->toString();
	std::string sb = ab->toString();
	printf ("concept comp %s\n"
           "          to %s\n", sa.c_str(), sb.c_str());

	// If they're the same atom, then clearly they match.
	if (aa == ab) return false;

	// Look for incoming links that are InheritanceLinks.
	// The "generalized concept" for this should be at the far end.
	Atom *ca = fl.follow_binary_link(aa, INHERITANCE_LINK);
	Atom *cb = fl.follow_binary_link(ab, INHERITANCE_LINK);

	sa = ca->toString();
	sb = cb->toString();
	printf ("gen comp %d %s\n"
           "         to %s\n", ca==cb, sa.c_str(), sb.c_str());

	if (ca == cb) return false;
	return true;
}

/* ======================================================== */
/**
 * Check to see if atom is a bound variable.
 * Heuristics are used to determine this: the local atom should
 * be an instance of a concept, whose dictionary word is _$qVar,
 * i.e. one of the bound variable names.
 *
 * XXX This heuristic should be re-examined/rethought at some point.
 *
 * If it is, return pointer to the canonical var.
 */
Atom * PatternMatch::is_var(Atom *atom)
{
	// The local atom will be an instance of a general concept...
	atom = fl.follow_binary_link(atom, INHERITANCE_LINK);
	if(!atom) return NULL;
	// and we want the "word" associated with this general concept.
	atom = fl.backtrack_binary_link(atom, WR_LINK);
	if(!atom) return NULL;

	Handle h = TLB::getHandle(atom);
	if (bound_vars.count(h)) return atom;
	return NULL;
}

/* ======================================================== */

/**
 * tree comparison failed, so erase the proposed solution.
 */
bool PatternMatch::erase_solution(Handle h)
{
	// std:map<Handle,Handle>::const_iterator it = var_solution.find(h);
	// if (*it) var_solution[h] = NULL;
	var_solution[h] = NULL;
	foreach_outgoing_handle(h, &PatternMatch::erase_solution, this);
	return false;
}

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
	Atom *canon_var = is_var(aa);
	if (canon_var)
	{
		// If ab is the very same var, then its a mismatch.
		if (aa == ab) return true;

		// Else, we have a candidate solution.
		// Make a record of it.
		var_solution[ha] = ab;
		var_solution[TLB::getHandle(canon_var)] = ab;
		return false;
	}

	// If they're the same atom, then clearly they match.
	// ... but only if aa is NOT a bound var.
	if (aa == ab)
	{
		var_solution[ha] = ab;
		return false;
	}

	// If one is null, but the other is not, there's clearly no match.
	if (aa && !ab) return true;
	if (ab && !aa) return true;

	// If types differ, then no match.
	if (aa->getType() != ab->getType()) return true;

std::string sta = aa->toString();
std::string stb = ab->toString();
printf ("tree_compare depth=%d comp %s\n"
        "                       to %s\n", depth, sta.c_str(), stb.c_str());

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
printf("tree_comp down link mismatch=%d\n", mismatch);
		return mismatch;
	}

	// If we are here, then we are comparing nodes.
	// The result of comparing nodes depends on the
	// node types.
	Type ntype = aa->getType();

	// DefinedLinguisticRelation nodes must match exactly;
	// so if we are here, there's already a mismatch.
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == ntype) return true;

	// Concept nodes can match if they inherit from the same concept.
	if (CONCEPT_NODE == ntype)
	{
		bool mismatch = concept_match(aa, ab);
		if (false == mismatch) var_solution[ha] = ab;
printf("tree_comp concept mist=%d\n", mismatch);
		return mismatch;
	}
	fprintf(stderr, "Error: unexpected node type %d %s\n", ntype,
	        ClassServer::getTypeName(ntype));

	std::string sa = aa->toString();
	std::string sb = ab->toString();
	fprintf (stderr, "unexpected depth=%d comp %s\n"
	                 "                      to %s\n",
	        depth, sa.c_str(), sb.c_str());

	return true;
}

/* ======================================================== */

bool PatternMatch::soln_up(Atom *as)
{
	Atom *ap = TLB::getAtom(curr_pred_handle);
	bool no_match = tree_compare(ap, as);

	// If no match, try the next one.
	if (no_match) return false;

	// Ahh ! found a match!
	// If we've navigated to the top of the predicate, then we're done!
	if (curr_pred_handle == curr_root) return true;

printf("have soln match, ");
prt(as);
printf("moving up the pred\n");
	// Move up the predicate, and hunt for a match, again.
	curr_soln_handle = TLB::getHandle(as);
	bool found = foreach_incoming_atom(curr_pred_handle,
	                &PatternMatch::pred_up, this);
printf("up pred find =%d\n", found);
	return found;
}

bool PatternMatch::pred_up(Atom *a)
{
	Handle h = TLB::getHandle(a);

	// Is this atom even a part of the predicate we are considering?
	// If not, try the next atom.
	bool valid = ot.is_node_in_tree(curr_root, h);
	if (!valid) return false;

	// Now, move up the solution outgoing set, looking for a match.
	curr_pred_handle = h;

	bool found = foreach_incoming_atom(curr_soln_handle,
	                     &PatternMatch::soln_up, this);

printf("duude upward soln find =%d\n", found);
	return found;
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
	Atom *atom = TLB::getAtom(ah);

std::string str = atom->toString();
printf ("\nduuude candidate %s\n", str.c_str());
	// Compare a predicate tree to a tree in the graph.
	// The compare is pair-wise, in parallel.
	depth = 1;
	bool mismatch = foreach_outgoing_atom_pair(normed_predicate[0], ah,
	                 &PatternMatch::tree_compare, this);
	depth = 0;

	if (mismatch)
	{
		erase_solution(ah);
		// Return false to try the next candidate.
		return false;
	}

str = atom->toString();
printf ("duuude --------------------- \npred zero solved %s\n", str.c_str());
	Handle ph = normed_predicate[0];
	predicate_solution[ph] = ah;

	// Now, search for an as-yet unsolved/unmatched predicate.
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

	// If there are no further predicates to solve,
	// we are done! Return true to terminate the search.
	if (UNDEFINED_HANDLE == pursue) return true;

printf("duude next handle is ");
prt(TLB::getAtom(pursue));
	// pursue is a pointer to a node that's shared between
	// several predicates. One of the predicates has been
	// solved, another has not.  We want to now traverse 
	// upwards from this node, to find the top of the 
	// unsolved predicate.
	curr_root = unsolved_pred;
	curr_soln_handle = var_solution[pursue];
	bool found = foreach_incoming_atom(pursue, &PatternMatch::pred_up, this);

printf("final up result = %d\n", found);

	return true;
}

/**
 * Solve a predicate.
 * Its understood that the input "graph" is a predicate, of sorts,
 * with the list of "bound vars" are to be solved for (or "evaluated")
 * bound vars must be, by definition, Nodes.
 */
void PatternMatch::match(void)
{
	if (normed_predicate.size() == 0) return;

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

	// Get type of the first item in the predicate list.
	Handle h = normed_predicate[0];
	Atom *a = TLB::getAtom(h);
	Type ptype = a->getType();

	// Plunge into the deep end - start looking at all viable
	// candidates in the AtomSpace.
	foreach_handle_of_type(atom_space, ptype,
	      &PatternMatch::do_candidate, this);

	print_solution();
}

void PatternMatch::print_solution(void)
{
	printf("\nSolution vector:\n");

	// Print out the solution vector.
	std::set<Handle>::const_iterator j;
	for (j=bound_vars.begin(); j != bound_vars.end(); j++)
	{
		Handle var = *j;
		Handle soln = var_solution[var];
printf("\nnyerh hare hare var=%p soln=%p\n", var, soln);
		Atom *av = TLB::getAtom(var);
		Atom *as = TLB::getAtom(soln);
prt(av);
		Node *nv = dynamic_cast<Node *>(av);
		Node *ns = dynamic_cast<Node *>(as);
		if (ns && nv)
		{
			printf("var %s solved by %s\n", 
			       nv->getName().c_str(), ns->getName().c_str());
		}
	}
}

/* ===================== END OF FILE ===================== */
