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
	std::string str = atom->toString();
	printf ("%s\n", str.c_str());
	return false;
}

/**
 * Return true, for example, if the node is _subj or _obj
 */
bool PatternMatch::is_ling_rel(Atom *atom)
{
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == atom->getType()) return true;
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
	normed_predicate.push_back(TLB::getHandle(atom));
	return false;
}

/**
 * Put predicate into "normal form".
 * In this case, a checp hack: remove all relations that 
 * are not "defined lingussitic relations", e.g. all but 
 * _subj(x,y) and _obj(z,w) relations. 
 */
void PatternMatch::filter(Handle graph, const std::vector<Handle> &bvars)
{
	bound_vars = bvars;
	normed_predicate.clear();
	foreach_outgoing_atom(graph, &PatternMatch::apply_rule, this);
}

/* ======================================================== */

// Return true if there's a mis-match.
bool PatternMatch::pair_compare(Atom *aa, Atom *ab)
{
	// If they're the same atom, then clearly they match.
	if (aa == ab) return false;

	// If types differ, then no match.
	if (aa->getType() != ab->getType()) return true;

	// If links, then compare link contents
	if (dynamic_cast<Link *>(aa))
	{
		Handle ha = TLB::getHandle(aa);
		Handle hb = TLB::getHandle(ab);
		depth ++;
		bool mismatch = foreach_outgoing_atom_pair(ha, hb,
		                 &PatternMatch::pair_compare, this);
		depth --;

		return mismatch;
	}

	// If we are here, then we are comparing nodes.
	// The result of comparing nodes depends on the 
	// node types.
	//
	// DefinedLinguisticRelation nodes must match exactly;
	// so if we are here, there's already a mismatch.
	if (DEFINED_LINGUISTIC_RELATIONSHIP_NODE == aa->getType()) return true;
	
	// Concept nodes can match if they inherit from the same concept.

	std::string sa = aa->toString();
	std::string sb = ab->toString();
	printf ("duuude dep=%d comp %s\n"
           "                to %s\n", depth, sa.c_str(), sb.c_str());

	return false;
}

bool PatternMatch::do_candidate(Atom *atom)
{
	// XXX Use the same basic filter rejection as was used to clean up
	// the predicate -- reject anything thats not a linguistic relation.
	Handle ah = TLB::getHandle(atom);
	bool keep = foreach_outgoing_atom(ah, &PatternMatch::is_ling_rel, this);
	if (!keep) return false;

	depth = 1;
	bool mismatch = foreach_outgoing_atom_pair(normed_predicate[0], ah, 
	                 &PatternMatch::pair_compare, this);
	depth = 0;

	if (mismatch) return false;

	std::string str = atom->toString();
	printf ("duuude have match %s\n", str.c_str());
	return false;
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
	for (i=bound_vars.begin(); i != bound_vars.end(); i++)
	{
		Atom *a = TLB::getAtom(*i);
		Node *n = dynamic_cast<Node *>(a);
		if (n)
		{
			printf(" bound var: %s\n", n->getName().c_str());
		}
	}

printf("\nnyerh hare hare\n");

	// Get type of the first item in the predicate list.
	Handle h = normed_predicate[0];
	Atom *a = TLB::getAtom(h);
	Type ptype = a->getType();

	// perform whole-hog searching
	foreach_handle_of_type(atom_space, ptype,
	      &PatternMatch::do_candidate, this);
}
