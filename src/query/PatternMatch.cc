/*
 * PatternMatch.cc
 *
 * Linas Vepstas February 2008
 */

#include "Foreach.h"
#include "Link.h"
#include "Node.h"
#include "PatternMatch.h"
#include "TLB.h"

using namespace opencog;

bool PatternMatch::prt(Atom *atom)
{
	std::string str = atom->toString();
	printf ("duuude its %s\n", str.c_str());
	return false;
}

bool PatternMatch::apply_rule(Atom *atom)
{
	std::string str = atom->toString();
	printf ("duude checking %s\n", str.c_str());
	return false;
}

/**
 * Put predicate into "normal form".
 * In this case, a checp hack: remove all but _subj(x,y) 
 * and __obj(z,w) rel's
 */
Handle PatternMatch::filter(Handle graph, const std::vector<Handle> &bound_vars)
{
	norm_outgoing.clear();
	foreach_outgoing_atom(graph, &PatternMatch::apply_rule, this);

	Link *norm_pred = new Link(ASSERTION_LINK, norm_outgoing);
	return TLB::getHandle(norm_pred);
}

/**
 * Solve a predicate.
 * Its understood that the input "graph" is a predicate, of sorts, 
 * with the list of "bound vars" are to be solved for (or "evaluated")
 * bound vars must be, by definition, Nodes.
 */
void PatternMatch::match(Handle graph, const std::vector<Handle> &bound_vars)
{
	size_t i;

	// Just print out the bound variables
	for (i=0; i<bound_vars.size(); i++)
	{
		Handle v = bound_vars[i];
		Atom *a = TLB::getAtom(v);
		Node *n = dynamic_cast<Node *>(a);
		if (n)
		{
			printf(" bound var: %s\n", n->getName().c_str());
		}
	}

	// print out some of the graph ...
	foreach_outgoing_atom(graph, &PatternMatch::prt, this);
}
