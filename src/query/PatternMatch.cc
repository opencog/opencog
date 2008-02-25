/*
 * PatternMatch.cc
 *
 * Linas Vepstas February 2008
 */

#include "Foreach.h"
#include "Node.h"
#include "PatternMatch.h"
#include "TLB.h"

using namespace opencog;

bool PatternMatch::prt(Atom *atom)
{
	std::string str = atom->toString();
	printf ("duuude its %s\n", str.c_str());
	return true;
}

/**
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
