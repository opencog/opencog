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
	norm_outgoing.push_back(TLB::getHandle(atom));
	return false;
}

/**
 * Put predicate into "normal form".
 * In this case, a checp hack: remove all relations that 
 * are not "defined lingussitic relations", e.g. all but 
 * _subj(x,y) and _obj(z,w) relations. 
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
