/*
 * AttentionalFocusCB.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  July 2014
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
#include "AttentionalFocusCB.h"

using namespace opencog;

// #define DEBUG
#ifdef DEBUG
   #define dbgprt(f, varargs...) printf(f, ##varargs)
#else
   #define dbgprt(f, varargs...)
#endif

bool AttentionalFocusCB::node_match(Handle& node1, Handle& node2)
{
	return node1 == node2 and
		node2->getSTI() > _as->getAttentionalFocusBoundary();
}

bool AttentionalFocusCB::link_match(LinkPtr& lpat, LinkPtr& lsoln)
{
	return DefaultPatternMatchCB::link_match(lpat, lsoln)
		and lsoln->getSTI() > _as->getAttentionalFocusBoundary();
}

IncomingSet AttentionalFocusCB::get_incoming_set(Handle h)
{
	const IncomingSet &incoming_set = h->getIncomingSet();

	// Discard the part of the incoming set that is below the
	// AF boundary.  The PM will look only at those links that
	// this callback returns; thus we avoid searching the low-AF
	// parts of the hypergraph.
	IncomingSet filtered_set;
	for (const auto& l : incoming_set)
		if (l->getSTI() > _as->getAttentionalFocusBoundary())
			filtered_set.push_back(l);

	// If nothing is in AF
	if (filtered_set.empty())
	{
		// XXX What shall we do here? Return the default or return empty ?
		// Returning empty completely halts the search up to this point
		// ... and maybe that is a good thing, right? But it might also
		// mean that there is an AF bug somewhere ... I think that
		// returning the empty set is really probably the right thing ...
		// XXX TODO test with PLN and FIXME ...
		filtered_set = incoming_set;
	}

	// The exploration of the set of patterns proceeds by going through
	// the incoming set, one by one.  So sorting the incoming set will
	// cause the exploration to look at the highest STI atoms first.
	std::sort(filtered_set.begin(), filtered_set.end(), compare_sti);

	return filtered_set;
}

void AttentionalFocusCB::validate_clauses(std::set<Handle>& vars,
                                          std::vector<Handle>& clauses)
{
}

void AttentionalFocusCB::perform_search(PatternMatchEngine *pme,
                                        std::set<Handle> &vars,
                                        std::vector<Handle> &clauses,
                                        std::vector<Handle> &negations)
{
	// In principle, we could start our search at some node, any node,
	// that is not a variable. In practice, the search begins by
	// iterating over the incoming set of the node, and so, if it is
	// large, a huge amount of effort might be wasted exploring
	// dead-ends.  Thus, it pays off to start the search on the
	// node with the smallest ("narrowest" or "thinnest") incoming set
	// possible.  Thus, we look at all the clauses, to find the
	// "thinnest" one.
	//
	// Note also: the user is allowed to specify patterns that have
	// no constants in them at all.  In this case, the search is
	// performed by looping over all links of the given types.

	size_t bestclause;
	Handle best_start = find_thinnest(clauses, _starter_pred, bestclause);

	if ((Handle::UNDEFINED != best_start) and
	    // (Handle::UNDEFINED != _starter_pred) and
	    (!vars.empty()))
	{
		_root = clauses[bestclause];
		dbgprt("Search start node: %s\n", best_start->toShortString().c_str());
		dbgprt("Start pred is: %s\n", _starter_pred->toShortString().c_str());

		// This should be calling the over-loaded virtual method
		// get_incoming_set(), so that, e.g. it gets sorted by attentional
		// focus in the AttentionalFocusCB class...
		IncomingSet iset = get_incoming_set(best_start);
		size_t sz = iset.size();
		for (size_t i = 0; i < sz; i++) {
			Handle h(iset[i]);
			dbgprt("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
			dbgprt("Loop candidate: %s\n", h->toShortString().c_str());
			bool rc = pme->do_candidate(_root, _starter_pred, h);
			if (rc) break;
		}

		// If we are here, we are done.
		return;
	}

	// If we are here, then we could not find a clause at which to start,
	// as, apparently, the clauses consist entirely of variables!! So,
	// basically, we must search the entire!! atomspace, in this case.
	// Yes, this hurts.
	_root = clauses[0];
	_starter_pred = _root;

	dbgprt("Start pred is: %s\n", _starter_pred->toShortString().c_str());

	// Get the set of types of the potential candidates
	std::set<Type> ptypes;
	if (_root->getType() == VARIABLE_NODE)
		ptypes = (*_type_restrictions)[_root];

	// XXX TODO -- as a performance optimization, we should try all
	// the different clauses, and find the one with the smallest number
	// of atoms of that type, or otherwise try to find a small ("thin")
	// incoming set to search over.

	// Plunge into the deep end - start looking at all viable
	// candidates in the AtomSpace.
	std::list<Handle> handle_set;
	_as->getHandleSetInAttentionalFocus(back_inserter(handle_set));

	// WARNING: if there's nothing in the attentional focus then get
	// the whole atomspace
	if (handle_set.empty())
		_as->getHandlesByType(back_inserter(handle_set), ATOM, true);

	// Filter by variable types
	if (not ptypes.empty()) {
		auto it = remove_if(handle_set.begin(), handle_set.end(),
		                    [&ptypes](const Handle& h) {
			                    return ptypes.find(h->getType()) == ptypes.end();
		                    });
		handle_set.erase(it, handle_set.end());
	}

	for (const Handle& h : handle_set)
	{
		dbgprt("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
		dbgprt("Loop candidate: %s\n", h->toShortString().c_str());
		bool rc = pme->do_candidate(_root, _starter_pred, h);
		if (rc) break;
	}
}
