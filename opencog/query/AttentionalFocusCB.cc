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

// Uncomment below to enable debug print
// #define DEBUG
#ifdef DEBUG
   #define dbgprt(f, varargs...) printf(f, ##varargs)
#else
   #define dbgprt(f, varargs...)
#endif

bool AttentionalFocusCB::node_match(const Handle& node1, const Handle& node2)
{
	return node1 == node2 and
		node2->getSTI() > _as->getAttentionalFocusBoundary();
}

bool AttentionalFocusCB::link_match(const LinkPtr& lpat, const LinkPtr& lsoln)
{
	return DefaultPatternMatchCB::link_match(lpat, lsoln)
		and lsoln->getSTI() > _as->getAttentionalFocusBoundary();
}

IncomingSet AttentionalFocusCB::get_incoming_set(const Handle& h)
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
		// Returning the empty set abandons the search in this direction.
		// Search will then backtrack and try a different direction.
		// ... and that is exactly what should be happening.
		// But it might also mean that there is an AF bug somewhere ...
		// Returning the empty set is right thing, but testing is needed.
		//
		// XXX TODO Currently, ForwardChainerUTest fails, when we
		// return the empty set ... this is surely because the
		// ForwardChainerUTest has not given sufficient stimulous to
		// to the right atoms, for this to actually work right.
		// So that test needs to be fixed FIXME ... or AF itself is
		// broken.
		// return filtered_set;
		filtered_set = incoming_set;
	}

	// The exploration of the set of patterns proceeds by going through
	// the incoming set, one by one.  So sorting the incoming set will
	// cause the exploration to look at the highest STI atoms first.
	std::sort(filtered_set.begin(), filtered_set.end(), compare_sti);

	return filtered_set;
}

// XXX FIXME, this is a bad-cut-n-paste job of the DefaultCB,
// with some unclear modifications..
bool AttentionalFocusCB::initiate_search(PatternMatchEngine *pme,
                                         const std::set<Handle> &vars,
                                         const std::vector<Handle> &clauses)
{
	_search_fail = false;

	bool found = neighbor_search(pme, vars, clauses);
	if (found) return true;
	if (not _search_fail) return false;

	// If we are here, then we could not find a clause at which to start,
	// as, apparently, the clauses consist entirely of variables!! So,
	// basically, we must search the entire!! atomspace, in this case.
	// Yes, this hurts.
	_root = clauses[0];
	_starter_term = _root;

	dbgprt("Start term is: %s\n", _starter_term->toShortString().c_str());

	// XXX TODO -- as a performance optimization, we should try all
	// the different clauses, and find the one with the smallest number
	// of atoms of that type, or otherwise try to find a small ("thin")
	// incoming set to search over.

	// Plunge into the deep end - start looking at all viable
	// candidates in the AtomSpace.
	HandleSeq handle_set;
	_as->getHandleSetInAttentionalFocus(back_inserter(handle_set));
// xxxxxxxxxxxxxx here xxxxxxxxxxx

	// WARNING: if there's nothing in the attentional focus then get
	// the whole atomspace  ... XXX This cannot possibly be right!!
	// If nothig is in attentional focus, then certainly, the whole
	// atomspace should NOT be searched!! XXX So this is utterly wrong!
	// However, both RuleUTest and ForwardChainerUTest fail unless
	// we search the whole atomspace.  So that means that these two
	// unit tests ae failing to correctly set up the attentional focus
	// boundary!  Boooo! the tests are broken, they need to be fixed...
	if (handle_set.empty())
		_as->getHandlesByType(back_inserter(handle_set), ATOM, true);

#if WHAT_THE_HUHH_THIS_CANT_BE_RIGHT
	// Get the set of types of the potential candidates
	// XXX FIXME ... this cannot be right; when-ever would the
	// first clause just be a variable, all by itself??
	std::set<Type> ptypes;
	if (_root->getType() == VARIABLE_NODE)
		ptypes = _type_restrictions->at(_root);


	// Filter by variable types
	if (not ptypes.empty()) {
		auto it = remove_if(handle_set.begin(), handle_set.end(),
		                    [&ptypes](const Handle& h) {
			                    return ptypes.find(h->getType()) == ptypes.end();
		                    });
		handle_set.erase(it, handle_set.end());
	}
#endif

#ifdef DEBUG
	size_t i = 0;
#endif
	for (const Handle& h : handle_set)
	{
		dbgprt("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
		dbgprt("Loop candidate (%lu/%lu): %s\n", ++i, handle_set.size(),
		       h->toShortString().c_str());
		bool rc = pme->explore_neighborhood(_root, _starter_term, h);
		if (rc) return true;
	}
	return false;
}
