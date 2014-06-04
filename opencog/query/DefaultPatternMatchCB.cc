/*
 * DefaultPatternMatchCB.cc
 *
 * Copyright (C) 2008,2009,2014 Linas Vepstas
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

#include "DefaultPatternMatchCB.h"
#include "PatternMatchEngine.h"

#include <opencog/atomspace/Foreach.h>

using namespace opencog;

// #define DEBUG 1
#if DEBUG
   #define dbgprt(f, varargs...) printf(f, ##varargs)
#else
   #define dbgprt(f, varargs...)
#endif

/* ======================================================== */

// Find a good place to start the search.
//
// The handle h points to a clause.  In principle, it is enough to
// simply find a constant in the clause, and just start there. In
// practice, this can be an awful way to do things. So, for example,
// most "typical" clauses will be of the form
//
//    EvaluationLink
//        PredicteNode "blah"
//        ListLink
//            VariableNode $var
//            ConceptNode  "item"
//
// Typically, the incoming set to "blah" will be huge, so starting the
// search there would be a poor choice. Typically, the incoming set to
// "item" will be much smaller, and so makes a better choice.  The code
// below tries to pass over "blah" and pick "item" instead.  It does so
// by comparing the size of the incoming sets of the two constants, and
// picking the one with the smaller ("thinner") incoming set. Note that
// this is a form of "greedy" search.
//
// Note that the algo performs a full-depth search to find this. That's
// OK, because typeical clauses are never deep.
//
// Note that the size of the incoming set really is a better measure,
// and not the depth.  So, for example, if "item" has a huge incoming
// set, but "blah" does not, then "blah" is a much better place to
// start.
//
// size_t& depth will be set to the depth of the thinnest constant found.
// Handle& start will be set to the link containing that constant.
// size_t& width will be set to the incoming-set size of the thinnest
//               constant found.
// The returned value will be the constant at which to start the search.
// If no constant is found, then the returned value is the undefnied
// handle.
//
Handle
DefaultPatternMatchCB::find_starter(Handle h, size_t& depth,
                                    Handle& start, size_t& width)
{

	// If its a node, then we are done. Don't modiy either depth or
	// start.
	Type t = h->getType();
	if (classserver().isNode(t)) {
		if (t != VARIABLE_NODE) {
			width = h->getIncomingSetSize();
			return h;
		}
		return Handle::UNDEFINED;
	}

	size_t deepest = depth;
	start = Handle::UNDEFINED;
	Handle hdeepest(Handle::UNDEFINED);
	size_t thinnest = SIZE_MAX;

	// Iterate over all the handles in the outgoing set.
	// Find the deepest one that contains a constant, and start
	// the search there.  If there are two at the same depth,
	// then start with the skinnier one.
	LinkPtr ll(LinkCast(h));
	const std::vector<Handle> &vh = ll->getOutgoingSet();
	for (size_t i = 0; i < vh.size(); i++) {

		size_t brdepth = depth + 1;
		size_t brwid = SIZE_MAX;
		Handle sbr(h);
		Handle s(find_starter(vh[i], brdepth, sbr, brwid));

		if (s != Handle::UNDEFINED
		    and (brwid < thinnest
		         or (brwid == thinnest and deepest < brdepth)))
		{
			deepest = brdepth;
			hdeepest = s;
			start = sbr;
			thinnest = brwid;
		}

	}
	depth = deepest;
	width = thinnest;
	return hdeepest;
}

bool DefaultPatternMatchCB::loop_candidate(Handle h)
{
	dbgprt("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
	dbgprt("Loop candidate: %s\n", h->toShortString().c_str());
	return _pme->do_candidate(_root, _starter_pred, h);
}

/**
 * Search for solutions/groundings over all of the AtomSpace, using
 * some "reasonable" assumptions for what might be searched for. Or,
 * to put it bluntly, this search method *might* miss some possible
 * solutions, for certain "unusual" search types. The trade-off is
 * that this search algo should really be quite fast for "normal"
 * search types.
 *
 * This search algo makes the following (important) assumptions:
 *
 * 1) If there are no variables in the clauses, then this will search
 *    over all links which have the same type as the first clause.
 *    Clearly, this kind of search can fail if link_match() callback
 *    was prepared to accept other link types as well.
 *
 * 2) If there are variables, then the search will begin at the first
 *    non-variable node in the first clause.  The search will proceed
 *    by exploring the entire incoming-set for this node, but no farther.
 *    If the node_match() callback is willing to accept a broader range
 *    of node matches, esp for this initial node, then many possible
 *    solutions will be missed.
 *
 * 3) If the clauses consist entirely of variables, the same search
 *    as described in 1) will be performed.
 *
 * The above describes the limits to the "typical" search that this
 * algo can do well. In particular, if the constraint of 2) can be met,
 * then the search can be quite rapid, since incoming sets are often
 * quite small; and assumption 2) limits the search to "nearby",
 * connected atoms.
 *
 * Note that the default implementation of node_match() and link_match()
 * in this class does satisfy both 1) and 2), so this algo will work
 * correctly if these two methods are not overloaded.
 *
 * If you overload node_match(), and do so in a way that breaks
 * assumption 2), then you will scratch your head, thinking
 * "why did my search fail to find this obvious solution?" The answer
 * will be for you to create a new search algo, in a new class, that
 * overloads this one, and does what you want it to.  This class should
 * probably *not* be modified, since it is quite efficient for the
 * "normal" case.
 */
void DefaultPatternMatchCB::perform_search(PatternMatchEngine *pme,
                         std::set<Handle> &vars,
                         std::vector<Handle> &clauses,
                         std::vector<Handle> &negations)
{
	_pme = pme;

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

	size_t thinnest = SIZE_MAX;
	size_t deepest = 0;
	size_t bestclause = 0;
	Handle best_start(Handle::UNDEFINED);
	_starter_pred = Handle::UNDEFINED;

	size_t nc = clauses.size();
	for (size_t i=0; i < nc; i++) {
		Handle h(clauses[i]);
		size_t depth = 0;
		size_t width = SIZE_MAX;
		Handle pred(Handle::UNDEFINED);
		Handle start(find_starter(h, depth, pred, width));
		if (start != Handle::UNDEFINED
		    and (width < thinnest
		         or (width == thinnest and depth > deepest)))
		{
			thinnest = width;
			deepest = depth;
			bestclause = i;
			best_start = start;
			_starter_pred = pred;
		}
	}

	if ((Handle::UNDEFINED != best_start) && (0 != vars.size()))
	{
		_root = clauses[bestclause];
		dbgprt("Search start node: %s\n", best_start->toShortString().c_str());
		dbgprt("Start pred is: %s\n", _starter_pred->toShortString().c_str());
		IncomingSet iset = get_incoming_set(best_start);
		size_t sz = iset.size();
		for (size_t i = 0; i < sz; i++) {
			bool rc = loop_candidate(Handle(iset[i]));
			if (rc) break;
		}
	}
	else
	{
		_root = clauses[0];
		_starter_pred = _root;

		dbgprt("Start pred is: %s\n", _starter_pred->toShortString().c_str());
		// Get type of the first item in the predicate list.
		Type ptype = _root->getType();

		// Plunge into the deep end - start looking at all viable
		// candidates in the AtomSpace.

		// XXX TODO -- as a performance optimization, we should try all
		// the different clauses, and find the one with the smallest number
		// of atoms of that type, or otherwise try to find a small ("thin")
		// incoming set to search over.
		AtomSpace *as = _pme->get_atomspace();
		as->foreach_handle_of_type(ptype,
		      &DefaultPatternMatchCB::loop_candidate, this);
	}
}

/* ======================================================== */

bool DefaultPatternMatchCB::post_link_match(LinkPtr& lpat, LinkPtr& lsoln)
{
	return false;
}

/* ===================== END OF FILE ===================== */
