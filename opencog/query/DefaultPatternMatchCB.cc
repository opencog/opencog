/*
 * DefaultPatternMatchCB.cc
 *
 * Copyright (C) 2008,2009 Linas Vepstas
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

/* ======================================================== */

Handle DefaultPatternMatchCB::find_starter(Handle h)
{
	AtomSpace *as = pme->get_atomspace();
	Type t = as->getType(h);
	if (classserver().isNode(t)) {
		if (t != VARIABLE_NODE) return h;
		return Handle::UNDEFINED;
	}

	starter_pred = h;
	const std::vector<Handle> &vh = as->getOutgoing(h);
	for (size_t i = 0; i < vh.size(); i++) {
		Handle hout = vh[i];
		Handle s = find_starter(hout);
		if (s != Handle::UNDEFINED) return s;
	}

	return Handle::UNDEFINED;
}

bool DefaultPatternMatchCB::loop_candidate(Handle h)
{
	// printf("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
	// printf("Loop candidate: %s\n", pme->get_atomspace()->atomAsString(h).c_str());
	return pme->do_candidate(root, starter_pred, h);
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
void DefaultPatternMatchCB::perform_search(PatternMatchEngine *_pme,
                         const std::vector<Handle> &vars,
                         const std::vector<Handle> &clauses,
                         const std::vector<Handle> &negations)
{
	pme = _pme;

	// Ideally, we start our search at some node, any node, that is
	// not a variable, that is in the first clause. If the first
	// clause consists entirely of variable nodes, then we are 
	// screwed, and must search over all links that have the same 
	// type as the first clause.  Alternately, if there are *no*
	// variables at all, then we must assume a very general match.
	Handle h = clauses[0];
	root = h;
	Handle start = find_starter(h);
	if ((Handle::UNDEFINED != start) && (0 != vars.size()))
	{
		// printf("Search start node: %s\n", as->atomAsString(start).c_str());
		// printf("Start pred is: %s\n", as->atomAsString(starter_pred).c_str());
		foreach_incoming_handle(start,
		                  &DefaultPatternMatchCB::loop_candidate, this);
	}
	else
	{
		starter_pred = root;

		// Get type of the first item in the predicate list.
		AtomSpace *as = pme->get_atomspace();
		Type ptype = as->getType(h);

		// Plunge into the deep end - start looking at all viable
		// candidates in the AtomSpace.
		as->foreach_handle_of_type(ptype,
		      &DefaultPatternMatchCB::loop_candidate, this);
	}
}

/* ===================== END OF FILE ===================== */
