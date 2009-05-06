/*
 * PatternMatchEngine.cc
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
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/TLB.h>

using namespace opencog;

/* ======================================================== */

Handle DefaultPatternMatchCB::find_starter(Handle h)
{
	Atom *a = TLB::getAtom(h);
	Link *l = dynamic_cast<Link *>(a);
	if (NULL == l)
	{
		Type t = a->getType();
		if (t != VARIABLE_NODE) return h;
		return Handle::UNDEFINED;
	}

	starter_pred = h;
	const std::vector<Handle> &vh = l->getOutgoingSet();
	for (size_t i = 0; i < vh.size(); i++) {
		Handle hout = vh[i];
		Handle s = find_starter(hout);
		if (s != Handle::UNDEFINED) return s;
	}

	return Handle::UNDEFINED;
}

bool DefaultPatternMatchCB::loop_candidate(Handle h)
{
	return pme->do_candidate(root,starter_pred,h);
}

/**
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
		// prtmsg("Search start node: ", start);
		foreach_incoming_handle(start,
		                  &DefaultPatternMatchCB::loop_candidate, this);
	}
	else
	{
		starter_pred = root;

		// Get type of the first item in the predicate list.
		Atom *a = TLB::getAtom(h);
		Type ptype = a->getType();

		// Plunge into the deep end - start looking at all viable
		// candidates in the AtomSpace.
		pme->get_atomspace()->foreach_handle_of_type(ptype,
		      &DefaultPatternMatchCB::loop_candidate, this);
	}
}

/* ===================== END OF FILE ===================== */
