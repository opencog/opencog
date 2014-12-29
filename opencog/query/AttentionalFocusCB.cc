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

bool AttentionalFocusCB::node_match(Handle& node1, Handle& node2)
{
	if (node1 == node2
			and node2->getAttentionValue()->getSTI()
					> _atom_space->getAttentionalFocusBoundary())
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool AttentionalFocusCB::link_match(LinkPtr& lpat, LinkPtr& lsoln)
{
	if (DefaultPatternMatchCB::link_match(lpat, lsoln))
	{
		return true;
	}
	if (lsoln->getAttentionValue()->getSTI()
			> _atom_space->getAttentionalFocusBoundary())
	{
		return false;
	}
	else
	{
		return true;
	}
}

IncomingSet AttentionalFocusCB::get_incoming_set(Handle h)
{
	const IncomingSet &incoming_set = h->getIncomingSet();

	// Discard the part of the incoming set that is below the
	// AF boundary.  The PM will look only at those links that
	// this callback returns; thus we avoid searching the low-AF
	// parts of the hypergraph.
	IncomingSet filtered_set;
	for (IncomingSet::const_iterator i = incoming_set.begin();
			i != incoming_set.end(); ++i)
	{
		Handle candidate_handle(*i);
		if (candidate_handle->getAttentionValue()->getSTI()
				> _atom_space->getAttentionalFocusBoundary())
		{
			filtered_set.push_back(LinkCast(candidate_handle));
		}
	}

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
