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

#include <opencog/attentionbank/AttentionalFocusCB.h>
#include <opencog/attentionbank/AttentionBank.h>

using namespace opencog;

// XXX FIXME -- do we need this function, at all?  Why isn't it
// sufficient to just do a normal pattern search, and weed out
// the atttention focus after the fact? I find it very hard to
// beleive that this provides any significant performance kick
// over a simpler, more modular design.

AttentionalFocusCB::AttentionalFocusCB(AtomSpace* as) :
	DefaultPatternMatchCB(as)
{
}

bool AttentionalFocusCB::node_match(const Handle& node1, const Handle& node2)
{
	return node1 == node2 and attentionbank(_as).atom_is_in_AF(node2);
}

bool AttentionalFocusCB::link_match(const PatternTermPtr& ptm, const Handle& lsoln)
{
	return DefaultPatternMatchCB::link_match(ptm, lsoln) and
		attentionbank(_as).atom_is_in_AF(lsoln);
}

IncomingSet AttentionalFocusCB::get_incoming_set(const Handle& h)
{
	IncomingSet incoming_set = h->getIncomingSet();

	// Discard the part of the incoming set that is below the
	// AF boundary.  The PM will look only at those links that
	// this callback returns; thus we avoid searching the low-AF
	// parts of the hypergraph.
	IncomingSet filtered_set;
	for (const auto& l : incoming_set) {
		if (attentionbank(_as).atom_is_in_AF(Handle(l)))
			filtered_set.push_back(l);
	}

	// If nothing is in AF
	if (filtered_set.empty())
	{
		// Returning the empty set abandons the search in this direction.
		// Search will then backtrack and try a different direction.
		// ... and that is exactly what should be happening.
		return filtered_set;
	}

	auto compare_sti = [&](const LinkPtr& lptr1, const LinkPtr& lptr2)->bool
	{
		return get_sti(Handle(lptr1)) > get_sti(Handle(lptr2));
	};

	// The exploration of the set of patterns proceeds by going through
	// the incoming set, one by one.  So sorting the incoming set will
	// cause the exploration to look at the highest STI atoms first.
	std::sort(filtered_set.begin(), filtered_set.end(), compare_sti);

	return filtered_set;
}
