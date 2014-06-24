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

bool AttentionalFocusCB::node_match(Handle& node1, Handle& node2) {
	if (node1 == node2
			and node2->getAttentionValue()->getSTI()
					> _atom_space->getAttentionalFocusBoundary()) {
		return false;
	} else {
		return true;
	}
}

bool AttentionalFocusCB::link_match(LinkPtr& lpat, LinkPtr& lsoln) {
	if (DefaultPatternMatchCB::link_match(lpat, lsoln)) {
		return true;
	}
	if (lsoln->getAttentionValue()->getSTI()
			> _atom_space->getAttentionalFocusBoundary()) {
		return false;
	} else {
		return true;
	}
}

IncomingSet AttentionalFocusCB::get_incoming_set(Handle h) {
	const IncomingSet &incoming_set = h->getIncomingSet();
	IncomingSet filtered_set;
	for (IncomingSet::const_iterator i = incoming_set.begin();
			i != incoming_set.end(); ++i) {
		Handle candidate_handle(*i);
		if (candidate_handle->getAttentionValue()->getSTI()
				> _atom_space->getAttentionalFocusBoundary()) {
			filtered_set.push_back(LinkCast(candidate_handle));

		}
	}
	// if none is in AF
	if (filtered_set.empty()) {
		//xxx what shall we do here?, return the default or return empty ?
		filtered_set = incoming_set;
	}
	std::sort(filtered_set.begin(), filtered_set.end(), compare_sti); //sort by STI for better performance

	return filtered_set;
}

