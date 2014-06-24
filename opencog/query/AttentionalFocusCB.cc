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
		filtered_set=incoming_set;
	}
	std::sort(filtered_set.begin(), filtered_set.end(), compare_sti); //sort by STI for better performance

	return filtered_set;
}
