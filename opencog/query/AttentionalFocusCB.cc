/*#include "AttentionalFocusCB.h"

using namespace opencog;


	bool opencog::AttentionalFocusCB::node_match(Handle& node1, Handle& node2) {
		// If equality, then a match.
		if (node1 == node2)
			return false;
		return true;
	}

	bool opencog::AttentionalFocusCB::link_match(LinkPtr& lpat, LinkPtr& lsoln) {
		if (lpat == lsoln)
			return false;

		if (lpat->getArity() != lsoln->getArity())
			return true;
		Type pattype = lpat->getType();
		Type soltype = lsoln->getType();

		// If types differ, no match,
		if (pattype != soltype)
			return true;
		return false;
	}

	IncomingSet opencog::AttentionalFocusCB::get_incoming_set(Handle h) {
		return h->getIncomingSet();
	}
	*/
