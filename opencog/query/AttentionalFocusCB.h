/*
 * AttentionalFocusCB.h
 *
 *  Created on: 17 Jun, 2014
 *      Author: misgana
 */

#ifndef _ATTENTIONAL_FOCUS_CB_H
#define _ATTENTIONAL_FOCUS_CB_H

#include <opencog/atomspace/AtomSpace.h>

#include "PatternMatchCallback.h"
#include "DefaultPatternMatchCB.h"

namespace opencog {

class AttentionalFocusCB:public virtual DefaultPatternMatchCB {

private:
	AtomSpace * _atom_space;
public:
	AttentionalFocusCB(AtomSpace * as) :
			 DefaultPatternMatchCB(as),_atom_space(as){
	}

	// Pass all the calls straight through, except one.
	bool node_match(Handle& node1, Handle& node2) {
		// If equality, then a match.
		if (node1 == node2)
			return false;
		return true;
	}

	bool link_match(LinkPtr& lpat, LinkPtr& lsoln) {
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

	IncomingSet get_incoming_set(Handle h) {
		return h->getIncomingSet();
	}

};
} //namespace opencog
#endif /* _ATTENTIONALFOCUSCB_H */
