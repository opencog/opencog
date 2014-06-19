/*
 * AttentionalFocusCB.h
 *
 *  Created on: 17 Jun, 2014
 *      Author: misgana
 */

#ifndef _ATTENTIONAL_FOCUS_CB_H
#define _ATTENTIONAL_FOCUS_CB_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include "PatternMatchCallback.h"
#include "DefaultPatternMatchCB.h"


#include "DefaultPatternMatchCB.h"

namespace opencog {


class AttentionalFocusCB: public virtual DefaultPatternMatchCB {



private:
	AtomSpace * _atom_space;
public:
	AttentionalFocusCB(AtomSpace * as) :
			DefaultPatternMatchCB(as), _atom_space(as) {
	}
	bool node_match(Handle& node1, Handle& node2);
	bool link_match(LinkPtr& lpat, LinkPtr& lsoln);
	IncomingSet get_incoming_set(Handle h);


};
} //namespace opencog
#endif /* _ATTENTIONALFOCUSCB_H */
