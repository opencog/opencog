#ifndef _ATTENTIONAL_FOCUS_CB_H
#define _ATTENTIONAL_FOCUS_CB_H

#include "PatternMatchCallback.h"
#include "DefaultPatternMatchCB.h"

namespace opencog {

class AttentionalFocusCB: public virtual DefaultPatternMatchCB {

private:
	AtomSpace * _atom_space;
	static bool compare_sti(LinkPtr lptr1,LinkPtr lptr2){
		return lptr1->getAttentionValue()->getSTI() > lptr2->getAttentionValue()->getSTI();
	}
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
