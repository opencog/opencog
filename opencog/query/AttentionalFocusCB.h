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
#ifndef _ATTENTIONAL_FOCUS_CB_H
#define _ATTENTIONAL_FOCUS_CB_H

#include "DefaultPatternMatchCB.h"

namespace opencog {

class AttentionalFocusCB: public virtual DefaultPatternMatchCB
{
private:
	static bool compare_sti(LinkPtr lptr1, LinkPtr lptr2)
	{
		return lptr1->getAttentionValue()->getSTI()
				> lptr2->getAttentionValue()->getSTI();
	}
public:
	AttentionalFocusCB(AtomSpace * as) :
			DefaultPatternMatchCB(as)
	{}
	bool node_match(Handle&, Handle&);
	bool link_match(LinkPtr&, LinkPtr&);
	IncomingSet get_incoming_set(Handle h);
	void perform_search(PatternMatchEngine *,
	                    std::set<Handle> &vars,
	                    std::vector<Handle> &clauses,
	                    std::vector<Handle> &negations);
};
} //namespace opencog
#endif /* _ATTENTIONALFOCUSCB_H */
