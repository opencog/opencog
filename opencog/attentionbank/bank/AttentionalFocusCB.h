/*
 * AttentionalFocusCB.h
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

#include <opencog/query/DefaultPatternMatchCB.h>

namespace opencog {

class AttentionalFocusCB: public virtual DefaultPatternMatchCB
{
public:
	AttentionalFocusCB(AtomSpace*);

	// Only match nodes if they are in the attentional focus
	bool node_match(const Handle&, const Handle&);

	// Only match links if they are in the attentional focus
	bool link_match(const PatternTermPtr&, const Handle&);

	// Only get incoming sets that are in the attentional focus
	IncomingSet get_incoming_set(const Handle&);
};

} //namespace opencog
#endif /* _ATTENTIONALFOCUSCB_H */
