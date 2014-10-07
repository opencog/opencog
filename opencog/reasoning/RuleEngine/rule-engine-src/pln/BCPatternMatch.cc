/*
 * BCPatternMatch.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  October 2014
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
#include "BCPatternMatch.h"
#include <opencog/guile/SchemeSmob.h>
BCPatternMatch::BCPatternMatch(AtomSpace * as) :
		Implicator(as), DefaultPatternMatchCB(as), AttentionalFocusCB(as), PLNImplicator(
				as), as_(as) {

}

BCPatternMatch::~BCPatternMatch() {

}

bool BCPatternMatch::node_match(Handle& node1, Handle& node2) {
	if (AttentionalFocusCB::node_match(node1, node2))
		return false;
	else
		return true;

}
bool BCPatternMatch::link_match(LinkPtr& lpat, LinkPtr& lsoln) {
	if (AttentionalFocusCB::link_match(lpat, lsoln))
		return false;
	else
		return true;

}

bool BCPatternMatch::grounding(const std::map<Handle, Handle> &var_soln,
		const std::map<Handle, Handle> &pred_soln) {
	Handle h = inst.instantiate(implicand, var_soln); //xxx would this create non existing atoms.
	if (Handle::UNDEFINED != h) {
		result_list.push_back(h);
	}
	return false;
}

HandleSeq BCPatternMatch::get_result_list() {
	return result_list;
}

void BCPatternMatch::clear_result_list() {
	result_list.clear();
}
