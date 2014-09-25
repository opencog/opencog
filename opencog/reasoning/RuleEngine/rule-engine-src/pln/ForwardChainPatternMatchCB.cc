/*
 * ForwardChainPatternMatchCB.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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

#include "ForwardChainPatternMatchCB.h"

ForwardChainPatternMatchCB::ForwardChainPatternMatchCB(AtomSpace * as,
		ForwardChainer * fc) :
		Implicator(as), DefaultPatternMatchCB(as), AttentionalFocusCB(as), PLNImplicator(
				as), as_(as), fc_(fc) {

}

ForwardChainPatternMatchCB::~ForwardChainPatternMatchCB() {
}

HandleSeq& ForwardChainPatternMatchCB::get_results() {
	return result_list;
}
bool ForwardChainPatternMatchCB::node_match(Handle& node1, Handle& node2) {
	if (not AttentionalFocusCB::node_match(node1, node2) or not fc_->search_in_af) {
		//force inference to be made only in the target list
		bool result = not fc_->is_in_target_list(node1);
		return result;

	} else {
		return true;
	}
}
bool ForwardChainPatternMatchCB::link_match(LinkPtr& lpat, LinkPtr& lsoln) {
	if (not AttentionalFocusCB::link_match(lpat, lsoln) or not fc_->search_in_af) {
		//force inference to be made only in the target list
		bool result = not fc_->is_in_target_list(Handle(lsoln));
		return result;
	} else {
		return true;
	}
}
bool ForwardChainPatternMatchCB::grounding(
		const std::map<Handle, Handle> &var_soln,
		const std::map<Handle, Handle> &pred_soln) {
	Handle h = inst.instantiate(implicand, var_soln);
	if (Handle::UNDEFINED != h) {
		result_list.push_back(h);
		fc_->add_to_target_list(h); //add to potential target list

		//add to chaining result
		HandleSeq hs = fc_->chaining_results;
		auto it = find_if(hs.begin(), hs.end(),[h](Handle hi) {return h.value() == hi.value();});
		if (it == hs.end())
			fc_->chaining_results.push_back(h);

	}
	return false;
}

