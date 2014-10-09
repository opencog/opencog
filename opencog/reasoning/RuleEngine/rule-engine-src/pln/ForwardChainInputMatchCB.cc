/*
 * ForwardChainInputMatchCB.cc
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

#include "ForwardChainInputMatchCB.h"

#include <opencog/guile/SchemeSmob.h>

ForwardChainInputMatchCB::ForwardChainInputMatchCB(AtomSpace * as,
		AtomSpace * target_list_as, ForwardChainer * fc) :
		Implicator(as), DefaultPatternMatchCB(as), AttentionalFocusCB(as), PLNImplicator(
				as), as_(as), fc_(fc) {
	set_instantiators_atom_space(target_list_as);
}

ForwardChainInputMatchCB::~ForwardChainInputMatchCB() {

}
void ForwardChainInputMatchCB::set_instantiators_atom_space(AtomSpace *as) {
	inst = Instantiator(as);
}

HandleSeq ForwardChainInputMatchCB::get_result_list(void) {
	return result_list;
}

bool ForwardChainInputMatchCB::grounding(
		const std::map<Handle, Handle> &var_soln,
		const std::map<Handle, Handle> &pred_soln) {
	Handle h = inst.instantiate(implicand, var_soln);
	if (Handle::UNDEFINED != h) {
		result_list.push_back(h);
		fc_->add_to_target_list(h); //add to potential target list
	}
	return false;
}
