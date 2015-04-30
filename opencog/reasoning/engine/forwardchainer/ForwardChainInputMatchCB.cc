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

using namespace opencog;

ForwardChainInputMatchCB::ForwardChainInputMatchCB(AtomSpace * as)
	: Implicator(as), DefaultPatternMatchCB(as),
	  AttentionalFocusCB(as), PLNImplicator(as), _as(as) {}

ForwardChainInputMatchCB::~ForwardChainInputMatchCB() {}

bool ForwardChainInputMatchCB::grounding(
		const std::map<Handle, Handle> &var_soln,
		const std::map<Handle, Handle> &pred_soln) {
	Handle h = inst.instantiate(implicand, var_soln);
	if (Handle::UNDEFINED != h)
		_input_match.push_back(h);
	return false;
}

void ForwardChainInputMatchCB::set_instantiators_atom_space(AtomSpace *as) {
	inst = Instantiator(as);
}

HandleSeq ForwardChainInputMatchCB::get_input_matches(void) {
	return _input_match;
}
