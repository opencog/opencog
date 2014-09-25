/*
 * ForwardChainInputMatchCB.h
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

#ifndef FORWARDCHAININPUTMATCHCB_H_
#define FORWARDCHAININPUTMATCHCB_H_

#include "ForwardChainer.h"

#include <opencog/query/DefaultImplicator.h>

using namespace opencog;

class ForwardChainer;
class ForwardChainInputMatchCB: public virtual PLNImplicator {
private:
	AtomSpace * as_;
	ForwardChainer * fc_;
public:
	/**
	 * @param main_as the big main atomspace where initial target is fetched from
	 * @param target_list_as the mini chaining specific atomspace where targets is copied from the main atomspace
	 *  where PLN rules are applied on target lists for new knowledge discovery.
	 *
	 *  one can set the above to  the same atomspace object if there is no intention of chaining in a separate atomspace
	 */
	ForwardChainInputMatchCB(AtomSpace * main_as, AtomSpace * target_list_as,
			ForwardChainer * fc);
	virtual ~ForwardChainInputMatchCB();
	/**
	 *set the atomspace of the Instantiator member of Implicator object
	 */
	void set_instantiators_atom_space(AtomSpace * as);

	HandleSeq get_result_list(void);

	/**
	 * callback handler passed to the PatternMatcher. Called when fully grounded soln is found.
	 */
	bool grounding(const std::map<Handle, Handle> &var_soln,
			const std::map<Handle, Handle> &pred_soln);

};

#endif /* FORWARDCHAININPUTMATCHCB_H_ */
