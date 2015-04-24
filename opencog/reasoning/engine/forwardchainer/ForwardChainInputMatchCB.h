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

#include <opencog/query/DefaultImplicator.h>

namespace opencog {

class ForwardChainInputMatchCB: public virtual PLNImplicator {
private:
	AtomSpace* _as;
	HandleSeq _input_match;
public:
	/**
	 * @param main_as the main atomspace where initial source is fetched from
	 * @param source_list_as the chaining specific atomspace where sources is copied from the main atomspace
	 *  where PLN rules are applied on source lists for new knowledge discovery.
	 *
	 *  one can set the above two pointers to  the same atomspace object (wich now is the default way)
	 *  if there is no intention of chaining in a separate atomspace.
	 */
	ForwardChainInputMatchCB(AtomSpace * as);
	virtual ~ForwardChainInputMatchCB();

	/**
	 * callback handler passed to the PatternMatcher. Called when fully grounded soln is found.
	 */
	bool grounding(const std::map<Handle, Handle> &var_soln,
			const std::map<Handle, Handle> &pred_soln);

	/**
	 *set the atomspace of the Instantiator member of Implicator object
	 */
	void set_instantiators_atom_space(AtomSpace * as);
	/**
	 * @return the matched inputs
	 */
	HandleSeq get_input_matches(void);
};

} // ~namespace opencog

#endif /* FORWARDCHAININPUTMATCHCB_H_ */
