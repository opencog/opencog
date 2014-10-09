/*
 * ForwardChainPatternMatchCB.h
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

#ifndef FORWARDCHAINPATTERNMATCHCB_H_
#define FORWARDCHAINPATTERNMATCHCB_H_

#include "ForwardChainer.h"

#include <opencog/query/DefaultImplicator.h>

using namespace opencog;

class ForwardChainer;
class ForwardChainPatternMatchCB: public virtual PLNImplicator {
private:
	AtomSpace * as_;
	ForwardChainer * fc_;
public:
	ForwardChainPatternMatchCB(AtomSpace * as, ForwardChainer * fc);
	virtual ~ForwardChainPatternMatchCB();
	HandleSeq& get_results();

	//the follwing callbacks are used for guiding the PM to look only the target list
	//based on step 3 of http://wiki.opencog.org/w/New_PLN_Chainer_Design#Overall_Forward_Chaining_Process
	bool node_match(Handle& node1, Handle& node2);
	bool link_match(LinkPtr& lpat, LinkPtr& lsoln);

	/**
	 * A callback handler of the Pattern matcher used to store references to new conclusion the target list
	 */
	bool grounding(const std::map<Handle, Handle> &var_soln,
				const std::map<Handle, Handle> &pred_soln);
};

#endif /* FORWARDCHAINPATTERNMATCHCB_H_ */
