/*
 * ForwardChainer.h
 *
 * Copyright (C) 2014,2015 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>
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

#ifndef FORWARDCHAINERX_H_
#define FORWARDCHAINERX_H_

#include "FCMemory.h"
#include <opencog/reasoning/RuleEngine/rule-engine-src/ControlPolicyParamLoader.h>

using namespace opencog;

class ForwardChainerCallBack;
class ForwardChainer {
private:
	ControlPolicyParamLoader* cpolicy_loader_;
	string _conf_path ="reasoning/RuleEngine/rules/pln/default_cpolicy.json";
	AtomSpace * as_;
	FCMemory* fcmem_;
	/**
	 * initialize config methods
	 */
	void init();
	//TODO this is duplicate method
	/**
	 * choose a random target to start forward chaining with. This is useful when there is no target
	 * specified ahead to the forward chaining process.
	 * @param as - the atomspace instance from which target is selected
	 */
	Handle choose_random_target(AtomSpace *);
	void add_to_target_list(Handle h);
	void init_target(Handle target);
public:
	ForwardChainer(AtomSpace * as, string conf_path = "");
	virtual ~ForwardChainer();
	void do_chain(ForwardChainerCallBack& fcb, Handle htarget =
			Handle::UNDEFINED);
     HandleSeq get_chaining_result(void);
};

#endif /* FORWARDCHAINERX_H_ */
