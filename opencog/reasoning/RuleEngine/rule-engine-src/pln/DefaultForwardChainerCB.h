/*
 * DefaultForwardChainerCB.h
 *
 * Copyright (C) 2015 Misgana Bayetta
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

#ifndef DEFAULTFORWARDCHAINERCB_H_
#define DEFAULTFORWARDCHAINERCB_H_

#include "ForwardChainerCallBack.h"
#include "ForwardChainInputMatchCB.h"
#include "ForwardChainPatternMatchCB.h"
#include "PLNCommons.h"


class FCMemory;
class DefaultForwardChainerCB: public virtual ForwardChainerCallBack {
private:
	const float _ctv_fitnes = 0.9; // from the PLN book

	PLNCommons * pcommon_;
	AtomSpace * as_;
	ForwardChainInputMatchCB*  fcim_;
	ForwardChainPatternMatchCB*  fcpm_;

	map<Handle, string> choose_variable(Handle h);
	Handle target_to_pmimplicant(Handle htarget,
			map<Handle, string> hnode_vname_map);
	void add_to_target_list(Handle h);
	template<class Type> Type tournament_select(map<Type, float> tfitnes_map);
	/**
	 * calculates fitness values in target_list_atom_space using the formula F = s^x * c^(2-x)
	 * where s is strength,c is confidence and x is some fixed value
	 * @param h - a handle
	 * @return a fitness value
	 */
	float target_tv_fitness(Handle h);
public:
	DefaultForwardChainerCB(AtomSpace* as);
	virtual ~DefaultForwardChainerCB();

//callbacks
	virtual Rule* choose_rule(FCMemory& fcmem);
	virtual HandleSeq choose_input(FCMemory& fcmem);
	virtual Handle choose_next_target(FCMemory& fcmem);
	virtual HandleSeq apply_rule(FCMemory& fcmem);
};

#endif /* DEFAULTFORWARDCHAINERCB_H_ */
