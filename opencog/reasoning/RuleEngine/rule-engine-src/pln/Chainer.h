/*
 * Chainer.h
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
#ifndef CHAINER_H_
#define CHAINER_H_

#include <opencog/query/PatternMatch.h>

using namespace opencog;

/**
 * Abstract class for forward and backward chaining control policy
 * all derived concrete are supposed to implement the PM callbacks and
 * pure virtuals here
 */
class Chainer {
private:
	const float ctv_fitnes; // = 0.9; // this might need to be configurable at runtime
protected:
	AtomSpace * main_atom_space; // knowledge base atomspace
	AtomSpace * target_list_atom_space;
	Handle hinitial_target;
	PatternMatch chaining_pm;
	const std::string conf_path; // = "rule-engine.conf";
public:
	Chainer(AtomSpace *);
	virtual ~Chainer();
	virtual void do_chain(Handle htarget) = 0;
	virtual void choose_rule(void) = 0;
	/**
	 * calculates fitness values in target_list_atom_space using the formula F = s^x * c^(2-x)
	 * where s is strength,c is confidence and x is some fixed value
	 * @param h - a handle
	 * @return a fitness value
	 */
	float target_tv_fitness(Handle h);
	void set_htarget(Handle& h);
};
#endif /* CHAINER_H_ */
