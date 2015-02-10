/*
 * FCMemory.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>   2015
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
#include "FCMemory.h"

FCMemory::FCMemory(AtomSpace* as) {
	as_ = as;
}

FCMemory::~FCMemory() {
}

void FCMemory::update_target_list(HandleSeq input) {
	for (Handle i : input) {
		if (find(target_list_.begin(), target_list_.end(), i)
				== target_list_.end())
			target_list_.push_back(i);
	}
}

vector<Rule*> FCMemory::get_rules(void) {
	return rules_;
}
void FCMemory::set_target(Handle target) {
	cur_target_ = target;
	target_list_.push_back(cur_target_);
}
HandleSeq FCMemory::get_target_list(void) {
	return target_list_;
}
bool FCMemory::is_search_in_af(void) {
	return search_in_af_;
}
Rule* FCMemory::get_cur_rule(void) {
	return cur_rule_;
}
void FCMemory::set_cur_rule(Rule* r) {
	cur_rule_ = r;
}
void FCMemory::add_rules_product(int iteration, HandleSeq product) {
	for (Handle p : product) {
		Inference inf;
		inf.iter_step = iteration;
		inf.applied_rule = cur_rule_;
		inf.inf_product.push_back(p);
		inf_history_.push_back(inf);
	}
}
void FCMemory::add_inference(int iter_step, HandleSeq product,
		HandleSeq matched_nodes) {
	Inference inf;
	inf.applied_rule = cur_rule_;
	inf.iter_step = iter_step;
	for (Handle p : product)
		inf.inf_product.push_back(p);
	for (Handle mn : matched_nodes)
		inf.matched_nodes.push_back(mn);
	inf_history_.push_back(inf);
}
Handle FCMemory::get_cur_target(void) {
	return cur_target_;
}

bool FCMemory::isin_target_list(Handle h) {
	for (Handle hi : target_list_) {
		//TODO recursive search might be the right way
		if (hi.value() == h.value())
			return true;
	}
	return false;
}

HandleSeq FCMemory::get_result() {
	HandleSeq result;
	for (Inference i : inf_history_)
		result.insert(result.end(), i.inf_product.begin(), i.inf_product.end());
	return result;
}

vector<Inference>& FCMemory::get_inf_history() {
	return inf_history_;
}

vector<Rule*> FCMemory::get_applied_rules(void) {
	vector<Rule*> applied_rules;
	for (Inference i : inf_history_) {
		if (find(applied_rules.begin(), applied_rules.end(), i.applied_rule)
				== applied_rules.end())
			applied_rules.push_back(i.applied_rule);
	}
	return applied_rules;
}
