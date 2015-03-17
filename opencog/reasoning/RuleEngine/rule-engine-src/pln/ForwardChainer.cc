/*
 * ForwardChainer.cc
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

#include "ForwardChainer.h"
#include "ForwardChainerCallBack.h"
#include "PLNCommons.h"

#include <opencog/query/PatternMatch.h>
#include <opencog/reasoning/RuleEngine/rule-engine-src/Rule.h>

ForwardChainer::ForwardChainer(AtomSpace * as, string conf_path /*=""*/) :
		as_(as) {
	fcmem_ = new FCMemory(as_);
	if (conf_path != "")
		_conf_path = conf_path;
	init();
}

ForwardChainer::~ForwardChainer() {
	delete cpolicy_loader_;
	delete fcmem_;
}

void ForwardChainer::init() {
	cpolicy_loader_ = new JsonicControlPolicyParamLoader(as_, _conf_path);
	cpolicy_loader_->load_config();
	fcmem_->search_in_af_ = cpolicy_loader_->get_attention_alloc();
	fcmem_->rules_ = cpolicy_loader_->get_rules();
	fcmem_->cur_rule_ = nullptr;
}

void ForwardChainer::do_chain(ForwardChainerCallBack& fcb,
		Handle htarget/*=Handle::UNDEFINED*/) {
	int iteration = 0;
	auto max_iter = cpolicy_loader_->get_max_iter();
	init_target(htarget);
	while (iteration < max_iter /*OR other termination criteria*/) {
		//add more premise to hcurrent_target by pattern matching
		HandleSeq input = fcb.choose_input(*fcmem_);
		fcmem_->update_target_list(input);
		//choose the best rule to apply
		vector<Rule*> rules = fcb.choose_rule(*fcmem_);
		map<Rule*, float> rule_weight;
		for (Rule* r : rules)
			rule_weight[r] = r->get_cost();
		PLNCommons pc(as_);
		auto r = pc.tournament_select(rule_weight);
		//if no rules matches the pattern of the target,choose another target if there is, else end forward chaining.
		if (not r)
			return;
		fcmem_->cur_rule_ = r;
		//apply rule
		HandleSeq product = fcb.apply_rule(*fcmem_);
		fcmem_->add_rules_product(iteration, product);
		fcmem_->update_target_list(product);
		//next target
		fcmem_->set_target(fcb.choose_next_target(*fcmem_));
		iteration++;
	}
}

void ForwardChainer::init_target(Handle htarget) {
	if (htarget == Handle::UNDEFINED)
		fcmem_->set_target(choose_random_target(as_)); //start FC on a random target
	else
		fcmem_->set_target(htarget);
}

Handle ForwardChainer::choose_random_target(AtomSpace * as) {
	//choose a random atoms to start forward chaining with
	HandleSeq hs;
	if (cpolicy_loader_->get_attention_alloc())
		as->getHandleSetInAttentionalFocus(back_inserter(hs));
	else
		as->getHandlesByType(back_inserter(hs), ATOM, true);
	Handle rand_target;
	for (;;) {
		Handle h = hs[rand() % hs.size()];
		Type t = as->getType(h);
		if (t != VARIABLE_NODE and t != BIND_LINK and t != IMPLICATION_LINK) {
			rand_target = h;
			break;
		}
	}
	return rand_target;
}

HandleSeq ForwardChainer::get_chaining_result() {
	return fcmem_->get_result();
}
