/*
 * DefaultForwardChainerCB.cc
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

#include "DefaultForwardChainerCB.h"
#include "PLNCommons.h"

#include <opencog/query/PatternMatch.h>
#include <opencog/guile/SchemeEval.h>
#include <opencog/guile/SchemeSmob.h>

DefaultForwardChainerCB::DefaultForwardChainerCB(AtomSpace* as) :
		ForwardChainerCallBack(as) {
	as_ = as;
	fcim_ = new ForwardChainInputMatchCB(as);
	fcpm_ = new ForwardChainPatternMatchCB(as);
}

DefaultForwardChainerCB::~DefaultForwardChainerCB() {
	delete fcim_;
	delete fcpm_;
}

//choose rule based on premises of rule matching the target
//uses temporary atomspace to limit the search space and avoid
vector<Rule*> DefaultForwardChainerCB::choose_rule(FCMemory& fcmem)
{
	//create temporary atomspace and copy target
	AtomSpace rule_atomspace;
	SchemeEval *sc = get_evaluator(&rule_atomspace);
	Handle target = fcmem.get_cur_target();
	if (target == Handle::UNDEFINED or NodeCast(target))
		throw InvalidParamException(TRACE_INFO,
				"Needs a target atom of type LINK");
	sc->eval(SchemeSmob::to_string(target));
	//create bindlink with target as an implicant
	PLNCommons pc(&rule_atomspace);
	Handle target_cpy = pc.replace_nodes_with_varnode(target, NODE);
	Handle bind_link = pc.create_bindLink(target_cpy, false);
	//copy rules to the temporary atomspace
	vector<Rule*> rules = fcmem.get_rules();
	for (Rule* r : rules)
		sc->eval(SchemeSmob::to_string(r->get_handle()));
	//pattern match
	DefaultImplicator imp(&rule_atomspace);
	PatternMatch pm;
	try {
		pm.do_bindlink(bind_link, imp);
	} catch (InvalidParamException& e) {
		cout << "VALIDATION FAILED:" << endl << e.what() << endl;
	}
	//get matched bindLinks
	auto matchings = imp.result_list;
	if (matchings.empty())
		return vector<Rule*> { };
	HandleSeq bindlinks;
	for (Handle hm : matchings) {
		HandleSeq check_bindlink;
		HandleSeq parents;
		pc.get_top_level_parent(hm, parents);
		for (Handle hp : parents) {
			if (as_->getType(hp) == BIND_LINK
					and find(bindlinks.begin(), bindlinks.end(), hp)
							== bindlinks.end())
				check_bindlink.push_back(hp);
		}
		//make sure matches are actually part of the premise list rather than the output of the bindLink
		for (Handle hb : check_bindlink) {
			auto outgoing = [this](Handle h) {return as_->getOutgoing(h);};
			Handle hpremise = outgoing(outgoing(hb)[1])[0]; //extracting premise from (BindLink((ListLinK..)(ImpLink (premise) (..))))
			if (pc.exists_in(hpremise, hm))
				bindlinks.push_back(hb);
		}
	}
	//transfer back bindlinks to main atomspace and  get their handle
	SchemeEval *scm = get_evaluator(as_);
	HandleSeq copied_back;
	for (Handle h : bindlinks)
		copied_back.push_back(scm->eval_h(SchemeSmob::to_string(h)));
	//find the rules containing the bindLink in copied_back
	vector<Rule*> matched_rules;
	for (Rule* r : rules)
		if (find(copied_back.begin(), copied_back.end(), r->get_handle())
				!= copied_back.end())
			matched_rules.push_back(r);
	return matched_rules;
}

HandleSeq DefaultForwardChainerCB::choose_input(FCMemory& fcmem) {
	Handle htarget = fcmem.get_cur_target();
	//get everything associated with the target handle
	HandleSeq inputs = as_->getNeighbors(htarget,true,true,LINK,true);
	return inputs;
}

Handle DefaultForwardChainerCB::choose_next_target(FCMemory& fcmem) {
	HandleSeq tlist = fcmem.get_target_list();
	map<Handle, float> tournament_elem;
	PLNCommons pc(as_);
	for (Handle t : tlist) {
		float fitness = pc.target_tv_fitness(t);
		tournament_elem[t] = fitness;
	}
	return pc.tournament_select(tournament_elem);
}

//TODO applier should check on atoms (Inference.matched_atoms when Inference.Rule =Cur_Rule), for mutex rules
HandleSeq DefaultForwardChainerCB::apply_rule(FCMemory& fcmem) {
	Rule * cur_rule = fcmem.get_cur_rule();
	fcpm_->set_fcmem(&fcmem);
	PatternMatch pm;
	try {
		pm.do_bindlink(cur_rule->get_handle(), *fcpm_);
	} catch (InvalidParamException& e) {
		cout << "VALIDATION FAILED:" << endl << e.what() << endl;
	}
	return fcpm_->get_products();
}
