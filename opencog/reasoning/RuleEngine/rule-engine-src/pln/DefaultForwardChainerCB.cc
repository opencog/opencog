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

#include <opencog/guile/SchemeEval.h>
#include <opencog/guile/SchemeSmob.h>

DefaultForwardChainerCB::DefaultForwardChainerCB(AtomSpace* as) :
		ForwardChainerCallBack(as) {
	as_ = as;
	pcommon_ = new PLNCommons(as_);
	fcim_ = new ForwardChainInputMatchCB(as);
	fcpm_ = new ForwardChainPatternMatchCB(as);
}

DefaultForwardChainerCB::~DefaultForwardChainerCB() {
	delete pcommon_;
	delete fcim_;
	delete fcpm_;
}

//-----------------callbacks-------------------------------------------------------------------------------//

//choose rule based on premises of rule matching the target
//uses temporay atomspace to limit the search space
Rule* DefaultForwardChainerCB::choose_rule(FCMemory& fcmem) {
	//create temporary atomspace and copy target
	AtomSpace rule_atomspace;
	SchemeEval sc(&rule_atomspace);
	Handle target = fcmem.get_cur_target();
	if(target == Handle::UNDEFINED or NodeCast(target))
		throw InvalidParamException(TRACE_INFO,"Needs a target atom of type LINK");
	sc.eval(SchemeSmob::to_string(target));

	//create bindlink with target as an implicant
	PLNCommons pc(&rule_atomspace);
	Handle target_cpy = pc.replace_nodes_with_varnode(target, NODE);
	Handle bind_link = pc.create_bindLink(target_cpy, false);

	//copy rules to the temporary atomspace
	vector<Rule*> rules = fcmem.get_rules();
	for (Rule* r : rules)
		sc.eval_h(SchemeSmob::to_string(r->get_handle()));

	//pattern match
	DefaultImplicator imp(&rule_atomspace);
	PatternMatch pm;
	try {
		pm.do_bindlink(bind_link, imp);
	} catch (InvalidParamException& e) {
		cout << "VALIDATION FAILED:" << endl << e.what() << endl;
	}

	//get matched bindLinks
	auto matching_rules = imp.result_list;
	if(matching_rules.empty()) return nullptr;
	HandleSeq bindlinks;
	for (Handle h : matching_rules) {
		HandleSeq parents;
		pc.get_top_level_parent(h, parents);
		auto it = std::find_if(parents.begin(), parents.end(),
				[this](Handle h) {return (as_->getType(h) == BIND_LINK);});
		if(it == parents.end())
			bindlinks.push_back(*it);
	}
	//transfer back to main atomspace and  get the matching bindLinks
	SchemeEval scm(as_);
	HandleSeq copied_back;
	for(Handle h:bindlinks)
		copied_back.push_back(scm.eval_h(SchemeSmob::to_string(h)));

	//find the rules containing the bindlinks in main_as_bindlinks and select one
	map<Rule*,float> rule_weight_map;
	for(Rule* r:rules)
		if(find(copied_back.begin(),copied_back.end(),r->get_handle()) != copied_back.end())
			rule_weight_map[r] = r->get_priority();
	return tournament_select(rule_weight_map);
}

HandleSeq DefaultForwardChainerCB::choose_input(FCMemory& fcmem) {
	Handle htarget = fcmem.get_cur_target();
	HandleSeq inputs;
	if (NodeCast(htarget)) {
		inputs = _as->getIncoming(htarget);
	}

	if (LinkCast(htarget)) {
		map<Handle, string> hnode_vname_map = choose_variable(htarget);
		Handle implicant = target_to_pmimplicant(htarget, hnode_vname_map);
		Handle bindLink = _commons->create_bindLink(implicant);
		//find inputs by pattern matching with custom callback fcim_
		PatternMatch pm;
		try {
			pm.do_bindlink(bindLink, *fcim_);
		} catch (InvalidParamException& e) {
			cout << "VALIDATION FAILED:" << endl << e.what() << endl;
		}
		inputs = fcim_->get_input_matches();
	}
	return inputs;
}

Handle DefaultForwardChainerCB::choose_next_target(FCMemory& fcmem) {
	HandleSeq tlist = fcmem.get_target_list();
	map<Handle, float> tournament_elem;
	for (Handle t : tlist) {
		float fitness = target_tv_fitness(t);
		tournament_elem[t] = fitness;
	}
	return tournament_select(tournament_elem);
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

//-------------------private helper methods------------------------------------------------------------------------------//

map<Handle, string> DefaultForwardChainerCB::choose_variable(Handle htarget) {
	vector<Handle> candidates = _commons->get_nodes(htarget, vector<Type>());
	map<Handle, HandleSeq> node_iset_map;
	//enforce not choosing two or more variables in the same Link
	for (auto it = candidates.begin(); it != candidates.end(); ++it) {
		HandleSeq hs = _as->getIncoming(*it);
		if (distance(candidates.begin(), it) == 0) {
			node_iset_map[*it] = hs;
		} else {
			bool has_same_link = false;
			for (auto i = node_iset_map.begin(); i != node_iset_map.end();
					++i) {
				HandleSeq tmp;
				sort(hs.begin(), hs.end());
				sort(i->second.begin(), i->second.end());
				set_intersection(hs.begin(), hs.end(), i->second.begin(),
						i->second.end(), back_inserter((tmp)));
				if (tmp.size() > 0) {
					has_same_link = true;
					break;
				}
			}
			if (!has_same_link)
				node_iset_map[*it] = hs;
		}
	}
	map<Handle, string> hnode_vname_map;
	for (auto it = node_iset_map.begin(); it != node_iset_map.end(); ++it) {
		Handle h = it->first;
		hnode_vname_map[h] = ("$var-" + NodeCast((h))->getName());
	}
	return hnode_vname_map;
}

Handle DefaultForwardChainerCB::target_to_pmimplicant(Handle htarget,
		map<Handle, string> hnode_vname_map) {
	Type link_type;
	HandleSeq hsvariablized;

	if (LinkCast(htarget)) {
		LinkPtr p_htarget = LinkCast(htarget);
		link_type = p_htarget->getType();
		HandleSeq hsoutgoing = _as->getOutgoing(htarget);
		for (auto i = hsoutgoing.begin(); i != hsoutgoing.end(); ++i) {
			Handle htmp = target_to_pmimplicant(*i, hnode_vname_map);
			hsvariablized.push_back(htmp);
		}
		return _as->addLink(link_type, hsvariablized, TruthValue::TRUE_TV());
	} else {
		if (NodeCast(htarget)) {
			auto it_var = hnode_vname_map.find(htarget); //TODO replace by find-if for linear complexity
			NodePtr p_htarget = NodeCast(htarget);
			if (it_var != hnode_vname_map.end())
				return _as->addNode(VARIABLE_NODE, it_var->second,
						TruthValue::TRUE_TV());
			else
				return htarget;
		}
	}
	return Handle::UNDEFINED; //unreachable?
}

template<class Type> Type DefaultForwardChainerCB::tournament_select(
		map<Type, float> tfitnes_map) {
	if (tfitnes_map.size() == 1) {
		return tfitnes_map.begin()->first;
	}

	map<Type, float> winners;
	int size = tfitnes_map.size() / 2; //TODO change the way tournament size is calculated
	for (auto i = 0; i < size; i++) {
		int index = (random() % tfitnes_map.size());
		auto it = tfitnes_map.begin();
		advance(it, index);
		winners[it->first] = it->second;
	}
	auto it = winners.begin();
	Type hbest = it->first;
	float max = it->second;
	for (; it != winners.end(); ++it) {
		if (it->second > max) {
			hbest = it->first;
			max = it->second;
		}
	}
	return hbest;
}

float DefaultForwardChainerCB::target_tv_fitness(Handle h) {
	TruthValuePtr ptv = _as->getTV(h);
	confidence_t c = ptv->getConfidence();
	strength_t s = ptv->getMean();

	return (pow((1 - s), _ctv_fitnes) * (pow(c, (2 - _ctv_fitnes))));
}

