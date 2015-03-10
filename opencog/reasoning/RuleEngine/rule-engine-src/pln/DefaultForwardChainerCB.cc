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

//-----------------callbacks-------------------------------------------------------------------------------//

//choose rule based on premises of rule matching the target
//uses temporary atomspace to limit the search space and avoid
vector<Rule*> DefaultForwardChainerCB::choose_rule(FCMemory& fcmem) {
	//create temporary atomspace and copy target
	AtomSpace rule_atomspace;
	SchemeEval sc(&rule_atomspace);
	Handle target = fcmem.get_cur_target();
	if (target == Handle::UNDEFINED or NodeCast(target))
		throw InvalidParamException(TRACE_INFO,
				"Needs a target atom of type LINK");
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
	SchemeEval scm(as_);
	HandleSeq copied_back;
	for (Handle h : bindlinks)
		copied_back.push_back(scm.eval_h(SchemeSmob::to_string(h)));
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

//-------------------private helper methods------------------------------------------------------------------------------//

map<Handle, string> DefaultForwardChainerCB::choose_variable(Handle htarget) {
	PLNCommons pc(as_);
	vector<Handle> candidates = pc.get_nodes(htarget, vector<Type>());
	map<Handle, HandleSeq> node_iset_map;
	//enforce not choosing two or more variables in the same Link
	for (auto it = candidates.begin(); it != candidates.end(); ++it) {
		HandleSeq hs = as_->getIncoming(*it);
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
		map<Handle, string> hnode_vname_map)
{
	if (LinkCast(htarget)) {
	   HandleSeq hsvariablized;
		LinkPtr p_htarget = LinkCast(htarget);
		Type link_type = p_htarget->getType();
		HandleSeq hsoutgoing = as_->getOutgoing(htarget);
		for (auto i = hsoutgoing.begin(); i != hsoutgoing.end(); ++i) {
			Handle htmp = target_to_pmimplicant(*i, hnode_vname_map);
			hsvariablized.push_back(htmp);
		}
		Handle h(as_->addLink(link_type, hsvariablized));
		h->setTruthValue(TruthValue::TRUE_TV());
		return h;
	} else {
		if (NodeCast(htarget)) {
			auto it_var = hnode_vname_map.find(htarget); //TODO replace by find-if for linear complexity
			NodePtr p_htarget = NodeCast(htarget);
			if (it_var != hnode_vname_map.end()) {
				Handle h = as_->addNode(VARIABLE_NODE, it_var->second);
				h->setTruthValue(TruthValue::TRUE_TV());
            return h;
         }
			else
				return htarget;
		}
	}
	return Handle::UNDEFINED; //unreachable?
}
