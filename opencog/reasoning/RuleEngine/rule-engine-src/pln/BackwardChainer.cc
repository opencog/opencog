/*
 * BackwardChainer.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  October 2014
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
#include "BackwardChainer.h"

#include <opencog/guile/SchemeSmob.h>

BackwardChainer::BackwardChainer(AtomSpace * as) :
		Chainer(as), as_(as) {
	commons_ = new PLNCommons(as_);
	bcpm_ = new BCPatternMatch(as_);
}

BackwardChainer::~BackwardChainer() {
	delete commons_;
	delete bcpm_;
}

void BackwardChainer::choose_rule() {

}

Handle BackwardChainer::select_rule(HandleSeq& hseq_rule) {
	//apply selection criteria to select one amongst the matching rules

	//xxx return random for the purpose of integration testing before going
	//for a complex implementation of this function
	return hseq_rule[random() % hseq_rule.size()];
}

HandleSeq BackwardChainer::query_rule_base(Handle htarget) {
	Handle hbind_link = commons_->create_bindLink(htarget);
#if DEBUG
	cout << "QUERY-RB:" << endl << SchemeSmob::to_string(hbind_link) << endl;
#endif
	chaining_pm.do_bindlink(hbind_link, *bcpm_);
	commons_->clean_up_bind_link(hbind_link);

	auto result = bcpm_->get_result_list();
	bcpm_->clear_result_list(); //makes sure on each query only new results are returned
	return result;
}

HandleSeq BackwardChainer::query_knowledge_base(Handle htarget) {
	Handle hbind_link = commons_->create_bindLink(htarget);
#if DEBUG
	cout << "QUERY-KB:" << endl << SchemeSmob::to_string(hbind_link) << endl;
#endif
	chaining_pm.do_bindlink(hbind_link, *bcpm_);
	commons_->clean_up_bind_link(hbind_link);

	auto result = bcpm_->get_result_list();
	bcpm_->clear_result_list(); //for making sure on each query new results are returned
	return result;
}

map<Handle, HandleSeq> BackwardChainer::join_premise_vgrounding_maps(
		const Handle& logical_link,
		const map<Handle, map<Handle, HandleSeq> >& premise_var_grounding_map) {
	map<Handle, HandleSeq> result;
	for (auto pvg_it = premise_var_grounding_map.begin();
			pvg_it != premise_var_grounding_map.end(); ++pvg_it) {
		map<Handle, HandleSeq> var_groundings = pvg_it->second;
		if (pvg_it == premise_var_grounding_map.begin())
			result = var_groundings;
		else {
			for (auto vg_it = var_groundings.begin();
					vg_it != var_groundings.end(); vg_it++) {
				if (result.count(vg_it->first) == 1) {
					HandleSeq vg1 = vg_it->second;
					HandleSeq vg2 = result[vg_it->first];
					HandleSeq common_values;
					sort(vg1.begin(), vg1.end());
					sort(vg2.begin(), vg2.end());
					if (as_->getType(logical_link) == AND_LINK) {
						set_intersection(vg1.begin(), vg1.end(), vg2.begin(),
								vg2.end(), back_inserter(common_values));

					}

					if (as_->getType(logical_link) == OR_LINK)
						set_union(vg1.begin(), vg1.end(), vg2.begin(),
								vg2.end(), back_inserter(common_values));

					result[vg_it->first] = common_values;

				} else
					result[vg_it->first] = vg_it->second;
			}
		}
	}
	return result;
}

Handle BackwardChainer::get_unvisited_logical_link(HandleSeq& llinks,
		HandleSeq& visited) {
	HandleSeq result;
	for (Handle h : llinks) {
		Type t = as_->getType(h);
		auto it = find(logical_link_types_.begin(), logical_link_types_.end(),
				t);
		if (it != logical_link_types_.end()) {
			auto i = find(visited.begin(), visited.end(), h);
			if (i == visited.end())
				result.push_back(h);
		}
	}
	if (not result.empty())
		return result[0];

	return Handle::UNDEFINED;
}

HandleSeq BackwardChainer::get_grounded(HandleSeq result) {
	HandleSeq grounded;
	for (Handle h : result) {
		HandleSeq var_containing = commons_->get_nodes(h, vector<Type> {
				VARIABLE_NODE });
		if (var_containing.empty())
			grounded.push_back(h);
	}
	return grounded;
}

HandleSeq BackwardChainer::filter_rules(HandleSeq result) {
	HandleSeq rules;
	for (Handle h : result) {
		HandleSeq links = as_->getIncoming(h);
		for (Handle l : links) {
			Type t = as_->getType(l);
			if (t == LIST_LINK) {
				HandleSeq incoming = as_->getIncoming(l);
				if (not incoming.empty()) {
					if (as_->getType(incoming[0]) == EXECUTION_LINK) {
						HandleSeq hs = as_->getIncoming(incoming[0]);
						if (not hs.empty()) {
							if (as_->getType(hs[0]) == IMPLICATION_LINK)
								rules.push_back(hs[0]);
						}
					}
				}
			}
			if (t == IMPLICATION_LINK)
				if (as_->getOutgoing(l)[1] == h)
					rules.push_back(l);
		}
	}
	return rules;
}

HandleSeq BackwardChainer::filter_grounded_experssions(HandleSeq handles) {
	HandleSeq grounded;
	for (Handle h : handles)
		if (commons_->get_nodes(h, vector<Type> { VARIABLE_NODE }).empty())
			grounded.push_back(h);
	return grounded;
}

Handle BackwardChainer::get_root_logical_link(Handle himplication_link)
		throw (opencog::InvalidParamException) {
	if (as_->getType(himplication_link) != IMPLICATION_LINK)
		throw InvalidParamException(TRACE_INFO,
				"input should be implication link");
	HandleSeq outg = as_->getOutgoing(himplication_link);
	Handle implicant = outg[0];
	if (find(logical_link_types_.begin(), logical_link_types_.end(),
			as_->getType(implicant)) != logical_link_types_.end()) {
		return implicant;
	} else
		return Handle::UNDEFINED;
}

map<Handle, HandleSeq> BackwardChainer::get_logical_link_premises_map(
		Handle& himplication_link) throw (opencog::InvalidParamException) {
	if (as_->getType(himplication_link) != IMPLICATION_LINK)
		throw InvalidParamException(TRACE_INFO,
				"input should be implication link");
	Handle root_llink = as_->getOutgoing(himplication_link)[0];
	map<Handle, HandleSeq> logical_link_premise_map;
	HandleSeq logical_links;

	Type t = as_->getType(root_llink);
	auto it = find(logical_link_types_.begin(), logical_link_types_.end(), t);
	if (it != logical_link_types_.end()) {
		logical_links.push_back(root_llink);
		do {
			Handle llink = logical_links[logical_links.size() - 1];
			logical_links.pop_back();
			HandleSeq premises = as_->getOutgoing(llink);
			for (Handle h : premises) {
				logical_link_premise_map[llink].push_back(h);
				//check if h is a logical link type and push it for building next iter map
				t = as_->getType(h);
				it = find(logical_link_types_.begin(),
						logical_link_types_.end(), t);
				if (it != logical_link_types_.end())
					logical_links.push_back(h);

			}
		} while (not logical_links.empty());
		return logical_link_premise_map;
	} else
		return map<Handle, HandleSeq> { { Handle::UNDEFINED, HandleSeq {
				root_llink } } };
}
HandleSeq BackwardChainer::chase_var_values(Handle& hvar,
		vector<map<Handle, HandleSeq>>& inference_list, HandleSeq& results) {
	for (auto it = inference_list.begin(); it != inference_list.end(); ++it) {
		map<Handle, HandleSeq> var_value = *it;
		if (var_value.count(hvar) != 0) {
			HandleSeq values = var_value[hvar];
			for (Handle h : values) {
				if (as_->getType(h) == VARIABLE_NODE) {
					HandleSeq new_values;
					HandleSeq hseq = chase_var_values(h, inference_list,
							new_values);
					results.insert(results.end(), hseq.begin(), hseq.end());
				} else {
					results.push_back(h);
				}
			}
		}
	}
	return results;
}
map<Handle, HandleSeq> BackwardChainer::ground_target_vars(Handle& hgoal,
		vector<map<Handle, HandleSeq>>& inference_list) {
	map<Handle, HandleSeq> vg_map;
	HandleSeq hgoal_vars = commons_->get_nodes(hgoal, vector<Type> {
			VARIABLE_NODE });

	for (map<Handle, HandleSeq> vgm : inference_list) {
		for (auto it = vgm.begin(); it != vgm.end(); ++it) {

			Handle hvar = it->first;
			auto i = find(hgoal_vars.begin(), hgoal_vars.end(), hvar);

			if (i != hgoal_vars.end()) {
				HandleSeq values;
				HandleSeq groundings = it->second;
				for (Handle h : groundings) {
					if (as_->getType(h) == VARIABLE_NODE) {
						HandleSeq val;
						val = chase_var_values(h, inference_list, val);
						values.insert(values.end(), val.begin(), val.end());
					} else {
						values.push_back(h);
					}
					//erase duplicates i.e union
					sort(values.begin(), values.end());
					values.erase(unique(values.begin(), values.end()),
							values.end());
				}
				if (vg_map.count(hvar) == 0)
					vg_map[hvar] = values;
				else {
					HandleSeq existing = vg_map[hvar];
					sort(existing.begin(), existing.end());
					sort(values.begin(), values.end());
					HandleSeq hs_combined;
					set_union(existing.begin(), existing.end(), values.begin(),
							values.end(), back_inserter(hs_combined));
					vg_map[hvar] = hs_combined;
				}
			}
		}
	}
	return vg_map;
}
map<Handle, HandleSeq> BackwardChainer::unify_to_empty_set(Handle&htarget) {
	HandleSeq vars = commons_->get_nodes(htarget,
			vector<Type> { VARIABLE_NODE });
	map<Handle, HandleSeq> result;
	for (Handle h : vars)
		result[h] = HandleSeq { Handle::UNDEFINED };
	return result;
}

map<Handle, HandleSeq> BackwardChainer::unify(Handle& htarget, Handle& match,
		map<Handle, HandleSeq>& result) {
	HandleSeq vars = commons_->get_nodes(htarget,
			vector<Type> { VARIABLE_NODE });
	if (LinkCast(htarget)) {
		HandleSeq target_outg = as_->getOutgoing(htarget);
		HandleSeq match_outg = as_->getOutgoing(match);
		assert(target_outg.size()==match_outg.size()); //TODO throw exception instead
		for (vector<Handle>::size_type i = 0; i < target_outg.size(); i++) {
			if (as_->getType(target_outg[i]) == VARIABLE_NODE)
				result[target_outg[i]].push_back(match_outg[i]);
			else
				unify(target_outg[i], match_outg[i], result);
		}
	} else if (as_->getType(htarget) == VARIABLE_NODE)
		result[htarget].push_back(match);

	return result;
}
map<Handle, HandleSeq> BackwardChainer::do_bc(Handle& hgoal) {
#ifdef DEBUG
	cout << endl << "DO BC CALLED WITH:" << endl << SchemeSmob::to_string(hgoal)
	<< endl;
	cout << "QUERY KNOWLEDGE BASE" << endl;
#endif

	HandleSeq kb_match = filter_grounded_experssions(
			query_knowledge_base(hgoal)); //TODO filter grounded grounded representations so the next condition would never be fooled
	if (kb_match.empty()) {
#ifdef DEBUG
		cout << "QUERYING RULE BASE" << endl;
#endif
		HandleSeq rules = filter_rules(query_rule_base(hgoal));
		if (rules.empty()) {
#ifdef DEBUG
			cout << "NOTHING FOUND" << endl;
#endif
			return unify_to_empty_set(hgoal);
		} else {
			Handle rule = select_rule(rules); //TODO use all rules for found here.
			Handle stadardized_rule = commons_->create_with_unique_var(rule);
			bc_generated_rules.push_back(stadardized_rule); //for later removal
#ifdef DEBUG
					cout << "RULE FOUND" << SchemeSmob::to_string(stadardized_rule)
					<< endl;
#endif
			map<Handle, HandleSeq> out;
			Handle implicand = as_->getOutgoing(stadardized_rule)[1];
			inference_list_.push_back(unify(hgoal, implicand, out));
#ifdef DEBUG
			cout << endl << "INFERENCE LIST UPDATE" << endl;
			print_inference_list();
#endif
			map<Handle, HandleSeq> solution = backward_chain(implicand,
					stadardized_rule);
			inference_list_.push_back(solution);
#ifdef DEBUG
			cout << endl << "FINAL INFERENCE LIST" << endl;
			print_inference_list();
			cout << endl << "BINDING GOAL: " << SchemeSmob::to_string(hgoal)
			<< endl;
#endif
			return ground_target_vars(hgoal, inference_list_);
		}
	} else {
		vector<map<Handle, HandleSeq>> kb_results;
		HandleSeq solns = get_grounded(kb_match); //find existing ones
		map<Handle, HandleSeq> out;
		for (Handle soln : solns) {
#ifdef DEBUG
			cout << "FOUND " << SchemeSmob::to_string(soln) << endl;
#endif
			kb_results.push_back(unify(hgoal, soln, out));
		}
		return ground_target_vars(hgoal, kb_results);
	}

	return unify_to_empty_set(hgoal);
}
map<Handle, HandleSeq> BackwardChainer::backward_chain(Handle& htarget,
		Handle& rule) {
	vector<map<Handle, HandleSeq>> results;
	Handle root_logical_link = get_root_logical_link(rule);

	if (root_logical_link == Handle::UNDEFINED) {
//eg. ImplicationLink (Inheritance $x "human") (InheritanceLink "$x" "bipedal")) has no logical links
		Handle implicant = as_->getOutgoing(rule)[0];
		return do_bc(implicant);
	}
	//build a tree of the the logical links and premises( premises could by themselves be a logical link) as a map
	map<Handle, HandleSeq> logical_link_premise_map =
			get_logical_link_premises_map(rule);
	map<Handle, map<Handle, HandleSeq>> premise_var_ground_map;

	HandleSeq visited_logical_link;
	HandleSeq evaluated_premises;
	visited_logical_link.push_back(root_logical_link); //start from the root

	//go down deep until a logical link maps only to set of premises in the logical_link_premise_map object
	//e.g start from (and x y) instead of (and (and x y) x) and start backward chaining  from there. i.e bottom up.
	while (not visited_logical_link.empty()) {
		Handle logical_link = visited_logical_link.back();
		visited_logical_link.pop_back();

		HandleSeq premises = logical_link_premise_map[logical_link];
		Handle llink = get_unvisited_logical_link(premises, evaluated_premises);
		if (llink != Handle::UNDEFINED) {
			visited_logical_link.push_back(llink);
			continue;
		}

		for (Handle premise : premises) {
			auto i = find(evaluated_premises.begin(), evaluated_premises.end(),
					premise);
			if (i == evaluated_premises.end()) {
				auto var_grounding = do_bc(premise);
				premise_var_ground_map[premise] = var_grounding;
				evaluated_premises.push_back(premise);
			}
		}
#ifdef DEBUG
		cout << endl << "PREMISE VAR GROUNDING MAP" << endl;
		print_premise_var_ground_mapping(premise_var_ground_map);
#endif
		auto v = join_premise_vgrounding_maps(logical_link,
				premise_var_ground_map);
#ifdef DEBUG
		cout << endl << "AFTER JOINING WITH "
		<< (as_->getType(logical_link) == AND_LINK ? "AND" : "OR")
		<< endl;
		print_var_value(v);
#endif
		results.push_back(v);		//add to results
#ifdef DEBUG
				cout << endl << "INFERENCE LIST UPDATE" << endl;
				print_inference_list();
#endif
		evaluated_premises.push_back(logical_link);
	}

	return ground_target_vars(htarget, results);
}

map<Handle, HandleSeq>& BackwardChainer::get_chaining_result() {
	return chaining_result_;
}

void BackwardChainer::remove_generated_rules() {
	for (vector<Handle>::size_type i = 0; i < bc_generated_rules.size(); i++) {
		Handle h = bc_generated_rules.back();
		commons_->clean_up_implication_link(h);
		bc_generated_rules.pop_back();
	}
}

void BackwardChainer::do_chain(Handle init_target) {
	chaining_result_.clear();
	chaining_result_ = do_bc(init_target);
	remove_generated_rules(); //clean variables
}

#ifdef DEBUG
void BackwardChainer::print_inference_list() {
	for (auto it = inference_list_.begin(); it != inference_list_.end(); ++it) {
		map<Handle, HandleSeq> var_ground = *it;
		for (auto j = var_ground.begin(); j != var_ground.end(); ++j) {
			cout << "[VAR:" << SchemeSmob::to_string(j->first) << endl;
			HandleSeq hs = j->second;
			for (Handle h : hs)
			cout << "\tVAL:" << SchemeSmob::to_string(h) << endl;
		}
		cout << "]" << endl;
	}
}
void BackwardChainer::print_premise_var_ground_mapping(
		const map<Handle, map<Handle, HandleSeq>>& premise_var_ground_map) {
	for (auto it = premise_var_ground_map.begin();
			it != premise_var_ground_map.end(); ++it) {
		cout << "PREMISE:" << endl << SchemeSmob::to_string(it->first) << endl;
		map<Handle, HandleSeq> var_ground = it->second;
		print_var_value(var_ground);
	}
}
void BackwardChainer::print_var_value(
		const map<Handle, HandleSeq>& var_ground) {
	for (auto j = var_ground.begin(); j != var_ground.end(); ++j) {
		cout << "[VAR:" << SchemeSmob::to_string(j->first) << endl;
		HandleSeq hs = j->second;
		for (Handle h : hs)
		cout << "\tVAL:" << SchemeSmob::to_string(h) << endl;
	}
	cout << "]" << endl;
}
#endif

