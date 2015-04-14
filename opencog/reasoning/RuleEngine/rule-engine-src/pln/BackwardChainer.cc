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

#include <opencog/atomutils/AtomUtils.h>
#include <opencog/atoms/bind/BindLink.h>

using namespace opencog;

BackwardChainer::BackwardChainer(AtomSpace * as)
    : _as(as)
{
	_commons = new PLNCommons(_as);
	_bcpm = new BCPatternMatch(_as);
}

BackwardChainer::~BackwardChainer()
{
	delete _commons;
	delete _bcpm;
}

/**
 * The public entry point for backward chaining.
 *
 * @param init_target  the initial atom to start the chaining
 */
void BackwardChainer::do_chain(Handle init_target)
{
	_chaining_result.clear();
	_chaining_result = do_bc(init_target);

	//clean variables
	remove_generated_rules();
}

map<Handle, HandleSeq>& BackwardChainer::get_chaining_result()
{
	return _chaining_result;
}

void BackwardChainer::choose_rule()
{

}

/**
 * The main recursive backward chaining method.
 *
 * @param hgoal  the atom to do backward chaining on
 * @return       ???
 */
map<Handle, HandleSeq> BackwardChainer::do_bc(Handle& hgoal)
{
	// TODO filter grounded representations so the next condition would never be fooled
	HandleSeq kb_match = filter_grounded_experssions(query_knowledge_base(hgoal));

	if (kb_match.empty())
	{
		HandleSeq rules = filter_rules(query_rule_base(hgoal));

		if (rules.empty())
		{
			// nothing found
			return unify_to_empty_set(hgoal);
		}
		else
		{
			// TODO use all rules for found here.
			Handle rule = select_rule(rules);

			Handle stadardized_rule = _commons->replace_nodes_with_varnode(rule);

			//for later removal
			_bc_generated_rules.push_back(stadardized_rule);

			map<Handle, HandleSeq> out;
			Handle implicand = _as->getOutgoing(stadardized_rule)[1];
			_inference_list.push_back(unify(hgoal, implicand, out));

			map<Handle, HandleSeq> solution = apply_rule(implicand, stadardized_rule);
			_inference_list.push_back(solution);

			return ground_target_vars(hgoal, _inference_list);
		}
	}
	else
	{
		vector<map<Handle, HandleSeq>> kb_results;
		HandleSeq solns = get_grounded(kb_match); //find existing ones

		map<Handle, HandleSeq> out;
		for (Handle soln : solns)
		{
			kb_results.push_back(unify(hgoal, soln, out));
		}

		return ground_target_vars(hgoal, kb_results);
	}

	return unify_to_empty_set(hgoal);
}

/**
 * Apply a rule to an atom.
 *
 * @param htarget   the atom in which the rule will be applied
 * @param rule      the rule to apply
 * @return          ???
 */
map<Handle, HandleSeq> BackwardChainer::apply_rule(Handle& htarget, Handle& rule)
{
	vector<map<Handle, HandleSeq>> results;
	Handle root_logical_link = get_root_logical_link(rule);

	if (root_logical_link == Handle::UNDEFINED)
	{
		//eg. ImplicationLink (Inheritance $x "human") (InheritanceLink "$x" "bipedal")) has no logical links
		Handle implicant = _as->getOutgoing(rule)[0];
		return do_bc(implicant);
	}

	//build a tree of the the logical links and premises( premises could by themselves be a logical link) as a map
	map<Handle, HandleSeq> logical_link_premise_map = get_logical_link_premises_map(rule);
	map<Handle, map<Handle, HandleSeq>> premise_var_ground_map;

	HandleSeq visited_logical_link;
	HandleSeq evaluated_premises;
	visited_logical_link.push_back(root_logical_link); //start from the root

	//go down deep until a logical link maps only to set of premises in the logical_link_premise_map object
	//e.g start from (and x y) instead of (and (and x y) x) and start backward chaining  from there. i.e bottom up.
	while (not visited_logical_link.empty())
	{
		Handle logical_link = visited_logical_link.back();
		visited_logical_link.pop_back();

		HandleSeq premises = logical_link_premise_map[logical_link];
		Handle llink = get_unvisited_logical_link(premises, evaluated_premises);

		if (llink != Handle::UNDEFINED)
		{
			visited_logical_link.push_back(llink);
			continue;
		}

		for (Handle premise : premises)
		{
			auto i = find(evaluated_premises.begin(), evaluated_premises.end(),
					premise);
			if (i == evaluated_premises.end())
			{
				auto var_grounding = do_bc(premise);
				premise_var_ground_map[premise] = var_grounding;
				evaluated_premises.push_back(premise);
			}
		}

		auto v = join_premise_vgrounding_maps(logical_link,
				premise_var_ground_map);

		results.push_back(v);		//add to results

		evaluated_premises.push_back(logical_link);
	}

	return ground_target_vars(htarget, results);
}

/**
 * Finds rule with their implicand matching the input.
 *
 * XXX what if a VariableNode is inside QuoteLink
 *
 * @param htarget    the input atom which will match the rules
 * @return           a list of rules
 */
HandleSeq BackwardChainer::query_rule_base(Handle htarget)
{
	Handle hbind_link = _commons->create_bindLink(htarget);

	logger().debug("QUERY-RB:\n" + hbind_link->toString());

	BindLinkPtr bl(BindLinkCast(hbind_link));
	bl->imply(_bcpm);

	_commons->clean_up_bind_link(hbind_link);

	auto result = _bcpm->get_result_list();
	_bcpm->clear_result_list(); //makes sure on each query only new results are returned

	return result;
}

/**
 * Finds atoms in the atomspace that match the VariableNodes in the input
 *
 * @param htarget
 * @return
 */
HandleSeq BackwardChainer::query_knowledge_base(Handle htarget)
{
	Handle hbind_link = _commons->create_bindLink(htarget);

	logger().debug("QUERY-KB:\n" + hbind_link->toString());

	BindLinkPtr bl(BindLinkCast(hbind_link));
	bl->imply(_bcpm);

	_commons->clean_up_bind_link(hbind_link);

	auto result = _bcpm->get_result_list();
	_bcpm->clear_result_list(); //for making sure on each query new results are returned
	return result;
}

/**
 * Find and return all handles containing variable and are within an implicationLink.
 *
 * @param handles
 * @return
 */
HandleSeq BackwardChainer::filter_rules(HandleSeq handles)
{
	HandleSeq rules;

	for (Handle h : handles)
	{
		HandleSeq links = _as->getIncoming(h);
		for (Handle l : links)
		{
			Type t = _as->getType(l);
			if (t == LIST_LINK)
			{
				HandleSeq incoming = _as->getIncoming(l);
				if (not incoming.empty())
				{
					if (_as->getType(incoming[0]) == EXECUTION_LINK)
					{
						HandleSeq hs = _as->getIncoming(incoming[0]);
						if (not hs.empty())
						{
							if (_as->getType(hs[0]) == IMPLICATION_LINK)
								rules.push_back(hs[0]);
						}
					}
				}
			}
			if (t == IMPLICATION_LINK)
				if (_as->getOutgoing(l)[1] == h)
					rules.push_back(l);
		}
	}

	return rules;
}

/**
 * Helper method for removing links with VariableNode.
 *
 * @param hpremise  a vector of atoms
 * @return          a cleaned vector with no link containing VariableNode
 */
HandleSeq BackwardChainer::filter_grounded_experssions(HandleSeq handles)
{
	HandleSeq grounded;

	for (Handle h : handles)
	{
		UnorderedHandleSet uhs = getAllUniqueNodes(h);

		if (std::none_of(uhs.begin(), uhs.end(), [](const Handle& n) { return NodeCast(n)->getType() == VARIABLE_NODE; }))
			grounded.push_back(h);
	}

	return grounded;
}

/**
 * maps variable to their groundings given a target handle with variables and a fully grounded matching handle
 * and adds the result to @param output
 * @param htarget the target with variable nodes
 * @param match a fully grounded matching handle with @param htarget
 * @param output a map object to store results
 * @return @param output a map of variable to their groundings
 */
map<Handle, HandleSeq> BackwardChainer::unify(Handle& htarget, Handle& match,
                                              map<Handle, HandleSeq>& result) {
	if (LinkCast(htarget)) {
		HandleSeq target_outg = _as->getOutgoing(htarget);
		HandleSeq match_outg = _as->getOutgoing(match);
		assert(target_outg.size()==match_outg.size()); //TODO throw exception instead
		for (vector<Handle>::size_type i = 0; i < target_outg.size(); i++) {
			if (_as->getType(target_outg[i]) == VARIABLE_NODE)
				result[target_outg[i]].push_back(match_outg[i]);
			else
				unify(target_outg[i], match_outg[i], result);
		}
	} else if (_as->getType(htarget) == VARIABLE_NODE)
		result[htarget].push_back(match);

	return result;
}

/**
 * maps @param htarget's variables wiht empty HandleSewq
 */
map<Handle, HandleSeq> BackwardChainer::unify_to_empty_set(Handle& htarget)
{
	logger().debug("[BC] Unify to empty set.");

	UnorderedHandleSet vars = get_outgoing_nodes(htarget, {VARIABLE_NODE});

	map<Handle, HandleSeq> result;
	for (Handle h : vars)
		result[h] = HandleSeq { Handle::UNDEFINED };
	return result;
}

/**
 *Given a target find a matching rule
 *@param target handle of the target
 */
Handle BackwardChainer::select_rule(HandleSeq& hseq_rule) {
	//apply selection criteria to select one amongst the matching rules

	//xxx return random for the purpose of integration testing before going
	//for a complex implementation of this function
	return hseq_rule[random() % hseq_rule.size()];
}


/**
 * @param connector
 * @param premise_var_grounding_map
 * @return a map of variable to groundings
 */
map<Handle, HandleSeq> BackwardChainer::join_premise_vgrounding_maps(
		const Handle& logical_link,
		const map<Handle, map<Handle, HandleSeq> >& premise_var_grounding_map)
{
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
					if (_as->getType(logical_link) == AND_LINK) {
						set_intersection(vg1.begin(), vg1.end(), vg2.begin(),
								vg2.end(), back_inserter(common_values));

					}

					if (_as->getType(logical_link) == OR_LINK)
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

/**
 * Looks if first list input exists in,VARIABLE_NODE the second ,gets that aren't contained in the second list
 * and returns the first amongst the non contained ones
 * @param connectors
 * @param visited
 * @return
 */
Handle BackwardChainer::get_unvisited_logical_link(HandleSeq& llinks,
		HandleSeq& visited) {
	HandleSeq result;
	for (Handle h : llinks) {
		Type t = _as->getType(h);
		auto it = find(_logical_link_types.begin(), _logical_link_types.end(),
				t);
		if (it != _logical_link_types.end()) {
			auto i = find(visited.begin(), visited.end(), h);
			if (i == visited.end())
				result.push_back(h);
		}
	}
	if (not result.empty())
		return result[0];

	return Handle::UNDEFINED;
}

/**
 * find and return all Handles with no variables
 * @param handles
 * @return vectors of handles with no variables in them
 */
HandleSeq BackwardChainer::get_grounded(HandleSeq result) {
	HandleSeq grounded;
	for (Handle h : result) {
		UnorderedHandleSet var_containing =
			get_outgoing_nodes(h, {VARIABLE_NODE});
		if (var_containing.empty())
			grounded.push_back(h);
	}
	return grounded;
}

/**
 * gets the roolt logical link in an implications link
 * eg. (ImplicationLink (AndLink .....) (...)) AndLink will be the root logical link
 * @param hrule
 * @return a handle if there is a root logical link or Handle::UNDEFINED if there is no
 */
Handle BackwardChainer::get_root_logical_link(Handle himplication_link)
		throw (opencog::InvalidParamException) {
	if (_as->getType(himplication_link) != IMPLICATION_LINK)
		throw InvalidParamException(TRACE_INFO,
				"input should be implication link");
	HandleSeq outg = _as->getOutgoing(himplication_link);
	Handle implicant = outg[0];
	if (find(_logical_link_types.begin(), _logical_link_types.end(),
			_as->getType(implicant)) != _logical_link_types.end()) {
		return implicant;
	} else
		return Handle::UNDEFINED;
}

/**
 *returns a map of connector link to set of premises connected xxx what if there is no connector?
 *eg. if implicatoinLink is
 *ImplicationLink(
 *  Andlink@1(
 *   Inheritance@1(
 *               (ConceptNode $x)
 *               (ConceptNode "Animal")
 *                )
 *            )
 *   AndLink@2(
 *           EvaluationLink@1(
 *                  (PredicateNode "eats")
 *                   ListLink(
 *                  (ConceptNode $x)
 *                  (ConceptNode "leaves")
 *                           )
 *                          )
 *           EvaluationLink@2(
 *                  (PredicateNode "eats")
 *                   ListLink(
 *                  (ConceptNode "$x")
 *                  (ConceptNode "flesh")
 *                           )
 *                          )
 *             )
 *    Inheritance@2(
 *               (ConceptNode $x)
 *               (ConceptNode "Omnivore")
 *                 )
 *               )
 *  will be returned as[Andlink@1->{Inheritance@1,AndLink@2},Andlink@2->{EvaluationLink@1,EvaluationLink@2}]
 *  where @n represents unique instance of links/connectors.its actually a BackInferenceTree(BIT) as a map
 *  without the use of tree DS.
 *  Note that implicand(consequent) has been rejected.the only interest here is the implicant(antecedent)
 *@ himplication_link a handle to an implication link
 *@ return a connector premise map found in the implicant of the implication link
 */
map<Handle, HandleSeq> BackwardChainer::get_logical_link_premises_map(
		Handle& himplication_link) throw (opencog::InvalidParamException) {
	if (_as->getType(himplication_link) != IMPLICATION_LINK)
		throw InvalidParamException(TRACE_INFO,
				"input should be implication link");
	Handle root_llink = _as->getOutgoing(himplication_link)[0];
	map<Handle, HandleSeq> logical_link_premise_map;
	HandleSeq logical_links;

	Type t = _as->getType(root_llink);
	auto it = find(_logical_link_types.begin(), _logical_link_types.end(), t);
	if (it != _logical_link_types.end()) {
		logical_links.push_back(root_llink);
		do {
			Handle llink = logical_links[logical_links.size() - 1];
			logical_links.pop_back();
			HandleSeq premises = _as->getOutgoing(llink);
			for (Handle h : premises) {
				logical_link_premise_map[llink].push_back(h);
				//check if h is a logical link type and push it for building next iter map
				t = _as->getType(h);
				it = find(_logical_link_types.begin(),
						_logical_link_types.end(), t);
				if (it != _logical_link_types.end())
					logical_links.push_back(h);

			}
		} while (not logical_links.empty());
		return logical_link_premise_map;
	} else
		return map<Handle, HandleSeq> { { Handle::UNDEFINED, HandleSeq {
				root_llink } } };
}

/**
 * looks for possible grounding of variable node in the entire inference list which was built through the backward chaining process
 * @param hvar a variable node whose possible values to be searched in the inference list
 * @param inference_list of variable to possible list of matches(to variableNode or ConceptNode) built in the prev inference steps
 * @param results a set of grounded nodes found for @param hvar
 */
HandleSeq BackwardChainer::chase_var_values(Handle& hvar,
		vector<map<Handle, HandleSeq>>& inference_list, HandleSeq& results) {
	for (auto it = inference_list.begin(); it != inference_list.end(); ++it) {
		map<Handle, HandleSeq> var_value = *it;
		if (var_value.count(hvar) != 0) {
			HandleSeq values = var_value[hvar];
			for (Handle h : values) {
				if (_as->getType(h) == VARIABLE_NODE) {
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

/**
 * matches the variables in the target to their groundings in the variable grounding map list
 * @param hgoal the target Handle consisting of variable nodes
 * @param var_grounding_map a variable to groundings map list
 * @return  a map of variable to all found groundings
 */
map<Handle, HandleSeq> BackwardChainer::ground_target_vars(Handle& hgoal,
		vector<map<Handle, HandleSeq>>& inference_list) {
	map<Handle, HandleSeq> vg_map;
	UnorderedHandleSet hgoal_vars = get_outgoing_nodes(hgoal, {VARIABLE_NODE});

	for (map<Handle, HandleSeq> vgm : inference_list) {
		for (auto it = vgm.begin(); it != vgm.end(); ++it) {

			Handle hvar = it->first;
			auto i = find(hgoal_vars.begin(), hgoal_vars.end(), hvar);

			if (i != hgoal_vars.end()) {
				HandleSeq values;
				HandleSeq groundings = it->second;
				for (Handle h : groundings) {
					if (_as->getType(h) == VARIABLE_NODE) {
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

/**
 * calls atomspace to remove each variables and links present the bc_gnerated_rules
 */
void BackwardChainer::remove_generated_rules() {
	for (vector<Handle>::size_type i = 0; i < _bc_generated_rules.size(); i++) {
		Handle h = _bc_generated_rules.back();
		_commons->clean_up_implication_link(h);
		_bc_generated_rules.pop_back();
	}
}

#ifdef DEBUG
void BackwardChainer::print_inference_list() {
	for (auto it = _inference_list.begin(); it != _inference_list.end(); ++it) {
		map<Handle, HandleSeq> var_ground = *it;
		for (auto j = var_ground.begin(); j != var_ground.end(); ++j) {
			cout << "[VAR:" << j->first->toString() << endl;
			HandleSeq hs = j->second;
			for (Handle h : hs)
			cout << "\tVAL:" << h->toString() << endl;
		}
		cout << "]" << endl;
	}
}
void BackwardChainer::print_premise_var_ground_mapping(
		const map<Handle, map<Handle, HandleSeq>>& premise_var_ground_map) {
	for (auto it = premise_var_ground_map.begin();
			it != premise_var_ground_map.end(); ++it) {
		cout << "PREMISE:" << endl << it->first->toString() << endl;
		map<Handle, HandleSeq> var_ground = it->second;
		print_var_value(var_ground);
	}
}
void BackwardChainer::print_var_value(
		const map<Handle, HandleSeq>& var_ground) {
	for (auto j = var_ground.begin(); j != var_ground.end(); ++j) {
		cout << "[VAR:" << j->first->toString() << endl;
		HandleSeq hs = j->second;
		for (Handle h : hs)
		cout << "\tVAL:" << h->toString() << endl;
	}
	cout << "]" << endl;
}
#endif

