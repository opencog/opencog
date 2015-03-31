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
#include "BCPatternMatch.h"

#include <opencog/atomutils/PatternUtils.h>
#include <opencog/atoms/bind/BindLink.h>

using namespace opencog;

BackwardChainer::BackwardChainer(AtomSpace* as, std::vector<Rule> rs)
    : _as(as)
{
	_commons = new PLNCommons(_as);
	_rules_set = rs;
}

BackwardChainer::~BackwardChainer()
{
	delete _commons;
}

/**
 * The public entry point for backward chaining.
 *
 * XXX TODO allow backward chaining 1 step (for mixing forward/backward chaining)
 *
 * @param init_target  the initial atom to start the chaining
 */
void BackwardChainer::do_chain(Handle init_target)
{
	_chaining_result.clear();

	do_bc(init_target);

	// results will contain a mapping of the variables in init_target
	// but it could point to other varialbes, so need to chase its
	// final grounding
	_chaining_result = ground_target_vars(init_target, _inference_list);

	//clean variables
	remove_generated_rules();
}

map<Handle, UnorderedHandleSet>& BackwardChainer::get_chaining_result()
{
	return _chaining_result;
}

/**
 * The main recursive backward chaining method.
 *
 * @param hgoal  the atom to do backward chaining on
 * @return       ???
 */
map<Handle, UnorderedHandleSet> BackwardChainer::do_bc(Handle& hgoal)
{
	// TODO filter grounded representations so the next condition would never be fooled
	// XXX what does the above meant?
	HandleSeq kb_match = filter_grounded_experssions(hgoal);

	if (kb_match.empty())
	{
		logger().debug("[BackwardChainer] Knowledge base empty");

		// find all rules whose implicand can be unified to hgoal
		std::vector<Rule> acceptable_rules = filter_rules(hgoal);

		logger().debug("[BackwardChainer] Found %d acceptable rules", acceptable_rules.size());

		// if no rules to backward chain on
		if (acceptable_rules.empty())
			return map<Handle, UnorderedHandleSet>();

		// XXX TODO use all rules found here; this will require branching on the
		// rules, so this will be the point where a variable could end up mapping
		// to more than one solution
		Rule standardized_rule = select_rule(acceptable_rules).gen_standardize_apart(_as);

		//for later removal
//		_bc_generated_rules.push_back(stadardized_rule);

		HandleSeq output = standardized_rule.get_implicand();
		map<Handle, Handle> implicand_mapping;

		// check which output can be unified and add to history
		for (Handle h : output)
		{
			if (not unify(h, hgoal, implicand_mapping))
				continue;

			logger().debug("[BackwardChainer] Found one implicand's output unifiable " + h->toShortString());
			break;
		}

		logger().debug("[BackwardChainer] Before chaining on rule %s", standardized_rule.get_name().c_str());
		logger().debug("[BackwardChainer] Inference history's size is %d", _inference_list.size());

		if (_inference_list.size() > 5)
			return map<Handle, UnorderedHandleSet>();

		print_inference_list();

		return bc_rule(standardized_rule, implicand_mapping);
	}

	map<Handle, UnorderedHandleSet> results;

	for (Handle soln : kb_match)
	{
		map<Handle, Handle> vgm;

		// should be possible to unify since kb_match is found by PM on hgoal
		unify(hgoal, soln, vgm);

		// construct the hgoal to all mappings here to be returned
		for (auto it = vgm.begin(); it != vgm.end(); ++it)
			results[it->first].emplace(it->second);
	}

	return results;
}

/**
 * Backward chaining on a rule's implicant.
 *
 * Get all the premises inside the implicant, and backward chain on each
 * premise separately.
 *
 * XXX TODO actually use the unified implicand to limit the implicant
 * Right now the whole atomspace is searched to match the implicant, which
 * might not be necessary...
 *
 * @param rule      the rule to apply
 * @return          ???
 */
map<Handle, UnorderedHandleSet> BackwardChainer::bc_rule(Rule& rule, const map<Handle, Handle>& mapping)
{
	Handle himplicant = rule.get_implicant();

	// add the mapping to the inference history
	map<Handle, UnorderedHandleSet> history;
	for (auto& m : mapping)
		history[m.first].emplace(m.second);
	_inference_list.push_back(history);

	// reverse ground the implicant with the grounding we found from unifying
	// the implicand
	Instantiator inst(_as);
	himplicant = inst.instantiate(himplicant, mapping);

	std::stack<Handle> visit_stack;
	visit_stack.push(himplicant);

	map<Handle, HandleSeq> logical_link_premise_map = get_logical_link_premises_map(himplicant);
	map<Handle, map<Handle, UnorderedHandleSet>> premise_var_ground_map;
	UnorderedHandleSet evaluated_premises;

	logger().debug("[BackwardChainer] Before looping rule's premises");

	while (not visit_stack.empty())
	{
		Handle premise = visit_stack.top();
		visit_stack.pop();

		// check if premise was already visited
		if (evaluated_premises.count(premise) == 1)
			continue;

		// if premise has more sub-premises
		if (logical_link_premise_map.count(premise) == 1)
		{
			HandleSeq sub_premises = logical_link_premise_map[premise];

			// check if all its sub-premises are already evaluated
			if (std::all_of(sub_premises.begin(), sub_premises.end(), [&](const Handle& h) { return evaluated_premises.count(h) == 1; }))
			{
				// if so, apply the logical operation to the solutions
				premise_var_ground_map[premise] = join_premise_vgrounding_maps(premise, premise_var_ground_map);
				evaluated_premises.emplace(premise);

				continue;
			}

			// if not all evaluated, add this premise back into the stack, but
			// put all its sub-premises on top, so they are evaluated first
			visit_stack.push(premise);

			for (auto p : sub_premises)
				visit_stack.push(p);

			continue;
		}

		logger().debug("[BackwardChainer] Backward chaining on sub-premise " + premise->toShortString());

		// if premise has no sub-premises (ie. not a logical condition), just
		// backward chain on it
		// XXX TODO instead of immediately chain on the premise, add it to a queue instead
		// ie. do something like BFS instead of DFS


		// some of its mapping might need to be removed by AND_LINK, but if they are in the _inference_list, we cannot tell

		premise_var_ground_map[premise] = do_bc(premise);
		evaluated_premises.emplace(premise);
	}

	map<Handle, UnorderedHandleSet> result = premise_var_ground_map[himplicant];

	// add the final joined result to inference history
	_inference_list.push_back(result);

	// get the final grounding
	return result;
}

/**
 * Find all rules in which the input could be an output.
 *
 * @param htarget   an input atom to match
 * @return          a vector of rules
 */
std::vector<Rule> BackwardChainer::filter_rules(Handle htarget)
{
	std::vector<Rule> rules;

	for (Rule& r : _rules_set)
	{
		HandleSeq output = r.get_implicand();
		bool unifiable = false;

		// check if any of the implicand's output can be unified to target
		for (Handle h : output)
		{
			std::map<Handle, Handle> mapping;

			if (not unify(h, htarget, mapping))
				continue;

			unifiable = true;
			break;
		}

		// move on to next rule if htarget cannot map to the output
		if (not unifiable)
			continue;

		rules.push_back(r);
	}

	return rules;
}

/**
 * Filter the atomspace and find grounded expressions matching input.
 *
 * @param htarget  the atom to pattern match against
 * @return         a vector of grounded expressions
 */
HandleSeq BackwardChainer::filter_grounded_experssions(Handle htarget)
{
	Handle hbind_link = _commons->create_bindLink(htarget);

	logger().debug("QUERY-KB:\n" + hbind_link->toString());

	BindLinkPtr bl(BindLinkCast(hbind_link));
	BCPatternMatch bcpm(_as);
	bcpm.implicand = bl->get_implicand();

	bl->imply(&bcpm);

	_commons->clean_up_bind_link(hbind_link);

	HandleSeq grounded;

	for (Handle h : bcpm.get_result_list())
	{
		FindVariables fv(VARIABLE_NODE);
		fv.find_vars(h);

		if (fv.varset.empty())
			grounded.push_back(h);
	}

	return grounded;
}

/**
 * Unify two atoms, finding a mapping that makes them equal.
 *
 * Unification is done by mapping VariableNodes from one atom to atoms in the
 * other.
 *
 * XXX TODO unify UNORDERED_LINK
 * XXX TODO check unifying same variable twice
 * XXX TODO check VariableNode inside QuoteLink
 * XXX TODO check Typed VariableNode
 * XXX TODO unify in both direction
 * XXX Should (Link (Node A)) be unifiable to (Node A))?  BC literature never
 * unify this way, but in AtomSpace context, (Link (Node A)) does contain (Node A)
 *
 * @param htarget    the target with variable nodes
 * @param hmatch     a fully grounded matching handle with @param htarget
 * @param output     a map object to store results and for recursion
 * @return           true if the two atoms can be unified
 */
bool BackwardChainer::unify(const Handle& htarget,
                            const Handle& hmatch,
                            map<Handle, Handle>& result)
{
	LinkPtr lptr_target(LinkCast(htarget));
	LinkPtr lptr_match(LinkCast(hmatch));

	// if unifying a link
	if (lptr_target && lptr_match)
	{
		HandleSeq target_outg = lptr_target->getOutgoingSet();

		// if the two links type are not equal
		if (lptr_target->getType() != lptr_match->getType())
		{
			result = std::map<Handle, Handle>();
			return false;
		}

		HandleSeq match_outg = lptr_match->getOutgoingSet();

		// if the two links are not of equal size, cannot unify
		if (target_outg.size() != match_outg.size())
		{
			result = std::map<Handle, Handle>();
			return false;
		}

		for (vector<Handle>::size_type i = 0; i < target_outg.size(); i++)
		{
			if (target_outg[i]->getType() == VARIABLE_NODE)
				result[target_outg[i]] = match_outg[i];
			else if (not unify(target_outg[i], match_outg[i], result))
				return false;
		}
	}
	else if (htarget->getType() == VARIABLE_NODE)
	{
		result[htarget] = hmatch;
	}

	return not result.empty();
}

/**
 * Given a target, select a candidate rule.
 *
 * XXX TODO apply selection criteria to select one amongst the matching rules
 *
 * @param rules   a vector of rules to select from
 * @return        one of the rule
 */
Rule BackwardChainer::select_rule(const std::vector<Rule>& rules)
{
	//xxx return random for the purpose of integration testing before going
	//for a complex implementation of this function
	return rules[random() % rules.size()];
}


/**
 * Apply the logical link to the solution from its sub-premisies.
 *
 * @param logical_link               the top logical link
 * @param premise_var_grounding_map  the solutions from the sub-premises
 * @return                           a map of variable to groundings
 */
map<Handle, UnorderedHandleSet> BackwardChainer::join_premise_vgrounding_maps(
		const Handle& logical_link,
		const map<Handle, map<Handle, UnorderedHandleSet> >& premise_var_grounding_map)
{
	map<Handle, UnorderedHandleSet> result;

	// for each sub-premise, look at its solution
	for (auto pvg_it = premise_var_grounding_map.begin();
	     pvg_it != premise_var_grounding_map.end();
	     ++pvg_it)
	{
		// get the sub-premise's solution
		map<Handle, UnorderedHandleSet> var_groundings = pvg_it->second;

		// first sub-premise, just add all its solution
		if (pvg_it == premise_var_grounding_map.begin())
		{
			result = var_groundings;
			continue;
		}

		// for each variable
		for (auto vg_it = var_groundings.begin();
			 vg_it != var_groundings.end();
			 ++vg_it)
		{
			Handle key = vg_it->first;
			UnorderedHandleSet vals = vg_it->second;

			// if OR_LINK, the mapping of common variable can be merged
			if (logical_link->getType() == OR_LINK)
			{
				result[key].insert(vals.begin(), vals.end());
				continue;
			}

			// if AND_LINK, the mapping of common variable must agree
			if (logical_link->getType() == AND_LINK)
			{
				UnorderedHandleSet common_vals;
				UnorderedHandleSet old_vals = result[key];

				for (auto o : old_vals)
				{
					if (vals.count(o) == 1)
						common_vals.emplace(o);
				}

				result[key] = common_vals;
			}
		}
	}

	return result;
}

/**
 * Returns a map of connector link to set of premises connected.
 *
 * eg. if the implicant is
 *
 *    Andlink@1
 *       Inheritance@1
 *          ConceptNode $x
 *          ConceptNode "Animal"
 *       AndLink@2
 *          EvaluationLink@1
 *             PredicateNode "eats"
 *             ListLink
 *                ConceptNode $x
 *                ConceptNode "leaves"
 *          EvaluationLink@2
 *             PredicateNode "eats"
 *             ListLink
 *                ConceptNode "$x"
 *                ConceptNode "flesh"
 *
 * will be returned as
 *
 *    Andlink@1->{Inheritance@1,AndLink@2}
 *    Andlink@2->{EvaluationLink@1,EvaluationLink@2}
 *
 * where @n represents unique instance of links/connectors.  It'ss actually
 * a Back Inference Tree (BIT) as a map without the use of tree.
 *
 * @param implicant    a handle to the implicant
 * @return             a mapping of the above structure
 */
map<Handle, HandleSeq> BackwardChainer::get_logical_link_premises_map(Handle& himplicant)
{
	map<Handle, HandleSeq> logical_link_premise_map;
	std::stack<Handle> visit_stack;

	visit_stack.push(himplicant);

	while (not visit_stack.empty())
	{
		Handle head = visit_stack.top();
		visit_stack.pop();

		// if not a logical link, continue
		if (find(_logical_link_types.begin(), _logical_link_types.end(), head->getType()) == _logical_link_types.end())
			continue;

		for (Handle h : LinkCast(head)->getOutgoingSet())
		{
			logical_link_premise_map[head].push_back(h);
			visit_stack.push(h);
		}

	}

	return logical_link_premise_map;
}

/**
 * Looks for possible grounding of variable node in the input inference list.
 *
 * Does the main recursive chasing for ground_target_vars()
 *
 * @param hvar            a variable node whose possible values to be searched in the inference list
 * @param inference_list  variable to possible list of matches
 * @param results         a set of grounded solution found
 */
UnorderedHandleSet BackwardChainer::chase_var_values(
        Handle& hvar,
        const vector<map<Handle, UnorderedHandleSet>>& inference_list,
        UnorderedHandleSet& results)
{
	// check the inference history to see where hvar appear
	for (map<Handle, UnorderedHandleSet> vgm : inference_list)
	{
		// each time it appear, add its mapping to the results
		if (vgm.count(hvar) != 0)
		{
			UnorderedHandleSet groundings = vgm[hvar];

			for (Handle h : groundings)
			{
				if (h->getType() == VARIABLE_NODE)
					chase_var_values(h, inference_list, results);
				else
					results.emplace(h);
			}
		}
	}

	return results;
}

/**
 * Matches the variables in the target to their groundings.
 *
 * This method will chase the mapping, so if $x->$y, $y->"dog", then in the
 * end we will get $x->"dog".
 *
 * @param hgoal            the target Handle consisting of variable nodes
 * @param inference_list   a variable to groundings map list
 * @return                 a map of variable to all found groundings
 */
map<Handle, UnorderedHandleSet> BackwardChainer::ground_target_vars(
        Handle& hgoal,
        vector<map<Handle, UnorderedHandleSet>>& inference_list)
{
	map<Handle, UnorderedHandleSet> vg_map;

	// find all VariableNode inside hgoal, but not those inside QuoteLink
	FindVariables fv(VARIABLE_NODE);
	fv.find_vars(hgoal);

	// check all inference history
	for (map<Handle, UnorderedHandleSet> vgm : inference_list)
	{
		// check the mapping generated at each point
		for (auto it = vgm.begin(); it != vgm.end(); ++it)
		{
			Handle hvar = it->first;

			// if hvar is in hgoal
			if (fv.varset.count(hvar) == 1)
			{
				UnorderedHandleSet groundings = it->second;

				for (Handle h : groundings)
				{
					UnorderedHandleSet results;

					if (h->getType() == VARIABLE_NODE)
						chase_var_values(h, inference_list, results);

					vg_map[hvar].insert(results.begin(), results.end());
				}
			}
		}
	}

	return vg_map;
}

/**
 * calls atomspace to remove each variables and links present the bc_gnerated_rules
 */
void BackwardChainer::remove_generated_rules()
{
//	for (vector<Handle>::size_type i = 0; i < _bc_generated_rules.size(); i++) {
//		Handle h = _bc_generated_rules.back();
//		_commons->clean_up_implication_link(h);
//		_bc_generated_rules.pop_back();
//	}
}

#ifdef DEBUG
void BackwardChainer::print_inference_list()
{
	for (auto it = _inference_list.begin(); it != _inference_list.end(); ++it) {
		map<Handle, UnorderedHandleSet> var_ground = *it;
		for (auto j = var_ground.begin(); j != var_ground.end(); ++j) {
			UnorderedHandleSet hs = j->second;
			std::string mapping;
			for (Handle h : hs)
				mapping += "\tVAL:" + h->toString() + "\n";

			logger().debug("[BackwardChainer] VAR:" + j->first->toString() + mapping);
		}
	}
}

void BackwardChainer::print_premise_var_ground_mapping(
		const map<Handle, map<Handle, HandleSeq>>& premise_var_ground_map)
{
	for (auto it = premise_var_ground_map.begin();
			it != premise_var_ground_map.end(); ++it) {
		cout << "PREMISE:" << endl << it->first->toString() << endl;
		map<Handle, HandleSeq> var_ground = it->second;
		print_var_value(var_ground);
	}
}

void BackwardChainer::print_var_value( const map<Handle, HandleSeq>& var_ground)
{
	for (auto j = var_ground.begin(); j != var_ground.end(); ++j) {
		cout << "[VAR:" << j->first->toString() << endl;
		HandleSeq hs = j->second;
		for (Handle h : hs)
			cout << "\tVAL:" << h->toString() << endl;
	}
	cout << "]" << endl;
}
#endif

