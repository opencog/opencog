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

#include <opencog/atomutils/FindUtils.h>
#include <opencog/atoms/bind/PatternUtils.h>
#include <opencog/atoms/bind/SatisfactionLink.h>

using namespace opencog;

BackwardChainer::BackwardChainer(AtomSpace* as, std::vector<Rule> rs)
    : _as(as)
{
	_rules_set = rs;

	// create a garbage subspace with _as as parent, so codes acting on
	// _as will see also see _garbage, but codes looking at _garbage
	// will not see stuff in _as
	_garbage_subspace = new AtomSpace(_as);
}

BackwardChainer::~BackwardChainer()
{
	// this will presumably remove all temp atoms from _as
	delete _garbage_subspace;
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

	_targets_stack = std::stack<Handle>();
	_targets_stack.push(init_target);

	int i = 0;

	while (not _targets_stack.empty())
	{
		// XXX TODO targets selection should be done here, by first changing
		// the stack to other data types

		Handle top = _targets_stack.top();
		_targets_stack.pop();

		logger().debug("[BackwardChainer] Before do_bc");

		VarMultimap subt = do_bc(top);
		VarMultimap& old_subt = _inference_history[top];

		logger().debug("[BackwardChainer] After do_bc");

		// add the substitution to inference history
		for (auto& p : subt)
			old_subt[p.first].insert(p.second.begin(), p.second.end());

		i++;
		// debug quit
		if (i == 5)
			break;
	}

	_chaining_result = _inference_history[init_target];

	//clean variables
	//remove_generated_rules();
}

VarMultimap& BackwardChainer::get_chaining_result()
{
	return _chaining_result;
}



/**
 * The main recursive backward chaining method.
 *
 * @param hgoal  the atom to do backward chaining on
 * @return       ???
 */
VarMultimap BackwardChainer::do_bc(Handle& hgoal)
{
//	// check if this goal is already grounded
//	if (_inference_history.count(hgoal) == 1)
//		return _inference_history[hgoal];

	HandleSeq free_vars = get_free_vars_in_tree(hgoal);

	// check whether this goal has free variables and worth exploring
	if (free_vars.empty())
	{
		logger().debug("[BackwardChainer] Boring goal with no free var, skipping " + hgoal->toShortString());
		return VarMultimap();
	}

//	VarMultimap results;

//	// check if all free vars has a solution in the inference history already
//	// XXX no good, does not handle And/Or/Not
//	for (Handle& h : free_vars)
//	{
//		for (auto& p : _inference_history)
//		{
//			VarMultimap& vgm = p.second;

//			if (vgm.count(h) == 0)
//				continue;

//			results[h].insert(vgm[h].begin(), vgm[h].end());
//		}
//	}

//	if (results.size() == free_vars.size())
//		return results;

	std::vector<VarMap> kb_vmap;

	// else, either try to ground, or backward chain
	HandleSeq kb_match = match_knowledge_base(hgoal, kb_vmap);

	if (kb_match.empty())
	{
		// if logical link, break it up, add each to the targets stack, and return
		if (_logical_link_types.count(hgoal->getType()) == 1)
		{
			HandleSeq sub_premises = LinkCast(hgoal)->getOutgoingSet();

			for (Handle& h : sub_premises)
				_targets_stack.push(h);

			return VarMultimap();
		}

		// find all rules whose implicand can be unified to hgoal
		std::vector<Rule> acceptable_rules = filter_rules(hgoal);

		// if no rules to backward chain on, no way to solve this goal
		if (acceptable_rules.empty())
			return VarMultimap();

		// XXX TODO use all rules found here; this will require branching
		Rule standardized_rule = select_rule(acceptable_rules).gen_standardize_apart(_garbage_subspace);

		Handle himplicant = standardized_rule.get_implicant();
		HandleSeq outputs = standardized_rule.get_implicand();
		VarMap implicand_mapping;

		// a rule can have multiple output, and only one will unify to our goal
		// so try to find the one output that works
		for (Handle h : outputs)
		{
			VarMap temp_mapping;

			if (not unify(h, hgoal, temp_mapping))
				continue;

			// wrap all the mapped result inside QuoteLink, so that variables
			// will be handled correctly for the next BC step
			for (auto& p : temp_mapping)
				implicand_mapping[p.first] = _garbage_subspace->addAtom(createLink(QUOTE_LINK, p.second));

			logger().debug("[BackwardChainer] Found one implicand's output unifiable " + h->toShortString());
			break;
		}

		// reverse ground the implicant with the grounding we found from
		// unifying the implicand
		Instantiator inst(_garbage_subspace);
		himplicant = inst.instantiate(himplicant, implicand_mapping);

		// find all matching premises
		// XXX TODO include typed variable node checking
		std::vector<VarMap> vmap_list;
		HandleSeq possible_premises = match_knowledge_base(himplicant, vmap_list);

		std::stack<Handle> to_be_added_to_targets;
		VarMultimap results;

		// for each set of possible premises, check if they already satisfy the goal
		for (Handle& h : possible_premises)
		{
			vmap_list.clear();

			bool need_bc = false;
			HandleSeq grounded_premises = match_knowledge_base(h, vmap_list);

			// check each grounding to see if any has no variable
			for (size_t i = 0; i < grounded_premises.size(); ++i)
			{
				Handle& g = grounded_premises[i];
				VarMap& m = vmap_list[i];

				FindAtoms fv(VARIABLE_NODE);
				fv.search_set(g);

				// if some grounding cannot solve the goal, will need to BC
				if (not fv.varset.empty())
				{
					need_bc = true;
					continue;
				}

				// this is a grounding that can solve the goal, so apply it
				inst.instantiate(hgoal, m);

				// add the grounding to the return results
				for (Handle& h : free_vars)
					results[h].emplace(m[h]);
			}

			if (not need_bc)
				continue;

			// XXX TODO premise selection would be done here to
			// determine wheter to BC on a premise

			// break out any logical link and add to targets
			if (_logical_link_types.count(h->getType()) == 0)
			{
				to_be_added_to_targets.push(h);
				continue;
			}

			HandleSeq sub_premises = LinkCast(h)->getOutgoingSet();

			for (Handle& h : sub_premises)
				to_be_added_to_targets.push(h);
		}

		if (not to_be_added_to_targets.empty())
		{
			// add itself back to target, since this is not completely solved
			_targets_stack.push(hgoal);

			while (not to_be_added_to_targets.empty())
			{
				_targets_stack.push(to_be_added_to_targets.top());
				to_be_added_to_targets.pop();
			}
		}

		return results;
	}
	else
	{
		logger().debug("[BackwardChainer] Matched something in knowledge base, storying the grounding");

		VarMultimap results;
		bool goal_readded = false;

		for (size_t i = 0; i < kb_match.size(); ++i)
		{
			Handle& soln = kb_match[i];
			VarMap& vgm = kb_vmap[i];

			logger().debug("[BackwardChainer] Looking at grounding " + soln->toShortString());

			// check if there is any free variables in soln
			HandleSeq free_vars = get_free_vars_in_tree(soln);

			// if there are free variables, add this soln to the target stack
			// XXX should the free_vars be checked against inference history to
			// see if a solution exist first?
			if (not free_vars.empty())
			{
				// add the goal back in target list since one of the solution
				// has variables inside
				if (not goal_readded)
				{
					_targets_stack.push(hgoal);
					goal_readded = true;
				}

				_targets_stack.push(soln);
			}

			// construct the hgoal to all mappings here to be returned
			for (auto it = vgm.begin(); it != vgm.end(); ++it)
				results[it->first].emplace(it->second);
		}

		return results;
	}
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
 * Find all atoms in the AtomSpace matching the pattern.
 *
 * @param htarget  the atom to pattern match against
 * @return         a vector of matched atoms
 */
HandleSeq BackwardChainer::match_knowledge_base(Handle htarget, vector<VarMap>& vmap)
{
	// get all VariableNodes (unquoted)
	FindAtoms fv(VARIABLE_NODE);
	fv.search_set(htarget);

	// unbundle clause if necessary
	HandleSeq terms;
	if (_logical_link_types.count(htarget->getType()))
		terms = LinkCast(htarget)->getOutgoingSet();
	else
		terms.push_back(htarget);

	logger().debug("[BackwardChainer] Matching knowledge base with %s and %d variables", htarget->toShortString().c_str(), fv.varset.size());

	SatisfactionLinkPtr sl(createSatisfactionLink(fv.varset, terms));
	BCPatternMatch bcpm(_as);

	sl->satisfy(&bcpm);

	logger().debug("[BackwardChainer] After running pattern matcher");

	vector<map<Handle, Handle>> var_solns = bcpm.get_var_list();
	vector<map<Handle, Handle>> pred_solns = bcpm.get_pred_list();

	HandleSeq results;

	logger().debug("[BackwardChainer] Pattern matcher found %d matches", var_solns.size());

	for (size_t i = 0; i < var_solns.size(); i++)
	{
		// don't want matched clause that is part of a rule
		if (std::any_of(_rules_set.begin(), _rules_set.end(),
		                [&](Rule& r) { return is_atom_in_tree(r.get_handle(), pred_solns[i][htarget]); }))
			continue;

		// don't want matched clause that is in the garbage space
		if (_garbage_subspace->getAtom(pred_solns[i][htarget]) != Handle::UNDEFINED)
			continue;

		// don't want matched clause already in inference history
		if (_inference_history.count(pred_solns[i][htarget]) == 1)
			continue;

		// XXX don't want clause that are already in _targets_stack?
		// no need? since things on targets stack are in inference history

		results.push_back(pred_solns[i][htarget]);
		vmap.push_back(var_solns[i]);
	}

	return results;
}

/**
 * Unify two atoms, finding a mapping that makes them equal.
 *
 * Use the Pattern Matcher to do the heavy lifting of unification from one
 * specific atom to another, let it handles UnorderedLink, VariableNode in
 * QuoteLink, etc.
 *
 * XXX TODO check Typed VariableNode
 * XXX TODO unify in both direction?
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
                            VarMap& result)
{
	logger().debug("[BackwardChainer] starting unify");

	// lazy way of restricting PM to be between two atoms
	AtomSpace temp_space;

	Handle temp_htarget = temp_space.addAtom(htarget);
	Handle temp_hmatch = temp_space.addAtom(hmatch);

	FindAtoms fv(VARIABLE_NODE);
	fv.search_set(temp_htarget);

	HandleSeq terms;
	terms.push_back(temp_htarget);

	SatisfactionLinkPtr sl(createSatisfactionLink(fv.varset, terms));
	BCPatternMatch bcpm(&temp_space);

	sl->satisfy(&bcpm);

	// if no grounding
	if (bcpm.get_var_list().size() == 0)
		return false;

	logger().debug("[BackwardChainer] unify found %d mapping", bcpm.get_var_list().size());

	std::vector<std::map<Handle, Handle>> pred_list = bcpm.get_pred_list();
	std::vector<std::map<Handle, Handle>> var_list = bcpm.get_var_list();

	VarMap good_map;

	// go thru each solution, and get the first one that map the whole temp_hmatch
	// XXX TODO branch on the various groundings
	for (size_t i = 0; i < pred_list.size(); ++i)
	{
		for (auto& p : pred_list[i])
		{
			if (is_atom_in_tree(p.second, temp_hmatch))
			{
				good_map = var_list[i];
				i = pred_list.size();
				break;
			}
		}
	}

	// change the mapping from temp_atomspace to current atomspace
	for (auto& p : good_map)
	{
		Handle var = p.first;
		Handle grn = p.second;

		logger().debug("[BackwardChainer] unified " + var->toShortString() + " to " + grn->toShortString());

		// getAtom should get the equivlent atom in this atomspace
		result[_as->getAtom(var)] = _as->getAtom(grn);
	}

	return true;
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
VarMultimap BackwardChainer::join_premise_vgrounding_maps(
		const Handle& logical_link,
		const map<Handle, VarMultimap>& premise_var_grounding_map)
{
	VarMultimap result;

	// for each sub-premise, look at its solution
	for (auto pvg_it = premise_var_grounding_map.begin();
	     pvg_it != premise_var_grounding_map.end();
	     ++pvg_it)
	{
		// get the sub-premise's solution
		VarMultimap var_groundings = pvg_it->second;

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
VarMultimap BackwardChainer::ground_target_vars(Handle& hgoal)
{
	return VarMultimap();

//	// check if inference history has a grounding for hgoal yet
//	if (_inference_history.count(hgoal) == 0)
//		return VarMultimap;

//	VarMultimap& grounding = _inference_history[hgoal];

//	map<Handle, UnorderedHandleSet> vg_map;

//	// find all VariableNode inside hgoal, but not those inside QuoteLink
//	FindAtoms fv(VARIABLE_NODE);
//	fv.search_set(hgoal);

//	// check all inference history
//	for (map<Handle, UnorderedHandleSet> vgm : inference_list)
//	{
//		// check the mapping generated at each point
//		for (auto it = vgm.begin(); it != vgm.end(); ++it)
//		{
//			Handle hvar = it->first;

//			// if hvar is in hgoal
//			if (fv.varset.count(hvar) == 1)
//			{
//				UnorderedHandleSet groundings = it->second;

//				for (Handle h : groundings)
//				{
//					UnorderedHandleSet results;

//					if (h->getType() == VARIABLE_NODE)
//						chase_var_values(h, inference_list, results);

//					vg_map[hvar].insert(results.begin(), results.end());
//				}
//			}
//		}
//	}

//	return vg_map;
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

