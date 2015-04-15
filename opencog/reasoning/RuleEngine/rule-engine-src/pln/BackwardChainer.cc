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

#include <opencog/util/random.h>

#include <opencog/atomutils/FindUtils.h>
#include <opencog/atoms/bind/PatternUtils.h>
#include <opencog/atoms/bind/SatisfactionLink.h>

using namespace opencog;

BackwardChainer::BackwardChainer(AtomSpace* as, std::vector<Rule> rs)
    : _as(as)
{
	_rules_set = rs;

	// create a garbage superspace with _as as parent, so codes acting on
	// _garbage will see stuff in _as, but codes acting on _as will not
	// see stuff in _garbage
	_garbage_superspace = new AtomSpace(_as);
}

BackwardChainer::~BackwardChainer()
{
	// this will presumably remove all temp atoms
	delete _garbage_superspace;
}

/**
 * Set the initial target for backward chaining.
 *
 * @param init_target   Handle of the target
 */
void BackwardChainer::set_target(Handle init_target)
{
	_init_target = init_target;
}

/**
 * The public entry point for full backward chaining.
 */
void BackwardChainer::do_full_chain()
{
	_inference_history.clear();

	_targets_stack = std::stack<Handle>();
	_targets_stack.push(_init_target);

	int i = 0;

	while (not _targets_stack.empty())
	{
		do_step();

		i++;
		// debug quit
		if (i == 5)
			break;
	}
}

/**
 * Do a single step of backward chaining.
 */
void BackwardChainer::do_step()
{
	// XXX TODO targets selection should be done here, by first changing
	// the stack to other data types

	Handle top = _targets_stack.top();
	_targets_stack.pop();

	logger().debug("[BackwardChainer] Before do_step_bc");

	VarMultimap subt = do_bc(top);
	VarMultimap& old_subt = _inference_history[top];

	logger().debug("[BackwardChainer] After do_step_bc");

	// add the substitution to inference history
	for (auto& p : subt)
		old_subt[p.first].insert(p.second.begin(), p.second.end());
}

/**
 * Get the current result on the initial target, if any.
 *
 * @return a VarMultimap mapping each variable to all possible solutions
 */
VarMultimap& BackwardChainer::get_chaining_result()
{
	return _inference_history[_init_target];
}

/**
 * The main recursive backward chaining method.
 *
 * @param hgoal  the atom to do backward chaining on
 * @return       the solution found for this goal, if any
 */
VarMultimap BackwardChainer::do_bc(Handle& hgoal)
{
	HandleSeq free_vars = get_free_vars_in_tree(hgoal);

	// Check whether this goal has free variables and worth exploring
	if (free_vars.empty())
	{
		logger().debug("[BackwardChainer] Boring goal with no free var, "
		               "skipping " + hgoal->toShortString());
		return VarMultimap();
	}

	std::vector<VarMap> kb_vmap;

	// Else, either try to ground, or backward chain
	HandleSeq kb_match = match_knowledge_base(hgoal, kb_vmap);

	if (kb_match.empty())
	{
		// If logical link, break it up, add each to the targets
		// stack, and return
		if (_logical_link_types.count(hgoal->getType()) == 1)
		{
			HandleSeq sub_premises = LinkCast(hgoal)->getOutgoingSet();

			for (Handle& h : sub_premises)
				_targets_stack.push(h);

			return VarMultimap();
		}

		// Find all rules whose implicand can be unified to hgoal
		std::vector<Rule> acceptable_rules = filter_rules(hgoal);

		// If no rules to backward chain on, no way to solve this goal
		if (acceptable_rules.empty())
			return VarMultimap();

		// XXX TODO use all rules found here; this will require branching
		Rule standardized_rule = select_rule(acceptable_rules).gen_standardize_apart(_garbage_superspace);

		Handle himplicant = standardized_rule.get_implicant();
		HandleSeq outputs = standardized_rule.get_implicand();
		VarMap implicand_mapping;

		// A rule can have multiple outputs, and only one will unify
		// to our goal so try to find the one output that works
		for (Handle h : outputs)
		{
			VarMap temp_mapping;

			if (not unify(h, hgoal, temp_mapping))
				continue;

			// Wrap all the mapped result inside QuoteLink, so that variables
			// will be handled correctly for the next BC step
			for (auto& p : temp_mapping)
				implicand_mapping[p.first] =
					_garbage_superspace->addAtom(createLink(QUOTE_LINK, p.second));

			logger().debug("[BackwardChainer] Found one implicand's output "
			               "unifiable " + h->toShortString());
			break;
		}

		// Reverse ground the implicant with the grounding we found from
		// unifying the implicand
		Instantiator inst(_garbage_superspace);
		himplicant = inst.instantiate(himplicant, implicand_mapping);

		// Find all matching premises
		// XXX TODO include typed variable node checking
		std::vector<VarMap> vmap_list;
		HandleSeq possible_premises = match_knowledge_base(himplicant, vmap_list);

		std::stack<Handle> to_be_added_to_targets;
		VarMultimap results;

		// For each set of possible premises, check if they already
		// satisfy the goal
		for (Handle& h : possible_premises)
		{
			vmap_list.clear();

			bool need_bc = false;
			HandleSeq grounded_premises = match_knowledge_base(h, vmap_list);

			// Check each grounding to see if any has no variable
			for (size_t i = 0; i < grounded_premises.size(); ++i)
			{
				Handle& g = grounded_premises[i];
				VarMap& m = vmap_list[i];

				FindAtoms fv(VARIABLE_NODE);
				fv.search_set(g);

				// If some grounding cannot solve the goal, will need to BC
				if (not fv.varset.empty())
				{
					need_bc = true;
					continue;
				}

				// This is a grounding that can solve the goal, so apply it
				inst.instantiate(hgoal, m);

				// Add the grounding to the return results
				for (Handle& h : free_vars)
					results[h].emplace(m[h]);
			}

			if (not need_bc)
				continue;

			// XXX TODO premise selection would be done here to
			// determine wheter to BC on a premise

			// Break out any logical link and add to targets
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
			// Add itself back to target, since this is not completely solved
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
		logger().debug("[BackwardChainer] Matched something in knowledge base, "
		               "storying the grounding");

		VarMultimap results;
		bool goal_readded = false;

		for (size_t i = 0; i < kb_match.size(); ++i)
		{
			Handle& soln = kb_match[i];
			VarMap& vgm = kb_vmap[i];

			logger().debug("[BackwardChainer] Looking at grounding "
			               + soln->toShortString());

			// Check if there is any free variables in soln
			HandleSeq free_vars = get_free_vars_in_tree(soln);

			// If there are free variables, add this soln to the target stack
			if (not free_vars.empty())
			{
				// Add the goal back in target list since one of the
				// solution has variables inside
				if (not goal_readded)
				{
					_targets_stack.push(hgoal);
					goal_readded = true;
				}

				_targets_stack.push(soln);
			}

			// Construct the hgoal to all mappings here to be returned
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
 * @param vmap     an output list of mapping for variables in htarget
 * @return         a vector of matched atoms
 */
HandleSeq BackwardChainer::match_knowledge_base(Handle htarget,
                                                vector<VarMap>& vmap)
{
	// Get all VariableNodes (unquoted)
	FindAtoms fv(VARIABLE_NODE);
	fv.search_set(htarget);

	// Unbundle clause if necessary
	HandleSeq terms;
	if (_logical_link_types.count(htarget->getType()))
		terms = LinkCast(htarget)->getOutgoingSet();
	else
		terms.push_back(htarget);

	logger().debug("[BackwardChainer] Matching knowledge base with "
	               " %s and %d variables",
	               htarget->toShortString().c_str(), fv.varset.size());

	// Pattern Match on _as to avoid matching stuff in our garbage space
	SatisfactionLinkPtr sl(createSatisfactionLink(fv.varset, terms));
	BCPatternMatch bcpm(_as);

	sl->satisfy(&bcpm);

	logger().debug("[BackwardChainer] After running pattern matcher");

	vector<map<Handle, Handle>> var_solns = bcpm.get_var_list();
	vector<map<Handle, Handle>> pred_solns = bcpm.get_pred_list();

	HandleSeq results;

	logger().debug("[BackwardChainer] Pattern matcher found %d matches",
	               var_solns.size());

	for (size_t i = 0; i < var_solns.size(); i++)
	{
		// don't want matched clause that is part of a rule
		if (std::any_of(_rules_set.begin(), _rules_set.end(),
		                [&](Rule& r) { return is_atom_in_tree(r.get_handle(), pred_solns[i][htarget]); }))
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
 * @param result     an output VarMap mapping varibles from target to match
 * @return           true if the two atoms can be unified
 */
bool BackwardChainer::unify(const Handle& htarget,
                            const Handle& hmatch,
                            VarMap& result)
{
	logger().debug("[BackwardChainer] starting unify " + htarget->toShortString() + " to " + hmatch->toShortString());

	// lazy way of restricting PM to be between two atoms
	// XXX FIXME the new PM is complaining VariableNode has no type restriction...
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
		result[_garbage_superspace->getAtom(var)] = _garbage_superspace->getAtom(grn);
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
	return rand_element(rules);
}

