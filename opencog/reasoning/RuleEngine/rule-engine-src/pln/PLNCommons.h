/*
 * PLNCommons.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Oct 2014
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
#ifndef PLNCOMMONS_H_
#define PLNCOMMONS_H_

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/types.h>

using namespace std;
using namespace opencog;
/**
 * Reusable functions in the PLN module
 */
class PLNCommons {
public:
	AtomSpace * as_;
	const float FITNESS_PARAM = 0.9;
	PLNCommons(AtomSpace *as);
	virtual ~PLNCommons();
	/**
	 * creates a BindLink instance that could be passed to to PatternMatching module
	 * @param himplicant - an implicant part of the BindLink must have a variable node.
	 * @return - a Handle to the BindLink instance created
	 */
	Handle create_bindLink(Handle himplicant, bool is_quoted = false)
			throw (opencog::InvalidParamException);
	/**
	 * Given an atom (a link or node), Find and return all the nodes associated
	 * @param hinput - an atoms to be looked
	 * @param required_nodes - a list of nodes to look for. if vector is empty, all kinds of nodes are looked
	 * @return - a set of nodes
	 */
	UnorderedHandleSet get_nodes(const Handle& hinput,
	                             vector<Type> required_nodes);
	/**
	 * checks if a handle already exists in a HandleSeq
	 */
	bool exists_in(Handle& hlink, Handle& h);
	/**
	 *wraps all variables of the input handle in a QuoteLink. TODO adding flexibility for wrapping particular variables only
	 *@param himplicant a Handle containing variables
	 *@param as the atomspase containing himplicant
	 */
	Handle create_quoted(Handle himplicant);
	/**
	 * Remove created bind link and quote links inf exist
	 * @param hbindLink a reference to the bindLink to be removed
	 */
	void clean_up_bind_link(Handle& hbindLink)
			throw (opencog::InvalidParamException);
	/**
	 * Remove implication link created with unique vars
	 */
	void clean_up_implication_link(Handle& himplication_link)
			throw (opencog::InvalidParamException);
	/**
	 * Remove a variable node. All the incoming sets are also removed in a recursive manner.
	 * @param h a var node to be deleted
	 */
	void remove_vnode_containing_links(Handle& h);
	/**
	 * create a copy of the given implicatoin link with unique variables
	 */
	Handle replace_nodes_with_varnode(Handle& himplication_link, Type t =
			VARIABLE_NODE);

	/*
	 * Generate UUID
	 */
	string get_unique_name(Handle& h); //TODO implement this
	/**
	 * create a copy of the handle with a different variable name passed in @param var_uniq_var map input
	 */
	Handle change_node_types(Handle& h, map<Handle, Handle>& replacement_map);
	/**
	 * Get top level parent of the handle
	 * @return a Link or Handle::UNDEFINED if there is no
	 */
	void get_root_links(Handle h, HandleSeq& parents);
	template<class Type> Type tournament_select(map<Type, float> tfitnes_map) {
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
	/**
	 * calculates fitness values in target_list_atom_space using the formula F = s^x * c^(2-x)
	 * where s is strength,c is confidence and x is some fixed value
	 * @param h - a handle
	 * @return a fitness value
	 */
	float target_tv_fitness(Handle h);
};

#endif /* PLNCOMMONS_H_ */
