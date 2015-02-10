/*
 * PLNCommons.h
 *
 *  Created on: 9 Oct, 2014
 *      Author: misgana
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
	PLNCommons(AtomSpace *as);
	virtual ~PLNCommons();
	/**
	 * creates a BindLink instance that could be passed to to PatternMatching module
	 * @param himplicant - an implicant part of the BindLink must have a variable node.
	 * @return - a Handle to the BindLink instance created
	 */
	Handle create_bindLink(Handle himplicant,bool is_quoted = false) throw (opencog::InvalidParamException);
	/**
	 * Given an atom (a link or node), Find and return all the nodes associated
	 * @param hinput - an atoms to be looked
	 * @param required_nodes - a list of nodes to look for. if vector is empty, all kinds of nodes are looked
	 * @return - a set of nodes
	 */
	HandleSeq get_nodes(const Handle& hinput, vector<Type> required_nodes);
	/**
	 * checks if a handle already exists in a HandleSeq
	 */
	bool exists(HandleSeq hseq, Handle h);
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
	void clean_up_bind_link(Handle& hbindLink) throw (opencog::InvalidParamException);
	/**
	 * Remove implication link created with unique vars
	 */
	void clean_up_implication_link(Handle& himplication_link) throw (opencog::InvalidParamException);
	/**
	 * Remove a variable node. All the incoming sets are also removed in a recursive manner.
	 * @param h a var node to be deleted
	 */
	void remove_vnode_containing_links(Handle& h);
	/**
	 * create a copy of the given implicatoin link with unique variables
	 */
	Handle replace_nodes_with_varnode(Handle& himplication_link,Type t=VARIABLE_NODE);

	/*
	 * Generate UUID
	 */
	string get_unique_name(Handle& h); //TODO implement this
	/**
	 * create a copy of the handle with a different variable name passed in @param var_uniq_var map input
	 */
	Handle change_node_types(Handle& h, map<Handle, Handle>& replacement_map,
			Type t) {
		Handle hcpy;
		if (LinkCast(h)) {
			HandleSeq hs = as_->getOutgoing(h);
			HandleSeq hs_cpy;
			for (Handle hi : hs) {
				if (NodeCast(hi)) {
					if (t == NODE || as_->getType(hi) == t)
						hs_cpy.push_back(replacement_map[hi]);
					else
						hs_cpy.push_back(hi);
				} else if (LinkCast(hi)) {
					hs_cpy.push_back(change_node_types(hi, replacement_map, t));
				}
			}
			hcpy = as_->addLink(as_->getType(h), hs_cpy, as_->getTV(h));
		} else if (NodeCast(h)) {
			if (t == NODE || as_->getType(h) == t)
				hcpy = replacement_map[h];
			else
				hcpy = h;
		}

		return hcpy;
	}
	/**
	 * Get top level parent of the handle
	 * @return a Link or Handle::UNDEFINED if there is no
	 */
	void get_top_level_parent(Handle h,HandleSeq parents);
};

#endif /* PLNCOMMONS_H_ */
