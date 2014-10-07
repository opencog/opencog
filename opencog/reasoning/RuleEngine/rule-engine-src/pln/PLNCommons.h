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

class PLNCommons {
public:
	AtomSpace * as_;
	PLNCommons(AtomSpace *as);
	virtual ~PLNCommons();
	/**
	 * creates a BindLink instance that could be passed to to PatternMatching module
	 * @param himplicant - an implicant part of the BindLink must have a variable node(no checking is done now).
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
	Handle create_with_unique_var(Handle& himplication_link);

	/*
	 * Generate UUID
	 */
	string get_unique_name(Handle& h); //TODO implement this
	/**
	 * create a copy of the handle with a different variable name as depicted @param var_uniq_var map input
	 */
	Handle replace_vars(Handle& h,map<Handle,Handle>& var_uniq_var_map);
};

#endif /* PLNCOMMONS_H_ */
