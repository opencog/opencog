/*
 * ForwardChainer.h
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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
#ifndef FORWARDCHAINER_H_
#define FORWARDCHAINER_H_

#include "Chainer.h"
#include "ForwardChainInputMatchCB.h"
#include "ForwardChainPatternMatchCB.h"

#include <opencog/guile/SchemeEval.h>

using namespace std;

class ForwardChainPatternMatchCB;
class ForwardChainInputMatchCB;
class ForwardChainer: public virtual Chainer {
private:
	friend class ForwardChainerUTest;
	int ITERATION_SIZE;
	bool search_in_af; // = true;

	friend class ForwardChainInputMatchCB;
	friend class ForwardChainPatternMatchCB;
	HandleSeq target_list_; /*potential target list*/
	HandleSeq chaining_results;

	HandleSeq prev_chosen_targets;
	vector<string> bind_link_name_;  // the variable name assigned to a BindLink
	//  eg. (define human (BindLink ...)human is the variable name
	Handle hcurrent_choosen_rule_; // = Handle::UNDEFINED; //Handle to chosen BindLink on a praticular step of forward chaining

	ForwardChainInputMatchCB * fcim_;
	ForwardChainPatternMatchCB * fcpm_;
	SchemeEval * scm_eval_;
	/**
	 * chooses a set of nodes to be replaced by a VariableNode with a name for each
	 * @param htarget - the target from which nodes are chosen to be replaced by VariableNode
	 * @return a map of Handle to a node to be replaced by VariableNode and name for the replacing VariableNode
	 */
	map<Handle, string> choose_variable(Handle htarget);
	/**
	 * Given an atom (a link or node), Find and return all the nodes associated
	 * @param hinput - an atoms to be looked
	 * @param required_nodes - a list of nodes to look for. if vector is empty, all kinds of nodes are looked
	 * @return - a set of nodes
	 */
	vector<Handle> get_nodes(Handle hinput, vector<Type> required_nodes);
	/**
	 * create a BindLink instance that could be passed to to PatternMatching module
	 * @param himplicant - an implicant part of the BindLink must have a variable node(no checking is done now).
	 * @return - a Handle to the BindLink instance created
	 */
	Handle create_bindLink(Handle himplicant)
			throw (opencog::InvalidParamException);
	/**
	 * a callback handler for forward chaining invocation from scm shell
	 */
	void do_forward_chaining(Handle hinitial_target);
	/**
	 * checks if a handle already exists in a HandleSeq
	 */
	bool exists(HandleSeq& hseq, Handle& h);
	/**
	 *Initialize the forward chaining listener
	 */
	void init(void);
	/**
	 * tournament selection xxx tournament size is 50% of input
	 * @param hfitness_map - a handle fitness map
	 */
	Handle tournament_select(map<Handle, float> hfitness_map);
	/**
	 * check if a handle is in the potential target list
	 * @param h A handle to be looke in the target list
	 */
	bool is_in_target_list(Handle h);
public:
	ForwardChainer(AtomSpace * as);
	~ForwardChainer();
	/**
	 * Converts a target link or node to a variable containing link or variable replaced node
	 * and creats the new node in the target_list_atom_space
	 * eg. InheritanceLink
	 * 	      ConceptNode Cat
	 * 	      ConceptNode Animal
	 * 	   lets choose cat to be changed by VariableNode $x
	 * 	   			InheritanceLink
	 * 	   				ConceptNode $x
	 * 	   				ConceptNode Animal
	 * @param htarget - the input atom to be converted
	 * @param vname_vnode_map - a set of tuples of name string and node to be replaced by VariableNode
	 * @return - a new atom(node or link) created by changing part of htarget to variable nodes
	 */
	Handle target_to_pmimplicant(Handle htarget,
			map<Handle, string> vname_vnode_map);
	/**
	 * choose next target from potential target list based on some factors
	 *  (xxx currently based on fitness) other factors might be included later
	 */
	Handle choose_target_from_list(HandleSeq hs_list);
	/**
	 * choose a random target to start forward chaining with. This is useful when there is no target
	 * specified ahead to the forward chaining process.
	 * @param as - the atomspace instance from which target is selected
	 */
	Handle choose_target_from_atomspace(AtomSpace *);
	/**
	 * chaining main entry point
	 * @param htarget - the target atom which the forward chaining will be, if htarget is Handle::UNDEFINED
	 *                  chaining will be over the entire atomspace
	 */
	void do_chain(Handle htarget);
	/**
	 * Choose additional premises for the rule via one of two options: A) searching the focus set
	 * specified, or B) searching the potential-target list itself. In each case, search proceeds using
	 * the PatternMatcher (with an appropriate callback to guide the PMs search based on the
	 * fitness functions for the additional premises)
	 */
	void choose_input(Handle target);
	/**
	 * chooses a bindLink name from the available set of BindLinks in  the bind_link_name and then executes
	 * the scm command (cog-fc-bind choosen_bindLink) so that the handle object to the bindLink name will be
	 * stored in hcurrent_bind_link
	 */
	void choose_rule(void);
	/**
	 * Loads all configuration information regarding the forward chaining process
	 * including loading of chaining rules to target_list_atom_space
	 */
	void load_fc_conf();
	/**
	 *returns the handle of a BindLink instance Given the scheme variable name for a bindLink which
	 *returns is already loaded in to the atomspace
	 *@param name - the scheme variable name for the BindLink epxpression
	 *@return - a handle to the BindLink instance
	 */
	Handle get_hbindLink(string& name);
	/**
	 * adds a reference to a a node or a a link and its member nodes in the target_list
	 * @param h- a handle to a node or a link
	 */
	void add_to_target_list(Handle h);
	/**
	 *returns the inferences made
	 */
	HandleSeq get_chaining_result(void);
};

#endif /* FORWARDCHAINER_H_ */

