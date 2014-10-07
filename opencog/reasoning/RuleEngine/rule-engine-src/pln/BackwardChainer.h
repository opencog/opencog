/*
 * BackwardChainer.h
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
#ifndef BACKWARDCHAINER_H_
#define BACKWARDCHAINER_H_

#include "PLNCommons.h"
#include "BCPatternMatch.h"
#include "Chainer.h"

//#define DEBUG 1
using namespace opencog;
/**
 * Backward chaining falls in to two cases
 *  1.Truth value query - Given a target atom whose truth value is not known and a pool of atoms,find a way
 *    to estimate the truth value of the target Atom,via combining the atoms in the pool  using the inference rule/
 *     eg. The target is "Do people breath"(InheritanceLink people breath)...the truth value of the target is estima
 *     ated via doing the inference "People are animals,animals breathe,therefore people breathe."
 *  2.Variable fulfillment query - Given a target Link(Atoms may be Nodes or Links) with one or more VariableAtoms
 *  among its targets,figure what atoms may be put in place of these VariableAtoms,so as to give the target Link a
 *  hight strength * confidence (i.e "hight truth value")
 *  eg. What breathes( InheritanceLink $X breath ) can be fulfilled by pattern matching
 *   whenever there are are multiple values to fill $X we will use fitness value measure to choose the best
 *  other compound example is what breathes and adds
 *  ANDLink
 *  	InheritanceLink $X Breath
 *  	InheritanceLink $X adds
 *
 *  Anatomy of a single inferene
 *  ============================
 *  A single inference step may be viewed as follows
 *  1.Choose inference Rule R and a tuple of Atoms that collectively match the input condition of the rule
 *  2.Apply choosen rule R to the chosen input Atoms
 *  3.Create an ExecutionLink recording the output found
 *  4.In addition to retaining this ExecutionLink in the Atomspace.also save the copy of it in the InferenceRepository(
 *  this is not needed for the very first implementation,but will be very useful once PLN is in regular use.)
 */
class BackwardChainer: public virtual Chainer {
private:
	friend class BackwardChainerUTest;
	PLNCommons * commons_;
	BCPatternMatch * bcpm_;
	map<Handle, HandleSeq> chaining_result_;
	vector<map<Handle, HandleSeq>> inference_list_;
    HandleSeq bc_generated_rules; //need to be removed at the end of the backward chaining process
	vector<Type> logical_link_types_ = { AND_LINK, OR_LINK }; //xxx any additional link should be reflected in @method join_premise_vgrounding_maps

	/**
	 * Looks if first list input exists in,VARIABLE_NODE the second ,gets that aren't contained in the second list
	 * and returns the first amongst the non contained ones
	 * @param connectors
	 * @param visited
	 * @return
	 */
	Handle get_unvisited_logical_link(HandleSeq& connectors,
			HandleSeq& visited);
	/**
	 * matches the variables in the target to their groundings in the variable grounding map list
	 * @param hgoal the target Handle consisting of variable nodes
	 * @param var_grounding_map a variable to groundings map list
	 * @return  a map of variable to all found groundings
	 */
	map<Handle, HandleSeq> ground_target_vars(Handle& hgoal,
			vector<map<Handle, HandleSeq>>& var_grounding_map);
	/**
	 * maps a variable to its grounding
	 * in failing to do so.
	 * @param htarget the goal
	 */
	map<Handle, HandleSeq> do_bc(Handle& htarget);
	/**
	 * maps a variable to its grounding
	 * in failing to do so.
	 * @param htarget the goal
	 */
	map<Handle, HandleSeq> backward_chain( Handle& htarget, Handle& rule);
	/**
	 * gets the roolt logical link in an implications link
	 * eg. (ImplicationLink (AndLink .....) (...)) AndLink will be the root logical link
	 * @param hrule
	 * @return a handle if there is a root logical link or Handle::UNDEFINED if there is no
	 */
	Handle get_root_logical_link(Handle hrule)
			throw (opencog::InvalidParamException);
	/**
	 * find and return all Handles with no variables
	 * @param handles
	 * @return vectors of handles with no variables in them
	 */
	HandleSeq get_grounded(HandleSeq handles);
	/**
	 * Find and return all handles containing variable and are within an implicationLink
	 * @param handles
	 * @return
	 */
	HandleSeq filter_rules(HandleSeq handles);
	/**
	 *
	 * @param hpremise
	 * @return
	 */
	HandleSeq filter_grounded_experssions(HandleSeq handles);
	/**
	 * Finds rule with their implicand matching the input @param hpremise
	 * @param hpremise
	 * @return a list of rules
	 */
	HandleSeq query_rule_base(Handle hpremise);
	/**
	 * Finds rule with their implicand matching the input @param hpremise
	 * @param hpremise
	 * @return a list of rules
	 */
	HandleSeq query_knowledge_base(Handle hpremise);
	/**
	 * @param connector
	 * @param premise_var_grounding_map
	 * @return a map of variable to groundings
	 */
	map<Handle, HandleSeq> join_premise_vgrounding_maps(const Handle& connector,
			const map<Handle, map<Handle, HandleSeq> >& premise_var_grounding_map);
	/**
	 * maps variable to their groundings given a target handle with variables and a fully grounded matching handle
	 * and adds the result to @param output
	 * @param htarget the target with variable nodes
	 * @param match a fully grounded matching handle with @param htarget
	 * @param output a map object to store results
	 * @return @param output a map of variable to their groundings
	 */
	map<Handle, HandleSeq> unify(Handle& htarget, Handle& match,
			map<Handle, HandleSeq>& output);
	/**
	 * maps @param htarget's variables wiht empty HandleSewq
	 */
	map<Handle, HandleSeq> unify_to_empty_set(Handle& htarget);
	/**
	 *Given a target find a matching rule
	 *@param target handle of the target
	 */
	Handle select_rule(HandleSeq& hseq_rule);
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
	map<Handle, HandleSeq> get_logical_link_premises_map(
			Handle& himplication_link) throw (opencog::InvalidParamException);
	/**
	 * looks for possible grounding of variable node in the entire inference list which was built through the backward chaining process
	 * @param hvar a variable node whose possible values to be searched in the inference list
	 * @param inference_list of variable to possible list of matches(to variableNode or ConceptNode) built in the prev inference steps
	 * @param results a set of grounded nodes found for @param hvar
	 */
	HandleSeq chase_var_values(Handle& hvar,
			vector<map<Handle, HandleSeq>>& inference_list, HandleSeq& results);
    /**
     * calls atomspace to remove each variables and links present the bc_gnerated_rules
     */
	void remove_generated_rules();

#if DEBUG
	void print_inference_list();
	void print_premise_var_ground_mapping(const map<Handle,map<Handle,HandleSeq>>&);
	void print_var_value(const map<Handle,HandleSeq>&);
#endif

public:
	AtomSpace * as_;
	BackwardChainer(AtomSpace * as);
	~BackwardChainer();
	HandleSeq rule_set; //set of matching rules
	std::map<int, HandleSeq> step_inference_map; //for holding inference history

	void do_chain(Handle init_target);
	map<Handle, HandleSeq>& get_chaining_result();
	void choose_rule();
};
#endif /* BACKWARDCHAINER_H_ */
