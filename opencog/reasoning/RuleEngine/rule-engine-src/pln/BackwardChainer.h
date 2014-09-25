/*
 * BackwardChainer.h
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
#ifndef BACKWARDCHAINER_H_
#define BACKWARDCHAINER_H_
#include <opencog/query/AttentionalFocusCB.h>

using namespace opencog;
/*
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
class BackwardChainer: public Chainer, public AttentionalFocusCB {
private:
	/**
	 * Initialize target list
	 */
public:
	HandleSeq target; //set of targets
	HandleSeq rule; //set of matching rules
	/*
	 * Initialize the potential target list atom with initial target atom
	 *Repeat:
	 * 1.Choose the next,target from the potential-target list (via stochastic selection base on weights that
	 *  are assigned to potential targets based on multiple factors)
	 * 2.Choose that rule to apply to the target ( selecting f among the admissbile rules in the rul base via
	 *  stochastic selection base on the wehths fo the rules in the current context).(Non that each rul asom indicates
	 *  a fitness function to be used for its premises base non the fitness function for its conclusion)
	 * 3.Chose premises for the rule via searching the "focus set" specified,suing the pattermatcher(with an appropriate
	 * callback to guie the PM's  search based on the fitness functions for the premises)
	 * 4.Check if one fo the stopping criteria has been met.if so exit the loop and stop repeating.
	 * 5.Push the selected premises an to the  potential target list
	 * 6.REturn to the start of the loop
	 */

	/*once inference has been carried out it can be represented in the Atomspace 
	 * eg.
	 * ExecutionLink?
	 * 	GroundedSchemaNode: PLNDeductionRule
	 * 	ListLink
	 * 		HypotheticalLink
	 * 			InheritanceLink people animal <tv1>
	 * 		HypotheticalLink
	 * 			InheritanceLink animal breath <tv2>
	 * 		HypotheticalLink
	 * 			InheritanceLink people breath <tv3>
	 */
//void record_chaining(RuleRepository applied_rule,HandleSeq args,Handle result);
};

#endif /* BACKWARDCHAINpER_H_ */
