/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * Written by   Ari Heljakka <heljakka@gmail.com>
 *              Joel Pitt <joel@fruitionnz.com>
 * All Rights Reserved
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
#ifndef _INFERENCE_RULE_H
#define _INFERENCE_RULE_H

#include <opencog/atomspace/TruthValue.h>
#include "../PLN.h"
#include "../PLNUtils.h"

const TruthValue& getTV(pHandle); 
#define NO_DIRECT_PRODUCTION Btr<std::set<BoundVertex > > \
    attemptDirectProduction(meta outh) { \
        return Btr<std::set<BoundVertex> >(); \
    }

namespace opencog { namespace pln {

/** Rule is a superclass for types of inference rules.
 *
 * In determining the proper input on the basis of getting a specific output:
 *
 * For some rules, the exact input atom can be determined.  For other rules,
 * only constraints on the atom properties can be determined.  Eg. AND(A, B, C)
 * can be derived from AND(A,B) & AND(B,C) etc. ie. from any collection of
 * AndLinks with A, B, and C occurring somewhere. This can only be expressed as
 * a MP (metapredicate).
 */
class Rule
{
public:
    // Maximum number of inputs a rule will ever take?
    const int RULE_INPUT_ARITY_MAX;

    typedef std::vector<BBvtree> MPs;
    typedef std::set<MPs> setOfMPs;

    typedef std::vector<meta> MPsIn;
    typedef std::set<MPsIn> setOfMPsIn;

protected:
    /**
     * The <tt>i</tt>th entry in the inputFilter contains the vtree that
     * describes what a valid input atom is for argument \c i. In the outgoing
     * tree, a bare ATOM designates a wild card:
     *
     * e.g. if the root is a NOT_LINK and it has 1 child which is just ATOM,
     * then it matches to any NOT_LINK with 1 child. The ATOM can also be at the
     * root, in which case any atom will do.
     */
    MPsIn inputFilter;
    // mutable std::map<atom, setOfMPs, lessatom_ignoreVarNameDifferences> o2iMetaCache;

    //! If <tt>freeInputArity == true</tt>, then the # of args is not pre-determined.
    bool freeInputArity;

    //! Atom table interface
    iAtomSpaceWrapper *destTable;

    //! Whether the the Rule can be computed?
    bool computable;

    //! Priority affects the order in which rules are preferred when doing inference.
    float priority;
    
    /**
     * Each subclass will have its generic requirements for input, stored in
     * its \c inputFilter, plus extra requirements for each desired output. The
     * extras are given by this method.
     *
     * If the extra filter includes the restrictions on of the input filter too,
     * then the method should set overrideInputFilter True on return.
     *
     * @param outh ???
     * @param overrideInputFilter Whether the method overrides inputFilter.
     * @return The extra requirements filter.
     */
    virtual std::set<MPs> o2iMetaExtra(meta outh, bool& overrideInputFilter) const=0;
    // virtual MPs* o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const=0;

public:
    /** Rule constructor
     * @param _destTable Pointer to the AtomTable interface.
     * @param _freeInputArity Are the number of arguments predetermined?
     * @param _computable Whether the rule can be computed?
     * @param _name The name of the rule.
     */
    Rule(iAtomSpaceWrapper *_destTable,
        bool _freeInputArity,
        bool _computable,
        std::string _name = "");

	Rule();

    virtual ~Rule();

    /** Get the priority of the rule for use by inference heuristics.
     *
     * @return The Rule's priority.
     */
    float getPriority() const { return priority; }

    /** Set the Rule's priority that is used by inference heuristics.
     *
     * @param _priority The Rule's priority.
     */
    void setPriority(float _priority) { priority = _priority; }

    /** The generic rule computation method.  This method must be implemented in
     * subclasses.  If the rule can take the args in any order, the \c Rule
     * object must take care of ordering them.
     *
     * @param h The vertices to compute on.
     * @param CX Context to use for rule computation. Currently unused.
     * @todo A future implementation may include 'bool ordered_already'
     * parameter to speed up.
     */
    virtual BoundVertex compute(const std::vector<Vertex>& h, pHandle CX = PHANDLE_UNDEFINED) const=0;

    //! A computation method on BoundVertex
    //! @see Rule::compute(const std::vector<Vertex>& h, pHandle CX = PHANDLE_UNDEFINED)
    BoundVertex compute(const std::vector<BoundVertex>& h, pHandle CX = PHANDLE_UNDEFINED) const;

    //! Try to call rule as a direct producer
    virtual Btr<std::set<BoundVertex> > attemptDirectProduction(meta h)=0;

    //! Just calls compute()
    BoundVertex operator() (const std::vector<Vertex> h, pHandle CX = PHANDLE_UNDEFINED) const
        { return compute(h,CX); }

    /** Check validity.
     * If this fails, then it re-orders the premises in order to gain it.
     * (not sure about this ???)
     *
     * @param h The vertices to check validity for.
     * @return Whether the provided vertices fit the rule requirements or not.
     */
    bool validate(const std::vector<Vertex>& h) const;

    /** ARI: Another alternative for checking validity. ???
     *
     * @param args The vertices to check validity for.
     * @return Whether the provided vertices fit the rule requirements or not.
     */
    virtual bool validate2 (MPs& args) const=0;

    /** Only perform compute if arguments are valid. ???
     *
     * @param h The vertices to compute the rule for
     * @param CX ???
     * @return The result of the rule being computed.
     */
    BoundVertex computeIfValid (const std::vector<Vertex>& h, pHandle CX = PHANDLE_UNDEFINED) const;

    //Handle compute(Handle h1) const;
    //Handle compute(Handle h1, Handle h2) const;
    //Handle compute(Handle h1, Handle h2, Handle h3) const;

    /** ???
     * @param meta ???
     * @return ???
     */
    setOfMPs o2iMeta(meta outh) const;

    //! Get the inputFilter for this Rule.
    MPsIn& getInputFilter() { return inputFilter; }

    //! Does the Rule have unfixed input arity?
    bool hasFreeInputArity() const { return freeInputArity; }
    bool isComputable() const { return computable; }

    /** Copy a bunch of vertices from src to dest ???
     *
     * @param src Source
     * @param dest Destination
     */
    static void CloneArgs(const Rule::MPs& src, Rule::MPs& dest);

    friend class RuleApp;

    //! Only for logging purposes.
    std::string name; 
};


/**
 * Note: enum RULE is currently unused
 */
enum RULE
{
    PTL_AND,
    SimpleAND,
    SimpleAND2,
    SimpleAND3,
    SimpleAND4,
    SimpleAND5,
    ForAll,
    PLNPredicate,

    ORPartition,
    OR,

    ANDPartition,

    NotEvaluation,
    UnorderedLinkPermutation,

    ANDBreakdown1,
    ANDBreakdown2,
    ANDBreakdown3,
    ANDBreakdown4,
    ANDBreakdown5,
    ORBreakdown,

    StrictImplicationBreakdown,
    ImplicationBreakdown,
    ImplicationConstruction,
    ImplicationTailExpansion,

    VariableInstantiation,

    ScholemFunctionProduction,
    
    ChildSubstitution,

    NOTElimination,

    Deduction_Implication,
    Deduction_Inheritance,
    Inversion_Implication,
    Inversion_Inheritance,
    Revision,

    MetaPredicateExecution,

    SubSetEval_ConceptNode,

    Equi2Impl,
    Equi2Sim,
    Inh2Imp,
    Inh2Sim,
    Inh2Eval,
    Sim2Inh,
    Mem2Inh,
    Mem2Eval,
    Imp2Inh,

    IntImp2Ext,
    IntInh2Ext,
    ExtImp2Int,
    ExtInh2Int,
    ExtImpl2Subset,
    ExtEqui2ExtSim,
    
    CrispTheorem,

    Lookup,
    Hypothesis,

    CrispUnification,
    StrictCrispUnification,

// Irrelevant Rules (the ones only used at startup, plus tautology)
    Tautology,

    OR2AND,
    Exist2ForAll,
    Exist
};

//! Total number of rules in PLN (Not necessarily the number that are active
//! though).
static const int NUMBER_OF_RULES (((int)Exist)+1); // Exist being the last rule!

}} // namespace opencog { namespace pln {

#endif // _INFERENCE_RULE_H
