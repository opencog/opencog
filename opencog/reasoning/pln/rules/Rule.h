/*
 * Copyright (C) 2008 by OpenCog Foundation
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
#include "../AtomSpaceWrapper.h"

#define NO_DIRECT_PRODUCTION Btr<std::set<BoundVertex > > \
    attemptDirectProduction(meta outh, bool fresh = false) const {  \
        return Btr<std::set<BoundVertex> >(); \
    }

namespace opencog { namespace pln {

/** Rule is a superclass for types of inference rules.
 *
 * In determining the proper input on the basis of getting a specific output:
 *
 * For some rules, the exact input atom can be determined.  For other rules,
 * only constraints on the atom properties can be determined.  Eg. And(A, B, C)
 * can be derived from And(A,B) & And(B,C) etc. ie. from any collection of
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
     *
     * Note that the inputFilter is not used when making child nodes in the BIT.
     * See o2iMetaExtra for that. It's only used in forward chaining. See also
     * i2oType in GenericRule.
     */
    MPsIn inputFilter;
    // mutable std::map<atom, setOfMPs, lessatom_ignoreVarNameDifferences> o2iMetaCache;

    //! If <tt>freeInputArity == true</tt>, then the # of args is not pre-determined.
    bool freeInputArity;

    //! Atom table interface
    AtomSpaceWrapper* asw;

    //! Whether the the Rule is a Composer.
    //! If it is a Composer then it needs premises to derive the truth value
    //! of the conclusion, otherwise it is called a Generator
    bool composer;

    //! Priority affects the order in which rules are preferred when doing inference.
    // the higher the priority value is the higher priority the rule has
    float priority;
    
    /**
     * Each subclass will have its generic requirements for input, stored in
     * its \c inputFilter, plus extra requirements for each desired output. The
     * extras are given by this method.
     *
     * If the extra filter includes the restrictions on the input filter too,
     * then the method should set overrideInputFilter True on return.
     *
     * The above may be incorrect (see the doc for o2iMeta()) -- JaredW.
     *
     * @param outh The MetaPredicate describing the desired output for this
     * Rule.
     * @param overrideInputFilter Whether the method overrides inputFilter.
     *                            if it is unchanged then it is considered false
     * @return The extra requirements filter.
     */
    virtual setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const=0;
    // virtual MPs* o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const=0;

    virtual meta targetTemplate() const { return meta( (vtree*)NULL); }

public:
    /** Rule constructor
     * @param _asw Pointer to the AtomSpace interface.
     * @param _freeInputArity Are the number of arguments predetermined?
     * @param _composer Whether the rule has premises.
     * @param _name The name of the rule.
     */
    Rule(AtomSpaceWrapper *_asw,
         bool _freeInputArity,
         bool _composer,
         std::string _name = "");
    
    Rule();

    virtual ~Rule();

    /** The generic rule computation method.  This method must be implemented in
     * subclasses.  If the rule can take the args in any order, the \c Rule
     * object must take care of ordering them.
     *
     * @param h The vertices to compute on.
     * @param CX Context to use for rule computation. Currently unused.
     * @param fresh allows atoms to be added with the same name/outgoing set.
     *              If fresh == false and the atom already exist then the new
     *              truth value is merged (via TruthValue::merge) with the old.
     *              Otherwise (fresh == true) then a new dummy context
     *              is associated to that new truth value.
     * @todo A future implementation may include 'bool ordered_already'
     * parameter to speed up.
     */
    virtual BoundVertex compute(const VertexSeq& h,
                                pHandle CX = PHANDLE_UNDEFINED,
                                bool fresh = true) const=0;

    /** Perform some checks then run the other version of compute. Note that
     * this is the method that is actually called by the back and forward
     * chainers.
     * @see Rule::compute(const VertexSeq& h, pHandle CX = PHANDLE_UNDEFINED)
     */
    BoundVertex compute(const std::vector<BoundVertex>& h,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;

    //! Try to call rule as a direct producer (Generator)
    virtual Btr<std::set<BoundVertex> > attemptDirectProduction(meta h,
                                                                bool fresh = true) const=0;

    //! Just calls compute()
    BoundVertex operator() (const VertexSeq h,
                            pHandle CX = PHANDLE_UNDEFINED,
                            bool fresh = true) const
        { return compute(h,CX); }

    /** Check validity.
     * If this fails, then it re-orders the premises in order to gain it.
     * (not sure about this ???)
     *
     * @param h The vertices to check validity for.
     * @return Whether the provided vertices fit the rule requirements or not.
     */
    bool validate(const VertexSeq& h) const;

    /** ARI: Another alternative for checking validity. Only used by deduction,
     * to avoid things like "Imp A A" where an atom implies itself. This method
     * should be refactored out.
     *
     * @param args The vertices to check validity for.
     * @return Whether the provided vertices fit the rule requirements or not.
     */
    virtual bool validate2(MPs& args) const=0;

    /** Only perform compute if arguments are valid. ???
     *
     * @param h The vertices to compute the rule for
     * @param CX Context to use for rule computation. Currently unused.
     * @return The result of the rule being computed.
     */
    BoundVertex computeIfValid (const VertexSeq& h,
                                pHandle CX = PHANDLE_UNDEFINED,
                                bool fresh = true) const;

    //Handle compute(Handle h1) const;
    //Handle compute(Handle h1, Handle h2) const;
    //Handle compute(Handle h1, Handle h2, Handle h3) const;

    /** Called while the BIT is determining whether to create a new child node
     * with this Rule. Used by BITNode::expandRule(). Determines the target for
     * the child node.
     *
     * Calls the o2iMetaExtra method in the subclass.
     * If that method sets overrideInputFilter to true, this returns its result.
     * Otherwise, this returns an empty set - that is, there are no possible
     * child nodes in this case.
     *
     * @param outh a metapredicate specifying the target for that BITNode.
     * @return a set of vectors of MetaPredicates. Each vector is one possible
     * input, with a MetaPredicate for each input slot (maybe -- JaredW).
     */
    setOfMPs o2iMeta(meta outh) const;

    //! Get the inputFilter for this Rule.
    MPsIn& getInputFilter() { return inputFilter; }

    virtual setOfMPs fullInputFilter() const;

    //! Does the Rule have unfixed input arity?
    bool hasFreeInputArity() const { return freeInputArity; }
    bool isComposer() const { return composer; }

    /** Copy a bunch of vertices from src to dest ???
     *
     * @param src Source
     * @param dest Destination
     */
    static void CloneArgs(const Rule::MPs& src, Rule::MPs& dest);

    friend class RuleApp;

    // Used for logging purposes and retrieving rule based on its name
    std::string name; 

    const std::string& getName() const { return name; }
};


/**
 * Note: enum RULE is currently unused
 */
enum RULE
{
    PLN_AND,
    SimpleAnd,
    SimpleAnd2,
    SimpleAnd3,
    SimpleAnd4,
    SimpleAnd5,
    ForAll,
    PLNPredicate,

    OrPartition,
    Or,

    AndPartition,

    NotEvaluation,
    UnorderedLinkPermutation,

    AndBreakdown1,
    AndBreakdown2,
    AndBreakdown3,
    AndBreakdown4,
    AndBreakdown5,
    OrBreakdown,

    StrictImplicationBreakdown,
    ImplicationBreakdown,
    ImplicationConstruction,
    ImplicationTailExpansion,

    VariableInstantiation,

    ScholemFunctionProduction,
    
    ChildSubstitution,

    NotElimination,

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

    Or2And,
    Exist2ForAll,
    Exist
};

typedef boost::shared_ptr<Rule> RulePtr;

//! Total number of rules in PLN (Not necessarily the number that are active
//! though).
static const int NUMBER_OF_RULES (((int)Exist)+1); // Exist being the last rule!

}} // namespace opencog { namespace pln {

#endif // _INFERENCE_RULE_H
