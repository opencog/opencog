/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#include "../PLN.h"
#include "Rules.h"
#include "RuleProvider.h"

#include <opencog/util/Logger.h>

#include <boost/foreach.hpp>

namespace opencog { namespace pln {

RuleProvider::RuleProvider(void)
{
}

template<typename T>
void delete_op(T* r) { delete r; } 

void RuleProvider::AddRule(Rule* r, float priority)
{
    push_back(r);
    r->setPriority(priority);
}

RuleProvider::~RuleProvider(void)
{
    for_each(begin(), end(), &delete_op<Rule>);
}

VariableRuleProvider::VariableRuleProvider(void)
{
}

VariableRuleProvider::~VariableRuleProvider(void)
{
}
/*
void VariableRuleProvider::CreateCustomCrispUnificationRules()
{
iAtomSpaceWrapper* parent = ASW();
	
}
*/
DefaultVariableRuleProvider::DefaultVariableRuleProvider(void)
{
    
    iAtomSpaceWrapper* parent = ASW();
    
    Btr<std::set<pHandle> > ForAll_handles = parent->getHandleSet(FORALL_LINK, "");
    
    foreach(pHandle fah, *ForAll_handles)
        AddRule(new CustomCrispUnificationRule(fah, parent), 7.5f);
    
    cprintf(-1, "Added %u CrispUnificationRules.\n", (unsigned int) size());
    
    AddRule(new LookupRule(parent), 20.0f);

/// StrictCrispUnification always requires Hypothesis, too!	
//	AddRule(new StrictCrispUnificationRule(parent), 7.5f);
//	AddRule(new CrispUnificationRule(parent), 7.5f); ///Alternative implementation

    float ANDEvaluatorPriority = 10.0f;
/// haxx:: \todo ANDRule sometimes confuses the order of atoms in the 
/// outgoing vector of the resulting ANDLink. Ie. the order is not the same
/// as the order in which the arguments were inputted. Eg. compute(a, b) may give
/// AND(b,a). This is not acceptable because all PLN code assumes that the ANDLinks
/// are ordered properly. This is especially necessary when ANDLinks are used
/// as SequentialANDLinks, but there is another basic cause for it, too.
//	AddRule(new ANDRule(parent), ANDEvaluatorPriority);

    AddRule(new ORRule(parent), 10.0f);
    
    AddRule(new SimpleANDRule<1>(parent), ANDEvaluatorPriority - 1.0f);
    AddRule(new SimpleANDRule<2>(parent), ANDEvaluatorPriority - 1.1f);
    AddRule(new SimpleANDRule<3>(parent), ANDEvaluatorPriority - 1.2f);
    //	AddRule(new SimpleANDRule<4>(parent), ANDEvaluatorPriority - 1.3f);
    //	AddRule(new SimpleANDRule<5>(parent), ANDEvaluatorPriority - 1.4f);
    
    AddRule(new ANDPartitionRule(parent), 10.0f);
    AddRule(new NotEvaluatorRule(parent), 10.0f);
    
    AddRule( new ScholemFunctionProductionRule(parent), 20.0f);
    //	AddRule(new FORALLRule(parent,NULL), 5.0f);
    //	AddRule( new PLNPredicateRule(parent,NULL), 5.0f);
    
    //	AddRule(new ImplicationBreakdownRule(parent), 9.0f);
    AddRule(new StrictImplicationBreakdownRule(parent), 9.0f);
    
    //	AddRule(new ImplicationTailExpansionRule(parent), 10.0f);
    //	AddRule(new ImplicationConstructionRule(parent), 10.0f);
//	AddRule(new InversionRule<IMPLICATION_LINK>(parent), 7.0f);
    //AddRule(new DeductionRule<DeductionSimpleFormula, IMPLICATION_LINK>(parent), 8.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(parent, IMPLICATION_LINK), 8.0f);
    //AddRule(new InversionRule<INHERITANCE_LINK>(parent), 7.0f);
    AddRule(new InversionRule(parent, INHERITANCE_LINK), 7.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(parent, INHERITANCE_LINK), 8.0f);
    
    //	AddRule(new ORPartitionRule(parent), 10.0f);
    AddRule(new CrispTheoremRule(parent), 10.0f);
    
    AddRule(new Int2ExtRule(parent, IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    AddRule(new Int2ExtRule(parent, INHERITANCE_LINK, EXTENSIONAL_INHERITANCE_LINK), 10.0f);
    AddRule(new Ext2IntRule(parent, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    AddRule(new Ext2IntRule(parent, EXTENSIONAL_INHERITANCE_LINK, INHERITANCE_LINK), 10.0f);
    
    AddRule(new HypothesisRule(parent), 30.0f);
    //	AddRule(new SimSubstRule1(parent), 5.0f);
    
    /* The rest of the Rules have rarely or never been used. Some of them just won't work. */
    
    /*	AddRule(new UnorderedLinkPermutationRule(parent), 10.0f);
	AddRule(new VariableInstantiationRule(parent), 10.0f);
	AddRule(new NOTEliminationRule(parent), 10.0f
	AddRule(new Equi2ImplRule(parent), 10.0f
	AddRule(new Equi2Sim(parent), 10.0f;
	AddRule(new Inh2SimRule(parent), 10.0f;
	AddRule(new Sim2InhRule(parent), 10.0f;

	AddRule(new RevisionRule(parent);	
	AddRule(new MetaPredicateExecutionRule(parent);
	AddRule(new SubSetEvalRule<CONCEPT_NODE>(parent)
	AddRule(new Equi2SimRule(parent);
	AddRule(new Mem2InhRule(parent);
	AddRule(new Inh2ImpRule(parent);
	AddRule(new Imp2InhRule(parent);
	AddRule(new Mem2EvalRule(parent);
	AddRule(new Inh2EvalRule(parent);

	AddRule(new ExtImpl2SubsetRule(parent);
	AddRule(new ExtEqui2ExtSimRule(parent);
	AddRule(new TautologyRule(parent);
	AddRule(new OR2ANDRule(parent);
	AddRule(new Exist2ForAllRule(parent);
	AddRule(new ExistRule(parent);
*/
}

DefaultVariableRuleProvider::~DefaultVariableRuleProvider(void)
{
}

//template<typename FormulaType>
#define FormulaType InversionFormula
class GenericRule2 : public Rule
{
protected:
    mutable FormulaType formula;

public:
    virtual std::set<MPs> o2iMetaExtra(meta outh, bool& overrideInputFilter) const=0;
    //	virtual TruthValue** formatTVarray	(const vector<Vertex>& premiseArray, int* newN) const=0;
    
    ~GenericRule2() {}
    /// Always a Composer
    GenericRule2(iAtomSpaceWrapper *_destTable,
                 bool _FreeInputArity, std::string _name = "")
	: Rule(_destTable, _FreeInputArity, true, _name) {	}
    
    BoundVertex compute(const std::vector<Vertex>& premiseArray, Handle CX = Handle::UNDEFINED) const
    {
        return Vertex();
    }
    NO_DIRECT_PRODUCTION;
};

//template<Type InclusionLink>

#define InclusionLink 13
class InversionRule2 : public GenericRule2 //<InversionFormula>
{
protected:
    std::vector<Type> ti;
    
    ~InversionRule2() {}
    TruthValue** formatTVarray(const std::vector<Vertex>& premiseArray, int* newN) const
    {
        TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[3];
        
        return tvs;
    }
    std::vector<BoundVertex> r;
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        Rule::MPs ret;
        
        return makeSingletonSet(ret);
    }
    
public:
    InversionRule2(iAtomSpaceWrapper *_destTable)
	: GenericRule2/*<InversionFormula>*/ (_destTable, false, "InversionRule")
    {
    }
    bool validate2				(MPs& args) const { return true; }
    
    virtual meta i2oType(const std::vector<Vertex>& h) const
    {
        return	meta(new tree<Vertex>());
    }
    NO_DIRECT_PRODUCTION;
};
        
        
ForwardChainerRuleProvider::ForwardChainerRuleProvider(void)
{
    iAtomSpaceWrapper* parent = ASW();
    //AddRule(new InversionRule<INHERITANCE_LINK>(parent), 7.0f);
    //AddRule(new DeductionRule<DeductionSimpleFormula, IMPLICATION_LINK>(parent), 8.0f);
    //AddRule(new DeductionRule<DeductionSimpleFormula, INHERITANCE_LINK>(parent), 8.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(parent, ASSOCIATIVE_LINK), 8.0f);
    reset();
}

ForwardChainerRuleProvider::~ForwardChainerRuleProvider(void)
{

}

void ForwardChainerRuleProvider::reset()
{
    invalidRules.clear();
    current = NULL;
}

void ForwardChainerRuleProvider::setSeed(pHandle s)
{
    reset();
    seed = s;
}

Rule* ForwardChainerRuleProvider::findHighestPriorityRule()
{
    // get the highest priority rule that isn't in invalidRules;

    // if invalid and rule provider have the same number, then no more
    // rules available, return NULL
    if (invalidRules.size() == size()) {
        return NULL;
    }

    float highestPriority = 0.0f;
    float highestIndex = -1;

    for (unsigned int i=0; i < size(); i++) {
        if (find(invalidRules.begin(), invalidRules.end(), at(i))
                != invalidRules.end())
            continue;
        float p = at(i)->getPriority();
        if (p > highestPriority) {
            highestPriority = p;
            highestIndex = i;
        }
    }
    if (highestIndex >= 0)
        return at(highestIndex);
    else
        return NULL;

}

unsigned int ForwardChainerRuleProvider::getSeedIndex() { return seedIndex; }

Rule* ForwardChainerRuleProvider::nextRule()
{
    // 1. check seed is set, check if current has a Rule, if so
    // add it to invalidRules, set to NULL.
    if (seed == PHANDLE_UNDEFINED) {
        opencog::logger().warn("No seed set, so can't return an appropriate Rule via nextRule.");
        return NULL;
    }
    if (current) {
        invalidRules.push_back(current);
        current = NULL;
    }
    
    // 2. find highest priority rule that isn't in invalid rules
    Rule *r = findHighestPriorityRule();
    
    // 3. check if current seed can fit in rule, otherwise add to
    // invalid rules and goto 2.
    typedef weak_atom< meta > vertex_wrapper;
    bool foundValidArgSlot = false;
    while (r && !foundValidArgSlot ) {
        for (unsigned int i = 0; i < r->getInputFilter().size(); i++) {
            vertex_wrapper mp(r->getInputFilter()[i]);
            if (mp(seed)) {
                seedIndex = i;
                foundValidArgSlot = true;
                current = r;
                break;
            }
        }
        if (!foundValidArgSlot) {
            invalidRules.push_back(r);
            r = findHighestPriorityRule();
        }
    }
    return r;
}

}}
