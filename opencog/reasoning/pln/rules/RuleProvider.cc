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

using std::string;

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

struct EqRuleName
{
private:
    const string& _name;
public:
    EqRuleName(const string& name) : _name(name) {}
    bool operator()(Rule* r) {
        OC_ASSERT(r != NULL);
        return r->getName() == _name;
    }
};

const Rule* RuleProvider::findRule(const string& ruleName) const
{
    EqRuleName eq(ruleName);
    RuleProvider::const_iterator cit = find_if(begin(), end(), eq);
    if(cit == end())
        return NULL;
    else return *cit;
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
iAtomSpaceWrapper* asw = ASW();
	
}
*/
DefaultVariableRuleProvider::DefaultVariableRuleProvider(void)
{
    
    AtomSpaceWrapper* asw = ASW();
    
    Btr<std::set<pHandle> > ForAll_handles = asw->getHandleSet(FORALL_LINK, "");
    
    foreach(pHandle fah, *ForAll_handles)
        AddRule(new CustomCrispUnificationRule(fah, asw), 7.5f);
    
    cprintf(-1, "Added %u CustomCrispUnificationRules.\n", (unsigned int) size());
    
    AddRule(new LookupRule(asw), 20.0f);

/// StrictCrispUnification always requires Hypothesis, too!	
//	AddRule(new StrictCrispUnificationRule(asw), 7.5f);
//	AddRule(new CrispUnificationRule(asw), 7.5f); ///Alternative implementation

    float ANDEvaluatorPriority = 10.0f;
/// haxx:: \todo ANDRule sometimes confuses the order of atoms in the 
/// outgoing vector of the resulting ANDLink. Ie. the order is not the same
/// as the order in which the arguments were inputted. Eg. compute(a, b) may give
/// AND(b,a). This is not acceptable because all PLN code assumes that the ANDLinks
/// are ordered properly. This is especially necessary when ANDLinks are used
/// as SequentialANDLinks, but there is another basic cause for it, too.
//	AddRule(new ANDRule(asw), ANDEvaluatorPriority);

    AddRule(new ORRule(asw), 10.0f);
    
    AddRule(new SimpleANDRule<1>(asw), ANDEvaluatorPriority - 1.0f);
    AddRule(new SimpleANDRule<2>(asw), ANDEvaluatorPriority - 1.1f);
    AddRule(new SimpleANDRule<3>(asw), ANDEvaluatorPriority - 1.2f);
    //	AddRule(new SimpleANDRule<4>(asw), ANDEvaluatorPriority - 1.3f);
    //	AddRule(new SimpleANDRule<5>(asw), ANDEvaluatorPriority - 1.4f);
    
    AddRule(new ANDPartitionRule(asw), 10.0f);
    AddRule(new NotEvaluatorRule(asw), 10.0f);
    
    AddRule(new ScholemFunctionProductionRule(asw), 20.0f);
    
    AddRule(new SubsetEvalRule(asw), 10.0f);

    AddRule(new IntensionalInheritanceRule(asw), 10.f);

    //	AddRule(new FORALLRule(asw,NULL), 5.0f);
    //	AddRule( new PLNPredicateRule(asw,NULL), 5.0f);
    
    //	AddRule(new ImplicationBreakdownRule(asw), 9.0f);
    AddRule(new StrictImplicationBreakdownRule(asw), 9.0f);
    
    //	AddRule(new ImplicationTailExpansionRule(asw), 10.0f);
    //	AddRule(new ImplicationConstructionRule(asw), 10.0f);
//	AddRule(new InversionRule<IMPLICATION_LINK>(asw), 7.0f);
    //AddRule(new DeductionRule<DeductionSimpleFormula, IMPLICATION_LINK>(asw), 8.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(asw, IMPLICATION_LINK), 8.0f);
    //AddRule(new InversionRule<INHERITANCE_LINK>(asw), 7.0f);
    AddRule(new InversionRule(asw, INHERITANCE_LINK), 7.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(asw, INHERITANCE_LINK), 8.0f);
    
    //	AddRule(new ORPartitionRule(asw), 10.0f);
    AddRule(new CrispTheoremRule(asw), 10.0f);
    
    AddRule(new Int2ExtRule(asw, IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    AddRule(new Int2ExtRule(asw, INHERITANCE_LINK, SUBSET_LINK), 10.0f);
    AddRule(new Ext2IntRule(asw, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    AddRule(new Ext2IntRule(asw, SUBSET_LINK, INHERITANCE_LINK), 10.0f);
    
    AddRule(new HypothesisRule(asw), 30.0f);
    // general -> specific
    //AddRule(new SimSubstRule1(asw, false), -10000000.0f);
    AddRule(new SimSubstRule1(asw, false), 5.0f);
    // specific -> general
    AddRule(new SimSubstRule1(asw, true), 5.0f);
    
    /* The rest of the Rules have rarely or never been used. Some of them just won't work. */
    
    /*	AddRule(new UnorderedLinkPermutationRule(asw), 10.0f);
	AddRule(new VariableInstantiationRule(asw), 10.0f);
	AddRule(new NOTEliminationRule(asw), 10.0f
	AddRule(new Equi2ImplRule(asw), 10.0f
	AddRule(new Equi2Sim(asw), 10.0f;
	AddRule(new Inh2SimRule(asw), 10.0f;
	AddRule(new Sim2InhRule(asw), 10.0f;

	AddRule(new RevisionRule(asw);	
	AddRule(new MetaPredicateExecutionRule(asw);
	AddRule(new SubSetEvalRule<CONCEPT_NODE>(asw)
	AddRule(new Equi2SimRule(asw);
	AddRule(new Mem2InhRule(asw);
	AddRule(new Inh2ImpRule(asw);
	AddRule(new Imp2InhRule(asw);
	AddRule(new Mem2EvalRule(asw);
	AddRule(new Inh2EvalRule(asw);

	AddRule(new ExtImpl2SubsetRule(asw);
	AddRule(new ExtEqui2ExtSimRule(asw);
	AddRule(new TautologyRule(asw);
	AddRule(new OR2ANDRule(asw);
	AddRule(new Exist2ForAllRule(asw);
	AddRule(new ExistRule(asw);
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
    GenericRule2(AtomSpaceWrapper *_asw,
                 bool _FreeInputArity, string _name = "")
        : Rule(_asw, _FreeInputArity, true, _name) {}
    
    BoundVertex compute(const std::vector<Vertex>& premiseArray,
                        Handle CX = Handle::UNDEFINED) const
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
    InversionRule2(AtomSpaceWrapper *_asw)
	: GenericRule2/*<InversionFormula>*/ (_asw, false, "InversionRule")
    {
    }
    bool validate2(MPs& args) const { return true; }
    
    virtual meta i2oType(const std::vector<Vertex>& h) const
    {
        return	meta(new tree<Vertex>());
    }
    NO_DIRECT_PRODUCTION;
};
        
        
ForwardChainerRuleProvider::ForwardChainerRuleProvider(void)
{
    AtomSpaceWrapper* asw = GET_ASW;
    
    float ANDEvaluatorPriority = 10.0f;

    AddRule(new ORRule(asw), 10.0f);
    
    AddRule(new SimpleANDRule<1>(asw), ANDEvaluatorPriority - 1.0f);
    AddRule(new SimpleANDRule<2>(asw), ANDEvaluatorPriority - 1.1f);
    AddRule(new SimpleANDRule<3>(asw), ANDEvaluatorPriority - 1.2f);
    //  AddRule(new SimpleANDRule<4>(asw), ANDEvaluatorPriority - 1.3f);
    //  AddRule(new SimpleANDRule<5>(asw), ANDEvaluatorPriority - 1.4f);
    
////    AddRule(new ANDPartitionRule(asw), 10.0f);
    AddRule(new NotEvaluatorRule(asw), 10.0f);
    
////    AddRule(new ScholemFunctionProductionRule(asw), 20.0f);
    
    // FC: Broken due to TableGather not handling Node Type vertexes
////    AddRule(new SubsetEvalRule(asw), 10.0f);

////    AddRule(new IntensionalInheritanceRule(asw), 10.f);

    //  AddRule(new FORALLRule(asw,NULL), 5.0f);
    //  AddRule( new PLNPredicateRule(asw,NULL), 5.0f);
    
    //  AddRule(new ImplicationBreakdownRule(asw), 9.0f);
    AddRule(new StrictImplicationBreakdownRule(asw), 9.0f);
    
    //  AddRule(new ImplicationTailExpansionRule(asw), 10.0f);
    //  AddRule(new ImplicationConstructionRule(asw), 10.0f);
//  AddRule(new InversionRule<IMPLICATION_LINK>(asw), 7.0f);
    //AddRule(new DeductionRule<DeductionSimpleFormula, IMPLICATION_LINK>(asw), 8.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(asw, IMPLICATION_LINK), 8.0f);
    //AddRule(new InversionRule<INHERITANCE_LINK>(asw), 7.0f);
    AddRule(new InversionRule(asw, INHERITANCE_LINK), 7.0f);
    AddRule(new InversionRule(asw, ASSOCIATIVE_LINK), 7.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(asw, INHERITANCE_LINK), 8.0f);
    // This next one is just for the wordpairs demo.
    AddRule(new DeductionRule<DeductionSimpleFormula>(asw, ASSOCIATIVE_LINK), 8.0f);
    AddRule(new DeductionRule<DeductionSimpleFormula>(asw, SIMILARITY_LINK), 8.0f);
    
    //  AddRule(new ORPartitionRule(asw), 10.0f);
////    AddRule(new CrispTheoremRule(asw), 10.0f);
    
    AddRule(new Int2ExtRule(asw, IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    AddRule(new Int2ExtRule(asw, INHERITANCE_LINK, SUBSET_LINK), 10.0f);
    AddRule(new Ext2IntRule(asw, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    AddRule(new Ext2IntRule(asw, SUBSET_LINK, INHERITANCE_LINK), 10.0f);
    
    
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

// no longer used in new version.
Rule* ForwardChainerRuleProvider::nextRule()
{
/*    // 1. check seed is set, check if current has a Rule, if so
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
    */
}

}}
