/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include <opencog/atomspace/AtomSpaceImpl.h>

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/macros.h>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

//#define DPRINTF printf
#define DPRINTF(...)

namespace opencog { namespace pln {

using std::string;

ReferenceRuleProvider& referenceRuleProvider()
{
    static ReferenceRuleProvider* rrp = NULL;
    if (rrp == NULL) {
        DPRINTF("Creating new reference rule provider\n");
        rrp = new ReferenceRuleProvider();
    }
    // need to add any other rules?
    return *rrp;
}

const float ReferenceRuleProvider::instantiationRulePriority(7.5f);

ReferenceRuleProvider::~ReferenceRuleProvider(void)
{
}

void ReferenceRuleProvider::registerWatcher(RuleProvider* rp)
{
    boost::mutex::scoped_lock lock(watchersLock);
    watchers.push_back(rp);
    foreach(RulePtr r, instantiationRules) {
        rp->addRule(r->getName(),instantiationRulePriority);
    }
    
}

void ReferenceRuleProvider::unregisterWatcher(RuleProvider* rp)
{
    boost::mutex::scoped_lock lock(guard);
    std::vector<RuleProvider*>::iterator ri;
    for (ri = watchers.begin(); ri != watchers.end(); ri++ ) {
        if (*ri == rp) break;
    }
    if (ri != watchers.end()) watchers.erase(ri);
}

RuleProvider::RuleProvider(void)
{
}

RuleProvider::~RuleProvider(void)
{
    boost::mutex::scoped_lock lock(guard);
    // Unregister from the referenceRuleProvider list
    referenceRuleProvider().unregisterWatcher(this);
}

void RuleProvider::watchInstantiationAtoms() {
    // separate guard for this bool?
    if (!watchingForInstantiationAtoms) {
        referenceRuleProvider().registerWatcher(this);
        watchingForInstantiationAtoms = true;
    }
}

void RuleProvider::addRule(const string& ruleName, float priority)
{
    if (this == &referenceRuleProvider())
        throw opencog::RuntimeException(TRACE_INFO, "Can't add rule to reference ruleprovider.");
    // Check it exists in the reference repository
    RulePtr r = referenceRuleProvider().findRule(ruleName);
    boost::mutex::scoped_lock lock(guard);
    // If so, give it a priority
    rulePriorities[ruleName] = priority;
}

void RuleProvider::removeRule(const string& ruleName)
{
    boost::mutex::scoped_lock lock(guard);
    // Check it exists in the reference repository
    std::map<const std::string, float>::iterator i = rulePriorities.find(ruleName);
    if (i == rulePriorities.end())
        throw opencog::InvalidParamException(TRACE_INFO, ("No rule with name " + ruleName).c_str());
    rulePriorities.erase(i);
}

void RuleProvider::setPriority(const std::string& ruleName, float priority) {
    boost::mutex::scoped_lock lock(guard);
    std::map<const std::string, float>::const_iterator i = rulePriorities.find(ruleName);
    if (i == rulePriorities.end());
        throw opencog::InvalidParamException(TRACE_INFO, ("No rule with name " + ruleName).c_str());
    rulePriorities[ruleName] = priority;
}

float RuleProvider::getPriority(const std::string& ruleName) {
    boost::mutex::scoped_lock lock(guard);
    std::map<const std::string, float>::const_iterator i = rulePriorities.find(ruleName);
    if (i == rulePriorities.end())
        throw opencog::InvalidParamException(TRACE_INFO, ("No rule with name " + ruleName).c_str());
    else return i->second;
}

RulePtr ReferenceRuleProvider::addRule(Rule* r, float priority)
{
    boost::mutex::scoped_lock lock(guard);
    RulePtr rp(r);
    rules.find(rp->getName());
    // TODO prevent double addition of existing rule!!
    DPRINTF("Rule added %s\n",rp->getName().c_str());
    rules[rp->getName()] = rp;
    rulePriorities[rp->getName()] = priority;
    lock.unlock();
    return rp;
}

bool RuleProvider::empty() const {
    return rulePriorities.size() == 0 ? true : false;
}

struct EqRuleName
{
private:
    const string& _name;
public:
    EqRuleName(const string& name) : _name(name) {}
    bool operator()(const std::pair<std::string,RulePtr>& other) {
        return other.first == _name;
    }
};

RulePtr RuleProvider::findRule(const string& ruleName) const
{
    boost::mutex::scoped_lock lock(guard);
    // Check rule is in our rulePriorities
    std::map<const std::string, float>::const_iterator i = rulePriorities.find(ruleName);
    if (i == rulePriorities.end())
        throw opencog::InvalidParamException(TRACE_INFO, ("No rule with name " + ruleName).c_str());
    RuleProvider* rp = &referenceRuleProvider();
    return rp->findRule(ruleName);
}


RulePtr ReferenceRuleProvider::findRule(const string& ruleName) const
{
    EqRuleName eq(ruleName);
    boost::mutex::scoped_lock lock(guard);
    std::map<const std::string,RulePtr>::const_iterator cit = find_if(rules.begin(), rules.end(), eq);
    if(cit == rules.end())
        throw opencog::InvalidParamException(TRACE_INFO, ("No rule with name " + ruleName).c_str());
    else return cit->second;
}

std::vector<std::string> RuleProvider::getRuleNames() const
{
    boost::mutex::scoped_lock lock(guard);
    std::vector<std::string> result;
    std::map<const std::string, float>::const_iterator cit = rulePriorities.begin();
    while (cit != rulePriorities.end()) {
        result.push_back(cit->first);
        cit++;
    }
    return result;
}

VariableRuleProvider::VariableRuleProvider(void)
{
}

VariableRuleProvider::~VariableRuleProvider(void)
{
}

void RuleProvider::printPriorities()
{
    printf("Priorities of rules\n");
    std::map<const std::string, float>::const_iterator cit = rulePriorities.begin();
    while (cit != rulePriorities.end()) {
        printf(" %s - %.3f\n", cit->first.c_str(), cit->second);
        cit++;
    }
}

void ReferenceRuleProvider::printRules()
{
    printf("Instantiation Rules\n");
    foreach(RulePtr r, instantiationRules) {
        printf(" %s\n", r->getName().c_str());
    }
    printf("All Rules\n");
    std::map<const std::string, float>::const_iterator cit = rulePriorities.begin();
    while (cit != rulePriorities.end()) {
        printf(" %s\n", cit->first.c_str());
        cit++;
    }
}

bool ReferenceRuleProvider::handleAddSignal(AtomSpaceImpl* a, Handle h)
{
    AtomSpaceWrapper* asw = ASW();
    pHandle ph = asw->realToFakeHandle(h, NULL_VERSION_HANDLE);
    
    Type t = a->getType(h);
    RulePtr rulePtr;
    if (t == FORALL_LINK) {
        rulePtr = addRule(new ForAllInstantiationRule(t, ph, asw),
                instantiationRulePriority);
    } else if (t == AVERAGE_LINK) {
        rulePtr = addRule(new AverageInstantiationRule(t, ph, asw),
                instantiationRulePriority);
    }
    if (rulePtr) {
        instantiationRules.push_back(rulePtr);
        boost::mutex::scoped_lock lock(watchersLock);
        foreach(RuleProvider* rp, watchers)
            rp->addRule(rulePtr->getName(), instantiationRulePriority);
    }
    return false;
}

bool ReferenceRuleProvider::handleRemoveSignal(AtomSpaceImpl* a, Handle h)
{
    AtomSpaceWrapper* asw = ASW();
    pHandle ph = asw->realToFakeHandle(h, NULL_VERSION_HANDLE);
    
    Type t = a->getType(h);
    std::string name;
    if (t == FORALL_LINK) {
        name = ForAllInstantiationRulePrefixStr;
    } else if (t == AVERAGE_LINK) {
        name = AverageInstantiationRulePrefixStr;
    }
    if (name.length() > 0) {
        name += boost::lexical_cast<std::string>(ph);
        // remove from ReferenceRuleProvider
        removeRule(name);
    }
    return false;
}

void ReferenceRuleProvider::removeRule(const std::string& ruleName)
{
    boost::mutex::scoped_lock lock(guard);
    EqRuleName eq(ruleName);
    std::map<std::string,RulePtr>::iterator cit = find_if(rules.begin(), rules.end(), eq);
    if(cit == rules.end())
        throw opencog::InvalidParamException(TRACE_INFO, ("No rule with name " + ruleName).c_str());

    // Remove from instantiation rules if it exists within the list
    std::vector<RulePtr>::iterator ri;
    for (ri = instantiationRules.begin(); ri != instantiationRules.end(); ri++ ) {
        if ((*ri)->getName() == ruleName) break;
    }
    if (ri != instantiationRules.end()) {
        boost::mutex::scoped_lock lock(watchersLock);
        foreach(RuleProvider* rp, watchers) {
            rp->removeRule((*ri)->getName());
        }
        instantiationRules.erase(ri);
    }
    // Remove from referenceRuleProvider priorities
    std::map<const std::string, float>::iterator i = rulePriorities.find(ruleName);
    rulePriorities.erase(i);

    // Remove from master rule map
    rules.erase(cit);
    DPRINTF("Rule removed %s\n",cit->second->getName().c_str());
}

ReferenceRuleProvider::ReferenceRuleProvider(void)
{
    
    AtomSpaceWrapper* asw = ASW();
    AtomSpace* atomspace = asw->getAtomSpace();
    
    c_add = atomspace->addAtomSignal(
            boost::bind(&ReferenceRuleProvider::handleAddSignal, this, _1, _2));
    c_remove = atomspace->removeAtomSignal(
            boost::bind(&ReferenceRuleProvider::handleRemoveSignal, this, _1, _2));
    assert(c_add.connected() && c_remove.connected());

    addRule(new LookupRule(asw), 20.0f);

/// StrictCrispUnification always requires Hypothesis, too!	
//	addRule(new StrictCrispUnificationRule(asw), 7.5f);
//	addRule(new CrispUnificationRule(asw), 7.5f); ///Alternative implementation

    float AndEvaluatorPriority = 10.0f;
/// haxx:: \todo AndRule sometimes confuses the order of atoms in the 
/// outgoing vector of the resulting AndLink. Ie. the order is not the same
/// as the order in which the arguments were inputted. Eg. compute(a, b) may give
/// And(b,a). This is not acceptable because all PLN code assumes that the AndLinks
/// are ordered properly. This is especially necessary when AndLinks are used
/// as SequentialAndLinks, but there is another basic cause for it, too.
//	addRule(new AndRule(asw), AndEvaluatorPriority);

    addRule(new OrRule(asw), 10.0f);
    
    addRule(new SimpleAndRule<1>(asw), AndEvaluatorPriority - 1.0f);
    addRule(new SimpleAndRule<2>(asw), AndEvaluatorPriority - 1.1f);
    addRule(new SimpleAndRule<3>(asw), AndEvaluatorPriority - 1.2f);
    //addRule(new SimpleAndRule<4>(asw), AndEvaluatorPriority - 1.3f);
    //addRule(new SimpleAndRule<5>(asw), AndEvaluatorPriority - 1.4f);
    
    addRule(new AndPartitionRule(asw), -100.0f);
    //addRule(new AndBreakdownRule(asw, 2), 10.0f);
    //addRule(new AndBreakdownRule(asw, 3), 10.0f);

    addRule(new NotRule(asw), 10.0f);
    
    addRule(new ScholemFunctionProductionRule(asw), 20.0f);
    
    addRule(new SubsetEvalRule(asw, CONCEPT_NODE), 10.0f);
    addRule(new IntensionalInheritanceRule(asw, CONCEPT_NODE), 10.f);
    // FC needs these with less restrictive typing for some reason...
    addRule(new SubsetEvalRule(asw, ATOM), 1.0f);
    addRule(new IntensionalInheritanceRule(asw, ATOM), 1.0f);

    //addRule(new ForAllRule(asw,NULL), 5.0f);
    //addRule( new PLNPredicateRule(asw,NULL), 5.0f);
    
    //addRule(new ImplicationBreakdownRule(asw), 9.0f);
    addRule(new StrictImplicationBreakdownRule(asw), 9.0f);
    
    //addRule(new ImplicationTailExpansionRule(asw), 10.0f);
    //addRule(new ImplicationConstructionRule(asw), 10.0f);
    addRule(new InversionRule(asw, INHERITANCE_LINK), 7.0f);
    addRule(new InversionRule(asw, ASSOCIATIVE_LINK), -100.0f);
    addRule(new InversionRule(asw, IMPLICATION_LINK), -100.0f);
    addRule(new DeductionRule<DeductionSimpleFormula>(asw, IMPLICATION_LINK), 8.0f);
    addRule(new DeductionRule<DeductionSimpleFormula>(asw, INHERITANCE_LINK), 8.0f);
    addRule(new DeductionRule<DeductionSimpleFormula>(asw, ASSOCIATIVE_LINK), 8.0f);
    addRule(new DeductionRule<DeductionSimpleFormula>(asw, SIMILARITY_LINK), 8.0f);
    // Enabling SubsetDeductionRule seems to make the inference trail printing
    // mechanism enter an infinite loop
    //addRule(new DeductionRule<DeductionSimpleFormula>(asw, SUBSET_LINK), 8.0f);
    
    //addRule(new OrPartitionRule(asw), 10.0f);
    addRule(new CrispTheoremRule(asw), 10.0f);
    
    addRule(new Int2ExtRule(asw, IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    addRule(new Int2ExtRule(asw, INHERITANCE_LINK, SUBSET_LINK), 10.0f);
    addRule(new Ext2IntRule(asw, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    addRule(new Ext2IntRule(asw, SUBSET_LINK, INHERITANCE_LINK), 10.0f);
    
    addRule(new Equi2ImpRule(asw), 10.0f);

    addRule(new HypothesisRule(asw), 30.0f);
    // general -> specific
    addRule(new InheritanceSubstRule(asw, false), 5.0f);
    // specific -> general; can be handled by general->specific, plus
    // InversionRule.
    //    addRule(new InheritanceSubstRule(asw, true), 5.0f);
    
    /* The rest of the Rules have rarely or never been used. Some of them just won't work. */
    
    /*	addRule(new UnorderedLinkPermutationRule(asw), 10.0f);
	addRule(new VariableInstantiationRule(asw), 10.0f);
	addRule(new NotEliminationRule(asw), 10.0f
	addRule(new Equi2ImplRule(asw), 10.0f
	addRule(new Equi2Sim(asw), 10.0f;
	addRule(new Inh2SimRule(asw), 10.0f;
	addRule(new Sim2InhRule(asw), 10.0f;

	addRule(new RevisionRule(asw);	
	addRule(new MetaPredicateExecutionRule(asw);
	addRule(new SubSetEvalRule<CONCEPT_NODE>(asw)
	addRule(new Equi2SimRule(asw);
	addRule(new Mem2InhRule(asw);
	addRule(new Inh2ImpRule(asw);
	addRule(new Imp2InhRule(asw);
	addRule(new Mem2EvalRule(asw);
	addRule(new Inh2EvalRule(asw);

	addRule(new ExtImpl2SubsetRule(asw);
	addRule(new ExtEqui2ExtSimRule(asw);
	addRule(new TautologyRule(asw);
	addRule(new Or2AndRule(asw);
	addRule(new Exist2ForAllRule(asw);
	addRule(new ExistRule(asw);
*/

    // Contextual rules
    addRule(new ContextualizerRule(asw), 5.0f);
    addRule(new DecontextualizerRule(asw), 4.0f);
    addRule(new ContextFreeToSensitiveRule(asw), 1.0f);
}

//template<typename FormulaType>
#define FormulaType InversionFormula
class GenericRule2 : public Rule
{
protected:
    mutable FormulaType formula;

public:
    virtual std::set<MPs> o2iMetaExtra(meta outh, bool& overrideInputFilter) const=0;
    //	virtual TVSeq formatTVarray	(const VertexSeq& premiseArray) const=0;
    
    ~GenericRule2() {}
    /// Always a Composer
    GenericRule2(AtomSpaceWrapper *_asw,
                 bool _FreeInputArity, string _name = "")
        : Rule(_asw, _FreeInputArity, true, _name) {}
    
    BoundVertex compute(const VertexSeq& premiseArray,
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
    TVSeq formatTVarray(const VertexSeq& premiseArray) const
    {
        TVSeq tvs(3);
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
    
    virtual meta i2oType(const VertexSeq& h) const
    {
        return	meta(new tree<Vertex>());
    }
    NO_DIRECT_PRODUCTION;
};

#define USE_RULES_BESIDES_DEDUCTION
ForwardComposerRuleProvider::ForwardComposerRuleProvider(void)
{
    AtomSpaceWrapper* asw = GET_ASW; OC_UNUSED(asw);

    float AndEvaluatorPriority = 10.0f;

#ifdef USE_RULES_BESIDES_DEDUCTION
////addRule(new SimpleAndRule<1>(asw), AndEvaluatorPriority - 1.0f);
    addRule("SimpleAndRule2", AndEvaluatorPriority - 1.1f);
    addRule("SimpleAndRule3", AndEvaluatorPriority - 1.2f);
    //  addRule(new SimpleAndRule<4>(asw), AndEvaluatorPriority - 1.3f);
    //  addRule(new SimpleAndRule<5>(asw), AndEvaluatorPriority - 1.4f);

//// Needs a fullInputFilter method to deal with the variable arity.
    // Also not actually used in any of the demos.
    addRule("AndPartitionRule", 10.0f);
    addRule("NotRule", 10.0f);

    // FC: Have to use ATOM due to TableGather not handling Node Type vertexes
    addRule("SubsetEvalRuleAtom", 10.0f);

    addRule("IntensionalInheritanceRuleAtom", 10.f);

    //  addRule(new ForAllRule(asw,NULL), 5.0f);
    //  addRule( new PLNPredicateRule(asw,NULL), 5.0f);

    //  addRule(new ImplicationBreakdownRule(asw), 9.0f);
    addRule("ModusPonensRule", 9.0f); //StrictImplicationBreakdownRule

    //  addRule(new ImplicationTailExpansionRule(asw), 10.0f);
    //  addRule(new ImplicationConstructionRule(asw), 10.0f);
//  addRule(new InversionRule<IMPLICATION_LINK>(asw), 7.0f);
    //addRule(new DeductionRule<DeductionSimpleFormula, IMPLICATION_LINK>(asw), 8.0f);
    addRule("ImplicationDeductionRule", 8.0f);
    //addRule(new InversionRule<INHERITANCE_LINK>(asw), 7.0f);
    addRule("InversionRule", 7.0f);
    addRule("AssociativeInversionRule", 7.0f);
#endif
    addRule("InheritanceDeductionRule", 8.0f);
#ifdef USE_RULES_BESIDES_DEDUCTION
    // This next one is just for the wordpairs demo.
    addRule("AssociativeDeductionRule", 8.0f);
    addRule("SimilarityDeductionRule", 8.0f);

    //  addRule(new OrPartitionRule(asw), 10.0f);

////addRule(new CrispTheoremRule(asw), 10.0f);

    addRule("Link2Link(ImplicationLink=>MixedImplicationLink)", 10.0f);
    addRule("Link2Link(InheritanceLink=>SubsetLink)", 10.0f);
    addRule("Link2Link(ExtensionalImplicationLink=>MixedImplicationLink)", 10.0f);
    addRule("Link2Link(SubsetLink=>InheritanceLink)", 10.0f);
#endif
}

ForwardComposerRuleProvider::~ForwardComposerRuleProvider(void)
{

}

//// Generator RuleProvider
ForwardGeneratorRuleProvider::ForwardGeneratorRuleProvider(void)
{
	AtomSpaceWrapper* asw = GET_ASW; OC_UNUSED(asw);

    addRule("Lookup", 20.0f);
    addRule("ScholemFunctionProductionRule", 20.0f);
    addRule("Hypothesis", 30.0f);

    watchInstantiationAtoms();
}

ForwardGeneratorRuleProvider::~ForwardGeneratorRuleProvider(void)
{

}


DeductionRuleProvider::DeductionRuleProvider(void) {
    AtomSpaceWrapper* asw = GET_ASW; OC_UNUSED(asw);

    //  addRule(new ImplicationBreakdownRule(asw), 9.0f);
    addRule("ModusPonensRule", 9.0f);

    //  addRule(new ImplicationTailExpansionRule(asw), 10.0f);
    //  addRule(new ImplicationConstructionRule(asw), 10.0f);

    addRule("ImplicationDeductionRule", 8.0f);

    addRule("InheritanceDeductionRule", 8.0f);
    // This next one is just for the wordpairs demo.
    addRule("AssociativeDeductionRule", 8.0f);
    addRule("SimilarityDeductionRule", 8.0f);
}
DeductionRuleProvider::~DeductionRuleProvider(void) {

}

EvaluationRuleProvider::EvaluationRuleProvider(void) {
    AtomSpaceWrapper* asw = GET_ASW; OC_UNUSED(asw);

    float AndEvaluatorPriority = 10.0f;

    watchInstantiationAtoms();

    addRule("OrRule", 10.0f);

    addRule("SimpleAndRule1", AndEvaluatorPriority - 1.0f);
    addRule("SimpleAndRule2", AndEvaluatorPriority - 1.1f);
    addRule("SimpleAndRule3", AndEvaluatorPriority - 1.2f);
    //addRule("SimpleAndRule4", AndEvaluatorPriority - 1.2f);
    //addRule("SimpleAndRule5", AndEvaluatorPriority - 1.2f);

    // Needs a fullInputFilter method to deal with the variable arity.
    // Also not actually used in any of the demos.
    //addRule(new AndPartitionRule(asw), 10.0f);
    
    addRule("NotRule", 10.0f);

    addRule("SubsetEvalRule", 10.0f);

    addRule("IntensionalInheritanceRule", 10.f);

    //addRule(new ForAllRule(asw,NULL), 5.0f);
    //addRule( new PLNPredicateRule(asw,NULL), 5.0f);

    //addRule(new OrPartitionRule(asw), 10.0f);

    //addRule(new CrispTheoremRule(asw), 10.0f);

    // Subset2Inh looks for all possible SubsetLinks, then tries to produce
    // SubsetLinks containing those SubsetLinks, etc.
    // Each of these only takes about 1 step, because the BC doesn't count
    // different exact Atoms as different steps.
    //addRule(new Int2ExtRule(asw, IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    //addRule(new Ext2IntRule(asw, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK), 10.0f);
    addRule("Link2Link(InheritanceLink=>SubsetLink)", 10.0f);
    addRule("Link2Link(SubsetLink=>InheritanceLink)", 10.0f);

    // JaredW: Inversion is basically a conversion Rule, so maybe it should be here.
    // (i.e. it has exactly one way to produce each target, so not really combinatorial explosions).
    //addRule(new InversionRule<INHERITANCE_LINK>(asw), 7.0f);
    addRule("InversionRule", 7.0f);
    addRule("AssociativeInversionRule", 7.0f);
    addRule("ImplicationInversionRule", 7.0f);

    addRule("Lookup", 20.0f);
    addRule("ScholemFunctionProductionRule", 20.0f);
    addRule("Hypothesis", 30.0f);
}

EvaluationRuleProvider::~EvaluationRuleProvider(void) {

}

}}
