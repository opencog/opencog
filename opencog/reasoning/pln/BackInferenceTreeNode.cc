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

#include "PLN.h"

#ifdef WIN32
#pragma warning(disable : 4311)
#endif

#include <stdlib.h>
#include <time.h>

#include "rules/RuleProvider.h"
#include "rules/Rules.h"
//#include "spacetime.h"
#include "AtomSpaceWrapper.h"
#include <opencog/server/CogServer.h>
#include <opencog/util/copyif.h>
#include "BackInferenceTreeNode.h"
#include <boost/iterator/indirect_iterator.hpp>
#include <boost/foreach.hpp>

#ifdef USE_BITUBIGRAPHER
//! @todo
#include <opencog/ubigraph/BITUbigrapher.h>
#endif // USE_BITUBIGRAPHER

// TODELETE: wasn't updated, or used (to any effect)
//int haxxUsedProofResources = 0;
float temperature = 0.01;

namespace test
{
    long bigcount=0;

    double custom_duration = 0.0;
    clock_t custom_start, custom_finish;
    //! How long it takes to virtualise the target during BITNode::expandRule
    time_t custom_start2, custom_finish2;
    double custom_duration2 = 0.0;

    const bool LOG_WITH_NODE_ID = true;
}


namespace haxx
{
    extern opencog::pln::BITNodeRoot* bitnoderoot;

    //! @todo This data must persist even if the BITNodeRoot is deleted.
    std::map<pHandle,std::vector<pHandle> > inferred_from;
    std::map<pHandle,Rule*> inferred_with;
}

//! ARI: okay to delete design namespace?
namespace design
{
/*  typedef boost::make_recursive_variant<
        opencog::pln::BITNodeRoot*,
        opencog::pln::ParametrizedBITNode,
        set<tree<boost::recursive_variant_ > >
    >::type BITChild;
    tree<BITChild> newBITRoot;*/

/*    struct pBITNode : public tree<int>
    {
        int x;
    };
*/
/*  typedef boost::make_recursive_variant<
        opencog::pln::BITNodeRoot*,
        set<opencog::pln::pBITNode>,
        tree<boost::recursive_variant_ >
    >::type BITChild; */

/*    typedef boost::make_recursive_variant<
        opencog::pln::BITNodeRoot*,
        //pBITNode,
        opencog::pln::ParametrizedBITNode,
        set<tree<boost::recursive_variant_> >
    > BITChild;

    tree<BITChild> newBITRoot; */

//  boost::variant<opencog::pln::BITNodeRoot*, opencog::pln::set<ParametrizedBITNode>, BITChild> BITChild;
}

namespace opencog {
namespace pln {

using std::set;
using std::pair;
using std::vector;
using std::string;
using std::map;
using std::stringstream;
using std::endl;
using std::cout;

typedef pair<Rule*, vtree> directProductionArgs;

struct less_dpargs : public std::binary_function<directProductionArgs, directProductionArgs, bool>
{
    bool operator()(const directProductionArgs& lhs, const directProductionArgs& rhs) const
    {
        if (lhs.first < rhs.first)
            return true;
        if (lhs.first > rhs.first)
            return false;
        if (less_vtree()(lhs.second, rhs.second))
            return true;
        if (less_vtree()(rhs.second, lhs.second))
            return false;

        return false;
    }
};

}}

namespace haxx
{

using namespace opencog::pln;
static map<directProductionArgs, Btr<set<BoundVertex> >, less_dpargs> DirectProducerCache;

}

namespace opencog {
namespace pln {

//! This cache apparently gives a 30% speed up when active
const bool USE_GENERATOR_CACHE = false; 

const bool DIRECT_RESULTS_SPAWN = true;
static const float MIN_CONFIDENCE_FOR_RULE_APPLICATION = 0.00001f;
const bool PREVENT_LOOPS = false;

extern Btr<set<pHandle> > ForAll_handles;
static int ParametrizedBITNodes = 0;

void pr(pair<pHandle, pHandle> i);
void pr3(pair<pHandle, pHandle> i);

ParametrizedBITNode::ParametrizedBITNode(BITNode* _prover, Btr<bindingsT> _bindings)
: bindings(_bindings), prover(_prover)
{ prover->children.size(); ParametrizedBITNodes++; }

template<typename V, typename Vit, typename Tit>
void copy_vars(V& vars, Vit varsbegin, Tit bbvt_begin, Tit bbvt_end)
{
    copy_if(bbvt_begin, bbvt_end, inserter(vars, varsbegin), 
        bind(std::equal_to<Type>(), 
        bind(getTypeFun, bind(&_v2h, _1)),
        (Type)FW_VARIABLE_NODE));
}

static int more_count=0;

BITNode::BITNode()
: depth(0), root(0), Expanded(false), rule(NULL), my_bdrum(0.0f),
direct_results(Btr<set<BoundVertex> >(new set<BoundVertex>))

{
}

string BITNodeRoot::printUsers(BITNode* b)
{
    stringstream ss;
    foreach(BITNode* u, users[b])
        ss << "[" << (long)u << "] ";
    ss << endl;
    cout << ss.str();
    return ss.str();
}

string BITNodeRoot::printParents(BITNode* b) const
{
    stringstream ss;
    foreach(const parent_link<BITNode>& p, b->parents)
        ss << "[" << (long)p.link << "] ";
    ss << endl;
    cout << ss.str();
    return ss.str();
}

unsigned int BITNodeRoot::getTreeDepth() const { return treeDepth; }

void BITNodeRoot::setRecordingTrails(bool x) { recordingTrails = x; }
bool BITNodeRoot::getRecordingTrails() const { return recordingTrails; }

BITNodeRoot::BITNodeRoot(meta _target, RuleProvider* _rp, bool _rTrails,
        FitnessEvaluatorT _fe)
: inferenceNodes(0), exec_pool_sorted(false), rp(_rp), post_generalize_type(0),
  treeDepth(0)
{
    AtomSpaceWrapper *asw = GET_ASW;
    haxx::DirectProducerCache.clear();
    
    /// All CustomCrispUnificationRules must be re-created
    //ForAll_handles.reset();
    //RuleRepository::Instance().CreateCustomCrispUnificationRules();

    recordingTrails = _rTrails;
    fitnessEvaluator = _fe;
    
    rule = NULL;
    root = this;
    bound_target = meta(new vtree);
    if (!rp) // changed to NOT rp... otherwise we are overwriting the passed rp
      rp = new DefaultVariableRuleProvider;
    assert(!rp->empty());
    cprintf(3, "rp ok\n");
    haxx::bitnoderoot = this;

    vtree::iterator target_it = _target->begin();
    Type targetType = (Type) _v2h(*target_it);
    // Check whether the target inherits from either VARIABLE_SCOPE_LINK
    // or FORALL_LINK
    //! @todo This should be done for all quantifiers
    //! (put into a isQuantifier() method)
    post_generalize_type = asw->inheritsType((Type)(_v2h(*target_it)), VARIABLE_SCOPE_LINK)
                                    ? VARIABLE_SCOPE_LINK
                                    : asw->inheritsType((Type)(_v2h(*target_it)), FORALL_LINK)
                                        ? FORALL_LINK
                                        : 0;
    if (post_generalize_type) {
        // VARIABLE_SCOPE_LINK( arg_list, actual_atom ) -> select actual_atom:
        target_it = _target->begin(target_it);
        ++target_it;
    }

    raw_target = Btr<vtree>(new vtree(target_it));

    // The 1st child corresponds to "root variable scoper",
    // variable-bound clones of which will be spawned later on.
    // Those clones will then be owned by this Root.
    // To enable this, the 1st child MUST OWN the variables in the target atom
    // of the root.
    dummy_args.push_back(Btr<BoundVTree>(new BoundVTree(*raw_target)));

//  rawPrint(*raw_target, raw_target->begin(), -1);
    children.push_back(set<ParametrizedBITNode>());
    cprintf(3, "scoper...\n");
    BITNode* root_variable_scoper = createChild(0, NULL, dummy_args,
            Btr<BoundVTree>(new BoundVTree(make_vtree(NODE))),
            bindingsT(), NO_SIBLING_SPAWNING);
    cprintf(3, "scoper ok\n");
    set<Vertex> vars;
    copy_if(    raw_target->begin(),
                raw_target->end(),
                inserter(vars, vars.begin()),
                bind(std::equal_to<Type>(),
                    bind(getTypeFun, bind(&_v2h, _1)),
                        (Type)FW_VARIABLE_NODE));
    foreach(Vertex v, vars)
        varOwner[v].insert(root_variable_scoper);

    eval_results.push_back(set<VtreeProvider*>());
    cprintf(3, "Root ok\n");

#ifdef USE_BITUBIGRAPHER
    haxx::BITUSingleton = new BITUbigrapher;
    haxx::BITUSingleton->drawRoot(this);
    haxx::BITUSingleton->drawBITNode(this, children);
#endif
}

/// The complexity here results from bindings and virtuality.
/// The bindings may convert some atoms to either virtual or real atoms.
/// In the end, all links must be virtual. But simple conversion is not enough,
/// because if the root link is real, then it must be collected as a direct
/// result first.

void BITNode::ForceTargetVirtual(spawn_mode spawning)
{
    AtomSpaceWrapper *asw = GET_ASW;
    pHandle *ph = boost::get<pHandle>(&(*raw_target->begin()));
    
    if (ph && !asw->isType(*ph) && asw->getType(*ph) != FW_VARIABLE_NODE)
    {
        cprintf(2,"ForceTargetVirtual: Arg [%u] (exists).\n", *ph);
        
        Btr<set<BoundVertex> > directResult(new set<BoundVertex>);
        
        /// Though the target was conceived directly, it is under my pre-bindings!
        directResult->insert(BoundVertex(*ph, Btr<bindingsT>(new bindingsT)));

        addDirectResult(directResult, spawning);
        
        setTarget(meta(new vtree(make_vtree(*ph))), Btr<bindingsT>(new bindingsT));
    }

    //! @todo There's some redundancy here that should be removed...

    bound_target = ForceAllLinksVirtual(bound_target);
    raw_target = ForceAllLinksVirtual(raw_target);
}


BITNode* BITNodeRoot::createChild(int my_rule_arg_i, Rule* new_rule,
        const Rule::MPs& rule_args, BBvtree _target,
        const bindingsT& bindings, spawn_mode spawning)
{
    /// We ignore most of the args.
    BITNode* ret =  new BITNode(this,
                                this,
                                1,
                                0,
                                _target,
                                (Rule*)NULL,
                                rule_args,
                                target_chain);

    exec_pool.push_back(ret);   
    exec_pool_sorted = false;

    assert(ret->children.size() == 1 && ret->args.size() == 1);
    children[0].insert(ParametrizedBITNode(ret, Btr<bindingsT>(new bindingsT)));

    return ret;
}

int BITNodeRoot::getExecPoolSize() const { return exec_pool.size(); }
BITNodeRoot& BITNode::getBITRoot() const { return *root; }

Btr<set<BoundVertex> > BITNodeRoot::evaluate(set<const BITNode*>* chain) const
{
    //! @todo why is there an assert zero and below results commented out?
    assert(0);

    Btr<set<BoundVertex> > results;

//  Btr<set<BoundVertex> > results = (*children[0].begin()).prover->evaluate(chain);

    if (post_generalize_type)
    {
        BoundVertex VarScopeLink(Generalize(results, post_generalize_type));
        results->clear();
        results->insert(VarScopeLink);

        return results;
    }
    else
    {
        cprintf(0, "Results:\n");
        const float min_confidence = 0.0001f;
        Btr<set<BoundVertex> > nontrivial_results(new set<BoundVertex>);

        foreach(const BoundVertex& new_result, *results)
            if (GET_ASW->getTV(_v2h(new_result.value)).getConfidence() > min_confidence)
            {
                printTree(_v2h(new_result.value),0,0);

                nontrivial_results->insert(new_result);
            }
        return nontrivial_results;
    }
}



int BITNode::totalChildren() const
{
    int c=0;
    for (vector<set<ParametrizedBITNode> >::const_iterator i =
            children.begin();
            i!=children.end(); i++) {
        c+=i->size();
    }
    return c;
}

BITNodeRoot::~BITNodeRoot() {
    delete rp;
    foreach(BITNode* b, nodes) delete b;

#if 0
    // Attempt to use the varOwner map to delete FWVars
    // Had numerous complexities/issues, probably resolveable.
    
    //    std::map<Vertex, std::set<BITNode*> > varOwner;
    std::map<Vertex, std::set<BITNode*> >::const_iterator var;
    for (var = varOwner.begin(); var != varOwner.end(); var++) {
        // var->first
        // _v2h
        pHandle h = _v2h(var->first);
        // Why are some of them not valid pHandles?
        if (GET_ASW->isValidPHandle(h)) {
            std::cout << "Deleting " << h << std::endl;
            GET_ASW->removeAtom(_v2h(var->first));
        }
    }
#endif
}

BITNode::~BITNode() {
    assert(root);

    if (root == this)
        cprintf(3, "BITNodeRoot dying...");
    else
        cprintf(4, "BITNode dying... root now has %ld => %ld BITNodes\n",
            root->inferenceNodes, root->inferenceNodes-1);
    root->inferenceNodes--;
}
/*
        ParametrizedBITNode pn(this, plink.bindings);

        set<ParametrizedBITNode>::iterator ps = plink.link->children[plink.parent_arg_i].find(pn);

        if (ps != plink.link->children[plink.parent_arg_i].end())
            plink.link->children[plink.parent_arg_i].erase(ps);

        int a2 = plink.link->children[plink.parent_arg_i].size();
    }

    for (vector< set<ParametrizedBITNode> >::iterator i =children.begin();
        i != children.end(); i++)
    {
        for (set<ParametrizedBITNode>::iterator j=i->begin(); j!=i->end(); j++)
            delete j->prover;
    }
    children.clear();
}
*/

void BITNode::setTarget(meta _target, Btr<bindingsT> binds)
{
    raw_target = _target;
    bound_target = bind_vtree(*raw_target, *binds);
    counted_number_of_free_variables_in_target = number_of_free_variables_in_target();
}

void BITNode::create()
{
    assert(children.empty());

    tlog(-1, "New BITnode was created to prove:\n");
    rawPrint(*bound_target, bound_target->begin(),-1);

    tlog(-1, "This new BITnode needs %d args:\n", args.size());
    for (uint i = 0; i < args.size(); i++)
        rawPrint(*args[i],args[i]->begin(),-1);

    target_chain.insert(*bound_target);

    if (!rule || rule->isComposer())
    {
        children.insert(children.begin(), args.size(), set<ParametrizedBITNode>());
    }
    eval_results.insert(eval_results.begin(), args.size(), set<VtreeProvider*>());
    assert(rule || children.size() == 1);
    
    if (depth > getBITRoot().getTreeDepth()) {
        //tlog(5, "BIT has reached depth: %d", depth);
        std::cout << "BIT has reached depth: " << depth << std::endl;
        getBITRoot().setTreeDepth(depth);
    }
}

BITNode::BITNode( BITNodeRoot* _root,
    BITNode* _parent,
    unsigned int _depth,
    unsigned int _parent_arg_i,
    meta _target,
    Rule *_rule,
    const Rule::MPs& _args,
    const vtreeset& _target_chain,
    Btr<bindingsT> _pre_bindings,
    spawn_mode spawning,
    bool _create)
: raw_target(_target), depth(_depth), root(_root), Expanded(false),
rule(_rule), my_bdrum(0.0f), target_chain(_target_chain), args(_args)
{
    AtomSpaceWrapper *asw = GET_ASW;
    if (_parent)
        addNewParent(_parent, _parent_arg_i);

    try {         
        assert(!parents.empty() || !root);

        setTarget(_target, _pre_bindings);

        vtree::iterator rbt_it = bound_target->begin();
        bool rbt_isType = asw->isType(_v2h(*rbt_it));

        if (asw->isSubType(_v2h(*rbt_it), LINK)
            && ((!rbt_isType && !asw->getArity(_v2h(*rbt_it)))
                || (rbt_isType && !bound_target->number_of_children(rbt_it))
                )
            ) {
            rawPrint(*bound_target, rbt_it, -10);
            assert(0);
        }

        if (!root->rp)
        {
            root->rp = new DefaultVariableRuleProvider();
            tlog(3, "Default RuleProvider created.\n");
        }
        else tlog(3, "Parent passed on my RuleProvider.\n");

        my_bdrum = _parent->my_bdrum;

        direct_results = Btr<set<BoundVertex> >(new set<BoundVertex>);

        ForceTargetVirtual(spawning);

        if (_create) {
            tlog(2, "Creating...\n");
            create();
        }
    } catch(string s) {
      printf("Exception in BITNode::BITNode = %s\n",s.c_str());
      throw;
    } catch(...)  {
      printf("Unknown Exception in BITNode::BITNode!\n");
      throw;
    }
    root->inferenceNodes++;
}

bool BITNode::eq(BITNode* rhs) const
{
    return eq(rhs->rule, rhs->args, rhs->getTarget(), bindingsT());
}

template<typename T>
bool equal_indirect(const T& a, const T& b) { return *a == *b; }

bool BITNode::eq(Rule* r,  const Rule::MPs& _args, meta _target, const bindingsT& _pre_bindings) const
{
    try
    {
        bool ret=false;

        if (rule != r)
            ret = false;
        else if (rule && !rule->isComposer())
        {
            //  meta _final_target(bind_vtree(*_target, GetPreBindings()));
            meta _final_target(new vtree(*_target));
            ret = (*getTarget() == *_final_target);
        }
        else
        {
            assert(!args.empty());

            meta _final_target(new vtree(*_target));
            ret= (*getTarget() == *_final_target);

            ret = (args.size() == _args.size()
                && std::equal(args.begin(), args.end(), _args.begin(), &equal_indirect<meta>));
        }

        if (ret)
            tlog(3, ret?"EQ\n": "IN-EQ\n");

        return ret;
    } catch(PLNexception e) { puts(e.what()); puts("Apparently pre-bindings to eq() were inconsistent."); throw; }
}

BITNode* BITNode::HasChild(BITNode* new_child, int arg_i) const
{
    foreach(const ParametrizedBITNode& c, children[arg_i])
        if (c.prover->eq(new_child))
        {
            tlog(3,"Has this child already.\n");
            return c.prover;
        }

    return NULL;
}

BITNode* BITNode::HasChild(int arg_i, Rule* r, 
    const Rule::MPs& _args, meta _target, const bindingsT& _pre_bindings) const
{
    foreach(const ParametrizedBITNode& c, children[arg_i])
        if (c.prover->eq(r, _args, _target, _pre_bindings))
        {
            tlog(3,"Has this child already.\n");
            return c.prover;
        }

    return NULL;
}
    
bool BITNode::addNewParent(BITNode* parent, int argumentSlot, Btr<bindingsT>
        bindings)
{
    if (root == NULL) {
        logger().error("[BIT] BITNode has no root - failed to add parent link");
        return false;
    }
    // Create parent link
    parent_link<BITNode> pLink(parent, argumentSlot, bindings);
    // Add parent link
    parents.insert(pLink);

    // Let the BIT root know which nodes are dependent on this one now
    // (parent,
    root->users[this].insert(parent);
    // ...and users of parent)
    root->users[this].insert(root->users[parent].begin(),
            root->users[parent].end());
    return true;
}

BITNode* BITNode::findNode(BITNode* new_child) const
{
    BITNode* ret = NULL;
tlog(0,"Finding...\n");
    for (uint i=0; i<children.size(); i++)
        if ((ret=HasChild(new_child, i)) != NULL)
            return ret;

    //foreach (const set<BITNode*>& sBIT, children)
    for (uint i=0; i<children.size();i++)
        foreach(const ParametrizedBITNode& c, children[i])
            if ((ret=c.prover->findNode(new_child)) != NULL)
                return ret;

    return NULL;
}


BITNode* BITNode::findNode(Rule* new_rule, meta _target,
    const Rule::MPs& rule_args, const bindingsT& _pre_bindings) const
{
    /// _target has to be ForceAllLinksVirtual()ized beforehand!

    BITNode* ret = NULL;

    for (uint i=0; i<children.size(); i++)
        if ((ret=HasChild(i, new_rule, rule_args, _target, _pre_bindings)) != NULL)
            return ret;

    //foreach (const set<BITNode*>& sBIT, children)
    for (uint i=0; i<children.size();i++)
        foreach(const ParametrizedBITNode& c, children[i])
            if ((ret=c.prover->findNode(new_rule, _target, rule_args, _pre_bindings)) != NULL)
                return ret;

    return NULL;
}

//! Best Direct Result Under Me - Updater
//! ...used to apply a value to the BITNode class
struct BDRUMUpdater
{
    float val;
    BDRUMUpdater(float _val) : val(_val) {}
    void operator()(BITNode* b) { b->my_bdrum = val; }
};



/* Algorithm */ 


static int count111=0;

void BITNode::addDirectResult(Btr<set<BoundVertex> > directResult, spawn_mode spawning)
{
    AtomSpaceWrapper *asw = GET_ASW;
// If we were to store the results, it would cause inconsistency because we no longer store the prebindings.
//  direct_results->insert(directResult->begin(), directResult->end());

    foreach(const BoundVertex& bv, *directResult)
    {
        tlog(-2, "Added direct result:\n");
        printTree(_v2h(bv.value), 0, -2);
    }

    foreach(const BoundVertex& bv, *directResult)
    {
        if (bv.bindings)
        {
            // Remove un-owned bindings; they are inconsequential.
            ///! @todo Possibly the CrispUnificationRule should be prevented from
            /// producing them in the 1st place.
            bindingsT temp_binds(*bv.bindings);
            bv.bindings->clear();
            foreach(hpair hp, temp_binds)
                if (STLhas(root->varOwner, hp.first))
                    bv.bindings->insert(hp);
        }
        if (!bv.bindings || bv.bindings->empty())
            direct_results->insert(bv);
    }

    // Check whether we need to update the Best Direct Result Under Me
    bool bdrum_changed = false;
    foreach(BoundVertex bv, *directResult)
    {
        float confidence = asw->getTV(_v2h(bv.value)).getConfidence();
        if (confidence > my_bdrum)
        {
            my_bdrum = confidence;
            bdrum_changed = true;
        }
    }
    
    // If necessary, update all nodes under this one with the new my_bdrum value
    if (bdrum_changed)
        ApplyDown(BDRUMUpdater(my_bdrum));
    
    if (spawning && DIRECT_RESULTS_SPAWN)
    {
        cprintf(1,"SPAWN...\n");

        // Insert to pool the bound versions of all the other arguments of the parent
        foreach(const BoundVertex& bv, *directResult)
            if (bv.bindings && !bv.bindings->empty())
                root->spawn(bv.bindings);
            else //Proceed to Rule evaluation (if parent has other args already)
            {
                tlog(0, "Unbound result: notify parent...\n");
                pHandle hh = _v2h(bv.value);
                /*printTree(hh,0,2); */
                NotifyParentOfResult(new VtreeProviderWrapper(bv.value));
            }
    }
    else
        tlog(0,"A no-spawning process.\n");

    root->exec_pool_sorted = false; 
}

bool BITNode::inferenceLoopWith(meta req)
{
    if (this == root)
        return false;

    foreach(const vtree& prev_req, target_chain)
//      if (equalVariableStructure(prev_req, *req))  /// Would disallow for multiple consequtive deduction...
        if (prev_req == *req)
        {
            vtree* preq = const_cast<vtree*>(&prev_req);

            rawPrint(*preq, preq->begin(),3);
            tlog(3,"Loops! Equal var structure with:\n");
            rawPrint(*req, req->begin(),3);
            return true;
        }

    return false;
}   
    
bool BITNode::inferenceLoop(Rule::MPs reqs)
{
    foreach(meta req, reqs)
        foreach(const vtree& prev_req, target_chain)
        if (equalVariableStructure(prev_req, *req))
        {
            rawPrint(*req, req->begin(),3);
            return true;
        }

        return false;
}

// Filter

bool BITNode::obeysSubtreePolicy(Rule *new_rule, meta _target)
{
//  return !(asw->inheritsType(asw->getTypeV(*_target), NODE)
//      && new_rule->isComposer());

    return true;
}

// Filter

bool BITNode::obeysPoolPolicy(Rule *new_rule, meta _target)
{
    AtomSpaceWrapper *asw = GET_ASW;
    if (asw->inheritsType(asw->getTypeV(*_target), FW_VARIABLE_NODE))
        return false;

    /// This rejects all atoms that contain a link with >1 vars directly below it.
    /// \todo Goes over far more child atoms than necessary. Should be made smarter.
    for(vtree::post_order_iterator node = _target->begin_post(); node != _target->end_post(); ++node)
    {
        if (count_if(_target->begin(node), _target->end(node),
                     bind(std::equal_to<Type>(),
                          bind(getTypeFun, bind(&_v2h, _1)),
                          (Type)FW_VARIABLE_NODE ))
            > 1)
        {
            cprintf(-1, "Dis-obeys pool policy:\n");
            rawPrint(*_target, _target->begin(), -1);
            return false;
        }
    }

    return true;
}

void BITNode::findTemplateBIT(BITNode* new_node, BITNode*& template_node, bindingsT& template_binds) const
{
    foreach(BITNode* bit, root->BITNodeTemplates)
        if (bit->rule == new_node->rule && new_node->args.size() == bit->args.size())
        {
            template_binds.clear();

            uint i=0;
            for(; i<bit->args.size(); i++)
                if (!unifiesWithVariableChangeTo(GET_ASW, *new_node->args[i],
                                                 *bit->args[i], template_binds))
                    break;

            if (i==bit->args.size())
            {
                template_node = bit;
                return;
            }
        }   
    template_node = NULL;
}

BITNode* BITNode::createChild(unsigned int target_i, Rule* new_rule,
    const Rule::MPs& rule_args, BBvtree _target, const bindingsT& new_bindings,
    spawn_mode spawning)
{
    AtomSpaceWrapper *asw = GET_ASW;
/*    if (this->depth == haxx::maxDepth)
    {
        puts("haxx::maxDepth !!! "); //press enter");
        return NULL;
    }*/

    /// If any new requirement can, upon suitable substitutions,
    /// produce a requirement higher in the tree, reject it to avoid
    /// looping.
    if ((!PREVENT_LOOPS || !inferenceLoop(rule_args)))
    {
        Btr<bindingsT> bindings(new bindingsT());
        ///haxx::
        Btr<map<Vertex, Vertex> > pre_bindingsV(new map<Vertex, Vertex>);
        Btr<map<Vertex, Vertex> > new_bindingsV(toVertexMap(new_bindings.begin(), new_bindings.end()));

        if (!consistent<Vertex>(*pre_bindingsV, *new_bindingsV, pre_bindingsV->begin(), pre_bindingsV->end(), new_bindingsV->begin(), new_bindingsV->end()))
        {
            tlog(-1, "Binding INCONSISTENT. Child not created.\n");

            ///! @todo Check if coming here actually is allowed by
            /// the design, or whether it's a bug.

            return NULL;
        }
        /// Bindings from the node that spawned me:
        bindings->insert(new_bindings.begin(), new_bindings.end());
        
        /// Bindings from the Rule.o2i that produced my target:
        if (_target->bindings)
            bindings->insert(_target->bindings->begin(), _target->bindings->end());
        
        BITNode* new_node = new BITNode(
                root, this, depth+1, target_i, _target, new_rule, rule_args,
                target_chain, bindings, spawning, false);

        BITNode* template_node = NULL;
        Btr<bindingsT> template_binds(new bindingsT);
        
        if (new_rule && new_rule->isComposer()) //&& new_rule->name != "CrispUnificationRule")
        {
            if (test::bigcount >= 3175)
                cout << new_rule->name << endl;
            findTemplateBIT(new_node, template_node, *template_binds);

            if (template_node)
            {
                tlog(-1, "Found a template [%ld]\n", (long)template_node);

                delete new_node;

                if (this == template_node || hasAncestor(template_node))
                    return NULL;

                /// If I already have this node as a child
                foreach(const parent_link<BITNode>& p, template_node->parents)
                    if (p.link == this && p.parent_arg_i == target_i)
                    {
                        tlog(-1, "I already have this node as a child\n");
                        foreach(const parent_link<BITNode>& myp, parents)
                            tlog(-2, "Parent: %ld\n", (long)myp.link);
                        return template_node;
                    }

                children[target_i].insert(ParametrizedBITNode(template_node, template_binds));
                template_node->addNewParent(this, target_i, template_binds);

                return template_node;
            }
            else
                root->BITNodeTemplates.insert(new_node);
        }

        root->nodes.insert(new_node);

        new_node->create();

        tlog(2, "Created new BIT child [%ld]\n", (long)new_node);

        if (obeysPoolPolicy(new_rule, _target))
        {
            root->exec_pool.push_back(new_node);
            root->exec_pool_sorted = false;
        }

        children[target_i].insert(ParametrizedBITNode(new_node, Btr<bindingsT>(new bindingsT)));

        /// Identify the _new_ variables of this set
        /// And add _new_ variables to var dependency map for the new Child (-InferenceNode)

        /// Copy all vars from the args
        set<Vertex> vars;
        foreach(const BBvtree& bbvt, rule_args)
            copy_vars(vars, vars.begin(), bbvt->begin(), bbvt->end());

        /// Find all _new_ vars
        foreach(Vertex v, vars)
            if (!STLhas2(*_target, v))
            {
                root->varOwner[v].insert(new_node);
                cprintf(0,"[%ld] owns %s\n", (long)new_node, asw->getName(_v2h(v)).c_str());
            }

        return new_node;
    }

    return NULL;
}

/// spawn() is called with all the bindings that were made to produce some
/// direct (eg. lookup) result.
void BITNodeRoot::spawn(Btr<bindingsT> bindings)
{
    /// Only retain the bindings relevant to the varOwner
    map<BITNode*, bindingsT> clone_binds;
    foreach(hpair raw_pair, *bindings) //for $x=>A
        foreach(BITNode* bitn, varOwner[raw_pair.first])
            clone_binds[bitn].insert(raw_pair);
    /// clone_binds now has BITNode owns the variable as a key
    /// and a mapping from that variable to the binding.
    
    typedef pair<BITNode*, bindingsT> o2bT;
    foreach(const o2bT& owner2binds, clone_binds)
    {
        cprintf(-1,"spawn next[%ld]:\n", (long)owner2binds.first);

        foreach(hpair b, owner2binds.second)
            owner2binds.first->tryClone(b);
    }
}

/// If any of the bound vars are owned, we'll spawn.
///! @todo Actually Rules should probably not even need to inform us
/// about the bound vars if they are new, in which case this check would be redundant.
bool BITNodeRoot::spawns(const bindingsT& bindings) const
{
    foreach(hpair b, bindings)
        if (STLhas(varOwner, b.first))
            return true;

    return false;
}

bool BITNode::expandRule(Rule *new_rule, int target_i, BBvtree _target, Btr<bindingsT> bindings, spawn_mode spawning)
{   
    bool ret = true;
    try
    {
        tlog(-2, "Expanding rule... %s\n", (new_rule ? new_rule->name.c_str() : "?"));      

        if (!new_rule->isComposer())          
        {
            createChild(target_i, new_rule, Rule::MPs(), _target, *bindings, spawning);
        }
        else
        {
            tlog(-2, "Expanding Composer: %s\n", new_rule->name.c_str());
            rawPrint(*_target, _target->begin(), 2);

            if (!obeysSubtreePolicy(new_rule, _target))
            {
                tlog(3, "Our policy is to filter this kind of BIT nodes out.\n");
                return false;
            }

            /// Different argument vectors of the same rule (eg. ForAllRule) may have different bindings.

            time( &test::custom_start2 );

            meta virtualized_target(bindings ? bind_vtree(*_target,*bindings) : meta(new vtree(*_target)));
            ForceAllLinksVirtual(virtualized_target);

            time( &test::custom_finish2 );
            test::custom_duration2 += difftime( test::custom_finish2,
                                                test::custom_start2 );

            set<Rule::MPs> target_v_set = new_rule->o2iMeta(virtualized_target);
            
test::custom_duration += (double)(test::custom_finish - test::custom_start) / CLOCKS_PER_SEC;
                        
            if (target_v_set.empty())
            {
                tlog(3,"This rule is useless.\n");
                return false;
            }
            else
            {       
                tlog(2,"Rule.o2iMeta gave %d results\n",target_v_set.size());

                for (set<Rule::MPs>::iterator j =target_v_set.begin();
                     j!=target_v_set.end(); j++) {       
                    Btr<BoundVTree> jtree = (*j->begin());
                    Btr<bindingsT> combined_binds(new bindingsT(*bindings));
                    
                    /** If we spawn the new bindings that this Rule needs, then we will not create the new
                        unbound childnode at all. The new (bound) version of that child will be probably
                        created later.
                        New variables may have been introduced in these bindings of jtree, eg.
                        to get Imp(A, $2) we might have got arg vector of
                        Imp(A, $1) and Imp($1, $2) where $2 = And(B, $4).
                        Because the childnode will not be created here, $1 will simply disappear.
                        $2 = And(B, $4) will be submitted to spawning. The owner of $4 will then be
                        the owner of $2. We find this out by looking at the RHS of each binding that was
                        fed to the spawning process, and finding if there are un-owned variables there.
                        $4 was clearly unwoned.
                        
                        ToDo: we could maintain a list of un-owned variables and destroy them at proper times?
                    */
                    
                    if (spawning == ALLOW_SIBLING_SPAWNING
                        && jtree->bindings && root->spawns(*jtree->bindings))
                        root->spawn(jtree->bindings);
                    else
                    {
                        /// haxx::
                        /// The pre-bindings of a parent node must include the pre-bindings of Rule argument nodes.
                        /// Because all the arg nodes have the same bindings, we just grab the ones from the 1st arg
                        /// and insert them to this (new) parent node.

                        if (jtree->bindings)
                            try
                            {
                                insert_with_consistency_check(*combined_binds, jtree->bindings->begin(), jtree->bindings->end());
                            } catch(...) { puts("exception in expandRule (bindings combination)"); continue; }

                        BITNode* new_node = createChild(target_i, new_rule, *j,_target, *combined_binds, spawning);                 
                    }
                }
            
                return !children[target_i].empty();
            }
        }
    } catch(...) { tlog(0,"Exception in ExpandRule()"); throw; }

    return ret;
}

void BITNode::tryClone(hpair binding) const
{
    foreach(const parent_link<BITNode>& p, parents)
    {
        tlog(-2, "TryClone next...\n");

        // when 'binding' is from ($A to $B), try to find parent with bindings ($C to $A)
        map<pHandle, pHandle>::const_iterator it =
            find_if(p.bindings->begin(), p.bindings->end(),
                    bind(std::equal_to<pHandle>(),
                         bind(&second<pHandle,pHandle>, _1),
                         binding.first));
        
        if (p.bindings->end() != it) {
            // i.e. go up the tree as far as necessary until source of binding
            // chain found.
            p.link->tryClone(hpair(it->first, binding.second));
        }
        else {
            // Then create a bound VTree from the source.
            // Create Child with this new binding:
            Rule::MPs new_args;
            Rule::CloneArgs(this->args, new_args);
            
            // If validate2 fails... (which is only used by deduction)
            ///! @todo get rid of validate2.
            if (p.link && p.link->rule && !p.link->rule->validate2(new_args))
                continue;
            
            bindingsT single_bind;
            single_bind.insert(binding);
            
            // Bind the args
            foreach(BBvtree& bbvt, new_args) {
                bbvt = BBvtree(new BoundVTree(*bind_vtree(*bbvt, single_bind)));
                ForceAllLinksVirtual(bbvt);
            }
            Btr<BoundVTree> new_target(new BoundVTree(*bind_vtree(*this->raw_target, single_bind)));
            
            BITNode* new_node = p.link->createChild(p.parent_arg_i,
                                                    this->rule,
                                                    new_args,
                                                    new_target,
                                                    single_bind,
                                                    NO_SIBLING_SPAWNING); //Last arg probably redundant now
        }
    }
}

bool BITNode::hasAncestor(const BITNode* const _p) const
{
    return this != root && STLhas(root->users[(BITNode*)this], (BITNode*)_p);
}

const set<VtreeProvider*>& BITNodeRoot::infer(int& resources,
                                              float minConfidenceForStorage,
                                              float minConfidenceForAbort)
{
    AtomSpaceWrapper *asw = GET_ASW;
    
    if (raw_target == NULL) {
        puts("Target is null, aborting.\n");
        static set<VtreeProvider*> aborted;
        return aborted;
    }
        
    if (currentDebugLevel >= 4) {
        puts("Finally proving:");
        NMPrinter(NMP_ALL)(*raw_target, 4);
    }

    if (rule) puts("non-null rule");
    
    tlog(0, "variableScoper... %d\n", resources);
    tlog(0, "children %d\n", children.size());
    /// \todo Support minConfidenceForStorage

    BITNode* variableScoper = children[0].begin()->prover;

    cprintf(0,"children %ud\n", (unsigned int) children.size());

    // These are not supposed to propagate higher than variableScoper
    //assert(eval_results.empty());
    while(resources)
    {
        tlog(0, "Resources left %d\n", resources);

        resources--;
        expandFittest();

        tlog(0, "expandFittest() ok!\n");

        const vector<set<VtreeProvider*> >& eval_res_vector_set = eval_results;
        
        foreach(VtreeProvider* vtp, *eval_res_vector_set.begin())
        {
            tlog(0, "get next TV\n");
            assert(!asw->isType(vt2h(*vtp)));
            const TruthValue& etv = asw->getTV(vt2h(*vtp));
            if (!etv.isNullTv()) {
                if (etv.getConfidence() > minConfidenceForAbort)
                    return *eval_res_vector_set.begin();
                else 
                    tlog(0,"TV conf too low to stop now: %f\n", etv.getConfidence());
            }
        }
        tlog(0, "infer(): one step ok\n");
    }

    const vector<set<VtreeProvider*> >& eval_res_vector_set = variableScoper->getEvalResults();
    
    assert(!eval_res_vector_set.empty());

    return *eval_res_vector_set.begin();
}

bool BITNode::createChildren(int i, BBvtree arg,
                             Btr<bindingsT> bindings,
                             spawn_mode spawning)
{
    assert(!arg->empty());

    tlog(1,"arg #%d. To produce:\n", i);
    rawPrint(*arg, arg->begin(),1); 

    tlog(1,"Creating children...\n");

    if (PREVENT_LOOPS && inferenceLoopWith(arg))
    {
        HypothesisRule hr(ASW());
        expandRule(&hr, i, arg, bindings, spawning);
        tlog(-2,"LOOP! Assumed Hypothetically:");
        rawPrint(*arg, arg->begin(), 2);
    }
    else
    {
        foreach(Rule *r, *root->rp)
            expandRule(r, i, arg, bindings, spawning);
    }
    tlog(1,"Rule expansion ok!\n");

    if (children[i].empty())
    {
        tlog(1,"Arg %d proof failure.\n",i);

        return false;
    }
            
    return true;
}

void BITNode::createChildrenForAllArgs()
{
    tlog(1,"---createChildrenForAllArgs()\n");  
    
    for (uint i = 0; i < args.size(); i++)
        if (!createChildren(i, args[i],
                            Btr<bindingsT>(new bindingsT),
                            ALLOW_SIBLING_SPAWNING))
            break;
}

bool BITNode::CheckForDirectResults()
{
    AtomSpaceWrapper *asw = GET_ASW;
    if (!rule || rule->isComposer())
        return false;
    
    pHandle th = _v2h(*getTarget()->begin());
    if (!asw->isType(th) && asw->getType(th) == FW_VARIABLE_NODE) {
        tlog(-1,"Proof of FW_VARIABLE_NODE prohibited.\n");
        return true;
    }
    
    Btr<set<BoundVertex> > directResult;
    
    if (USE_GENERATOR_CACHE)
    {
        directProductionArgs dp_args(rule, *bound_target);
        
        map<directProductionArgs, Btr<set<BoundVertex> >, less_dpargs>::iterator ex_it =
            haxx::DirectProducerCache.find(dp_args);
        
        directResult = (haxx::DirectProducerCache.end() != ex_it
                        ? ex_it->second
                        : haxx::DirectProducerCache[dp_args] = rule->attemptDirectProduction(bound_target));
        
        if (haxx::DirectProducerCache.end() == ex_it)
            tlog(-1,"attemptDirectProduction. Cache size %d. Target:\n", haxx::DirectProducerCache.size());
        else
            tlog(-1,"CACHED DirectProduction. Cache size %d. Target:\n", haxx::DirectProducerCache.size());
    }
    else
        directResult = rule->attemptDirectProduction(bound_target);

    if (directResult && !directResult->empty())
    {
        addDirectResult(directResult, ALLOW_SIBLING_SPAWNING);              
            
        return true;
    }
    else
    {
        tlog(3,"No direct child results generated.\n");
        return false;           
    }
}

void BITNode::expandNextLevel()
{
    AtomSpaceWrapper *asw = GET_ASW;
    tlog(-2, "Expanding with fitness %.4f\n", fitness());
    tlog(-2, "In expansion pool? %s\n", (STLhas2(root->exec_pool, this)? "YES":"NO"));
    rawPrint(*getTarget(), getTarget()->begin(), -2);
    printArgs();
    if (asw->getType(_v2h(*getTarget()->begin())) == FW_VARIABLE_NODE)    
        tlog(2, "Target is FW_VARIABLE_NODE! Intended? Dunno.\n");
    tlog(-2, " %d children exist already\n", children.size());

    // Remove this BITNode from the root's execution pool
    root->exec_pool.remove_if(bind2nd(std::equal_to<BITNode*>(), this));

    if (!Expanded) {
        CheckForDirectResults();
        createChildrenForAllArgs();
        Expanded = true;
#ifdef USE_BITUBIGRAPHER
        haxx::BITUSingleton->drawBITNode(this, children);
#endif
    }
    else {
        for (uint i = 0; i < args.size(); i++) {   
            foreach(const ParametrizedBITNode& bisse, children[i])
                bisse.prover->expandNextLevel();

            if (children[i].empty()) {
                tlog(1,"Arg %d proof failure.\n",i);        
                break;
            }           
        }       
    }
}

/* Algorithm: Evaluation */
bool BITNode::NotifyParentOfResult(VtreeProvider* new_result) const
{
    AtomSpaceWrapper *asw = GET_ASW;
    Vertex v = *(*new_result).getVtree().begin();
    pHandle h = _v2h(v);
    assert(!asw->isType(h));

    stats::Instance().ITN2atom[(BITNode*)this].insert(v); //*new_result->getVtree().begin());

    foreach(const parent_link<BITNode>& p, parents)
        p.link->EvaluateWith(p.parent_arg_i, new_result);

    return true;
}
#if 1
void BITNode::EvaluateWith(unsigned int arg_i, VtreeProvider* new_result)
{
    AtomSpaceWrapper *asw = GET_ASW;
//  tlog(-1, "ARG %d:\n", arg_i);
//  printTree(v2h(new_result), 0,-1);
    pHandle h_new_result = _v2h(*new_result->getVtree().begin());
    printArgs();

    /// If any of the existing results are equal in structure and
    /// higher confidence than the new one, skip it.

    foreach(VtreeProvider* old_result, eval_results[arg_i])
        if (IsIdenticalHigherConfidenceAtom(_v2h(*old_result->getVtree().begin()), h_new_result))
            return;

    eval_results[arg_i].insert(new_result);

#if FORMULA_CAN_COMPUTE_WITH_EMPTY_ARGS
    foreach(const set<VtreeProvider>& vset, eval_results)
        if (vset.empty())
            return;
#endif

    if (rule)
    {
        vector<Btr<set<VtreeProvider*> > > child_results;

        // Fill in child_results:
        // arg_i will only contain this result
        // other slots will contain their existing result(s) (if any)
        for (uint i=0;i<args.size();i++)
        {
            child_results.push_back(Btr<set<VtreeProvider*> >(new set<VtreeProvider*>));

            if (i != arg_i)
                *child_results[i] = eval_results[i];
            else
                child_results[i]->insert(new_result);

/*          foreach(const BoundVertex& bv, *miv_set)
                if (bv.bindings)
                    removeRecursionFromHandleHandleMap(bv.bindings); */
        }

        // Set of possible {vectors of args for each slot}?
        set<vector<VtreeProvider*> > argVectorSet;
        // Commented this out, this seems to do nothing... -- Joel
        //int s1 = argVectorSet.empty() ? 0 : argVectorSet.begin()->size();

//      removeRecursion(child_results);
        WithLog_expandVectorSet<vector<VtreeProvider*>,
                                set<VtreeProvider*>,
                                vector<Btr<set<VtreeProvider*> > >::const_iterator >
            (child_results, argVectorSet);

        BoundVertex next_result;

        for (set<vector<VtreeProvider*> >::iterator a = argVectorSet.begin();
             a!= argVectorSet.end(); a++)
            if (!a->empty() && (rule->hasFreeInputArity() 
                                || a->size() >= rule->getInputFilter().size()))
            {
                /// Arg vector size excession prohibited.

                const vector<VtreeProvider*>& rule_args = *a;

                int s2 = rule_args.size();

                boost::indirect_iterator<vector<VtreeProvider*>::const_iterator, const VtreeProvider > ii;
                RuleApp* ruleApp = NULL;
                ValidateRuleArgs(rule_args.begin(), rule_args.end());

                foreach(VtreeProvider* ra, rule_args)
                {
                    pHandle h = _v2h(*ra->getVtree().begin());
                    if (!asw->isSubType(h, HYPOTHETICAL_LINK) &&
                        asw->getTV(h).getConfidence() < MIN_CONFIDENCE_FOR_RULE_APPLICATION)
                        goto next_args;
                }

                tlog(-1, "Evaluating...\n");

                ruleApp = new RuleApp(rule);
                next_result = ruleApp->compute(rule_args.begin(), rule_args.end());

                assert(!asw->isType(_v2h(next_result.value)));
                
                ii = rule_args.begin();

                if (validRuleResult(next_result,
                        boost::indirect_iterator<vector<VtreeProvider*>::const_iterator,
                            const VtreeProvider>(rule_args.begin()),
                        boost::indirect_iterator<vector<VtreeProvider*>::const_iterator,
                            const VtreeProvider>(rule_args.end()),
                        Btr<bindingsT>(new bindingsT()))) {

                    NotifyParentOfResult(ruleApp);

                    root->hsource[_v2h(next_result.value)] = const_cast<BITNode*>(this);

                    if (root->getRecordingTrails()) {
                        foreach(VtreeProvider* v, rule_args)
                        {
//                          root->inferred_from[_v2h(next_result.value)].push_back(_v2h(v.value));
                            haxx::inferred_from[_v2h(next_result.value)].push_back(
                                    _v2h(*v->getVtree().begin()));
//                          root->inferred_with[_v2h(next_result.value)] = rule;
                            haxx::inferred_with[_v2h(next_result.value)] = rule;
                        }                       
                    }
                }
next_args:; //! @todo replace goto!
                //! @todo memory leak! Segfaults if we try to free this memory.. not sure why.
                //! Probably because the destructor deletes rule arguments?
                // delete ruleApp;
            }
    }
    else
    {
        tlog(-3, "Target produced!\n");

        if (this != root && root)
        {
            foreach(const set<VtreeProvider*>& eval_res_set, eval_results)
                foreach(const parent_link<BITNode>& p, parents)
                    p.link->eval_results[0].insert(eval_res_set.begin(), eval_res_set.end());
        }
            
/*      foreach(const set<VtreeProvider>& eval_res_set, GetEvalResults())
            foreach(const BoundVertex& eval_res, eval_res_set)
                printTree(v2h(eval_res.value),0,-3);
*/
    }

}

/// \todo Currently always uses ForAllRule to generalize. Should be different for varScopeLink
/// \todo Currently return value topology is wrong.

BoundVertex BITNodeRoot::Generalize(Btr<set<BoundVertex> > bvs, Type _resultT) const
{
    vector<Vertex> ForAllArgs;
    Vertex v = ATOM;
    BoundVertex new_result(v);

    const float min_confidence = 0.0001f;

    if (!bvs->empty())
    {
        cprintf(0,"\n");
        tlog(0,"Generalizing results:\n");

        foreach(const BoundVertex& b, *bvs)
            if (GET_ASW->getTV(_v2h(b.value)).getConfidence() > min_confidence)
            {
                printTree(_v2h(b.value),0,0);
                ForAllArgs.push_back(b.value);
            }

        if (_resultT == FORALL_LINK) {
            //new_result = FORALLRule(ASW(), Handle::UNDEFINED).compute(ForAllArgs);
            new_result = FORALLRule(ASW(), PHANDLE_UNDEFINED, FORALL_LINK).compute(ForAllArgs);
        } 
        else {
            //new_result = PLNPredicateRule(ASW(), Handle::UNDEFINED).compute(ForAllArgs);
            new_result = PLNPredicateRule(ASW(), PHANDLE_UNDEFINED, VARIABLE_SCOPE_LINK).compute(ForAllArgs);
        }

        tlog(0,"\nCombining %d results for final unification. Result was:\n", ForAllArgs.size());
        printTree(_v2h(new_result.value),0,0);
    }
    else
        tlog(1,"NO Results for the root query.\n");

    return new_result;
}



/* Fitness evaluation */



int BITNode::number_of_free_variables_in_target() const
{
    AtomSpaceWrapper *asw = GET_ASW;
    /// Use set<> to prevent re-counting of the already-found Handles
    
    set<pHandle> vars;
    
    for(vtree::iterator v  = getTarget()->begin(); v != getTarget()->end(); v++)
        if (asw->getType(_v2h(*v)) == FW_VARIABLE_NODE)
            vars.insert(_v2h(*v));   

    tlog(4,"number_of_free_variables_in_target: %d\n", vars.size());
        
    return (int)vars.size();
}

float BITNode::my_solution_space() const
{
    return counted_number_of_free_variables_in_target - getTarget()->size()*100.0f;
}

float BITNode::fitness() const
{       
    const float CONFIDENCE_WEIGHT = 10000.0f;
    const float DEPTH_WEIGHT = 100.0f;
    const float SOLUTION_SPACE_WEIGHT = 0.01f;
    const float RULE_PRIORITY_WEIGHT = 0.0001f;

//tlog(3,"fitness(): %f %f %f %f %f\n",my_bdrum, -depth, _bdrum, -1.0f*_bdrum,-depth -_bdrum);  
    
    return  -1.0f*CONFIDENCE_WEIGHT     * my_bdrum
            -1.0f*DEPTH_WEIGHT          * depth
            -1.0f*SOLUTION_SPACE_WEIGHT * my_solution_space()
            +1.0f*RULE_PRIORITY_WEIGHT  * (rule ? rule->getPriority() : 0);
    
/*  \todo Use arity in the spirit of the following:

int missing_arity = (int)rule->getInputFilter().size();
        
        foreach(Btr<set<BoundVertex> > rset, child_results)
        {
            if (!rset->empty())
                missing_arity--;
        }
        
        
        return RULE_PRIORITY_WEIGHT*priority[rule] / (missing_arity+1) / (DEPTH_PRIORITY_WEIGHT*depth)
                - _bdrum;
*/      
}

void BITNode::findFittest(BITNode*& bisse, float& best_fitness)
{
    bisse = (*root->exec_pool.begin());
    best_fitness = bisse->fitness();
    float next_fitness = 0.0f;
    foreach(BITNode* bis, root->exec_pool)
        if (best_fitness < (next_fitness=bis->fitness()))
        {
            best_fitness = next_fitness;
            bisse = bis;
        }
}

float all_best_fitness = 0.0f;

void BITNode::expandFittest()
{
    if (SOFTMAX == root->fitnessEvaluator)
    {
        /*  tlog(2,"Fitness table: (%d)\n", root->exec_pool.size());

        currentDebugLevel=2;

        printFitnessPool();

        currentDebugLevel=-4;
        */
        //////////
        //foreach(BackInferenceTreeNode* b, exec_pool)
        //  partition += 

        // Get the fitness of all nodes in the execution pool
        vector<double> fitnesses;
        transform(root->exec_pool.begin(), root->exec_pool.end(), back_inserter(fitnesses), std::mem_fun(&BITNode::fitness));

        //float partition = o;
        //const float temperature = 0.1f;

        int accuracy = RAND_MAX-1; //1000*1000;
        //float selection_weight_coordinate = exp(selection / temperature);

        double total_weight = 0.0, accumulated_weight = 0.0; 


        foreach(double Qb, fitnesses)
        {
            // NOTE! Too low temperature will kill!
            assert(temperature > 0.00001);
//          double a1 = exp((-1.0/Qb) / 0.00001);
            total_weight += exp((-1.0/Qb) / temperature);
        }

        /*time_t seconds;
        time(&seconds);

        srand((unsigned int) seconds);*/

        /*cout<< rand() << endl;
        cout<< rand() << endl;
        cout<< rand() << endl;
        */
        int r = rand();
        double selection = (1.0f/accuracy) * (r%accuracy) * total_weight;

        accumulated_weight = 0.0f; 
        int i=0;
        BITNodeRoot::exec_poolT::iterator ei = root->exec_pool.begin();

        foreach(double Qb, fitnesses)
        {
            //  partition += exp((-1.0/Qb) / temperature);
            accumulated_weight += exp((-1.0/Qb) / temperature);
            if (accumulated_weight > selection)
                break;
            i++;
            ei++;
        }

        // If we didn't reach 'selection' even after the last step, we're in trouble.
        assert(accumulated_weight > selection);
        /*
        printf("EXPANDING:  %.8f / %.8f / %.8f/ %.8f/ %d\n", selection, exp((-1.0f/fitnesses[i]) / temperature), accumulated_weight,
        total_weight, r);
        (*ei)->tlog(-4, ": %f / %d [%d]\n", (*ei)->fitness(), (*ei)->children.size(), (int)(*ei));
        */
        (*ei)->expandNextLevel();

        return;
    }
    else if (root->fitnessEvaluator == RANDOM)
    {
        BITNodeRoot::exec_poolT::iterator ei = root->exec_pool.begin();
        int e = rand() % root->exec_pool.size();
        for (int i=0; i < e; i++)
            ++ei;

        (*ei)->expandNextLevel();
    }
    else
    {
        print_progress();

        BITNode* bisse = NULL;
    
        if (!root->exec_pool.empty())
        {
            if (currentDebugLevel>1) //Sort and print
            {
                if (!root->exec_pool_sorted)
                {
                    root->exec_pool.sort(BITNodeFitnessCompare());
                    root->exec_pool_sorted = true;
                }

                std::list<BITNode*>::iterator i = root->exec_pool.begin();
                (*i)->tlog(3, ": %f / %d [%ld]\n", (*i)->fitness(), (*i)->children.size(), (long)(*i));

                for (++i; i != root->exec_pool.end(); i++)      
                    (*i)->tlog(3, ": %f / %d [%ld]\n", (*i)->fitness(), (*i)->children.size(), (long)(*i));

                bisse = (*root->exec_pool.begin());
            }
            else //Just find the best one
            {
                float best_fitness = 0.0f;
                findFittest(bisse, best_fitness);
                if (all_best_fitness > best_fitness)
                    all_best_fitness = best_fitness;

                cprintf(-2, "%.4f / %.4f\n", all_best_fitness, best_fitness);
            }

            bisse->expandNextLevel();
        }
    }
}

bool BITNodeFitnessCompare::operator()(BITNode* lhs, BITNode* rhs) const
{
//return lhs>rhs;
    
    if (!lhs)
        return false;
    if (!rhs)
        return true;

    float lfit = lhs->fitness();
    float rfit = rhs->fitness();
    float fit_diff = lfit - rfit;
//printf("%f_", fit_diff);  
    const float fitness_epsilon = 0.00000001f;
    return (fit_diff > fitness_epsilon ||
            ((fabs(fit_diff) < fitness_epsilon) && lhs>rhs));
}



/* Action */
string BITNodeRoot::extract_plan(pHandle h, unsigned int level,
                                 vtree& do_template, pHandleSeq& plan) const
{
    AtomSpaceWrapper *asw = GET_ASW;
    map<pHandle, vtree> bindings;
    stringstream ss;
    
    if (asw->isType(h)) {
        ss << "Can't make plan for a NULL/virtual target" << endl;
    }
    
    map<pHandle,Rule*> ::const_iterator rule = haxx::inferred_with.find(h);
    if (rule != haxx::inferred_with.end()) {
        foreach(pHandle arg_h, haxx::inferred_from[h]) {
            if (unifiesTo(GET_ASW, do_template, make_vtree(arg_h),
                          bindings, bindings, true)) {
                ss << "Satisfies do_template:" << endl; 
                ss << printTree(arg_h,level+1,0);
                plan.push_back(arg_h);
            }
            ss << extract_plan(arg_h, level+1, do_template, plan);
        }
    }
    return ss.str();
}

string BITNodeRoot::extract_plan(pHandle h) const
{
    AtomSpaceWrapper *asw = GET_ASW;
    stringstream ss;
    vtree do_template = mva((pHandle)EVALUATION_LINK,
                            NewNode(PREDICATE_NODE, "do"),
                            mva((pHandle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$999999999")));
    pHandleSeq plan;
    ss << extract_plan(h,0,do_template,plan);
    ss << "PLAN BEGIN" << endl;
    for (pHandleSeq::reverse_iterator i = plan.rbegin(); i!=plan.rend(); i++)
        printTree(*i,0,-10);
    ss << "PLAN END" << endl;
    if (plan.size()>0) {
        ss << "[plan found, exiting]" << endl;
    }
    return ss.str();
}

string BITNode::loopCheck() const
{
    stringstream ss;
    ss << "-" << depth << " " << (long) this << ":" << totalChildren() << endl;
    for (vector<set<ParametrizedBITNode> >::const_iterator i = children.begin();
            i!=children.end(); i++) {
        foreach(const ParametrizedBITNode& pbit, *i) {
            ss << "  ";
            ss << pbit.prover->loopCheck();
        }
    }
    return ss.str();
}

void BITNodeRoot::setTreeDepth(const unsigned int newDepth) { treeDepth =
newDepth; }

#if 0
void test_pool_policy()
{
/*  Rule* deductionR1 = RuleRepository::Instance().rule[Deduction_Implication];
    Rule* deductionR2 = RuleRepository::Instance().rule[Deduction_Inheritance];
*/
    Rule* deductionR2 = NULL; //Should have no effect

    assert(obeysPoolPolicy(deductionR2,
        Btr<tree<Vertex> > (new tree<Vertex>(mva((pHandle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "killed"),
                    mva((pHandle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )))));

    assert(!obeysPoolPolicy(deductionR2,
        Btr<tree<Vertex> > (new tree<Vertex>(mva((pHandle)EVALUATION_LINK,
                    NewNode(PREDICATE_NODE, "killed"),
                    mva((pHandle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(FW_VARIABLE_NODE, "$target")
                            )
            )))));

    assert(obeysPoolPolicy(deductionR2,
        Btr<tree<Vertex> > (new tree<Vertex>(mva((pHandle)EVALUATION_LINK,
                    NewNode(FW_VARIABLE_NODE, "$action"),,
                    mva((pHandle)LIST_LINK,
                                NewNode(FW_VARIABLE_NODE, "$killeri"),
                                NewNode(CONCEPT_NODE, "Osama")
                            )
            )))));

    puts("Pool policy test ok!");
}

#endif

/* PRINTING METHODS */

string BITNode::printResults() const
{
    std::stringstream ss;
    foreach(const set<VtreeProvider*>& vset, eval_results)
    {
        ss << "[ ";
        cout << "[ "; 
        foreach(VtreeProvider* vtp, vset)
            ss << printTree(_v2h(*vtp->getVtree().begin()),0,-10);
//          printf("%d ", _v2h(bv.value));
        ss << "]" << endl;
        cout << "]" << endl;
    }
    return ss.str();
}

string BITNode::print(int loglevel, bool compact, Btr<set<BITNode*> > usedBITNodes) const
{
    stringstream ss;
    stringstream ss1;

    // Initialise the set of BITNodes that have already been printed
    if (usedBITNodes == NULL) {
        usedBITNodes = Btr<set<BITNode*> >(new set<BITNode*>());
    }

#define prlog cprintf ///! @todo replace prlog with logger
    if (rule) {
        if (compact) {
            ss << repeatc(' ', depth*3) << rule->name << endl;
            //prlog(loglevel, ss.str().c_str());
        } else {
            ss << repeatc(' ', depth*3) << rule->name << " ([" << (long)this
                << "])" << endl;
            ss << repeatc(' ', (depth+1)*3) << "[ ";
            if (direct_results) {
                foreach(const BoundVertex& bv, *direct_results)
                    ss << _v2h(bv.value) << " ";
            }
            ss << "]\n";
        }
    } else {
        ss << "root" << endl;
    }
    ss1 << ss.str();
    prlog(loglevel, ss.str().c_str());
    ss.clear();
    
    if (STLhas2(*usedBITNodes, this)) {
        // If this subtree has already been printed...
        ss << repeatc(' ', (depth+1)*3) << "(loop)" << endl;
        ss1 << ss.str();
        prlog(loglevel, ss.str().c_str());
        ss.clear();
    } else {
        usedBITNodes->insert((BITNode*)this);

        int ccount=0;
        for (vector<set<ParametrizedBITNode> >::const_iterator i =
                children.begin(); i!=children.end(); i++) {
            if (compact)
                ss << repeatc(' ', (depth+1)*3) << "#" << ccount << endl;
            else {
                ss << repeatc(' ', (depth+1)*3) << "ARG #" << ccount << ":" << endl;
                ss << repeatc(' ', (depth+1)*3) << "---" << endl;
            }
            ccount++;
            int n_children = i->size();

            ss1 << ss.str();
            prlog(loglevel, ss.str().c_str());
            ss.clear();
            foreach(const ParametrizedBITNode& pbit, *i) {
                if (!compact || STLhas(stats::Instance().ITN2atom, pbit.prover))
                    ss1 << pbit.prover->print(loglevel, compact, usedBITNodes);
            }
        }
    }
    return ss1.str();
}

static int _trail_print_more_count = 0;

string BITNodeRoot::printTrail(pHandle h, unsigned int level) const
{
    AtomSpaceWrapper *asw = GET_ASW;
    stringstream ss;
    if (h == PHANDLE_UNDEFINED || asw->isType(h))
        ss << "Error, trying to print trail for NULL / Virtual atom." << endl;

    map<pHandle,Rule*> ::const_iterator rule = haxx::inferred_with.find(h);
    if (rule != haxx::inferred_with.end())
    {
        string name;
        
        if (find(rp->begin(), rp->end(), rule->second) == rp->end()) {
            name = "<!!! INVALID RULE !!!>";
        } else {
            name = rule->second->getName();
        }
    
        ss << repeatc(' ', level*3) << "[" << h << "] was produced by applying ";
        ss << name << " to:\n";

        map<pHandle,vector<pHandle> >::const_iterator h_it = haxx::inferred_from.find(h);
        assert (h_it != haxx::inferred_from.end());

        NMPrinter nmp(NMP_ALL, 0,
                NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE, 0,
                level+1);

        foreach(pHandle arg_h, h_it->second)
        {
            ss << nmp.toString(arg_h, -10);
            ss << printTrail(arg_h, level+1);
        }
    }
    else
        ss << repeatc(' ', level*3) << "which is trivial (or axiom).\n";
    return ss.str();
}

string BITNodeRoot::printTrail(pHandle h) const
{
    stringstream ss;
    AtomSpaceWrapper* asw = ASW();
    if (asw->isValidPHandle(h)) {
        ss << printTree(h,0,0);
        ss << printTrail(h,0);
        cout << ss.str();
    } else {
        ss << "Unknown pHandle " << h << endl;
    }
    return ss.str();
}

string BITNode::printArgs() const
{
    stringstream ss;
    if (args.size() == 0) {
        ss << "No arguments to BITNode [" << (long)this << endl;
        return ss.str();
    }
    ss << "BITNode [" << (long)this << "] has " << args.size() << " args:" <<endl;
    foreach(meta _arg, args)
        ss << NMPrinter(NMP_ALL).toString(*_arg, -2);
    cprintf(-2,ss.str().c_str());
    return ss.str();
}

string BITNode::printFitnessPool()
{
    stringstream ss;
    ss << tlog(0,"Fitness table (%d):\n", root->exec_pool.size());

    if (!root->exec_pool.empty()) {
        if (!root->exec_pool_sorted) {
            root->exec_pool.sort(BITNodeFitnessCompare());
            root->exec_pool_sorted = true;
        }

        for (std::list<BITNode*>::iterator i = root->exec_pool.begin();
                i != root->exec_pool.end(); i++) {
            ss << (*i)->tlog( 0, "fitness: %f / C:%d / P:%d\n", (*i)->fitness(),
                    (*i)->children.size(), (*i)->parents.size() );
        }
    }
    return ss.str();
}

static bool bigcounter = true;

// haxxUsedProofResources wasn't being printed, even before I commented out???
// -- JaredW
string BITNode::tlog(int debugLevel, const char *format, ...) const
{
    stringstream ss;
    if (debugLevel > currentDebugLevel) return "";

    //if (test::bigcount == 601) {
    //    ss << "Debug feature." << endl;
    //}

    ss << (bigcounter? (++test::bigcount) : depth) << " Pool="
        << (unsigned int) root->exec_pool.size() << "/" << root->inferenceNodes
        /*<< haxxUsedProofResources*/ << " [" << (long)this << "-"
        << (rule ? (rule->name.c_str()) : "ROOT") << "] ";

    char buf[5000];
    va_list ap;
    va_start(ap, format);
    int answer = vsprintf(buf, format, ap);
    ss << buf;

    cout << ss.str().c_str();
    fflush(stdout);
    va_end(ap);
    return ss.str();
}

string BITNode::printChildrenSizes() const
{
    stringstream ss;
    //if (currentDebugLevel>=3)
    //    ss << "next chi...0";
    ss << tlog(3,"next chi...0");
    for(uint c=0; c< children.size(); c++)
    {
        ss << tlog(3,"(%d:%d), ", c, children[c].size());
        ss << tlog(3,"\n");
    }
    return ss.str();
}

string BITNode::printTarget() const
{
    stringstream ss;
    ss << "Raw target:\n";
    ss << rawPrint(*raw_target, raw_target->begin(),0);
    ss << "Bound target:\n";
    ss << rawPrint(*getTarget(), getTarget()->begin(),0);
    cprintf(0, ss.str().c_str());
    return ss.str();
}


/*
/// No longer needed? MAY NOT BE UP2DATE:
/// Children are also cloned, but results (gained so far) will be shared instead.
BITNode* BITNode::Clone() const
    {
        BITNode* ret = new BITNode(*this);
        inferenceNodes++; 
        
        vector<set<BITNode*> > new_children;
        
        for (vector<set<BITNode*> >::iterator childi = ret->children.begin();
                childi != ret->children.end(); childi++)
        {
//      foreach(set<BITNode*>& child, ret->children)
            set<BITNode*> new_c_set;
            
            //for (set<BITNode*>::iterator bit=childi->begin();bit!=childi->end();bit++)
            foreach(BITNode* const& bitree, *childi)
                new_c_set.insert(bitree->Clone());
            
            new_children.push_back(new_c_set);
        }
        
        ret->children = new_children;
        
        ret->target = Btr<vtree>(new vtree(*target));
        foreach(meta m, args)
            ret->args.push_back(Btr<vtree>(new vtree(*m)));

        return ret;     
    }

struct target_binder
    {       
        bindingsT bindings;
        target_binder(const bindingsT& _bindings) : bindings(_bindings) {}
        void operator()(BITNode* b)
        {
            cprintf(3,make_subst_buf(bindings).c_str());
                        
                meta target = b->getTarget();

                cprintf(3,"Before bind:");
                rawPrint(*target,target->begin(),3);
            
                b->setTarget(bind_vtree(*target, bindings));
                
                cprintf(3,"After:");
                rawPrint(*b->getTarget(),b->getTarget()->begin(),3);
        }
    };
    
BITNode* BITNode::Bind(bindingsT b)
    {
        ApplyDown(target_binder(b));
        return this;        
    }
*/  

/*#ifdef WIN32

bool indirect_less_BITNode::operator()(BITNode* lhs, BITNode* rhs) const
{
    if (lhs->rule < rhs->rule)
        return true;
    else if (lhs->rule > rhs->rule)
        return false;

    if (lhs->args.size() < rhs->args.size())
        return true;
    else if (lhs->args.size() > rhs->args.size())
        return false;

    /// If target-determined
    if (lhs->rule && !lhs->rule->isComposer())
    {
        if (less_vtree()(*lhs->getTarget(), *rhs->getTarget()))
            return true;
        else if (less_vtree()(*rhs->getTarget(), *lhs->getTarget()))
            return false;
    }
    else
    {
        for(int i=0; i<lhs->args.size(); i++)
            if (less_vtree()(lhs->args[i]->getStdTree(), rhs->args[i]->std_tree()))
                return true;
            else if (less_vtree()(rhs->args[i]->getStdTree(), lhs->args[i]->std_tree()))
                return false;
    }

    return false;
}

struct indirect_less_BITNode : public binary_function<BITNode*, BITNode*, bool>
{
    bool operator()(BITNode* lhs, BITNode* rhs) const;
};

typedef set<BITNode*, indirect_less_BITNode> BITNodestoreT;

struct BITNodehash :  public stdext::hash_compare<BITNode*>
{
    /// hash function
    size_t operator()(BITNode* b) const
    {
        size_t ret = 0;
        ret += (int)b->rule;

        ret += BoundVTree(*b->getTarget()).getFingerPrint();

        foreach(Btr<BoundVTree> bvt, b->args)
            ret += bvt->getFingerPrint();

        return ret;
    }
    bool operator()(BITNode* lhs, BITNode* rhs) const
    {
        if (lhs->rule < rhs->rule)
            return true;
        else if (lhs->rule > rhs->rule)
            return false;

        if (lhs->args.size() < rhs->args.size())
            return true;
        else if (lhs->args.size() > rhs->args.size())
            return false;

        /// If target-determined
        if (lhs->rule && !lhs->rule->isComposer())
        {
            /// \todo Should look at std_tree forms of targets here!

            if (less_vtree()(*lhs->getTarget(), *rhs->getTarget()))
                return true;
            else if (less_vtree()(*rhs->getTarget(), *lhs->getTarget()))
                return false;
        }
        else
        {
            for(int i=0; i<lhs->args.size(); i++)
                if (less_vtree()(lhs->args[i]->getStdTree(), rhs->args[i]->std_tree()))
                    return true;
                else if (less_vtree()(rhs->args[i]->getStdTree(), lhs->args[i]->std_tree()))
                    return false;
        }

        return false;
    }
};

#include <hash_set>

typedef hash_set<BITNode*, BITNodehash> BITNodestoreT;

#else*/

#if 0
template<typename T1, typename T2, typename T3>
void combine_maps(const map<T1, T2>& m12, const map<T2, T3>& m23, map<T1, T2>& m13)
{
    typedef pair<T1, T2> p12T;
    typedef pair<T2, T3> p23T;
    foreach(p12T p12, m12)
    {
        map<T2,T3>::const_iterator it23f = m23.find(p12.first);
        map<T2,T3>::const_iterator it23s = m23.find(p12.second);

        m13[  ((m23.end() == it23f) ? p12.first  : it23f->second)]
            = ((m23.end() == it23s) ? p12.second : it23s->second);
    }
}
#endif

    /*
class priorityComp :  public std::binary_function<Rule*,Rule*,bool>
{
    const std::map<Rule*, float>& priority;
public:
    priorityComp(const std::map<Rule*, float>& _priority) : priority(_priority)
    {}

    bool operator()(Rule* x, Rule* y) //true if x has higher priority
    {
        std::map<Rule*, float>::const_iterator xi, yi;

        if ((yi=priority.find(y)) == priority.end())
            return true;
        else if ((xi=priority.find(x)) == priority.end())
            return false;
        else
            return (xi->second > yi->second);
    }
};
    
*/

#endif

}} //namespace opencog::pln




