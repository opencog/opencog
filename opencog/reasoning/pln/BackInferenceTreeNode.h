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

/** @file
 *
 * BackInferenceTree related code.
 * Original prototype code by Ari A. Heljakka Sep/2006.
 * Port by Joel Pitt 2008.
 * 
 * @author Ari A. Heljakka
 * @author Joel Pitt
 * @author Jared Wigmore
 * @date 2006-2010
 *
 */
#ifndef BACK_INFERENCE_TREE_NODE
#define BACK_INFERENCE_TREE_NODE

#define FORMULA_CAN_COMPUTE_WITH_EMPTY_ARGS 0

#include <stack>
#include <exception>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include "AtomSpaceWrapper.h"
#include "rules/RuleApp.h"
#include "utils/NMPrinter.h"

#include <opencog/util/iostreamContainer.h>

#include "FitnessEvaluator.h"

class BITNodeUTest;
class PLNUTest;

/* Don't define USE_BITUBIGRAPHER unless cmake actually found Ubigraph
 * installed */
#ifdef HAVE_UBIGRAPH
// XXX Doing this here creates a circular dpendency between PLN and ubigrapher,
// which breaks the build. So disable for now.
// #define USE_BITUBIGRAPHER
#endif

typedef unsigned int BITNodeID;

namespace opencog{namespace pln{ class BITNode;}}
namespace haxx {
    BITNode* getBITNode(BITNodeID id);
    const std::map<pHandle, std::vector<pHandle> > & get_inferred_from();
    const std::map<pHandle, RulePtr> & get_inferred_with();
}


namespace opencog {

#ifdef USE_BITUBIGRAPHER
class BITUbigrapher;
#endif

namespace pln {

class BITNode;
class BITNodeRoot;  //!< Root of a BIT, controls inference
class RuleProvider; //!< Provide rules when expanding the tree
class InferenceCache;
template <typename _exec_poolT> struct ExpansionPoolUpdater;

/** A class for runtime errors during BIT expansion
  *
  * Currently not used, but should be used as code is refactored.
  */
class BITException : public std::runtime_error
{
public:
    BITException(const std::string& msg)
        : std::runtime_error(msg) { };
};

struct BITNodeFitnessCompare : public std::binary_function<BITNode*, BITNode*, bool>
{
    /**
     * If lhs and rsh and almost equal (that is their differences is below
     * a certain epsilon) then it returns lhs > rsh
     * otherwise it returns lhs->fitness() > rhs->fitness()
     */
    bool operator()(BITNode* lhs, BITNode* rhs) const;
};

/** 
 *  Store the knowledge of which argument slot I inhabit in my parent's Rule,
 *  and which variable-to-variable bindings the parent is applying to me, ie.
 *  "How does the parent view me"
 */
template<typename T>
struct parent_link
{
    T* link;
    uint parent_arg_i;
    Btr<bindingsT> bindings;

    parent_link() : link(NULL), parent_arg_i(0) {}
    parent_link( T* _link, uint _parent_arg_i,
            Btr<bindingsT> _bindings = Btr<bindingsT>(new bindingsT))
        : link(_link), parent_arg_i(_parent_arg_i), bindings(_bindings) {}

    bool operator<(const parent_link& rhs) const {
        // Check first whether links (usually BITNodes) are less than one
        // another. Then check whether arg slot is different, and then
        // whether the bindings are different.
        return link<rhs.link
            || (link==rhs.link && parent_arg_i < rhs.parent_arg_i)
            || (link==rhs.link && parent_arg_i == rhs.parent_arg_i &&
                    *bindings < *(rhs.bindings));
    }
};

/** The generic BITNode in which some variables may be bound to other variables.
 *
 * A single ParametrizedBITNode can be used to represent various trees by using
 * different variable bindings. These bindings are for variables internal to
 * the PLN backward chainer as opposed to the variables within PLN theory.
 */
class ParametrizedBITNode {
public:
    Btr<bindingsT> bindings;
    BITNode* prover;

    ParametrizedBITNode(BITNode* _prover, Btr<bindingsT> _bindings);

    bool operator<(const ParametrizedBITNode& rhs) const
    {
        return (prover<rhs.prover  || (prover==rhs.prover && bindings<rhs.bindings));
    }
};

/** The basic node in the Backward Inference proof Tree
 *
 * Notes about the Backward Inference Tree and the Backward Chaining process.
 * - Circularity is not tolerated, though the algorithm could probably be
 *   expressed in a way that would allow for this. However, it turns out that
 *   circularity is never needed (I am not sure if this can be rigorously
 *   proven; possibly so).
 * - Subtrees are recycled by storing them in BITNode "templates". A subtree is
 *   reached via a channel that has inheritance-bindings associated to it, so
 *   that if we are already proving Inh(A,$1), we don't need a new tree for
 *   proving Inh(A,$2), but instead use the same tree with inheritance-binding
 *   ($1->$2).
 * - FW_VARIABLE_NODEs are used to denote placeholders for "future atoms that
 *   the bw chainer finds". Normal VariableNodes denote variables in
 *   variable-laden links that are real and reside in the AtomSpace.
 *   FW_VARIABLE_NODEs should normally not be stored in the AtomSpace, but
 *   currenty there are.
 *
 * The basic BW chainer dispatch cycle for expanding the tree with 1 inference
 * step is (omitting procedures such as the argument place of each BITNode,
 * result validation, expansion pool filtering etc.):
 *
 * -# BITNodeRoot.executionPool.insert(root)
 * -# currentBITNode = getFittest(root.executionPool)
 * -# root.executionPool += currentBITNode.expand()
 * -# if (not sufficient(root.getResults())): goto #2
 *
 * where...
 *
 * getFittest(pool):
 * \code
 *   returns the fittest entry in pool. This depends on a fitness function
 *   related to tree pruning algorithm discussed in DAPPIE. 
 * \endcode
 * 
 * currentBITNode.expand():
 * \code
 *    For currentBITNode.target, put into RS all Rule objects that could create it
 *    For each Composer C in RS:
 *         childCreate(C,RS)
 *       For each Generator G in RS:
 *         put into GS all possible atoms produced by G that match currentBITNode.target
 *    For each Handle H in GS:
 *      If GS was produced without binding any FW variables:
 *        put H into currentBITNode.results
 *      Else For each variable bind (var_i => H_i) made to get H:
 *        For the owner O of var_i:
 *          O.tryClone(var_i => H_i)
 *      C.evaluate(H)
 * \endcode
 *
 * O.tryClone(var_i => H_i):
 * \code
 *    For each parent P of O:
 *      If P has inheritance-bindings (var2_i => var_i):
 *        P.tryClone(var2_i => H_i)
 *      Else:
 *        P.childCreate(a copy of O such that all instances of var_i have been
 *        replaced with H_i)
 * \endcode
 * 
 * P.childCreate(Rule R, args AS):
 * \code
 *    If root.recyclerPool does not contain a BITNode that can be produced as B'
 *    from (R,AS) by some variable bindings {(Var_i=>Var2_i)} of variables in AS
 *    to other variables, then:
 *      Add B' to root.recyclerPool
 *      B = B'
 *    Else:
 *      B = ParametrizedBITNode(B') with inheritance-bindings (Var_i=>Var2_i)
 *      such that B owns each new variable from R.o2i
 *      Mark P and all users of P as users of B.
 *    add B to P.children
 *    make P the parent of B
 *    return B
 * \endcode
 * 
 * BITNode.evaluate(latest_result) = try to compute all the results of the Rule
 * of this BITNode by using latest_result in conjunction with old results.
 * \code
 *    For each new_result in results:
 *      For each parent:
 *        call parent.Evaluate(new_result). 
 * \endcode
 * 
 *
 * The following two todos suggest approaches that aren't quite right -- JaredW
 * @todo Allow forward chaining using BIT... by accepting a wildcard target, and
 * only accepting results with reasonable confidence.
 *
 * @todo Tackle revision.
 * - first make sub-BITs shared between BIT grounded by a root.
 * - at the end of infer() - revise the TVs within the BIT.
 * - when a result is computed for a subtree, mark it as revised.
 *
 * @remarks BITNode supports pre-bindings extensively, but it's unclear whether
 * they are obsolete or not. Joel: It's also unclear whether anyone understands
 * what this remark really means.
 *
 */
class BITNode
{
    friend class BITNodeFitnessCompare;
    friend class BDRUMUpdater;
    friend class ExpansionPoolUpdater<std::list<BITNode*> >;
    friend class BITNodeRoot;
    friend class ExplicitlyEvaluatedBITNode;

    // Let unit tests inspect BITNode state
    friend class ::BITNodeUTest;
    friend class ::PLNUTest;
    friend class PLNModule;
#ifdef USE_BITUBIGRAPHER
    friend class opencog::BITUbigrapher;
#endif

//protected:
public:
    /** Whether to allow sibling spawning */
    enum spawn_mode { NO_SIBLING_SPAWNING = 0, ALLOW_SIBLING_SPAWNING };

    /// Contains the target of this inference state node
    meta bound_target, raw_target;

    /// For heuristics; store the nr so you don't have to re-count.
    uint counted_number_of_free_variables_in_target;

    /// parent is the set of parent_links that refer upwards in the BIT.
    /// A set is used to store the parent links because a node can have
    /// multiple parents when the parent node has different
    /// bindings
    mutable std::set<parent_link<BITNode> > parents;

    /// The results produced by combining all the possible results
    /// for my arguments (contained as sets), with an entry in the vector for
    /// each argument.
    std::vector<std::set<VtreeProvider*> > eval_results;

    /// Note that depth is not necessarily const because
    /// BITNode can be reused at different parts in the BIT
    /// which are not necessarily at the same depth,
    /// via for instance the ParametrizedBITNode class, which is a
    /// wrapper for BITNode, not sure how depth works then though.
    unsigned int depth;
    BITNodeRoot* root;

    bool Expanded;

    /// The Rule that the child_results of this state object's child_results will be
    /// associated with. This BITNode can be thought of as an implementation of
    /// this Rule
    RulePtr rule;
    
    /// bdrum = the Best (confidence of) Direct Result Under Me
    float my_bdrum;
    
    typedef std::set<vtree, less_tree_vertex> vtreeset;

    /// Targets of all atoms above me... obsolete but partially still functions
    vtreeset target_chain;

    /// NOTE: this is no longer used (except for logging).
    /// The direct results stored in this node. This approach is somewhat
    /// clumsy but gets the job done. The direct results that do not need
    /// evaluation. Could be referred to as "generators". One example is
    /// the LookUp rule which just checks if an atom exists.
    Btr<std::set<BoundVertex> > direct_results;

    /// The vector of targets of my children, which will then become Rule arguments.
    /// Refers downwards in the BIT.
    Rule::MPs args;

    /* METHODS */
    void setTarget(meta _target, Btr<bindingsT> binds);
    meta getTarget() const { return bound_target; }

    /// Apply an operator to all nodes that have this one as a node
    template<typename _opT, typename _stateT>
    bool ApplyUp(_opT _op, _stateT my_state) {
        _op(this);
        foreach(parent_link<BITNode> p, parents)
            p.link->ApplyUp(_op, my_state);

        return true;
    }

    /// Apply an operator to all nodes below this one (and to this one, if inclusive is true)
    template<typename opT> void ApplyDown(opT op, bool inclusive = true,
            Btr<std::set<BITNode*> > usedBITNodes = Btr<std::set<BITNode*> >())
    {
        using namespace boost;
        using namespace std;
        
        // Initialise the set of BITNodes that it has already been applied to
        if (usedBITNodes == NULL) {
            usedBITNodes = Btr<set<BITNode*> >(new set<BITNode*>());
        }

        if (STLhas2(*usedBITNodes, this)) {
            return;
        }

        usedBITNodes->insert(this);

        for (std::vector<std::set<ParametrizedBITNode> >::iterator i =  children.begin(); i!=children.end(); i++)
            foreach(const ParametrizedBITNode& pbit, *i)
                pbit.prover->ApplyDown<opT>(op, true, usedBITNodes);
        if (inclusive)
            op(this);
    }
    
    /// Apply an operator to all nodes below this one (and to this one) in reverse
    //! @todo Add an already-used list like ApplyDown() to deal with loops in the tree
    template<typename opT> bool ApplyDown2(opT op)
    {
        using namespace boost;
        
        if (!op(this))
            return false;

        for (std::vector<std::set<ParametrizedBITNode> >::iterator i =  children.begin(); i!=children.end(); i++)
            foreach(const ParametrizedBITNode& pbit, *i)
                if (!pbit.prover->ApplyDown2<opT>(op))
                    return false;

        return true;
    }

    /// Inform the parents of a new result
    bool NotifyParentOfResult(VtreeProvider* new_result) const;
    
    /// Evaluate the Rule with the given new result as arg #arg_i
    /// Allows one to add arguments one by one and evaluating when all slots
    /// filled. Alternatively there has been discussion to evaluate empty slots
    /// as arguments with 0 confidence.
    void EvaluateWith(unsigned int arg_i, VtreeProvider* new_result);
    
    /// Creation of child nodes
    /// Normally arg = args[rule_arg_i], but children with differently-bound
    /// targets may also be created.
    /// == expansion of the BIT.
    BITNode* createChild(unsigned int my_rule_arg_i, RulePtr new_rule,
                         const Rule::MPs& rule_args, BBvtree arg,
                         const bindingsT& bindings, spawn_mode spawning);
    bool createChildren(int rule_arg_i, BBvtree arg,
                        Btr<bindingsT> bindings, spawn_mode);
    void createChildrenForAllArgs();

    /// Check for whether the target atom can simply be Generated.
    /// Here, Generated = "direct"
    /// Note: Rules come in 2 flavours: Composers and Generators.
    /// See PLN implementation docs.
    bool CheckForDirectResults();
    BITNode* HasChild(int arg_i, RulePtr r, const Rule::MPs& rule_args,
                      meta target, const bindingsT& _pre_bindings) const;
    BITNode* HasChild(BITNode* new_child, int arg_i) const;

    /** Add a new parent BITNode.
     * @returns whether the parent was successfully added.
     * @note whenever a new BITNode starts to use this one,
     *       via some parametrization, it becomes my parent.
     */
    bool addNewParent(BITNode* parent, int argumentSlot, Btr<bindingsT>
            _bindings = Btr<bindingsT>(new bindingsT));

    bool inferenceLoop(Rule::MPs reqs);
    bool inferenceLoopWith(meta req);

    /// helper
    //! @todo Doesn't do anything
    template<typename IterT>
    void ValidateRuleArgs(IterT rule_args_begin, IterT rule_args_end) const
    {
        /// Check for wild FreeVariableNodes 
#if 0
        foreach(BoundVertex v, rule_args) {
            if (v.bindings)
            foreach(hpair t, *v.bindings) {
                if (nm->getType(t.second) == FW_VARIABLE_NODE) {
                    LOG(0, "FW_VARIABLE_NODE found on binding's RHS!");
                    getc(stdin);
                }
            }
        }
        AtomSpace *nm = CogServer::getAtomSpace();
#endif
        for(IterT bv = rule_args_begin; bv != rule_args_end; bv++)
            if (GET_ASW->getType(_v2h(*(*bv)->getVtree().begin())) == FW_VARIABLE_NODE) {
#if 0
                LOG(0, "FW_VARIABLE_NODE found on Rule args:"
                    +string(nm->getName(v2h(bv->value))));

                tlog(0,"Args were:\n");
                foreach(BoundVertex v, rule_args)
                    printTree(v2h(v.value),0,0);
#endif
            }
    }

    template<typename ArgIterT>
    bool validRuleResult( BoundVertex& new_result,
            ArgIterT rule_args_begin,
            ArgIterT rule_args_end,
            Btr<bindingsT> bindings_of_all_args) const
    {
        pHandle* ph;

        if (!(!new_result.bindings ||new_result.bindings->empty()))
            printSubsts(new_result, -1);
        assert(!new_result.bindings ||new_result.bindings->empty());

        if ((ph = boost::get<pHandle>(&new_result.value)) && *ph == PHANDLE_UNDEFINED)
        {
            puts("Rule returned PHANDLE_UNDEFINED! Args were:\n");

            for_each(rule_args_begin, rule_args_end,
                boost::bind<void>(
                    NMPrinter(NMP_DEFAULT,NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION,
                        NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE, -2),
                        bind(&VtreeProvider::getVtree, _1)
                    )
            );

            tlog(1,"Binds were (%d):\n", bindings_of_all_args->size());         

            foreach(hpair phh, *bindings_of_all_args)
            {
                printTree(phh.first,0,3);
                cprintf(3,"=>");
                printTree(phh.second,0,3);
            }           

            getc(stdin);getc(stdin);

            return false;
        }
        else
        {
            tlog(2,"::compute() resulted in: \n");
            printTree(boost::get<pHandle>(new_result.value),0,2);
            printSubsts(new_result,2);

            return true;
        }
    }

    /// Completely and exhaustively expand tree to the bottom.
    template<typename VectorT, typename SetT, typename VectorOfSetsIterT>
    void WithLog_expandVectorSet(const std::vector<Btr<SetT> >& child_results,
                                 std::set<VectorT>& argVectorSet) const
    {
        tlog(3,     "expandVectorSet(child_results, argVectorSet)...\n");

        expandVectorSet<VectorT, 
                        VectorOfSetsIterT,
                        std::insert_iterator<std::set<VectorT> > >(
                            child_results.begin(),
                            child_results.end(),
                            inserter(argVectorSet, argVectorSet.begin()));

        tlog(2,     "RESULT FROM expandVectorSet(child_results, argVectorSet) ((%d,%d), %d,%d)...\n", (child_results.size()>0?child_results[0]->size():0), 
            (child_results.size()>1?child_results[1]->size():0), argVectorSet.size(),   (argVectorSet.empty() ? 0 : argVectorSet.begin()->size()) );    

        if (!child_results.empty())
        { std::string bc = std::string("Arg Expansion results:\n");
        foreach(const VectorT& vbv, argVectorSet)
            if (!vbv.empty())
            {
                bc += "(";
                foreach(VtreeProvider* bv, vbv)
                    bc += i2str(_v2h(*bv->getVtree().begin())) + " ";
                bc += ")\n";
            }
            tlog(1, bc.c_str());
        }
    }
    
    /// Add the new result from a Generator, and spawn new subtrees when
    /// necessary according to the variable bindings that accompany the new
    /// result.
    /// I.e. it produces clones of this node (or higher ones), with vars
    /// replaced by results. They get put into the expansion pool.
    void addDirectResult(Btr<std::set<BoundVertex> > directResult,
                         spawn_mode spawning);

    /// Create children and do some (probably obsolete) pre-binding checks.
    bool expandRule(RulePtr rule, int target_i, BBvtree arg,
                    Btr<bindingsT> bindings, spawn_mode spawning);
    
    void clearResults();
    
    /// Heuristics helper value. See implementation for details.
    float my_solution_space() const;
    int number_of_free_variables_in_target() const;
    
    /// If target is a Handle, push it to the result set and replace
    /// the target with its virtual counter-part.
    void ForceTargetVirtual(spawn_mode spawning);
    
    /// Fitness-for-being-selected-for-expansion-next.
    /// The higher the better.
    ///
    /// JaredW: How the fitness works:
    /// The search is a breadth-first search in order of Rule priority.
    /// When a BITNode gets a result, no further BITNodes will be expanded below it.
    /// Within the breadth-first search, try out subgoals with a smaller solution space first.
    /// (In other words, with less FWVars == with less possible matches.)
    /// bdrum represents the best confidence of a result _above_ this BITNode.
    /// I've tested some of these criteria to be useful, and removed useless/harmful
    /// bits (see the target size in my_solution_space).
    /// I think the current search order will
    /// never try out a path when there is a better path available, but breadth-first is still not
    /// ideal, especially in that you can never use it to skip a path (i.e. it can't be combined with
    /// inference control based on Atom STI or previous inference statistics).
    float fitness() const;
    
    /// If inserting the rule invocation node in the subtree obeys our policy
    static bool obeysSubtreePolicy(RulePtr new_rule, meta arg);

    /// If inserting the rule invocation node in the expansion pool obeys
    /// our policy
    //! @todo Should still be static, but got difficulties.
    /*static */bool obeysPoolPolicy(RulePtr new_rule, meta arg, bool loosePoolPolicy = false);

    /// Find if we already have a BITNode like this such that we can re-use it
    /// by some template_bindings which are returned to the caller.
    void findTemplateBIT(BITNode* new_node, BITNode*& template_node,
                         bindingsT& template_binds) const;

    /// Try to clone this BITNode (under its existing parents) with a new
    /// binding applied.
    /// The cloning may fail if the result causes the arguments of the
    /// associated Rule to go invalid, but I think the check is not really
    /// made rigorously here.
    void tryClone(hpair binding) const;

public:
    /// Set of possible inputs for each index of the input vectors of the Rule
    /// associated with this node. Contains the BITNodes, as opposed to the
    /// results as in direct_results
    std::vector<std::set<ParametrizedBITNode> > children;

    /// The ID (serial number) of this BITNode.
    BITNodeID id;

    BITNode();
    BITNode(BITNodeRoot* _root,
            BITNode* _parent,
            unsigned int _depth,
            unsigned int _parent_arg_i,
            meta _target,
            RulePtr _rule,
            const Rule::MPs& _args,
            const vtreeset& _target_chain,
            Btr<bindingsT> _pre_bindings = Btr<bindingsT>(new bindingsT),
            spawn_mode spawning = NO_SIBLING_SPAWNING,
            bool _create = true);
    virtual ~BITNode();

    /// After construction, the object is comparable to others by
    /// using eq() methods. After Create(), it becomes fully usable.
    void create();

    /// Use for debugging
    std::string loopCheck() const;
    
    /// Use for debugging
    int totalChildren() const;

    BITNodeRoot& getBITRoot() const;
    
    /// Use for debugging
    bool hasAncestor(const BITNode* const _p) const;

    /// Whether the Rule for this BITNode is a Composer,
    /// or whether it's a Generator
    bool iscomposer() const { return rule->isComposer(); }

    const std::vector<std::set<VtreeProvider*> >& getEvalResults() const { return eval_results; }
    const std::set<parent_link<BITNode> >& getParents() const { return parents; }
    
    /// Look for a node equal to an existing dummy node
    BITNode* findNode(BITNode* new_child) const;

    /// Look for a node based on certain defining characteristics
    BITNode* findNode(RulePtr new_rule, meta _target, const Rule::MPs& rule_args,
                      const bindingsT& new_bindings) const;

    /// helpers
    bool eq(BITNode* rhs) const;
    bool eq(RulePtr r,  const Rule::MPs& _args, meta _target,
            const bindingsT& _pre_bindings) const;

    /// Expand whole tree level. Typically not called externally
    void expandNextLevel();
    
    // Printing utilities
    std::string tlog(int debugLevel, const char *format, ...) const;
    
    std::string printChildrenSizes() const;
    std::string print(int loglevel=0, bool compact=false, Btr<std::set<BITNode*> > UsedBITNodes = Btr<std::set<BITNode*> >()) const;
    std::string printFitnessPool(int logLevel = 0);
    std::string printTarget() const;
    std::string printArgs() const;
    std::string printResults() const; 
};

/** The root of a Backward Inference proof Tree
 *
 * The BITNodeRoot class works in the following way:
 * -# if the target of the query / inference is either a BindLink or
 * ForAllLink (they have slightly different math), then BITNodeRoot::infer will
 * carry out the unification steps and find all fitting results, but return the
 * Handle to the single atom such as ForAllLink [$x] (ImplicationLink (isBoy $x)
 * (isDumb $x)). It's TV can be retrieved from the atom table.
 * -# If the target is an atom of different type, then BITNodeRoot::infer will
 * return the set of atoms that satisfy the target description. Eg. if the
 * target is ImplicationLink (isBoy $x) (isDumb $x), then it will return all
 * ImpLinks that can be bound to this formula.
 */
class BITNodeRoot : public BITNode
{
    // Let unit tests inspect BITNode state
    friend class ::PLNUTest;
    friend class ::BITNodeUTest;
    friend class ::InferenceCache;
    friend class ExpansionPoolUpdater<std::list<BITNode*> >;

    friend class RuleApp;
public:

    ~BITNodeRoot();

//    // The list of all BITNodes (except the root variable scoper I think).
//    // Updated in BITNode::createChild; used to delete all BITNodes in the
//    // BITNodeRoot destructor.
//    std::set<BITNode*> nodes;

    // The class assumes ownership of the RuleProvider
    BITNodeRoot(meta _target, RuleProvider* _rp = NULL,
            bool _rTrails = true, FitnessEvaluatorT fe = BEST, InferenceCache* _cache = NULL);
    
    /**
     * Basic usage: 
     *
     * \code
     * Btr<BITNodeRoot> state = opencog::pln::PLNEvaluator::
     *     BIT_evaluate(opencog::pln::InferenceTaskParameters(NULL,target));
     * // or you can create the BITNodeRoot directly...
     * state->infer(ProofResources, minConfidenceForStorage, minConfidenceForAbort);
     * \endcode
     * 
     * @param Resources the max number of inference steps. The method returns
     * by ref the nr of actual steps taken.
     * @param minConfidenceForStorage the conf. threshold for storing an
     * inference result
     * @param minConfidenceForAbort the conf. threshold for finishing the
     * inference once "good enough" result found
     * @todo Use revision on the final result (probably have to do it through
     * AtomSpaceWrapper, since within PLN doesn't know about PLN dummy
     * contexts).
     */
    const std::set<VtreeProvider*>& infer(int& resources,
            float minConfidenceForStorage = 0.00000001f,
            float minConfidenceForAbort = 1.00f);

    /// Manual evaluation. Should not be needed anymore.
    Btr<std::set<BoundVertex> > evaluate(std::set<const BITNode*>* chain = NULL) const;

    /// Apply either ForAllRule or VariableScopeRule,
    /// depending on the _resultT type.
    BoundVertex Generalize(Btr<std::set<BoundVertex> >, Type _resultT) const;

    /// Extraction of an actionable plan from the proof tree of the atom with
    /// pHandle h.
    std::string extract_plan(pHandle h) const;
    std::string extract_plan(pHandle h, unsigned int level,
                             vtree& do_template, pHandleSeq& plan,
                             Btr<std::set<pHandle> > usedPHandles = Btr<std::set<pHandle> >() ) const;

    /// Find the fittest BITNode for expansion.
    void findFittest(BITNode*& bisse, float& best_fitness);

    /// Typically not called externally
    void expandFittest();

    // Statistics
    
    std::map<pHandle,BITNode*> hsource;
    long inferenceNodes; // counts the number of nodes in the BIT

    std::string printTrail(pHandle h) const;
    std::string printUsers(BITNode* b);
    std::string printParents(BITNode* b) const;

    int getExecPoolSize() const;
    unsigned int getTreeDepth() const;
    // Used by FC. If true, loosen the pool policy to allow multiple FWVars in an expression.
    // This seems to break some tests in BC, but makes FC support more cases of ForAll unification
    // for input into DeductionRule and ModusPonens.
    void setLoosePoolPolicy(bool x) { loosePoolPolicy = x; }
    // Keep the RuleProvider rather than deleting it. Currently required for forward chaining + trails.
    // Will be necessary when we reuse BITNodes between trees.
    void setKeepRP(bool x) { keepRP = true; }
    void setRecordingTrails(bool x=true);
    bool getRecordingTrails() const;

    FitnessEvaluatorT fitnessEvaluator;

protected:

    InferenceCache* BITcache;
    bool sharedBITCache;

    typedef std::list<BITNode*> exec_poolT;
    exec_poolT exec_pool;

    // A map of which FWVariableNode is owned by which BITNode(s).
    // Cloning a BITNode when a result is found for one argument would result in
    // remaining FWVars being owned by both BITNodes.
//    std::map<Vertex, std::set<BITNode*> > varOwner;

//    std::map<RulePtr, float> priority;

    // Store the depths of each BITNode from this particular root (target).
    // If it is even a descendant of this root.
    std::map<BITNode*, int> BITNodeDepths;

    Rule::MPs dummy_args;

    bool recordingTrails;

    /** Records which BITNodes use one another.
     * List is maintained in order to know which BITNodes directly
     * or indirectly use (ie. Have as a subtree, with possibly some
     * inheritance-bindings) BITNode b. This information is used (only) to
     * prevent circularity: a BITNode cannot indirectly or directly use itself.
     *
     * @note This should eventually be moved to a BITNodeCache (which doesn't
     * exist yet).
     */
//    std::map<BITNode*, std::set<BITNode*> > users;

    /// It's too slow to sort the pool after every insertion.

    bool exec_pool_sorted;
//    std::set<BITNode*> BITNodeTemplates;

    RuleProvider* rp;

    /** Whether the root is a generalization.
     * 0 = no, other = the result link type
     * e.g. ForallLink, BindLink, ThereExistsLink
     */
    Type post_generalize_type;
    
    // Methods
    bool spawns(const bindingsT& bindings) const;
    void spawn(Btr<bindingsT> bindings);

    BITNode* createChild(int my_rule_arg_i, RulePtr new_rule,
                         const Rule::MPs& rule_args, 
                         BBvtree arg, const bindingsT& bindings,
                         spawn_mode spawning);
                        
    /// Basically a helper function for the public printTrail(pHandle)
    //! @todo Support multiple paths better, and adjust the code that detects loops
    //std::string printTrail(pHandle h, unsigned int level, Btr<std::set<pHandle> > usedPHandles = Btr<std::set<pHandle> >()) const;
    std::string printTrail(VtreeProvider* vp, unsigned int level) const;


    void setTreeDepth(const unsigned int newDepth);
    // The current depth of the tree (i.e. the depth of the lowest BITNode)
    unsigned int treeDepth;

    bool loosePoolPolicy;
    bool keepRP;

    friend class BITNode;
};

/// A BITNode results of which are produced by (slow) manual evaluation
/// Obsolete but works. Contains interesting stats class, although the RuleApp
/// can generate richer statistics now.
class ExplicitlyEvaluatedBITNode : public BITNode
{
public:
    //! @todo Complete implementation
    BoundVertex evaluate1(int index);
    
    Btr<std::set<BoundVertex> > evaluate(std::set<const BITNode*>* chain = NULL) const;

    /// Use for brute force checking of whether the tree can be evaluated (but do not really evaluate).
    //! @todo This may no longer be up-2-date with the real evaluate()
    Btr<std::set<BoundVertex> > pseudo_evaluate() const;
};

    template<typename T>
    struct nofilter
    {
        T filteredBegin, filteredEnd;
        nofilter() {}
        nofilter(T begin, T end) : filteredBegin(begin), filteredEnd(end) {}
        void create(T begin, T end) {
            filteredBegin = begin;
            filteredEnd = end;
        }
    };
    template<typename T, typename T2>
    struct triviality_filter
    {
        T filteredBegin, filteredEnd;
        T2 filtered;
        triviality_filter() {}
        triviality_filter(T begin, T end) { Create(begin, end); }
        void create(T begin, T end) {
            for(T i = begin; i != end; ++i)
                if (i->first->isComposer())
                    filtered.insert(*i);

            filteredBegin   = filtered.begin();
            filteredEnd     = filtered.end();
        }
    };

    /*
        The stats class is for recording statistics pertaining to BITNodes.
        This approach may be obsoleted by an approach which uses RuleApp.
        Such an approach doesn't exist yet tho (at May 27, 2007).
    */

    class stats : public Singleton<stats>
    {
        friend class Singleton<stats>;
        stats() {}
        
    public:
        //Store the result set of each BITNode for later analysis.
        typedef std::map<BITNode*, std::set<Vertex> > ITN2atomT;
        typedef std::map<BITNode*, std::set<Vertex> >::iterator ITN2atomIteratorT;
        typedef std::map<BITNode*, std::set<Vertex> >::const_iterator ITN2atomIteratorConstT;
        typedef nofilter<ITN2atomIteratorConstT> nofilterT;
        typedef triviality_filter<ITN2atomIteratorConstT, ITN2atomT> triviality_filterT;

        ITN2atomT ITN2atom;
        template<typename filterT> // = nofilter<ITN2atomIteratorT> >
        void print(filterT filter = nofilterT()) const
        {

            puts("stats::print()\n");
            filter.create(ITN2atom.begin(), ITN2atom.end());

            for (std::map<BITNode*, std::set<Vertex> >::const_iterator  i =
                    filter.filteredBegin;
                    i != filter.filteredEnd;
                    i++) {
    //          printf("[%d]: ", (int)i->first);
                i->first->print(-10, true);
                foreach(Vertex v, i->second)
                {
                    printf("%u\n", _v2h(v));
                    NMPrinter(NMP_BRACKETED|NMP_TYPE_NAME |NMP_NODE_NAME|NMP_NODE_TYPE_NAME|NMP_TRUTH_VALUE|NMP_PRINT_TO_FILE, -10).print(_v2h(v));
                }
                printf("\n");
            }
            puts("---\n");
        }
    };

}}

#endif
