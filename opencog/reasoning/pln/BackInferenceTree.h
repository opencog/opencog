#ifndef BACK_INFERENCE_TREE
#define BACK_INFERENCE_TREE

#include <boost/bind.hpp>
#include <stack>
#include "RuleProvider.h"

using namespace reasoning;

namespace haxx
{
	extern reasoning::iAtomTableWrapper* defaultAtomTableWrapper;
}


namespace reasoning
{

typedef int CHAINER_STATUS;
const int INCOMPLETE=1, COMPLETE = 2, SOLUTION = 4, COMPLETE_EVALUATED = 8, FAILURE = 16;

class RuleRepository : public Singleton<RuleRepository>
{
protected:
	bool created;
	set<uint> CustomCrispUnificationRules;

	RuleRepository() : created(false) {}
	friend class Singleton<RuleRepository>;
public:
	reasoning::Rule* rule[500];
	void CreateRules();
	const set<uint>& GetCustomCrispUnificationRules() const { return CustomCrispUnificationRules; }
};

/**
Elementary operations:
- Look up the atoms that directly match to the target => produce a set of atoms
- Look up all the rules that could produce the target (given some input that may or may
not be possible to produce) => produce a set of rule stubs
- Expand a rule stub => for each input the stub requires, do lookup for atoms & rule stubs
- KillIfUseless => stub is removed 

	Policy:
		- Nr of 

- Upon expansion of a new tree, pass along the set of all targets that have been attempted
to prove higher in the tree, plus the new target, to produce a set of atoms whose proof
can no longer be re-started below this level.

*/

enum EXPAND_MODE { ALL = 1, SINGLE = 2, UNTIL_SOLUTION = 4 };

class BackInferenceTree;
	
//typedef Btr<BackInferenceTree> pBITree;
typedef BackInferenceTree* pBITree;

class BackInferenceTree_fitness_comp : public binary_function<pBITree, pBITree, bool>
{
public:
    bool operator()(pBITree lhs, pBITree rhs) const;
};

class BackInferenceTree;
/*
template<typename T>
struct parent_link
{
	parent_link() : link(NULL), parent_arg_i(0) {}
	parent_link(T* _link, uint _parent_arg_i) : link(_link), parent_arg_i(_parent_arg_i) {}
	T* link;
	uint parent_arg_i;
	bool operator<(const parent_link& rhs) const { return link<rhs.link || (link==rhs.link && parent_arg_i < rhs.parent_arg_i); }
};
*/

template<typename T>
struct parent_link
{
	parent_link() : link(NULL), parent_arg_i(0) {}
	parent_link(T* _link, uint _parent_arg_i, Btr<bindingsT> _bindings = Btr<bindingsT>(new bindingsT))
		: link(_link), parent_arg_i(_parent_arg_i), bindings(_bindings) {}
	T* link;
	uint parent_arg_i;
	Btr<bindingsT> bindings;

	bool operator<(const parent_link& rhs) const {
		return link<rhs.link
			|| (link==rhs.link && parent_arg_i < rhs.parent_arg_i)
			|| (link==rhs.link && parent_arg_i == rhs.parent_arg_i && bindings < rhs.bindings);
	}
};


enum spawn_mode { NO_SIBLING_SPAWNING = 0, ALLOW_SIBLING_SPAWNING };

/*#ifdef WIN32


struct BIThash :  public stdext::hash_compare<BackInferenceTree*>
{
	/// hash function
	size_t operator()(BackInferenceTree* b) const
	{
		size_t ret = 0;
		ret += (int)b->rule;

		ret += BoundVTree(*b->GetTarget()).getFingerPrint();

		foreach(Btr<BoundVTree> bvt, b->args)
			ret += bvt->getFingerPrint();

		return ret;
	}
	bool operator()(BackInferenceTree* lhs, BackInferenceTree* rhs) const
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
		if (lhs->rule && !lhs->rule->IsComputable())
		{
			/// \todo Should look at std_tree forms of targets here!

			if (less_vtree()(*lhs->GetTarget(), *rhs->GetTarget()))
				return true;
			else if (less_vtree()(*rhs->GetTarget(), *lhs->GetTarget()))
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

typedef hash_set<BackInferenceTree*, BIThash> BITstoreT;

#else*/

class indirect_less_BIT : public binary_function<BackInferenceTree*, BackInferenceTree*, bool>
{
public:
	bool operator()(BackInferenceTree* lhs, BackInferenceTree* rhs) const;
};

typedef set<BackInferenceTree*, indirect_less_BIT> BITstoreT;

/*#endif
*/


class BackInferenceTreeRoot;

class BackInferenceTree
{
	friend class indirect_less_BIT;
	friend class BackInferenceTree_fitness_comp;
	friend class bdrum_updater;
	friend class BackInferenceTreeRoot;
protected:
	meta	bound_target,
			raw_target;
	uint counted_number_of_free_variables_in_target;

	mutable set<parent_link<BackInferenceTree> > parents;
	
	/// BackInferenceTree supports pre-bindings extensively, but it's unclear whether they are
	/// obsolete or not.

	Btr<bindingsT> pre_bindings;

	float my_bdrum;
	unsigned int depth;
	
	BackInferenceTreeRoot* root;

	/// Possibly obsolete
	typedef set<vtree, less_tree_vertex> vtreeset;
	vtreeset target_chain;

	/// The Rule that the child_results of this state object's child_results will be
	/// associated with.
	Rule *rule;
	
	/// Contains the target of this inference state node
	//vector<InferenceTaskParameters> pars;
	//const InferenceTaskParameters& pars;
	Rule::MPs args;

	bool Expanded;

	/* METHODS */

	Btr<bindingsT> relevantBindings(const bindingsT& _pre_bindings) const;

	void SetTarget(meta _target);
	meta GetTarget() const { return bound_target; }

	template<typename _opT, typename _stateT> bool ApplyUp(_opT _op, _stateT my_state) {
		_op(this);
		foreach(parent_link<BackInferenceTree> p, parents)
			p.link->ApplyUp(_op, my_state);

		return true;
	}

	template<typename opT> void ApplyDown(opT op)
	{
		using namespace boost;
		
		for (vector<set<pBITree> >::iterator i =  children.begin(); i!=children.end(); i++)
			for_each(i->begin(), i->end(),
				bind(&BackInferenceTree::ApplyDown<opT>, _1, op));
		op(pBITree(this));
	}
	
	///Just evaluate in reverse order
	template<typename opT> bool ApplyDown2(opT op)
	{
		using namespace boost;
		
		if (!op(this))
			return false;

		for (vector<set<pBITree> >::iterator i =  children.begin(); i!=children.end(); i++)
			foreach(pBITree i2, *i)
				if (!i2->ApplyDown2<opT>(op))
					return false;

		return true;
	}

	BackInferenceTree* CreateChild(int my_rule_arg_i, Rule* new_rule, const Rule::MPs& rule_args, 
						BBvtree arg, const bindingsT& bindings,spawn_mode spawning);
	
	/// Normally arg = args[rule_arg_i], but children with differently-bound targets
	/// may also be created.

	void SimpleClone(const bindingsT&  new_bind) const;
	
	bool CreateChildren(int rule_arg_i, BBvtree arg, Btr<bindingsT> bindings, spawn_mode);
	void CreateChildrenForAllArgs();
	void CheckIfSolution(bool comprehensive = false);
	bool CheckForDirectResults();
	BackInferenceTree* HasChild(int arg_i, Rule* r,  const Rule::MPs& rule_args, meta target, const bindingsT& _pre_bindings) const;
	BackInferenceTree* HasChild(BackInferenceTree* new_child, int arg_i) const;
	BackInferenceTree* FindNode(Rule* new_rule, meta _target, const Rule::MPs& rule_args, const bindingsT& new_bindings) const;
	BackInferenceTree* FindNode(BackInferenceTree* new_child) const;
	void addNewParent(parent_link<BackInferenceTree> new_parent);

	bool inferenceLoop(Rule::MPs reqs);
	bool inferenceLoopWith(meta req);

	void spawn(Btr<bindingsT> bindings);

	/// helper
	void ValidateRuleArgs(const vector<BoundVertex>& rule_args) const;
	/// helper
	bool ValidRuleResult(BoundVertex& new_result, const vector<BoundVertex>& rule_args, Btr<bindingsT> bindings_of_all_args) const;
	/// helper
	void WithLog_expandVectorSet(const std::vector<Btr<BV_Set> >& multi_input_vector,
					 set<BV_Vector>& output_set) const;
	
	void ReleaseChildren();
	BackInferenceTree* Bind(bindingsT binding);
//	BackInferenceTree* Clone() const;

	void addDirectResult(Btr<set<BoundVertex> > directResult, spawn_mode spawning);
	bool expandRule(Rule *rule, int target_i, BBvtree arg,
		Btr<bindingsT> bindings, spawn_mode spawning);
	
	/// Use for brute force checking of whether the tree can be evaluated (but do not really evaluate).
	/// TODO: This may no longer be up-2-date with the real evaluate()
	Btr<set<BoundVertex> > pseudo_evaluate() const;

	void clearResults();
	
	float my_solution_space() const;
	int number_of_free_variables_in_target() const;
	
	/// If target is a Handle, push it to the result set and replace
	/// the target with its virtual counter-part.

	void ForceTargetVirtual(spawn_mode spawning);
	
	float fitness() const;
	
	/// If inserting the rule invocation node in the subtree obeys our policy
	static bool ObeysSubtreePolicy(Rule *new_rule, meta arg);

	/// If inserting the rule invocation node in the expansion pool obeys our policy
	static bool ObeysPoolPolicy(Rule *new_rule, meta arg);
		
public:
	/// set of possible inputs for each index of the input vectors of the Rule
	/// associated with this node,
	vector<set<pBITree> > children;
	
	/// The ultimate result set of this node. Once the Node is COMPLETE,
	/// this will no longer be updated. Until then, it'll be re-created
	/// on each new evaluation.
	Btr<set<BoundVertex> > my_results;

	BackInferenceTree();
	BackInferenceTree(	BackInferenceTreeRoot* _root,
						BackInferenceTree* _parent,
						unsigned int _depth,
						unsigned int _parent_arg_i,
						meta _target,
						Rule *_rule,
						const Rule::MPs& _args,
						const vtreeset& _target_chain,
						Btr<bindingsT> _pre_bindings = Btr<bindingsT>(new bindingsT),
						spawn_mode spawning = NO_SIBLING_SPAWNING,
						bool _create = true);
	virtual ~BackInferenceTree();

	void Create();	

	void expandNextLevel();
	void expandFittest();
	/// TODO: May not work. Possibly redundant anyway.
	void expandNext(EXPAND_MODE mode);
	/// TODO: May not work. Possibly redundant anyway.
	void expandSubtree();

	bool eq(BackInferenceTree* rhs) const;
	bool eq(Rule* r,  const Rule::MPs& _args, meta _target, const bindingsT& _pre_bindings) const;

	/// TODO: Complete implementation
	BoundVertex evaluate1(int index);

	Btr<set<BoundVertex> > evaluate(set<const BackInferenceTree*>* chain = NULL) const;

	const bindingsT& GetPreBindings() const { return *pre_bindings; }
	const set<parent_link<BackInferenceTree> >& GetParents() const { return parents; }

	int tlog(int debugLevel, const char *format, ...) const;
	void printChildrenSizes() const;
	void print() const;
	void printFitnessPool();
	void printTarget() const;
	void printArgs() const;


	//void relevantBindingsTest();
};

class BackInferenceTreeRoot : public BackInferenceTree
{
public:
	typedef list<BackInferenceTree*> exec_poolT;

	BackInferenceTreeRoot(meta _target,
						Btr<RuleProvider> _rp);

	exec_poolT exec_pool;

	BoundVertex Generalize(Btr<set<BoundVertex> >, Type _resultT) const;
	void extract_plan(Handle h) const;
	void extract_plan(Handle h, unsigned int level, vtree& do_template, vector<Handle>& plan) const;

	/// Overriden methods:

	Btr<set<BoundVertex> > evaluate(set<const BackInferenceTree*>* chain = NULL) const;

	/// Statistics
	map<Handle,vector<Handle> > inferred_from;
	map<Handle,Rule*> inferred_with;
	map<Handle,BackInferenceTree*> hsource;
	long InferenceNodes;



	void print_trail(Handle h) const;

protected:
	map<Vertex, BackInferenceTree*> varOwner;
	void print_trail(Handle h, unsigned int level) const;

	bool spawns(const bindingsT& bindings) const;

	std::map<Rule*, float> priority;

	Rule::MPs dummy_args;

	/// It's too slow to sort the pool after every insertion. Otherwise we'd do this:
	/// typedef set<BackInferenceTree*, BackInferenceTree_fitness_comp> exec_poolT;

	bool exec_pool_sorted;
	BITstoreT BITstore;

	Btr<RuleProvider> rp;

	/// 0 = no, other = the result link type
	Type post_generalize_type;


	BackInferenceTree* CreateChild(int my_rule_arg_i, Rule* new_rule, const Rule::MPs& rule_args, 
						BBvtree arg, const bindingsT& bindings,spawn_mode spawning);

	friend class BackInferenceTree;
};

}

#endif
