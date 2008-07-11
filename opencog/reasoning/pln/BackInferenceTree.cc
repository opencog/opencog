#include "PLN.h"

#ifdef WIN32
#pragma warning(disable : 4311)
#endif

#include <stdlib.h>
#include <time.h>

#include <boost/lambda/lambda.hpp>
#include <boost/foreach.hpp>

#include "Rules.h"
#include "PTLEvaluator.h"
#include "Rules.h"

#include "spacetime.h"

#include "AtomTableWrapper.h"

#include <CogServer.h> //"../core/ClassServer.h"
#include <utils2.h>
#include "Ptlatom.h"
#include "PTLEvaluator.h"
#include "BackInferenceTree.h"

//#ifndef USE_PSEUDOCORE
//	#include "RouletteSelector.h"
//	#include "BackwardInferenceTask.h"
//#endif

#define DIRECT_RESULTS_SPAWN 1

const float MIN_CONFIDENCE_FOR_RULE_APPLICATION = 0.00001f;

/*
For debugging in Win32:

#undef ATOM
#include <iostream>
#include <cstddef>
#include <windows.h>
*/
/*
namespace test
{
	long bigcount=0;

	double custom_duration = 0.0;
	clock_t custom_start, custom_finish;
	time_t custom_start2, custom_finish2;
	double custom_duration2 = 0.0;

	extern int _test_count;
	extern bool debugger_control;

	extern FILE *logfile;
	const bool LOG_WITH_NODE_ID = true;

}*/


namespace haxx
{
	extern bool AllowFW_VARIABLENODESinCore;
	extern uint maxDepth;
}

bool indirect_less_BIT::operator()(BackInferenceTree* lhs, BackInferenceTree* rhs) const
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
	if (lhs->rule && !lhs->rule->isComputable())
	{
		if (less_vtree()(*lhs->GetTarget(), *rhs->GetTarget()))
			return true;
		else if (less_vtree()(*rhs->GetTarget(), *lhs->GetTarget()))
			return false;
	}
	else
	{
		/*
		for(int i=0; i<lhs->args.size(); i++)
			if (less_vtree()(lhs->args[i]->getStdTree(), rhs->args[i]->std_tree()))
				return true;
			else if (less_vtree()(rhs->args[i]->getStdTree(), lhs->args[i]->std_tree()))*/
				return false;
	}

	return false;
}

namespace reasoning
{
//bool RECORD_TRAILS = false;
bool PREVENT_LOOPS = false;

const float infinite = 9999999999.0f;

void pr(pair<Handle, Handle> i);
void pr3(pair<Handle, Handle> i)
{
	printTree(i.first,0,0);
	printf("=>");
	printTree(i.second,0,0);
}

bool BackInferenceTree_fitness_comp::operator()(BackInferenceTree* lhs, BackInferenceTree* rhs) const
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

void AssignPriority(std::map<Rule*, float>& priority)
{
	for (int i=0;i<100;i++)
		priority[RuleRepository::Instance().rule[i]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[PTL_AND]] = 10.0f;
/*
	priority[RuleRepository::Instance().rule[AND2]] = 1.0f
	priority[RuleRepository::Instance().rule[AND3]] = 1.0f
	priority[RuleRepository::Instance().rule[AND4]] = 1.0f
	priority[RuleRepository::Instance().rule[AND5]] = 1.0f
	priority[RuleRepository::Instance().rule[AND85]] = 1.0f
*/
	priority[RuleRepository::Instance().rule[SimpleAND]] = priority[RuleRepository::Instance().rule[PTL_AND]] - 1.0f;
	priority[RuleRepository::Instance().rule[SimpleAND2]] = priority[RuleRepository::Instance().rule[PTL_AND]] - 1.1f;
	priority[RuleRepository::Instance().rule[SimpleAND3]] = priority[RuleRepository::Instance().rule[PTL_AND]] - 1.2f;
	priority[RuleRepository::Instance().rule[SimpleAND4]] = priority[RuleRepository::Instance().rule[PTL_AND]] - 1.3f;
	priority[RuleRepository::Instance().rule[SimpleAND5]] = priority[RuleRepository::Instance().rule[PTL_AND]] - 1.4f;
	priority[RuleRepository::Instance().rule[ForAll]] = 5.0f;
	priority[RuleRepository::Instance().rule[PLNPredicate]] = 5.0f;

	priority[RuleRepository::Instance().rule[ORPartition]] = 10.0f;
	priority[RuleRepository::Instance().rule[OR]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[ANDPartition]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[NotEvaluation]] = 10.0f;
	priority[RuleRepository::Instance().rule[UnorderedLinkPermutation]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[ANDBreakdown1]] = 4.0f;
	priority[RuleRepository::Instance().rule[ANDBreakdown2]] = 4.0f;
	priority[RuleRepository::Instance().rule[ANDBreakdown3]] = 3.0f;
	priority[RuleRepository::Instance().rule[ANDBreakdown4]] = 2.0f;
	priority[RuleRepository::Instance().rule[ANDBreakdown5]] = 1.0f;
	priority[RuleRepository::Instance().rule[ORBreakdown]] = 1.0f;
	
	priority[RuleRepository::Instance().rule[ImplicationBreakdown]] = 9.0f;
//	priority[RuleRepository::Instance().rule[ImplicationConstruction]] = 10.0f;
//	priority[RuleRepository::Instance().rule[ImplicationTailExpansion]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[VariableInstantiation]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[ScholemFunctionProduction]] = 20.0f;
	
	priority[RuleRepository::Instance().rule[NOTElimination]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[Deduction_Implication]] = 8.0f;
	priority[RuleRepository::Instance().rule[Deduction_Inheritance]] = 8.0f;
	priority[RuleRepository::Instance().rule[Inversion_Implication]] = 7.0f;
	priority[RuleRepository::Instance().rule[Inversion_Inheritance]] = 7.0f;
	
	///???
	priority[RuleRepository::Instance().rule[Revision]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[MetaPredicateExecution]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[SubSetEval_ConceptNode]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[Equi2Impl]] = 10.0f;
	priority[RuleRepository::Instance().rule[Equi2Sim]] = 10.0f;
	priority[RuleRepository::Instance().rule[Inh2Imp]] = 10.0f;
	priority[RuleRepository::Instance().rule[Inh2Sim]] = 10.0f;
	priority[RuleRepository::Instance().rule[Inh2Eval]] = 10.0f;
	priority[RuleRepository::Instance().rule[Sim2Inh]] = 10.0f;
	priority[RuleRepository::Instance().rule[Mem2Inh]] = 10.0f;
	priority[RuleRepository::Instance().rule[Mem2Eval]] = 10.0f;
	priority[RuleRepository::Instance().rule[Imp2Inh]] = 10.0f;

	priority[RuleRepository::Instance().rule[IntInh2Ext]] = 10.0f;
	priority[RuleRepository::Instance().rule[ExtImp2Int]] = 10.0f;
	priority[RuleRepository::Instance().rule[ExtInh2Int]] = 10.0f;
	priority[RuleRepository::Instance().rule[ExtImpl2Subset]] = 10.0f;
	priority[RuleRepository::Instance().rule[IntImp2Ext]] = 10.0f;
	priority[RuleRepository::Instance().rule[ExtEqui2ExtSim]] = 10.0f;
	priority[RuleRepository::Instance().rule[Tautology]] = 10.0f;
	
	priority[RuleRepository::Instance().rule[Lookup]] = 20.0f;
	priority[RuleRepository::Instance().rule[Hypothesis]] = 30.0f;
	
	priority[RuleRepository::Instance().rule[CrispUnification]] = 7.5f;
	priority[RuleRepository::Instance().rule[StrictCrispUnification]] = 7.5f;

/*	priority[RuleRepository::Instance().rule[OR2AND]] = 10.0f;
	priority[RuleRepository::Instance().rule[Exist2ForAll]] = 10.0f;
	priority[RuleRepository::Instance().rule[Exist]] = 10.0f;*/

}


struct bdrum_updater
{
	float val;
	bdrum_updater(float _val) : val(_val) {}
	void operator()(BackInferenceTree* b) { b->my_bdrum = val; }
};

static int more_count=0;

BackInferenceTree::BackInferenceTree()
: depth(0), Expanded(false), rule(NULL), my_bdrum(0.0f), pre_bindings(new bindingsT),
my_results(Btr<set<BoundVertex> >(new set<BoundVertex>))
{
}

BackInferenceTreeRoot::BackInferenceTreeRoot(meta _target, Btr<RuleProvider> _rp)
: InferenceNodes(0), exec_pool_sorted(false), post_generalize_type(0), rp(_rp) 
{
	this->pre_bindings = Btr<bindingsT>(new bindingsT);
	rule = NULL;
	root = this;
	bound_target = meta(new vtree);

	vtree::iterator target_it = _target->begin();
	post_generalize_type = inheritsType((Type)v2h(*target_it), VARIABLE_SCOPE_LINK)
									? VARIABLE_SCOPE_LINK
									: inheritsType((Type)v2h(*target_it), FORALL_LINK)
										? FORALL_LINK
										: 0;
	if (post_generalize_type)
	{
		/// VARIABLE_SCOPE_LINK( arg_list, actual_atom) -> select actual_atom:
		target_it = _target->begin(target_it);
		++target_it;
	}

	raw_target = Btr<vtree>(new vtree(target_it));

	/**
		The 1st child corresponds to "root variable scoper", variable-bound clones of which
		will be spawned later on. Those clones will then be owned by this Root.
		To enable this, the 1st child MUST OWN the variables in the target atom of the root.
	*/

	dummy_args.push_back(Btr<BoundVTree>(new BoundVTree(*raw_target)));

	rawPrint(*raw_target, raw_target->begin(), -1);

	children.push_back(set<BackInferenceTree*>());

	BackInferenceTree* root_variable_scoper =	CreateChild(0, NULL, dummy_args, Btr<BoundVTree>(new BoundVTree(make_vtree(ATOM))),
			bindingsT(), NO_SIBLING_SPAWNING);

	set<Vertex> vars;
	/*
	copy_if(	raw_target->begin(),
				raw_target->end(),
				inserter(vars, vars.begin()),
				bind(equal_to<Type>(),
					bind(getTypeFun, bind(&_v2h, _1)),
					(Type)FW_VARIABLE_NODE));
	*/
	foreach(Vertex v, vars)
		varOwner[v] = root_variable_scoper;

	AssignPriority(priority);
}

BackInferenceTree* BackInferenceTreeRoot::CreateChild(int my_rule_arg_i, Rule* new_rule, const Rule::MPs& rule_args, 
					BBvtree _target, const bindingsT& bindings,spawn_mode spawning)

{
	/// We ignore most of the args.
	BackInferenceTree* ret =	new BackInferenceTree(
								this,
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
	children[0].insert(ret);

	return ret;
}

Btr<set<BoundVertex> > BackInferenceTreeRoot::evaluate(set<const BackInferenceTree*>* chain) const
{
	Btr<set<BoundVertex> > results = (*children[0].begin())->evaluate(chain);

	if (post_generalize_type)
	{
		BoundVertex VarScopeLink = Generalize(results, post_generalize_type);
		results->clear();
		results->insert(VarScopeLink);

		return results;
	}
	else
	{
		printf("Results:\n");
		const float min_confidence = 0.0001f;
		Btr<set<BoundVertex> > nontrivial_results(new set<BoundVertex>);

		/*foreach(BoundVertex new_result, *results)
			if (getTruthValue(v2h(new_result.value))->getConfidence() > min_confidence)
			{
				printTree(v2h(new_result.value),0,0);

				nontrivial_results->insert(new_result);
			}
		*/
		return nontrivial_results;
	}
}

void BackInferenceTree::print() const
{
	if (!((more_count++)%25))
	{
		puts(" --- more ");
		if (getc(stdin) == 'q')
			return;
	}
	#define prlog printf
	if (rule)
	{
		string cbuf("[ ");
		if (my_results)
			foreach(const BoundVertex& bv, *my_results)
				cbuf += i2str((long)v2h(bv.value)) + " ";
		if (cbuf.empty())
			cbuf = "[]";
		prlog("%s%s ([%ld])\n", repeatc(' ', depth*3).c_str(), rule->name.c_str(), (long)this);
		prlog("%s%s]\n", repeatc(' ', (depth+1)*3).c_str(), cbuf.c_str());
	}
	else
		prlog("root\n");
	
	int ccount=0;
	for (vector<set<pBITree> >::const_iterator i =  children.begin(); i!=children.end(); i++)
	{
		prlog("%sARG #%d:\n", repeatc(' ', (depth+1)*3).c_str(), ccount);
		prlog("%s---\n", repeatc(' ', (depth+1)*3).c_str());
		ccount++;
		
		for_each(i->begin(), i->end(),
			mem_fun(&BackInferenceTree::print));
	}
}

void BackInferenceTree::addDirectResult(boost::shared_ptr<set<BoundVertex> > directResult, spawn_mode spawning)
{
	AtomSpace *nm = CogServer::getAtomSpace();
	my_results->insert(directResult->begin(), directResult->end());

	bool bdrum_changed = false;
	
	foreach(BoundVertex bv, *directResult)
	{
		float confidence = IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL;
			//nm->getTruthValue(v2h(bv.value))->getConfidence();
		if (confidence > my_bdrum)
		{
			my_bdrum = confidence;
			bdrum_changed = true;
		}
	}
	
	if (bdrum_changed)
		ApplyDown(bdrum_updater(my_bdrum));

/*if (parent != root)	
	for (uint arg=0;arg<parent->args.size();arg++)
	{
		tlog(3, "arg:...\n");
		rawPrint(*parent->args[arg], parent->args[arg]->begin(),3);
	}*/
	
	if (spawning)
	{
		printf("SPAWN...\n");

#if DIRECT_RESULTS_SPAWN
		/// Insert to pool the bound versions of all the other arguments of the parent
		foreach(const BoundVertex& bv, *directResult)
			if (bv.bindings)
				spawn(bv.bindings);

		printf("----------------------\n");
	}
	else
		tlog(0,"A no-spawning process.\n");

	tlog(3, "DirectResults added ok.\n");
	
	root->exec_pool_sorted = false;	
}

/*
/// Test usage sample for index2combination

void prn_combis(vector<int>& v);

//template<typename T>

int main(int argc, char* argv[])
{
	printf("Hello World!\n");

	vector<int> size_vector;
	size_vector.push_back(2);
	size_vector.push_back(3);
//	size_vector.push_back(4);
	size_vector.push_back(1);

	for (int i = 0; i < 30; i++)
	{
		vector<int> cv = index2combination(i, size_vector);

		prn_combis(cv);

		getc(stdin);
	}

	return 0;
}
*/

/// TODO: Allow evaluation-of-next-best combination by using the following functions.
/// This allows step-by-step evaluation.

vector<int> index2combination(int index, vector<int>& size_vector)
{
	int cumulative_product = 1;
	vector<int> ret;

	for (uint i=0; i < size_vector.size(); i++)
	{
		ret.push_back( (index / cumulative_product) % size_vector[i] );
		cumulative_product *= size_vector[i];
	}

	return ret;
}

void prn_combis(vector<int>& v)
{
	for (uint i=0;i<v.size();i++)
		printf("%d%c", v[i], ( (i<v.size()-1) ? ',' : ' '));
}


BoundVertex BackInferenceTree::evaluate1(int index)
{
	return BoundVertex();	
}

int BackInferenceTree::number_of_free_variables_in_target() const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	/// Use set<> to prevent re-counting of the already-found Handles
	
	set<Handle> vars;
	
	for(vtree::iterator v  = GetTarget()->begin(); v != GetTarget()->end(); v++)
		if (nm->getType(v2h(*v)) == FW_VARIABLE_NODE)
			vars.insert(v2h(*v));	

	tlog(4,"number_of_free_variables_in_target: %d\n", vars.size());
		
	return (int)vars.size();
}

void BackInferenceTree::SetTarget(meta _target)
{
	raw_target = _target;
	bound_target = bind_vtree(*raw_target, GetPreBindings());
	counted_number_of_free_variables_in_target = number_of_free_variables_in_target();
}

float BackInferenceTree::my_solution_space() const
{
	return counted_number_of_free_variables_in_target - GetTarget()->size()*100.0f;
}

float BackInferenceTree::fitness() const
{		
	const float CONFIDENCE_WEIGHT = 10000.0f;
	const float DEPTH_WEIGHT = 100.0f;
	const float SOLUTION_SPACE_WEIGHT = 0.01f;
	const float RULE_PRIORITY_WEIGHT = 0.0001f;

//tlog(3,"fitness(): %f %f %f %f %f\n",my_bdrum, -depth, _bdrum, -1.0f*_bdrum,-depth -_bdrum);	
	
	return 	-1.0f*SOLUTION_SPACE_WEIGHT	*my_solution_space()
			-1.0f*DEPTH_WEIGHT			*depth
			-1.0f*CONFIDENCE_WEIGHT		*my_bdrum
			+1.0f*RULE_PRIORITY_WEIGHT	*root->priority[rule];
	
/*	\todo Use arity in the spirit of the following:

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

void BackInferenceTree::printChildrenSizes() const
	{
		//if (currentDebugLevel>=3)
		//	puts("next chi...0");
		tlog(3,"next chi...0");
		for(uint c=0; c< children.size(); c++)
		{
			tlog(3,"next chi...1");
			children[c];
			tlog(3,"next chi...2", children[c].size());
			tlog(3,"(%d:%d), ", c, children[c].size());
			tlog(3,"\n");
		}
	}


/// BIN cannot have pre-bindings for vars which do not occur in the target nor in its parent's pre-bindings

Btr<bindingsT> BackInferenceTree::relevantBindings(const bindingsT& _pre_bindings) const
{
	Btr<bindingsT> relevantBindings(new bindingsT);

	foreach(hpair b, _pre_bindings)
	{
		foreach(const parent_link<BackInferenceTree>& p, parents)	
			if ( (p.link && p.link->pre_bindings && STLhas(p.link->GetPreBindings(), b.first))			
				||(find(raw_target->begin(), raw_target->end(), Vertex(b.first)) != raw_target->end()))
			{
				(*relevantBindings)[b.first] = b.second;
				goto next_var;
			}

			/// TODO: The following should be redundant, because all variables in args[] are
			/// my variables, and will not be bound at this node by other processes!
			/// OTOH, in my children, they may be bound.
			foreach(meta arg, args)	
				if (find(arg->begin(), arg->end(), Vertex(b.first)) != arg->end())
				{
					(*relevantBindings)[b.first] = b.second;
					goto next_var;
				}
next_var:;
	}

	/// All pre-bindings of my parents must be included, too.

	foreach(const parent_link<BackInferenceTree>& p, parents)	
			insert_with_consistency_check(*relevantBindings, p.link->GetPreBindings().begin(), p.link->GetPreBindings().end());

	return relevantBindings;
}

void BackInferenceTree::Create()
{
	assert(children.empty());

	tlog(2, "This new InferenceState needs %d args:\n", args.size());
	for (uint ari = 0; ari < args.size(); ari++)
		rawPrint(*args[ari],args[ari]->begin(),2);

	target_chain.insert(*bound_target);

	if (!rule || rule->isComputable())
	{
		children.insert(children.begin(), args.size(), set<BackInferenceTree*>());
	}
	assert(rule || children.size() == 1);
}

BackInferenceTree::BackInferenceTree(
    BackInferenceTreeRoot* _root,
    BackInferenceTree* _parent,
    unsigned int _depth,
    unsigned int _parent_arg_i,
    meta _target,
    Rule *_rule,
    const Rule::MPs& _args,
    const vtreeset& _target_chain,
    Btr<bindingsT> _pre_bindings,
    spawn_mode spawning,
    bool _create)
: raw_target(_target), root(_root), depth(_depth),
target_chain(_target_chain), Expanded(false), rule(_rule),
my_bdrum(0.0f), pre_bindings(new bindingsT), args(_args)
{
    AtomSpace *nm = CogServer::getAtomSpace();
    if (_parent)
        parents.insert(parent_link<BackInferenceTree>(_parent, _parent_arg_i));

    try
    {		  
        assert(!parents.empty() || !root);

        /// Include all relevant bindings of the ones that were passed on to me

        /// pre_bindings of a node will never be changed.
        /// Although new parents will in principle bring new pre_bindings,
        /// we do not accept parents that have any pre_bindings which this node does not have.

        pre_bindings = relevantBindings(*_pre_bindings);

        tlog(0, "Pre-binds: my=%d parent=? given=%d\n", pre_bindings->size(), _pre_bindings->size());

        SetTarget(_target);

        if (inheritsType(nm->getType(v2h(*bound_target->begin())), LINK) &&
            ((nm->isReal(v2h(*bound_target->begin())) && !nm->getArity(v2h(*bound_target->begin()))) ||
            (!nm->isReal(v2h(*bound_target->begin())) && !bound_target->number_of_children(bound_target->begin())))
            )
        {
            rawPrint(*bound_target, bound_target->begin(),0);
            assert(0);
        }

        if (!root->rp)
        {
            root->rp.reset(new DefaultVariableRuleProvider());
            tlog(3, "Default RuleProvider created.\n");
        }
        else
            tlog(3, "Parent passed on my RuleProvider.\n");

        my_bdrum = _parent->my_bdrum;

        my_results = Btr<set<BoundVertex> >(new set<BoundVertex>);

        ForceTargetVirtual(spawning);

        tlog(0, "ForceTargetVirtual ok.\n");

        if (_create)
        {
            tlog(2, "Creating...\n");
            Create();
        }
  } catch(string s) {
      printf("EXCEPTION IN BackInferenceTree::BackInferenceTree! %s\n",s.c_str());
      getc(stdin); getc(stdin); throw;
  } catch(...) {
      printf("EXCEPTION IN BackInferenceTree::BackInferenceTree!\n");
      getc(stdin); getc(stdin); throw;
  }
  root->InferenceNodes++;
}

bool BackInferenceTree::inferenceLoopWith(meta req)
{
	if (this == root)
		return false;

	foreach(const vtree& prev_req, target_chain)
		if (equalVariableStructure(prev_req, *req))
		{
			vtree* preq = const_cast<vtree*>(&prev_req);

			rawPrint(*preq, preq->begin(),3);
			tlog(3,"Loops! Equal var structure with:\n");
			rawPrint(*req, req->begin(),3);
			return true;
		}

	return false;
}	
	
bool BackInferenceTree::inferenceLoop(Rule::MPs reqs)
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

template<typename T1, typename T2>
bool myequal(T1 sA, T1 eA, T2 sB)
{
	while (sA != eA)
		if (*(sA++) != *(sB++))
			return false;

	return true;
}

bool BackInferenceTree::eq(BackInferenceTree* rhs) const
{
	return eq(rhs->rule, rhs->args, rhs->GetTarget(), rhs->GetPreBindings());
}

template<typename T>
bool equal_indirect(const T& a, const T& b) { return *a == *b; }

bool BackInferenceTree::eq(Rule* r,  const Rule::MPs& _args, meta _target, const bindingsT& _pre_bindings) const
{
	try
	{
		bool ret=false;

		if (rule != r)
			ret = false;
		else if (rule && !rule->isComputable())
		{
			//	meta _final_target(bind_vtree(*_target, GetPreBindings()));
			meta _final_target(new vtree(*_target));
			ret = (*GetTarget() == *_final_target);
		}
		else
		{
			assert(!args.empty());

			meta _final_target(new vtree(*_target));
			ret= (*GetTarget() == *_final_target);

			ret = (args.size() == _args.size()
				&& std::equal(args.begin(), args.end(), _args.begin(), &equal_indirect<meta>));
//haxx:: \todo changed into more conservative...
//				&& std::equal(args.begin(), args.end(), _args.begin(), &equalVariableStructure2));

/*			if (ret &&
				!(args.size() == _args.size()	&&
				  std::equal(args.begin(), args.end(), _args.begin(), &equal_indirect<meta>)))
			{
				rawPrint(*_target, _target->begin(), -1);
				puts("vs");
				rawPrint(*GetTarget(), GetTarget()->begin(), -1);

				foreach(meta arg, _args)
					rawPrint(*arg, arg->begin(), -1);
				printArgs();
			}*/
		}

		if (ret)
			tlog(3, ret?"EQ\n": "IN-EQ\n");

		return ret;
	} catch(PLNexception e) { puts(e.what()); puts("Apparently pre-bindings to eq() were inconsistent."); throw; }
}

BackInferenceTree* BackInferenceTree::HasChild(BackInferenceTree* new_child, int arg_i) const
{
	foreach(const pBITree& c, children[arg_i])
		if (c->eq(new_child))
		{
			tlog(3,"Has this child already.\n");
			return c;
		}

	return NULL;
}

BackInferenceTree* BackInferenceTree::HasChild(int arg_i, Rule* r, 
	const Rule::MPs& _args, meta _target, const bindingsT& _pre_bindings) const
{
	foreach(const pBITree& c, children[arg_i])
		if (c->eq(r, _args, _target, _pre_bindings))
		{
			tlog(3,"Has this child already.\n");
			return c;
		}

	return NULL;
}
	
bool BackInferenceTree::ObeysSubtreePolicy(Rule *new_rule, meta _target)
{
//	return !(inheritsType(nm->getTypeV(*_target), NODE)
//		&& new_rule->computable());

	return true;
}

bool BackInferenceTree::ObeysPoolPolicy(Rule *new_rule, meta _target)
{
	AtomSpace *nm = CogServer::getAtomSpace();
	if (inheritsType(nm->getTypeV(*_target), FW_VARIABLE_NODE))
		return false;

	return true;
}

void BackInferenceTree::addNewParent(parent_link<BackInferenceTree> new_parent)
{
/*	/// The new parent cannot have any additional pre_bindings.

	foreach(const hpair& b, new_parent.link->GetPreBindings())
	{
		bindingsT::const_iterator i = this->GetPreBindings().find(b.first);

		if (i == this->GetPreBindings().end() || i->second != b.second)
			throw PLNexception("addNewParent(): Inconsistent parent bindings!!!");
	}
*/
	parents.insert(new_parent);
}

BackInferenceTree* BackInferenceTree::FindNode(BackInferenceTree* new_child) const
{
	BackInferenceTree* ret = NULL;

	for (uint i=0; i<children.size(); i++)
		if ((ret=HasChild(new_child, i)) != NULL)
			return ret;

	//foreach (const set<BackInferenceTree*>& sBIT, children)
	for (uint i=0; i<children.size();i++)
		foreach(BackInferenceTree* c, children[i])
			if ((ret=c->FindNode(new_child)) != NULL)
				return ret;

	return NULL;
}


BackInferenceTree* BackInferenceTree::FindNode(Rule* new_rule, meta _target,
	const Rule::MPs& rule_args, const bindingsT& _pre_bindings) const
{
	/// _target has to be ForceAllLinksVirtual()ized beforehand!

	BackInferenceTree* ret = NULL;

	for (uint i=0; i<children.size(); i++)
		if ((ret=HasChild(i, new_rule, rule_args, _target, _pre_bindings)) != NULL)
			return ret;

	//foreach (const set<BackInferenceTree*>& sBIT, children)
	for (uint i=0; i<children.size();i++)
		foreach(BackInferenceTree* c, children[i])
			if ((ret=c->FindNode(new_rule, _target, rule_args, _pre_bindings)) != NULL)
				return ret;

	return NULL;
}

BackInferenceTree* BackInferenceTree::CreateChild(unsigned int target_i, Rule* new_rule,
	const Rule::MPs& rule_args, BBvtree _target, const bindingsT& new_bindings,
	spawn_mode spawning)
{
	if (this->depth == haxx::maxDepth)
	{
		puts("haxx::maxDepth !!! press enter");
		getc(stdin);
		return NULL;
	}

	/// If any new requirement can, upon suitable substitutions,
	/// produce a requirement higher in the tree, reject it to avoid
	/// looping.

	if ((!PREVENT_LOOPS || !inferenceLoop(rule_args)))
	{
		Btr<bindingsT> bindings(new bindingsT(GetPreBindings()));
		///haxx::
		Btr<map<Vertex, Vertex> > pre_bindingsV(toVertexMap(GetPreBindings().begin(), GetPreBindings().end()));
		Btr<map<Vertex, Vertex> > new_bindingsV(toVertexMap(new_bindings.begin(), new_bindings.end()));

		if (!consistent<Vertex>(*pre_bindingsV, *new_bindingsV, pre_bindingsV->begin(), pre_bindingsV->end(), new_bindingsV->begin(), new_bindingsV->end()))
		{
/*			puts("First bindings:");
			for_each(GetPreBindings().begin(), GetPreBindings().end(), &print_binding);
			puts("2nd bindings:");
			for_each(new_bindings.begin(), new_bindings.end(), &print_binding);
*/
			tlog(0, "Binding INCONSISTENT. Child not created.\n");
			//getc(stdin);getc(stdin);

			/// \todo Check if coming here actually is allowed by the design, or whether it's a bug.

			return NULL;
		}
		/// Bindings from the node that spawned me:
		bindings->insert(new_bindings.begin(), new_bindings.end());
		/// Bindings from the Rule.o2i that produced my target:
		if (_target->bindings)
			bindings->insert(_target->bindings->begin(), _target->bindings->end());
		
		BackInferenceTree* new_node = new BackInferenceTree(
				root, this, depth+1, target_i, _target, new_rule, rule_args,
				target_chain, bindings, spawning, false);

//tlog(0,"000000000000000000000000000\n");

		/// TEMPORARILY DISABLED THE FindNode BECAUSE IT'S A SPEED BOTTLENECK

		BackInferenceTree* existing_node = 
			this->HasChild(new_node, target_i); ///Search only 
///			NULL;
//			root->FindNode(new_node);
/*
\todo what the **** should we do with this?

		/// If the node would go to the pool, we'll control it with the BITstore. Otherwise we don't.
		if (ObeysPoolPolicy(new_rule, _target))
		{
			BITstoreT::iterator biti = root->BITstore.find(new_node);
			if (root->BITstore.end() == biti)
			{
				root->BITstore.insert(new_node);

				//printf("Added new node. Store size: %d / %d\n", BITstore.size(), root->exec_pool.size());
			}
			else
			{
				existing_node = *biti;

				//printf("Existing node. Store size: %d / %d\n", BITstore.size(), root->exec_pool.size());
			}
		}
*/
tlog(0,"------------------------------------------------------------\n");
		
		if (existing_node) ///If exists already
		{
			delete new_node;

			foreach(const parent_link<BackInferenceTree>& p, existing_node->parents)
				if (p.link == this &&
                        p.parent_arg_i == target_i)
					return existing_node;

			children[target_i].insert(existing_node);
			existing_node->addNewParent(parent_link<BackInferenceTree>(this, target_i));
			new_node = existing_node;
		}
		else
		{
			new_node->Create();

			tlog(2, "Created new BIT child [%ld]\n", (long)new_node);

			if (ObeysPoolPolicy(new_rule, _target))
			{
				root->exec_pool.push_back(new_node);
				root->exec_pool_sorted = false;
			}

			children[target_i].insert(new_node);
		}

		return new_node;
	}

	return NULL;
}

void BackInferenceTree::spawn(Btr<bindingsT> bindings)
{
	set<Vertex> bind_key_set;
	transform(bindings->begin(),
		bindings->end(),
		inserter(bind_key_set, bind_key_set.begin()),
		bind(&first<Handle,Handle>, _1));

	/// Each NEW target variable must be assigned an owner: the bound clone of the owner of the key variable.
	/// this is done in SimpleClone().

	/// Clone the owner of each newly bound variable so that the new bind is applied to the clone

	tlog(0, "Cloning...");
	set<BackInferenceTree*> already_cloned;
	foreach(Vertex v, bind_key_set)
	{

/*		if (!STLhas(root->varOwner, v))
		{
			for_each(bindings->begin(), bindings->end(), pr3);
			printTree(v2h(v),0,0);
			puts("my args:");
			foreach(meta _arg, args)
				rawPrint(*_arg, _arg->begin(), 0);
		}

		assert(STLhas(varOwner, v));*/

		/// Actually some LHS vars in bindings do not always have owners. We'll just forget abt them.

		if (STLhas(root->varOwner, v) && !STLhas(already_cloned, root->varOwner[v]))
		{
			root->varOwner[v]->SimpleClone(*bindings);
			already_cloned.insert(root->varOwner[v]);
		}
	}
}

/// If any of the bound vars are owned, we'll spawn.
/// TODO: Actually Rules should probably not even need to inform us
/// about the bound vars if they are new, in which case this check would be redundant.

bool BackInferenceTreeRoot::spawns(const bindingsT& bindings) const
{
	foreach(hpair b, bindings)
		if (STLhas(varOwner, b.first))
			return true;

	return false;
}

template<typename V, typename Vit, typename Tit>
void copy_vars(V& vars, Vit varsbegin, Tit bbvt_begin, Tit bbvt_end)
{
	/*
	copy_if(bbvt_begin, bbvt_end, inserter(vars, varsbegin), 
		bind(equal_to<Type>(), 
		bind(getTypeFun, bind(&_v2h, _1)),
		(Type)FW_VARIABLE_NODE));
	*/
}

bool BackInferenceTree::expandRule(Rule *new_rule, int target_i, BBvtree _target, Btr<bindingsT> bindings, spawn_mode spawning)
{
AtomSpace *nm = CogServer::getAtomSpace();	
	bool ret = true;
	try
	{
		tlog(2, "Expanding rule... %s\n", (new_rule ? new_rule->name.c_str() : "?"));		

		if (!new_rule->isComputable())			
		{
			CreateChild(target_i, new_rule, Rule::MPs(), _target, *bindings, spawning);
		}
		else
		{
			rawPrint(*_target, _target->begin(), 3);

			if (!ObeysSubtreePolicy(new_rule, _target))
			{
				tlog(3, "Our policy is to filter this kind of BIT nodes out.\n");
				return false;
			}

			/// Different argument vectors of the same rule (eg. ForAllRule) may have different bindings.

//time( &test::custom_start2 );

			meta virtualized_target(bindings ? bind_vtree(*_target, *bindings) : meta(new vtree(*_target)));
			ForceAllLinksVirtual(virtualized_target);

//time( &test::custom_finish2 );
//test::custom_duration2 += difftime( test::custom_finish2, test::custom_start2 );

			set<Rule::MPs> target_v_set = new_rule->o2iMeta(virtualized_target);
			//test::custom_duration += (double)(test::custom_finish - test::custom_start) / CLOCKS_PER_SEC;
						
			if (target_v_set.empty())
			{
				tlog(3,"This rule is useless.\n");
				return false;
			}
			else
			{		
				tlog(2,"Rule.o2i gave %d results\n",target_v_set.size());

				for (set<Rule::MPs>::iterator 	j =target_v_set.begin();
											j!=target_v_set.end();
											j++)
				{		
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
						spawn(jtree->bindings);
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

						if (!spawning)
							if (jtree->bindings)
							{
								int temp00 = 0; //currentDebugLevel;
								printf("OOOOOOO\n");
								//currentDebugLevel = temp00;
							}

							BackInferenceTree* new_node = CreateChild(target_i, new_rule, *j,_target, *combined_binds, spawning);					

							/// Identify the _new_ variables of this set
							/// And add _new_ variables to var dependency map for the new Child (-InferenceNode)

							set<Vertex> vars, new_vars;
							foreach(const BBvtree& bbvt, *j)
								copy_vars(vars, vars.begin(), bbvt->begin(), bbvt->end());

							printf("%u vars total\n", (unsigned int) vars.size());

							foreach(Vertex v, vars)
								if (!STLhas2(*virtualized_target, v))
								{
									if (STLhas(root->varOwner, v))
									{
										printf("%ld owns \n", (long)root->varOwner[v]);
										printTree(v2h(v),0,0);
										printf("I am %ld. Target:\n", (long)this);
										rawPrint(*virtualized_target, virtualized_target->begin(), 0);
										puts("new args:");
										foreach(meta _arg, new_node->args)
											rawPrint(*_arg, _arg->begin(), 0);
										puts("my args:");
										foreach(meta _arg, args)
											rawPrint(*_arg, _arg->begin(), 0);
									}

									assert(!STLhas(root->varOwner, v));

									root->varOwner[v] = new_node;

									printf("[%ld] owns %s\n", (long)new_node, nm->getName(v2h(v)).c_str());
								}
								/// When one of these vars is bound later, clone the child with arg list in which that var has been bound.
					}

				}
			
				return !children[target_i].empty();
			}
		}
	} catch(...) { tlog(0,"Exception in ExpandRule()"); throw; }

	  return ret;
}

void BackInferenceTree::SimpleClone(const bindingsT& added_binds) const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	tlog(0,"SImpleClone:");

	Btr<bindingsT> new_binds(new bindingsT(GetPreBindings()));
	try {
		insert_with_consistency_check(*new_binds, added_binds.begin(), added_binds.end());
	} catch(...) {
		tlog(0,"SimpleClone: inconsistent bindings");
		return;
	}

	Rule::MPs new_args;

	Rule::CloneArgs(this->args, new_args);

	/// Bind the args
	foreach(BBvtree& bbvt, new_args)
	{
		bbvt = BBvtree(new BoundVTree(*bind_vtree(*bbvt, *new_binds)));
		ForceAllLinksVirtual(bbvt);
	}

	foreach(const parent_link<BackInferenceTree>& p, parents)
	{
		/// Create Child with this new binding:

		if (p.link && p.link->rule && !p.link->rule->validate2(new_args))
			continue;

		BackInferenceTree* new_node = p.link->CreateChild(
			p.parent_arg_i,
			this->rule,
			new_args,
			Btr<BoundVTree>(new BoundVTree(*this->raw_target)),
			*new_binds,
			NO_SIBLING_SPAWNING); //Last arg probably redundant now

		if (!new_node)
			return;

		/// Each NEW target variable must be assigned an owner: the bound clone of the owner of the key variable.

		foreach(const hpair& b, added_binds)
			if (STLhas(root->varOwner, b.first) && root->varOwner[b.first] == this)
			{
				/// Isolate RHS variables

				set<Vertex> vars;
				vtree virtualized_b_second(make_vtree(b.second));

				copy_vars(vars, vars.begin(),virtualized_b_second.begin(), virtualized_b_second.end());

				printf("%u vars total\n", (unsigned int) vars.size());

				foreach(const Vertex& v, vars)
					if (!STLhas(root->varOwner, v))
					{
						printf("Node %s was new. Now owned by [%ld].\n", nm->getName(v2h(v)).c_str(), (long)new_node);
						root->varOwner[v] = new_node;
					}
			}
	}
}

static bool bigcounter = true;

int BackInferenceTree::tlog(int debugLevel, const char *format, ...) const
	{
	    //if (debugLevel > currentDebugLevel) return 0;

		 //if (test::bigcount == 75229)
		 { puts("Debug feature."); }
		
		 //printf("%d %d [(%d)] (%s): ", (bigcounter?(++test::bigcount):depth), root->InferenceNodes,
		//	(test::LOG_WITH_NODE_ID ? ((int)this) : 0),
		//	(rule ? (rule->name.c_str()) : "(root)"));
		
		//if (test::logfile)
		//	fprintf(test::logfile, "%d %d [(%d)] (%s): ", depth, root->InferenceNodes,
		//	(test::LOG_WITH_NODE_ID ? ((int)this) : 0),
		//	(rule ? (rule->name.c_str()) : "(root)"));
		
		char buf[5000];
		
	    va_list ap;
	    va_start(ap, format);
		int answer = vsprintf(buf, format, ap);
		
	    printf(buf);
		
		//if (test::logfile)
		//	fprintf(test::logfile, buf);
		
	    //fflush(stdout);
	    va_end(ap);
	    return answer;
	}

void BackInferenceTree::printTarget() const
{
	printf("Raw target:\n");
	rawPrint(*raw_target, raw_target->begin(),0);
	printf("Bound target:\n");
	rawPrint(*GetTarget(), GetTarget()->begin(),0);
}

set<const BackInferenceTree*> tempchain;

void BackInferenceTree::ValidateRuleArgs(const vector<BoundVertex>& rule_args) const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	tlog(1, "Rule args:\n");
	foreach(BoundVertex v, rule_args)
		printTree(v2h(v.value),0,2);

	/// Check for wild FreeVariableNodes 

	/*				foreach(BoundVertex v, rule_args)
	{
	if (v.bindings)
	foreach(hpair t, *v.bindings)
	{
	if (nm->getType(t.second) == FW_VARIABLE_NODE)
	{
	LOG(0, "FW_VARIABLE_NODE found on binding's RHS!");
	getc(stdin);
	}
	}
	}	*/

	for(vector<BoundVertex>::const_iterator bv = rule_args.begin(); bv != rule_args.end(); bv++)
		if (nm->getType(v2h(bv->value)) == FW_VARIABLE_NODE)
		{
			printf("FW_VARIABLE_NODE found on Rule args: ");

/*			tlog(0,"Bindings were:\n");
			foreach(hpair phh, *bindings_of_all_args)
			{
				printTree(phh.first,0,3);
				cprintf(3,"=>");
				printTree(phh.second,0,3);
			}*/
			tlog(0,"Args were:\n");
			foreach(BoundVertex v, rule_args)
				printTree(v2h(v.value),0,0);
		}
}

bool BackInferenceTree::ValidRuleResult(BoundVertex& new_result, const vector<BoundVertex>& rule_args, Btr<bindingsT> bindings_of_all_args) const
{
	Handle* ph;

	if ((ph = v2h(&new_result.value)) && !*ph)
	{
		tlog(1, "Rule returned NULL! Args were:\n");
		foreach(const BoundVertex& v, rule_args)
			printTree(v2h(v.value),0,1);

		tlog(1,"Binds were (%d):\n", bindings_of_all_args->size());			

		foreach(hpair phh, *bindings_of_all_args)
		{
			printTree(phh.first,0,3);
			printf("=>");
			printTree(phh.second,0,3);
		}			

		getc(stdin);getc(stdin);

		return false;
	}
	else
	{
		tlog(2,"::compute() resulted in: \n");
		printTree(v2h(new_result.value),0,2);
		printSubsts(new_result,2);
	
		return true;
	}
}	

void BackInferenceTree::WithLog_expandVectorSet(const std::vector<Btr<BV_Set> >& child_results,
					 set<BV_Vector>& argVectorSet) const
{
	tlog(3, 	"expandVectorSet(child_results, argVectorSet)...\n");

	//expandVectorSet(child_results, argVectorSet);

	tlog(2, 	"RESULT FROm expandVectorSet(child_results, argVectorSet) ((%d,%d), %d,%d)...\n", (child_results.size()>0?child_results[0]->size():0), 
		(child_results.size()>1?child_results[1]->size():0), argVectorSet.size(),	(argVectorSet.empty() ? 0 : argVectorSet.begin()->size()) );	

	if (!child_results.empty())
	{ string bc = string("Arg Expansion results:\n");
	foreach(vector<BoundVertex> vbv, argVectorSet)
		if (!vbv.empty())
		{
			bc += "(";
			foreach(BoundVertex bv, vbv)
				bc += i2str((int)v2h(bv.value)) + " ";
			bc += ")\n";
		}
		tlog(1, bc.c_str());
	}
}

Btr<set<BoundVertex> > BackInferenceTree::evaluate(set<const BackInferenceTree*>* chain) const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	Btr<set<BoundVertex> > ret(new set<BoundVertex>(*my_results));

	if (!chain)
	{
		chain = &tempchain;
		chain->clear();
	}

	if (STLhas(*chain, this))
	{
		printf("evaluate: circle!");
		//		getc(stdin);
		return ret;
	}
	chain->insert(this);

	tlog(1,"evalute()\n");
	printChildrenSizes();

	vector<Btr<set<BoundVertex> > > child_results;
	for (uint i=0;i<args.size();i++)
		child_results.push_back(Btr<set<BoundVertex> >(new set<BoundVertex>));

	for (uint c=0;c<children.size();c++)
		for (set<BackInferenceTree*>::const_iterator i =children[c].begin();
			i!=children[c].end(); i++)
		{
			Btr< set<BoundVertex> > i_ret = (*i)->evaluate(chain);
			if (!i_ret->empty())
				child_results[c]->insert(i_ret->begin(), i_ret->end());
		}


	if (rule)
	{
		std::set<vector<BoundVertex> > argVectorSet;
		WithLog_expandVectorSet(child_results, argVectorSet);

		for (std::set<vector<BoundVertex> >::iterator	a = argVectorSet.begin();
																		a!= argVectorSet.end();
																		a++)
			if (!a->empty() && (rule->hasFreeInputArity() || a->size() >= rule->getInputFilter().size()))
			{
				/// Arg vector size excession prohibited.

				const vector<BoundVertex>& rule_args = *a;
				
				Btr<bindingsT> bindings_of_all_args(new bindingsT(GetPreBindings()));
				
				/// Reconstruct bindings for the new_result. Consistency has been verified.
				/// Make all args use the same complete binding set.
				/// Although normally each BoundVertex has its own bindings already,
				/// Hypotheticals do not. So, effectively, this step is just to bind the
				/// variables of hypotheticals.

				/*for(vector<BoundVertex>::iterator bv = a->begin(); bv != a->end(); bv++)
					if (bv->bindings)
					{
						bindings_of_all_args->insert(bv->bindings->begin(), bv->bindings->end());
														
						bv->bindings = bindings_of_all_args;
					}		
			*/
				ValidateRuleArgs(rule_args);

				BoundVertex new_result;

				foreach(BoundVertex bv, rule_args)
				{
					//if (!nm->inheritsType(nm->getType(v2h(bv.value)), HYPOTHETICAL_LINK) &&
						//nm->getTruthValue(v2h(bv.value))->getConfidence() < MIN_CONFIDENCE_FOR_RULE_APPLICATION)
					//	goto next_args;

//					printf("(%.6f, ", nm->getTruthValue(v2h(bv.value))->getConfidence());
				}
//				printf(") OK\n");

				new_result = rule->compute(rule_args);
				
				if (ValidRuleResult(new_result, rule_args, bindings_of_all_args))
				{
					ret->insert(new_result);

					root->hsource[v2h(new_result.value)] = const_cast<BackInferenceTree*>(this);

					//if (RECORD_TRAILS)
						foreach(const BoundVertex& v, rule_args)
						{
							root->inferred_from[v2h(new_result.value)].push_back(v2h(v.value));
							root->inferred_with[v2h(new_result.value)] = rule;
						}						
				}
//next_args:;
			}
	}
	else return child_results[0];
	
	foreach(BoundVertex bv, *ret)
	{
		if (!bv.bindings)
			bv.bindings = Btr<bindingsT>(new bindingsT);

		bv.bindings->insert(GetPreBindings().begin(), GetPreBindings().end());

		tlog(1,"RET [%d]\n", v2h(bv.value));

//		if (bv.bindings)
//			for_each(bv.bindings->begin(), bv.bindings->end(), print_binding);
	}
		
	chain->erase(this);
	return ret;
}

/// \todo Currently always uses ForAllRule to generalize. Should be different for varScopeLink
/// \todo Currently return value topology is wrong.

BoundVertex BackInferenceTreeRoot::Generalize(Btr<set<BoundVertex> > bvs, Type _resultT) const
{
	vector<Vertex> ForAllArgs;
	BoundVertex new_result((Handle)ATOM);

	const float min_confidence = 0.0001f;

	if (!bvs->empty())
	{
		printf("\n");
		tlog(0,"Generalizing results:\n");

		foreach(const BoundVertex& b, *bvs)
		{
			//if (getTruthValue(v2h(b.value))->getConfidence() > min_confidence)
			//{
			//	printTree(v2h(b.value),0,0);
			//	ForAllArgs.push_back(b.value);
			//}
		}
		new_result = RuleRepository::Instance().rule[(_resultT == FORALL_LINK) ? ForAll : PLNPredicate]->compute(ForAllArgs);

		printf("\n");
		tlog(0,"Combining %d results for final unification. Result was:\n", ForAllArgs.size());
		printTree(v2h(new_result.value),0,0);
	}
	else
		tlog(1,"NO Results for the root query.\n");

	return new_result;
}

BackInferenceTree::~BackInferenceTree()
{
    printf("Dying... %ld => %ld\n", root->InferenceNodes, root->InferenceNodes-1);
    root->InferenceNodes--;
/// TODO: TEMPORARY DISABLED FOR DEBUGGING!!! RE-ENABLE!
//    ReleaseChildren();
}
	
void BackInferenceTree::ReleaseChildren()
{
	for (vector< set<BackInferenceTree*> >::iterator i =children.begin();
		i != children.end(); i++)
	{
		for (set<BackInferenceTree*>::iterator j=i->begin(); j!=i->end(); j++)
			delete *j;
	}
	children.clear();
}

void BackInferenceTree::printFitnessPool()
{
	tlog(0,"Fitness table: (%d)\n", root->exec_pool.size());
	getc(stdin);getc(stdin);

	if (!root->exec_pool.empty())
	{
		if (!root->exec_pool_sorted)
		{
			root->exec_pool.sort(BackInferenceTree_fitness_comp());
			root->exec_pool_sorted = true;
		}

		for (list<BackInferenceTree*>::iterator i = root->exec_pool.begin(); i != root->exec_pool.end(); i++)		
		{
			(*i)->tlog(0, ": %f / %d [%ld]\n", (*i)->fitness(), (*i)->children.size(), (long)(*i));

			if (!((more_count++)%25))
			{
				puts(" --- more ");
				getc(stdin);
			}
		}
	}
}


/// TODO: May not work. Possibly redundant anyway.
void BackInferenceTree::expandSubtree()
{
	for (uint c=0;c<children.size();c++)
		for (set<BackInferenceTree*>::const_iterator i =children[c].begin();
			i!=children[c].end(); i++)
			if (find(root->exec_pool.begin(), root->exec_pool.end(), *i) != root->exec_pool.end())
				(*i)->expandNextLevel();
}

void BackInferenceTree::expandFittest()
{
	tlog(1,"Fitness table: (%d)\n", root->exec_pool.size());

	print_progress();
	
	BackInferenceTree* bisse = NULL;
	
	if (!root->exec_pool.empty())
	{
		if (true) //Sort and print
		{
			if (!root->exec_pool_sorted)
			{
				root->exec_pool.sort(BackInferenceTree_fitness_comp());
				root->exec_pool_sorted = true;
			}

			for (list<BackInferenceTree*>::iterator i = root->exec_pool.begin(); i != root->exec_pool.end(); i++)		
				(*i)->tlog(1, ": %f / %d [%ld]\n", (*i)->fitness(), (*i)->children.size(), (long)(*i));
				
			bisse = (*root->exec_pool.begin());
		}
		else //Just find the best one
		{
			bisse = (*root->exec_pool.begin());
			float best_fitness = bisse->fitness();
			float next_fitness = 0.0f;
			foreach(BackInferenceTree* bis, root->exec_pool)
				if (best_fitness < (next_fitness=bis->fitness()))
				{
					best_fitness = next_fitness;
					bisse = bis;
				}
		}

		bisse->expandNextLevel();
	}
}

bool BackInferenceTree::CreateChildren(int i, BBvtree arg, Btr<bindingsT> bindings, spawn_mode spawning)
{
	assert(!arg->empty());

	tlog(1,"arg #%d. To produce:\n", i);
	rawPrint(*arg, arg->begin(),1); 

	tlog(1,"Creating children...\n");

	if (inferenceLoopWith(arg))
	{
		expandRule(RuleRepository::Instance().rule[Hypothesis], i, arg, bindings, spawning);
		tlog(2,"LOOP! Assumed Hypothetically:");
		rawPrint(*arg, arg->begin(), 2);
	}
	else
	{
		//for (uint j=0;j<root->rp->get().size();j++)
		//	expandRule(RuleRepository::Instance().rule[root->rp->get()[j]], i, arg, bindings, spawning);
	}
	tlog(1,"Rule expansion ok!\n");

	if (children[i].empty())
	{
		tlog(1,"Arg %d proof failure.\n",i);

		return false;
	}
			
	return true;
}

void BackInferenceTree::CreateChildrenForAllArgs()
{
	tlog(1,"---CreateChildrenForAllArgs()\n");	
	
	for (uint i = 0; i < args.size(); i++)
		if (!CreateChildren(i, args[i], Btr<bindingsT>(new bindingsT), ALLOW_SIBLING_SPAWNING))
			break;
}

bool BackInferenceTree::CheckForDirectResults()
{
	AtomSpace *nm = CogServer::getAtomSpace();
	if (!rule || rule->isComputable())
		return false;

	Handle th = v2h(*GetTarget()->begin());
	if (nm->isReal(th) && nm->getType(th) == FW_VARIABLE_NODE)
	{
		tlog(2,"Proof of FW_VARIABLE_NODE prohibited.\n");
		return true;
	}

	tlog(2,"attemptDirectProduction. Target:\n");
	rawPrint(*bound_target, bound_target->begin(), 2);
	
	boost::shared_ptr<set<BoundVertex> > directResult = rule->attemptDirectProduction(bound_target);
		
	if (directResult && !directResult->empty())
	{
tlog(2,"%d direct child_results generated:\n", directResult->size());
		addDirectResult(directResult, ALLOW_SIBLING_SPAWNING);				
tlog(4,"I now has %d direct child_results.\n", my_results->size());
			
		return true;
	}
	else
	{
		tlog(3,"NO direct child_results generated.\n");
		return false;			
	}
}

/// The complexity here results from bindings and virtuality.
/// The bindings may convert some atoms to either virtual or real atoms.
/// In the end, all links must be virtual. But simple conversion is not enough,
/// because if the root link is real, then it must be collected as a direct
/// result first.

void BackInferenceTree::ForceTargetVirtual(spawn_mode spawning)
{
	AtomSpace *nm = CogServer::getAtomSpace();
	Handle *ph = v2h(&(*raw_target->begin()));
	
	if (ph && nm->isReal(*ph) && nm->getType(*ph) != FW_VARIABLE_NODE)
	{
		printf("ForceTargetVirtual: Arg [%ld] (exists).\n", (long)*ph);
		
		boost::shared_ptr<set<BoundVertex> > directResult(new set<BoundVertex>);
		
		/// Though the target was conceived directly, it is under my pre-bindings!
		directResult->insert(BoundVertex(*ph, pre_bindings));

		addDirectResult(directResult, spawning);
		
		//SetTarget(ForceAllLinksVirtual(bound_target));
		SetTarget(meta(new vtree(make_vtree(*ph))));
	}

	/// TODO: There's some redundancy here that should be removed...

	bound_target = ForceAllLinksVirtual(bound_target);
	raw_target = ForceAllLinksVirtual(raw_target);
}

void BackInferenceTree::expandNextLevel()
{
	AtomSpace *nm = CogServer::getAtomSpace();
  try
  {
	if (nm->getType(v2h(*GetTarget()->begin())) == FW_VARIABLE_NODE)	 
		tlog(2, "Target is FW_VARIABLE_NODE! Intended? Dunno.\n");
	
	tlog(2, "Rule:%s: ExpandNextLevel (%d children exist)\n", (rule?(rule->name.c_str()):"(root)"), children.size());
	  
	root->exec_pool.remove_if(bind2nd(equal_to<BackInferenceTree*>(), this));
	
	print_progress();
	
	if (!Expanded) //children.empty() || (children[0].empty() && child_results[0]->empty()))
	{
		CheckForDirectResults();

		CreateChildrenForAllArgs();

		Expanded = true;
	}
	else
		for (uint i = 0; i < args.size(); i++)
		{
			tlog(1,"Expand children...\n");
			
			foreach(BackInferenceTree* bisse, children[i])
				bisse->expandNextLevel();
			
/*			for (set<BackInferenceTree*>::iterator k=children[i].begin();
													k!=children[i].end();)
				this->removeIfFailure(*(k++));*/
			
			if (children[i].empty())
			{
				tlog(1,"Arg %d proof failure.\n",i);
			
				break;
			}			
		}		
  } catch(...) { tlog(0,"Exception in ExpandNextLevel()"); throw; }
}

	Btr<set<BoundVertex> > BackInferenceTree::pseudo_evaluate() const
	{
		Btr<set<BoundVertex> > ret(new set<BoundVertex>(*my_results));
		vector<Btr<set<BoundVertex> > > pseudo_child_results;
		
tlog(3, "%d children for rule %s:\n", children.size(), (rule ? rule->name.c_str() : "(root)"));
		
		printChildrenSizes();
		
		assert(rule || children.size() == 1);
		
		for (uint c=0;c<children.size();c++)
		{
tlog(3, " Child %d with %d child_results...\n", c, children[c].size());
			for (set<BackInferenceTree*>::const_iterator i =children[c].begin();
													i!=children[c].end(); i++)
			{
				Btr< set<BoundVertex> > i_ret = (*i)->pseudo_evaluate();
				assert(i_ret);

tlog(3, "   nEw Result set's (from %s) size %d. Inserting...\n", (*i)->rule->name.c_str(), i_ret->size());

				if (!i_ret->empty())
					pseudo_child_results[c]->insert(i_ret->begin(), i_ret->end());

tlog(3, "   nOw Result set for arg %d has size %d!\n", c, pseudo_child_results[c]->size());
		  	}
		}
		
		std::set<vector<BoundVertex> > argVectorSet;
tlog(3, 	"expandVectorSet(child_results, argVectorSet)...\n");
		//expandVectorSet(pseudo_child_results, argVectorSet);
tlog(3, 	"RESULT FROm expandVectorSet(child_results, argVectorSet) ((%d,%d), %d,%d)...\n", (pseudo_child_results.size()>0?pseudo_child_results[0]->size():0), 
			(pseudo_child_results.size()>1?pseudo_child_results[1]->size():0), argVectorSet.size(),	(argVectorSet.empty() ? 0 : argVectorSet.begin()->size()) );
		
		if (rule)
		{
tlog(3, "Feeding children to rule %s:\n", rule->name.c_str());
			
			for (std::set<vector<BoundVertex> >::iterator	a = argVectorSet.begin();
															a!= argVectorSet.end();
															a++)
			if (!a->empty() && (rule->hasFreeInputArity() || a->size() >= rule->getInputFilter().size()))
			{
				BoundVertex new_result(Vertex((Handle)NULL), new bindingsT);
				foreach(BoundVertex bv, *a)
				{
					if (bv.bindings)
						new_result.bindings->insert(bv.bindings->begin(), bv.bindings->end());
				}
				ret->insert(new_result);
			}
		}
		else
		{
			if (!argVectorSet.empty())
			{
				for (std::set<vector<BoundVertex> >::iterator aset = argVectorSet.begin();
																aset!= argVectorSet.end();
																aset++)
					if (!aset->empty())
					{
						ret->insert( (*aset)[0] );
						break; //Only 1 result is needed 'cause we're at the root.
					}		
			}
			else
				tlog(0,"NO Results for the root query.\n");
				
		}
		
		return ret;
	}

void BackInferenceTreeRoot::print_trail(Handle h, unsigned int level) const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	if (!h || !nm->isReal(h))
		puts("NULL / Virtual? Syntax: t<enter> Handle#<enter>");
	map<Handle,Rule*> ::const_iterator rule = inferred_with.find(h);
	if (rule != inferred_with.end())
	{
		printf("%s[%ld] was produced by applying %s to:\n", repeatc(' ', level*3).c_str(), (long)h, rule->second->name.c_str());
		map<Handle,vector<Handle> >::const_iterator h_it = inferred_from.find(h);
		assert (h_it != inferred_from.end());
		foreach(Handle arg_h, h_it->second)
		{
			printTree(arg_h,level+1,0);
			print_trail(arg_h, level+1);
		}
	}
	else
		cout << repeatc(' ', level*3) << "which is trivial (or axiom).\n";
}

void BackInferenceTree::printArgs() const
{
	printf("%u args:\n", (unsigned int) args.size());
	foreach(meta _arg, args)
		rawPrint(*_arg, _arg->begin(), 0);
}

void BackInferenceTreeRoot::print_trail(Handle h) const
{
	printTree(h,0,0);
	print_trail(h,0);
}

void BackInferenceTreeRoot::extract_plan(Handle h, unsigned int level, vtree& do_template, vector<Handle>& plan) const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	map<Handle, vtree> bindings;
	
	if (!h || !nm->isReal(h))
		puts("NULL / Virtual? Syntax: t<enter> Handle#<enter>");
	
	map<Handle,Rule*> ::const_iterator rule = inferred_with.find(h);
	
	if (rule != inferred_with.end())
	{
		foreach(Handle arg_h, root->inferred_from[h])
		{
			if (unifiesTo(do_template, make_vtree(arg_h), bindings, bindings, true))
			{
				//printTree(arg_h,level+1,0);
				plan.push_back(arg_h);
			}
		
			extract_plan(arg_h, level+1, do_template, plan);
		}
	}
}

void BackInferenceTreeRoot::extract_plan(Handle h) const
{
	AtomSpace *nm = CogServer::getAtomSpace();
	vtree do_template = mva(EVALUATION_LINK,
							NewNode(PREDICATE_NODE, "do"),
							mva(LIST_LINK,
								NewNode(FW_VARIABLE_NODE, "$999999999")));

	vector<Handle> plan;
	extract_plan(h,0,do_template,plan);
	for (vector<Handle>::reverse_iterator i = plan.rbegin(); i!=plan.rend(); i++)
		printTree(*i,0,0);
}


/*Btr<BV_Set> BackInferenceSubState::w_evaluate()
{
	for_each(next_targets_set)
		for_each(next_targets_set[i])
			vectorset += i.w_evaluate();
	
	set<pair<Rule*,watom> > t = combine(vectorset, set< (R,arg_vector,bindings)>);
	
	for_each(t)
		ret.insert(t->first->compute(convert(t->second())));
}*/

/*

/// Obsolete insofar as status flag is no longer used


bool is_failure(BackInferenceTree* s) {
//				printf("Is: %d\n", s->Status());
					assert(s->Status()!=FAILURE || s->children.empty());
					return s->Status()==FAILURE;
}		

CHAINER_STATUS BackInferenceTree::Status() const { return my_status; }

///This manual check for SOLUTIONity is no longer needed:
	
	int issolution = 1;
	for (int k=0;k<child_results.size() && issolution;k++)
	{
		if (!(issolution = !child_results[k]->empty())) //if no direct soln for arg #k
			for (set<BackInferenceTree*>::iterator					m  = children[k].begin();
													!issolution &&	m != children[k].end();
													m++) //find if any childre of #k are solns
				issolution = ((*m)->Status() & SOLUTION);
	}
	if (issolution)
	{
		my_status |= SOLUTION;
		tlog(1, "This child is a SOLUTION.\n");
	}
	else
		tlog(3, "This child is NOT a solution.\n");

void BackInferenceTree::NotifyStatus(unsigned int arg_i, CHAINER_STATUS new_status)
	{
		assert(arg_i < status.size());
tlog(1,"Notified %d/%d\n", arg_i, new_status);
		
		if (new_status&SOLUTION)
			status[arg_i] |= SOLUTION;
		
		CheckIfSolution();
	}

/// Obsolete if pre-bindings are no longer used:

struct SpawnBoundSiblings
{
	Btr<bindingsT> bindings;
	
	SpawnBoundSiblings(Btr<bindingsT> _bindings)
	: bindings(_bindings) { }
	
	bool operator()(BackInferenceTree* t)
	{
		uint count=0;

		if (debugger_control)
			debugger_control = debugger_control;
		t->tlog(3, "Spawning ([%d]) with %s\n", (int)t, (t->rule? (t->rule->name.c_str()) : ""));

		if (t->GetParents().empty() || t->GetParents().begin()->link == t->root)
			return false;

		set<Vertex> bind_key_set;
		transform(bindings->begin(), bindings->end(), inserter(bind_key_set, bind_key_set.begin()), bind(&first<Handle,Handle>, _1));

		foreach(const parent_link<BackInferenceTree>& p, t->GetParents())
			for (uint arg=0;arg < p.link->args.size();arg++)
				if (arg != p.parent_arg_i)
				{
					/// Bindings for each children are not stored by the parent,
					/// nor are the children copied directly so that the bindings could be "filled in".
					/// However, the InferenceNode which spawned this one _does_ have all the necessary
					/// pre-bindings in place, and they were passed on in my 'bindings' member variable!

					if (overlap(p.link->args[arg]->begin(), p.link->args[arg]->end(), bind_key_set))
					{
						p.link->CreateChildren(arg, BBvtree(new BoundVTree(*p.link->args[arg])), bindings, NO_SIBLING_SPAWNING);

						count++;
					}
				}

t->tlog(-1, "Spawned %d new nodes.\n", count);
		
		return true;
	}
};

struct SpawnBoundSiblings2
{
	Btr<bindingsT> bindings;
	
	SpawnBoundSiblings2(Btr<bindingsT> _bindings)
	: bindings(_bindings) { }
	
	bool operator()(BackInferenceTree* t)
	{
		uint count=0;

		if (debugger_control)
			debugger_control = debugger_control;

		t->tlog(0, "Spawning ([%d])\n", (int)t);

		set<Vertex> bind_key_set;
		transform(bindings->begin(), bindings->end(), inserter(bind_key_set, bind_key_set.begin()), bind(&first<Handle,Handle>, _1));

			/// Bindings for each children are not stored by the parent,
			/// nor are the children copied directly so that the bindings could be "filled in".
			/// However, the InferenceNode which spawned this one _does_ have all the necessary
			/// pre-bindings in place, and they were passed on in my 'bindings' member variable!

			/// If any of the arguments overlaps, return false.

			for (uint arg=0;arg < t->args.size();arg++)
			{	
				rawPrint(*t->args[arg], t->args[arg]->begin(), 0);
				rawPrint(*t->GetTarget(), t->GetTarget()->begin(), 0);
				foreach(Vertex& v, bind_key_set)
					printTree(v2h(v),0,0);

				if (overlap(t->args[arg]->begin(), t->args[arg]->end(), bind_key_set))
				{
					t->CreateChildren(arg, BBvtree(new BoundVTree(*t->args[arg])), bindings, NO_SIBLING_SPAWNING);
					count++;
				}
			}
	
		/// If returns true, ApplyDown2 will continue downwards.

		return (count==0);
	}
};

/// Disabled. Possibly redundant.
void BackInferenceTree::spawn(Btr<bindingsT> bindings)
{
//	foreach(parent_link<BackInferenceTree>& p, parents)
		if (bindings) // && p.link!=root)
		{
			/// Pass on pre-bindings because they may contain variable-to-variable mappings,
			/// so the new bindings may only make sense in the light of those
			/// old pre-bindings. (There may be other reaons, too.)
			Btr<bindingsT> passed_bindings(new bindingsT(this->GetPreBindings()));
			try {
				insert_with_consistency_check(*passed_bindings, bindings->begin(), bindings->end());
			} catch(...) { puts("exception in addDirectResult (bindings combination)"); return; }

			/// remove recursion from the bindings here
			//				removeRecursionFromMapSimple<bindingsT::iterator,Handle>(passed_bindings->begin(), passed_bindings->end());

			/// A var may be bound to a link which has more vars in its outgoing set.

			removeRecursionFromHandleHandleMap(*passed_bindings);

			/// TODO: Simplify!
			/// Currently the only automatic way to remove recursion by looking _inside_ that set is as follows:

			/// 1. Convert handle=>handle to handle=>vtree
			bindingsVTreeT bindsWithVTree;
			foreach(hpair hp, *passed_bindings)
				bindsWithVTree[hp.first] = make_vtree(hp.second);

			/// 2. Remove recursion from handle=>vtree mapping

			removeRecursionFromMap<bindingsVTreeT::iterator, vtree::iterator>(bindsWithVTree.begin(), bindsWithVTree.end());

			/// 3. Deconvert handle=>vtree to handle=>handle

			typedef pair<Handle,vtree> phvt;
			foreach(phvt vp, bindsWithVTree)
			{
				(*passed_bindings)[vp.first] = make_real(vp.second);
			}

			//ApplyUp(SpawnBoundSiblings(passed_bindings), 0);
			root->ApplyDown2(SpawnBoundSiblings2(passed_bindings));
		}
//		else
//			tlog(3,"NO PARENT OR BINDINGS!\n");

}

/// No longer needed? MAY NOT BE UP2DATE:
/// Children are also cloned, but results (gained so far) will be shared instead.
BackInferenceTree* BackInferenceTree::Clone() const
	{
		BackInferenceTree* ret = new BackInferenceTree(*this);
		InferenceNodes++; 
		
		vector<set<BackInferenceTree*> > new_children;
		
		for (vector<set<BackInferenceTree*> >::iterator childi = ret->children.begin();
				childi != ret->children.end(); childi++)
		{
//		foreach(set<BackInferenceTree*>& child, ret->children)
			set<BackInferenceTree*> new_c_set;
			
			//for (set<BackInferenceTree*>::iterator bit=childi->begin();bit!=childi->end();bit++)
			foreach(BackInferenceTree* const& bitree, *childi)
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
		void operator()(BackInferenceTree* b)
		{
			cprintf(3,make_subst_buf(bindings).c_str());
						
				meta target = b->GetTarget();

				cprintf(3,"Before bind:");
				rawPrint(*target,target->begin(),3);
			
				b->SetTarget(bind_vtree(*target, bindings));
				
				cprintf(3,"After:");
				rawPrint(*b->GetTarget(),b->GetTarget()->begin(),3);
		}
	};
	
BackInferenceTree* BackInferenceTree::Bind(bindingsT b)
	{
		ApplyDown(target_binder(b));
		return this;		
	}
*/	

/*		BackInferenceTree* new_node = NULL;

		if (new_node) ///If exists already
			new_node->addNewParent(parent_link(this, target_i));
		else
		{
			new_node = new BackInferenceTree(
				root, this, depth+1, target_i, _target, new_rule, rp, rule_args,
				target_chain, bindings, spawning, false);
			BackInferenceTree* existing_node = HasChild(new_node, target_i);
			if (!existing_node)
			{
				new_node->Create();

				tlog(2, "Created new BIT child [%d]\n", (int)new_node);

				if (ObeysPoolPolicy(new_rule, _target))
				{
					root->exec_pool.push_back(new_node);
					exec_pool_sorted = false;
				}

				children[target_i].insert(new_node);
			}
			else
			{ delete new_node; new_node = existing_node; }
		}

		return new_node;*/

/*		BackInferenceTree* ret = HasChild(target_i, new_rule, rule_args, _target, bindings);
		if (!ret)
		{
			/// Disabled. Crashes.
			///BackInferenceTree* new_node = root->FindNode(new_rule, _target, rule_args, bindings);
			BackInferenceTree* new_node = NULL;

			if (new_node) ///If exists already
				new_node->addNewParent(parent_link(this, target_i));
			else
			{
				new_node = new BackInferenceTree(
					root, this, depth+1, target_i, _target, new_rule, rp, rule_args,
					target_chain, bindings, spawning, false);
				new_node->Create();

				tlog(2, "Created new BIT child [%d]\n", (int)new_node);

				if (ObeysPoolPolicy(new_rule, _target))
				{
					root->exec_pool.push_back(new_node);
					exec_pool_sorted = false;
				}
			}

			children[target_i].insert(new_node);

			return new_node;
		}*/

/*struct BINRuleArg
{
	BINRuleArg() : node(0), nr(0) {}
	BINRuleArg(BackInferenceTree* _node, int _nr) : node(_node), nr(_nr) {}
	BackInferenceTree* node;
	bool operator<(const BINRuleArg& rhs) const {
		return node<rhs.node || (node==rhs.node && nr<rhs.nr);
	}
	int nr;
};
*/
#endif

} //namespace reasoning
