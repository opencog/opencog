class ParametrizedProver;

class PreProver
{
	meta target;
	Btr<Rule> rule;
};

class Prover
{
	typedef vector< set<Btr<ParametrizedProver> > > ParametrizedSubProofVector;
	typedef vector< set<PreProver*> > SubPreProofVector;
	StandardizedTargetAtom target;
	Btr<Rule> rule;
	ParametrizedSubProofVector children;

	/// The result set of this node.
	//Btr<set<BoundVertex> > my_results;
	vector<set<BoundVertex> > my_results;

	bool TargetOnPath(StandardizedTargetAtom _target)
	{
		return target == _target || (this != root && parent->TargetOnPath(_target));
	}

	Prover(Btr<Rule> _rule, StandardizedTargetAtom _target)
		: rule(_rule)
	{
		assert(_target.valid());
		target = _target;
	}

	ParametrizedProver Recycle(PreProver* p) const
	{
		ParametrizedProver param_p(p);
		vector<Prover*, indirect_comp<Prover> >::const_iterator old_prover_it = RecyclerPool.find(param_p.prover);
		if (old_prover_it == RecyclerPool.end())
			RecyclerPool.insert(p.prover);
		else
			param_p.prover = *old_prover_it;

		return param_p;
	}

	virtual void expand()=0;

	bool operator==(const Prover& rhs) const
	{
		return this->rule == rhs.rule && this->target == rhs.target;
	}
	bool operator<(const Prover& rhs) const
	{
		if (this->rule < rhs.rule)
			return true;
		if (this->rule > rhs.rule)
			return false;

		if (this->target < rhs.target)
			return true;
		else return false;
	}
};

class ParametrizedProver
{
public:
	bindingsT bindings;
	Prover prover;
};

class Composer : public Proof
{
	SubPreProofVector GetProvers() const
	{
/*
For Composer's each arg template T:
UnificationGenerators and Composers are always created as potential producers of T (restricted by that BIT node's o2i() method).
If T contains variables, LookupGenerator is, too.
If T does not contain variables, then SingletonGenerators are, too. 
Composer.evaluate(latest_result) = try to compute all the results of the Rule of this BIT by using latest_result in conjunction with old results. For each new_result, for each parent: call parent->evaluate(new_result). 
Note: Parents are always Composers. 
*/
	}
	void expand()=0;
	{
		assert(rule->IsGenerator());

		SubPreProofVector pre_provers = GetProvers(target, children);

		children.insert(pre_provers.size(), set<Prover*>());

		foreach(const set<PreProver*>& prover_set, pre_provers)
		{
			set<Prover*> next_arg_provers;

			foreach(PreProver* p, prover_set)
				next_arg_provers.insert(Recycle(p));

			children.push_back(next_arg_provers);
		}
		exec_pool.erase(this);			
	}
};

class Generator : public Proof
{
	void expand()
	{
		BoundVertex next_result = FindNext();
		if (next_result.value)
		{
		}
		else
		{
			exec_pool.erase(this);
			Die();				
		}
	}

	void Notify() {
	///	\todo for each parent: parent->evaluate(with this->latest_result).
	}

	virtual BoundVertex FindNext()=0;
}

class Generator : public Generator
{
	BoundVertex FindNext()
	{
		BoundVertex ret((Handle)0);
		return ret;
	}
};

class Generator : public Generator
{
	BoundVertex FindNext()
	{
		BoundVertex ret((Handle)0);
		return ret;
	}
};

class Generator : public Generator
{
	BoundVertex FindNext()
	{
		BoundVertex ret((Handle)0);
		return ret;
	}
};

/**	@class UnificationGenerator
		RewriteGenerators and CrispUnificationGenerators 
*/
class UnificationGenerator : public Generator
{
	/**
	Try to rewrite the target template. If fails, die. If succeeds with non-empty set of bindings B, spawn B-bound copies of all 
	BIT nodes that own any of the variables bound by B. If succeeds with empty B, then Generator.Notify().
	\todo The target may contain variables - this should be prepared for.
	*/
	virtual BoundVertex FindNext()=0;
};

class CrispUnificationGenerators : public UnificationGenerator
{
	BoundVertex FindNext()
	{
		BoundVertex ret((Handle)0);
		return ret;
	}
};

class RewriteGenerators : public UnificationGenerator
{
	BoundVertex FindNext()
	{
		BoundVertex ret((Handle)0);
		return ret;
	}
};

/** @class LookupGenerator
\todo Generous Policy: If fails, do nothing. (On a later expansion, this node _may_ be able to find something!)
If succeeds with non-empty set of bindings B, spawn B-bound copies of all BIT nodes that own any of the variables bound by B.
If succeeds with empty set of bindings B, abort. (Should never happen - indicates an error.)
(Because this spawns new nodes, it does not Call Generator.Notify(), which will be called by the SingletonGenerators instead.)
*/

class LookupGenerator : public Generator
{
	queue<BoundVertex> results;
	
	/// Try to produce the Handle. If many, produce the next one with the highest tv.confidence among those which have not been produced by this node yet.

	BoundVertex FindNext()
	{
		BoundVertex ret((Handle)0);
		
		if (results.empty())
		{
			Btr<set<BoundVertex> > result_set = LookupRule().attemptDirectProduction(target);'
			if (!result_set->empty())
				copy(result_set->begin(), result_set->end(), inserter(results.begin()));
		}
		if (!results.empty())
		{
			ret = results.front();
			results.pop();
		}

		if (ret)
		{
			assert(!ret.bindings.empty());
			/// \todo Spawn B-bound copies of all BIT nodes that own any of the variables bound by B.
		}

		return ret;
	}
};


/**	@class SingletonGenerator
		\brief a Generator which can only produce 1 atom. These are HypLinkGenerator, UniqueLookupGenerator and SkolemLinkGenerator. 
*/
class SingletonGenerator : public Generator
{
	virtual BoundVertex FindNext()=0;
};

/** @class UniqueLookupGenerator A LookupGenerator which takes as its argument a template without variables. (TODO: SkolemLinkGenerator has not been tested for a long time.) 
Because variable count is always less in the targets of SingletonGenerators, they are evaluated before other Generators. 
*/
class UniqueLookupGenerator : public SingletonGenerator
{
	/// Try to produce the Handle. If many, produce the next one with the highest tv.confidence among those which have not been produced by this node yet.

	BoundVertex FindNext()
	{
		assert(!HasFWVars(target));

		return nm->getAtom(target);
	}
};

class HypLinkGenerator : public SingletonGenerator
{
	BoundVertex FindNext()
	{
		assert(!HasFWVars(target));

		return HypothesisRule().attemptDirectProduction(target);
	}
};

class SkolemLinkGenerator : public SingletonGenerator
{
	BoundVertex FindNext()
	{
		assert(!HasFWVars(target));

		return ScholemRule().attemptDirectProduction(target);
	}
};


/**

Generator: last best result proportional to fitness.

Expansion
     Identify tree node by (target, Rule) pair
       Use CrispU, not StrictCrispU
         InferenceLoop check back
       Each ForAll formula & "theorem" will appear as a new CrispUnificationRule and TheoremRule, respectively
       Make CrispUnificationRule use variable constraints in its return value and in all respects shape it after StrictCrispUnificationRule
       Make TheoremRule use variable constraints in its return value, as StrictCrispUnificationRule does
	       CrispU and TheoremRule are direct producers => IsComputable()=false
  Immediate evaluation: once a new result has been generated, propagate it upwards with Compute()'s as high as you can, to make it available for Lookups.
 LookupIterator which gets the next-highest-confidence result at each expansion step

LookupRule and HypothesisRule are no longer used. 
All ForAll(Implies(And(...), $x))) < tv0.confidence>=0.989 > are assumed binary and converted to RewriteGenerators (they rewrite atom templates to equivalent atom templates). All other ForAll formulas are converted to CrispUnificationGenerators (which apply CrispUnificationRule). 
StrictCrispUnificationRule which requires proving subtrees of the target atom is no longer used.
The simpler CrispUnificationRule is used instead.
This allows for check for circular proofs. (Yes, they cannot be prohibited if using StrictCrispUnificationRule. After that Rule was created, the circularity check was disabled, but now I'm re-enabling it...)
(But CrispUnificationRule now becomes a Generator.)

*/