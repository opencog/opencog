#ifndef RULES_H
#define RULES_H

#include "Rule.h"
#include <SimpleTruthValue.h>
#include "iAtomTableWrapper.h"
#include "CoreWrapper.h"
#include "Ptlatom.h"
#include "PTLFormulas.h"
#include "NMPrinter.h"

#include <Atom.h>
#include <ClassServer.h>
#include <CogServer.h>
#include <Link.h>
#include <Node.h>
#include <TLB.h>

#ifndef WIN32 
   float max(float a, float b);
#endif

#define USE_ALL_AVAILABLE_INFORMATION_FOR_AND_RULE_COMPUTATION 1
#define MAX_ARITY_FOR_PERMUTATION 5


		/** haxx:: \todo Temporarily disabled!
			This check does not hold if one of the args
			has been executed but the other one has not.
			Execution here means that Eval(!now) becomes Eval(35353).
			! is a hacky shorthand for grounded predicates for now.
			
			The real solution will be tointroduce an equality check which considers
			the unexecuted and executed forms equal.
		*/

#define CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE 0

#define Abs(a) ( ((a)>0) ? (a) : (-a) )

const bool RuleResultFreshness = true;

#define NO_DIRECT_PRODUCTION Btr<set<BoundVertex > > attemptDirectProduction(meta outh) { return Btr<set<BoundVertex> >(); }

using namespace opencog;

namespace reasoning
{
	
Handle AND2ORLink(Handle& andL, Type _ANDLinkType, Type _OR_LINK);
Handle OR2ANDLink(Handle& andL);
Handle AND2ORLink(Handle& andL);
Handle Exist2ForAllLink(Handle& exL);
pair<Handle,Handle> Equi2ImpLink(Handle&);
#define LINKTYPE_ASSERT(__cLink, __cLinkType) assert(inheritsType(CogServer::getAtomSpace()->getType(__cLink), __cLinkType))

Rule::setOfMPs makeSingletonSet(Btr<Rule::MPs> mp);
Vertex CreateVar(iAtomTableWrapper* atw, std::string varname);
Vertex CreateVar(iAtomTableWrapper* atw);

/*	Must define, for each rule:
	Handle compute(Handle* h,const int n, Handle CX = NULL) const
	MPs o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
	atom i2oType(Handle* h, const int n) const

	And inputFilter must be initialized in the constructor!
*/

BBvtree atomWithNewType(const tree<Vertex>& v, Type T);
BBvtree atomWithNewType(Handle h, Type T);
BBvtree atomWithNewType(const Vertex& v, Type T);
	
Rule::setOfMPs makeSingletonSet(Btr<Rule::MPs> mp);

template<typename FormulaType>
class GenericRule : public Rule
{
protected:
	mutable FormulaType f;

public:
	virtual set<MPs> o2iMetaExtra(meta outh, bool& overrideInputFilter) const=0;
	virtual meta i2oType						(const vector<Vertex>& h) const=0;

	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const=0;
	
	/// Always computable
	GenericRule(iAtomTableWrapper *_destTable, bool _FreeInputArity, std::string _name = "")
	: Rule(_destTable, _FreeInputArity, true, _name) {	}
		
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		const int n = (const int)premiseArray.size();
		printf("<Generic rule args>");
		for (int j=0;j<n;j++)
		{
			const Handle *ph = boost::get<Handle>(&premiseArray[j]);
			printf("[%lu]\n", (ph?*ph:0));
		}
			//printTree(premiseArray[j],0,3);
		printf("</Generic rule args>");

		assert(validate(premiseArray));
printf("formatTVarray...\n");
		int TVN = f.TVN;

		TruthValue** tvs = formatTVarray(premiseArray, &TVN);
printf("formatTVarray OK\n");
		if (!tvs)
		{
			printf("Warning only: GenericRule: TV array formatting failure.");
			return Vertex((Handle)NULL);
		}
		printf("Computing TV\n");

		TruthValue* retTV = f.compute(tvs, TVN);
		printf("TV computation ok");

		delete[] tvs;
		printf("Res freed.");

		/// i2otype gives the atom skeleton (tree) w/o TV. addLink attaches it to Core with TV

		Handle ret = destTable->addAtom(*i2oType(premiseArray),
			*retTV,
			true);	
//			false);
		
        delete retTV;
        
		assert(ret);
		
		printf("Atom added.");

//		printTree(ret,0,3);

		return Vertex(ret);
	}
	NO_DIRECT_PRODUCTION;
};

class NotEvaluatorRule : public GenericRule<NotFormula>
{
protected:
	TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new TruthValue*[2];

		const int N = (int)premiseArray.size();
		assert(N==1);

		//std::vector<Handle> real = premiseArray[0];
#if 0
// Welter's comment: this change is waiting for Ari's aproval 
		tvs[0] = (TruthValue*) &(TruthValue::TRIVIAL_TV());
#else
		tvs[0] = new SimpleTruthValue(0,0); //nm->getTV(premiseArray[0]);
		// TODO: create the TrivialTV to use here
#endif
		tvs[1] = (TruthValue*) &(CogServer::getAtomSpace()->getTV(boost::get<Handle>(premiseArray[0])));
		return tvs;
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

public:
	NotEvaluatorRule(iAtomTableWrapper *_destTable);
	meta i2oType(const vector<Vertex>& h) const;

	bool validate2				(MPs& args) const { return true; }
	NO_DIRECT_PRODUCTION;
};

/** @class LookupRule
	
*/

class LookupRule : public Rule
{
protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		return premiseArray[0];
	}
public:
	bool validate2				(MPs& args) const { return true; }
	LookupRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable, false, false, "Lookup")
	{
		//inputFilter.push_back(new atom(result));
	}
	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};

class HypothesisRule : public Rule
{
protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		return premiseArray[0];
	}
public:
	bool validate2				(MPs& args) const { return true; }
	HypothesisRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable, false, false, "Hypothesis")
	{
		//inputFilter.push_back(new atom(result));
	}
	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};
/*
class Inh2EvalRule : public GenericRule<TautologyFormula>
{
protected:
	mutable std::vector<Type> ti;

public:
	bool validate2(Rule::MPs& args) const { return true; }
	Inh2EvalRule(iAtomTableWrapper *_destTable)
	: GenericRule<TautologyFormula>(_destTable,false,"")
	{
		GenericRule<FormulaType>::name = "Inh2Eval";
		GenericRule<FormulaType>::inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)INHERITANCE_LINK,
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType((Type)boost::get<Handle>(*outh->begin()), EVALUATION_LINK))
			return Rule::setOfMPs();

		Rule::MPs ret;

		BBvtree ret_m(new BoundVTree(*outh));
		*ret_m->begin() = Vertex((Handle)SRC_LINK);
		ret.push_back(ret_m);
		
		overrideInputFilter = true;

		return makeSingletonSet(ret);
	}

	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new SimpleTruthValue*[1];

		assert(premiseArray.size()==1);

		tvs[0] = &(nm->getTV(boost::get<Handle>(premiseArray[0])));

		return tvs;
	}

	virtual meta i2oType(const vector<Vertex>& h) const
	{
		assert(1==h.size());

		meta ret = atomWithNewType(h[0], DEST_LINK);
cprintf(3,"i2otype() outputs: ");
#if 0
rawPrint(*ret, ret->begin(), 3);
#else
  NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME, 
                      NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION, 
                      NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE, 
                      3);
    printer.print(ret->begin());
#endif

		return ret;
	}
	NO_DIRECT_PRODUCTION;
};
*/

extern std::map<int, string> type2name;

Rule::setOfMPs makeSingletonSet(Rule::MPs& mp);
template<typename FormulaType, Type SRC_LINK, Type DEST_LINK>
class Link2LinkRule : public GenericRule<FormulaType>
{
protected:
//	mutable std::vector<Type> ti;

public:
	bool validate2				(Rule::MPs& args) const { return true; }
	Link2LinkRule(iAtomTableWrapper *_destTable)
	: GenericRule<FormulaType>(_destTable,false,"")
	{
		GenericRule<FormulaType>::name = "Link2Link(" + type2name[SRC_LINK] + "=>" + type2name[DEST_LINK] +")";
		GenericRule<FormulaType>::inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)SRC_LINK,
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType((Type)(int)boost::get<Handle>(*outh->begin()), DEST_LINK))
			return Rule::setOfMPs();

		Rule::MPs ret;

		BBvtree ret_m(new BoundVTree(*outh));
		*ret_m->begin() = Vertex((Handle)SRC_LINK);
		ret.push_back(ret_m);
		
		overrideInputFilter = true;

		return makeSingletonSet(ret);
	}

	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[1];

		assert(premiseArray.size()==1);

		tvs[0] = (TruthValue*) &(CogServer::getAtomSpace()->getTV(boost::get<Handle>(premiseArray[0])));

		return tvs;
	}

	virtual meta i2oType(const vector<Vertex>& h) const
	{
		assert(1==h.size());

		meta ret = atomWithNewType(h[0], DEST_LINK);
cprintf(3,"i2otype() outputs: ");
#if 0
rawPrint(*ret, ret->begin(), 3);
#else
	NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME, 
                      NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION, 
                      NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE,
					  3);
    printer.print(ret->begin());
#endif

		return ret;
	}
	NO_DIRECT_PRODUCTION;
};

typedef  Link2LinkRule<Mem2InhFormula, MEMBER_LINK, EXTENSIONAL_INHERITANCE_LINK> Mem2InhRule;
typedef  Link2LinkRule<Int2ExtFormula, IMPLICATION_LINK, MIXED_IMPLICATION_LINK>				IntImp2ExtRule;
//typedef  Link2LinkRule<Int2ExtFormula, INHERITANCE_LINK, MIXED_INHERITANCE_LINK>				IntInh2ExtRule;
typedef  Link2LinkRule<Int2ExtFormula, INHERITANCE_LINK, EXTENSIONAL_INHERITANCE_LINK>				IntInh2ExtRule;
typedef  Link2LinkRule<Ext2IntFormula, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK> ExtImp2IntRule;
typedef  Link2LinkRule<Ext2IntFormula, EXTENSIONAL_INHERITANCE_LINK, INHERITANCE_LINK> ExtInh2IntRule;
//typedef  Link2LinkRule<Ext2IntFormula, EXTENSIONAL_INHERITANCE_LINK, MIXED_INHERITANCE_LINK> ExtInh2IntRule;
typedef  Link2LinkRule<Inh2ImpFormula, INHERITANCE_LINK, IMPLICATION_LINK>					Inh2ImpRule;
typedef  Link2LinkRule<Imp2InhFormula, IMPLICATION_LINK, INHERITANCE_LINK>					Imp2InhRule;
typedef  Link2LinkRule<Mem2EvalFormula, MEMBER_LINK, EVALUATION_LINK> Mem2EvalRule;
//typedef  Link2LinkRule<Eval2InhFormula, EVALUATION_LINK, INHERITANCE_LINK> Eval2InhRule;
//typedef  Link2LinkRule<Formula, _LINK, _LINK> Rule;

template<typename FormulaType, Type OUTPUT_LINK_TYPE>
class QuantifierRule : public Rule
{
	Handle domain;
	FormulaType f;
public:
	bool validate2				(MPs& args) const { return true; }

	QuantifierRule(iAtomTableWrapper *_destTable, const Handle& _domain)
	: Rule(_destTable, false, true, "QuantifierRule"),
	domain(_domain) {
		inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)((OUTPUT_LINK_TYPE==FORALL_LINK) ? EXIST_LINK : FORALL_LINK),
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs(); //No support (yet)
	}

	//Domain should be inferred instead from the premis ConceptNodes!!!

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		const int n = (int)premiseArray.size();
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[n];
		int i;
		for (i = 0; i < n; i++)
			tvs[i] = (TruthValue*) &(CogServer::getAtomSpace()->getTV(boost::get<Handle>(premiseArray[i])));

		TruthValue* retTV = f.compute(tvs, n);	

		delete[] tvs;

//haxx::
		Handle ret = destTable->addLink(OUTPUT_LINK_TYPE, HandleSeq(),
				*retTV,
				RuleResultFreshness);	
//				false);

        delete retTV;

		return Vertex(ret);
	}
	
	NO_DIRECT_PRODUCTION;
};

typedef QuantifierRule<FORALLFormula, FORALL_LINK> FORALLRule;
typedef QuantifierRule<EXISTFormula, EXIST_LINK> ExistRule;
typedef QuantifierRule<PredicateTVFormula, VARIABLE_SCOPE_LINK> PLNPredicateRule;

template<Type InclusionLink>
class InversionRule : public GenericRule<InversionFormula>
{
protected:
//	mutable std::vector<Type> ti;

	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[3];

		assert(premiseArray.size()==1);
		AtomSpace *nm = CogServer::getAtomSpace();
		std::vector<Handle> nodes = nm->getOutgoing(boost::get<Handle>(premiseArray[0]));

		tvs[0] = (TruthValue*) &(nm->getTV(boost::get<Handle>(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(nodes[0]));
		tvs[2] = (TruthValue*) &(nm->getTV(nodes[1]));

		return tvs;
	}
	std::vector<BoundVertex> r;

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType((Type)(int)boost::get<Handle>(*outh->begin()), InclusionLink))
			return Rule::setOfMPs();
		
		Rule::MPs ret;
		ret.push_back(atomWithNewType(*outh,InclusionLink));
		tree<Vertex>::sibling_iterator top = ret[0]->begin();
		tree<Vertex>::sibling_iterator right = ret[0]->begin(top);
		tree<Vertex>::sibling_iterator left = right++;
		outh->swap(left, right);

		overrideInputFilter = true;

		return makeSingletonSet(ret);
	}

public:
	InversionRule(iAtomTableWrapper *_destTable)
	: GenericRule<InversionFormula> (_destTable, false, "InversionRule")
	{
		inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)InclusionLink,
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));		
	}
	bool validate2				(MPs& args) const { return true; }

	virtual meta i2oType(const vector<Vertex>& h) const
	{
		assert(1==h.size());
		Handle h0 = boost::get<Handle>(h[0]);
/*cprintf(1,"INV OLD ATOM:\n");
printTree(boost::get<Handle>(h[0]),0,1);
cprintf(1,"INV New order:\n");
printTree(child(boost::get<Handle>(h[0]),1),0,1);
printTree(child(boost::get<Handle>(h[0]),0),0,1);*/
		return	meta(new tree<Vertex>(mva((Handle)CogServer::getAtomSpace()->getType(h0),
						mva(child(h0,1)),
						mva(child(h0,0))
				)));
	}
	NO_DIRECT_PRODUCTION;
};

class ANDSubstRule : public Rule
{
protected:
	ANDSubstRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"ANDSubstRule")
	{}
public:
	bool validate2				(MPs& args) const { return true; }

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	NO_DIRECT_PRODUCTION;
};

class ImplicationRedundantExpansionRule : public Rule
{
protected:
	ImplicationRedundantExpansionRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"ImplicationRedundantExpansionRule")
	{}
public:
	bool validate2				(MPs& args) const { return true; }

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	NO_DIRECT_PRODUCTION;
};

class ArityFreeANDRule : public Rule
{
protected:
	ArityFreeANDRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"")
	{}
	SymmetricANDFormula fN;
	AsymmetricANDFormula f2;
public:
	bool validate2				(MPs& args) const { return true; }

	bool asymmetric(Handle* A, Handle* B) const;
	//Handle compute(Handle A, Handle B, Handle CX = NULL)  const; //std::vector<Handle> vh)
	BoundVertex computeSymmetric(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	void DistinguishNodes(const vector<Vertex>& premiseArray, set<Handle>& ANDlinks, set<Handle>& nodes) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const=0;
};

class ANDRule : public ArityFreeANDRule
{
public:
	ANDRule(iAtomTableWrapper *_destTable)
	: ArityFreeANDRule(_destTable)
	{
		name = "AND Evaluator Rule";
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	//Btr<set<BoundVertex > > attemptDirectProduction(meta outh);

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	
	/// Direct production was used here before. TODO: Check whether this should be resumed!
	NO_DIRECT_PRODUCTION;
};

Handle UnorderedCcompute(iAtomTableWrapper *destTable,
					Type linkT, const ArityFreeFormula<TruthValue,
			       TruthValue*>& fN, Handle* premiseArray, const int n, Handle CX=NULL);

/** @class ANDPartitionRule
	Partitions argument into smaller ANDLinks
*/

class ANDPartitionRule : public Rule
{
	SymmetricANDFormula fN;

public:
	ANDPartitionRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"ANDPartitionRule")
	{ }
	
	bool validate2				(MPs& args) const { return true; }
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	
	NO_DIRECT_PRODUCTION;
};

template<int N>
class SimpleANDRule : public ArityFreeANDRule
{
public:
	SimpleANDRule(iAtomTableWrapper *_destTable)
	: ArityFreeANDRule(_destTable)
	{
		name = "Simple AND Rule";
		for (int i = 0; i < N; i++)
			inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))
			));
	}
	bool validate2				(MPs& args) const { return true; }

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType(CogServer::getAtomSpace()->	getType(boost::get<Handle>(*outh->begin())), AND_LINK)
			|| outh->begin().number_of_children() != N)
			return Rule::setOfMPs();
		
		tree<Vertex>::iterator top = outh->begin();
		MPs ret;//(outh->begin(top), outh->end(top));

		//for (int i = 0; i < N; i++)
		
		for (tree<Vertex>::sibling_iterator i =outh->begin(top);
									i!=outh->end  (top); i++)
			ret.push_back(BBvtree(new BoundVTree(i)));

		overrideInputFilter = true;
		
		return makeSingletonSet(ret);
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		Handle *hs = new Handle[premiseArray.size()];
		transform(premiseArray.begin(), premiseArray.end(), &hs[0], GetHandle()); //mem_fun(
		const int n = (int)premiseArray.size();
		vector<Handle> dummy_outgoing;
		dummy_outgoing.push_back(v2h(premiseArray[0]));

		//printf("ANDRUle: [%d: %d] %s =>\n", N, v2h(premiseArray[0]), getTruthValue(v2h(premiseArray[0]))->toString().c_str());

/*		puts("ANDRule got args:");
		foreach(const Vertex& v, premiseArray)
			printTree(v2h(v),0,-3);
	*/
		//currentDebugLevel = 3;
		Handle ret = ((N>1)
			      ? UnorderedCcompute(destTable, AND_LINK, fN, hs,n,CX)
			      : destTable->addLink(AND_LINK, dummy_outgoing, getTruthValue(v2h(premiseArray[0])),
				RuleResultFreshness));
		    delete[] hs;

		      //		printf("=> ANDRUle: %s:\n", ret, getTruthValue(ret)->toString().c_str());
		      //		printTree(ret,0,-3);
		      //currentDebugLevel = -3;

		return Vertex(ret);
	}
	
	NO_DIRECT_PRODUCTION;
};

class ORRule : public GenericRule<ORFormula>
{
public:
	ORRule(iAtomTableWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	
	NO_DIRECT_PRODUCTION;
	
	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const;
public:
	bool validate2				(MPs& args) const { return true; }

	virtual meta i2oType(const vector<Vertex>& h) const;
};

/** @class ORPartitionRule
	Partitions argument into, like, OR(A, OR(B, OR(C, D)))
*/

class ORPartitionRule : public Rule
{
	ORRule* regularOR;
public:

	ORPartitionRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable, true, true, "ORPartitionRule") 
	{
		regularOR = new ORRule(_destTable);
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	bool validate2				(MPs& args) const { return true; }
	
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	
	NO_DIRECT_PRODUCTION;
};

/// (x->A) => A.

class ImplicationBreakdownRule : public Rule
{
public:
	NO_DIRECT_PRODUCTION;

	ImplicationBreakdownRule(iAtomTableWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2				(MPs& args) const { return true; }
};

/// (x, x->A) => A.

class StrictImplicationBreakdownRule : public Rule
{
public:
	NO_DIRECT_PRODUCTION;

	StrictImplicationBreakdownRule(iAtomTableWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2				(MPs& args) const { return true; }

};

class ImplicationTailExpansionRule : public Rule
{
public:
	NO_DIRECT_PRODUCTION;

	ImplicationTailExpansionRule(iAtomTableWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;

	bool validate2				(MPs& args) const { return true; }
};

class CrispTheoremRule : public Rule
{
public:
	static map<vtree, vector<vtree> ,less_vtree> thms;
	NO_DIRECT_PRODUCTION;

	CrispTheoremRule(iAtomTableWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;

	bool validate2				(MPs& args) const { return true; }
};

class BaseCrispUnificationRule : public Rule
{
protected:
public:

	BaseCrispUnificationRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"CrispUnificationRule")
	{

		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)HYPOTHETICAL_LINK))
			));
	}

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;

	NO_DIRECT_PRODUCTION;
};

class CustomCrispUnificationRuleComposer : public BaseCrispUnificationRule
{
	Handle ForallLink;
public:

	CustomCrispUnificationRuleComposer(Handle _ForallLink, iAtomTableWrapper *_destTable)
	: BaseCrispUnificationRule(_destTable), ForallLink(_ForallLink) {}

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

//	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2(MPs& args) const { return true; }

	NO_DIRECT_PRODUCTION;
};

class CustomCrispUnificationRule : public Rule
{
protected:
	Handle ForallLink;
public:

	CustomCrispUnificationRule(Handle _ForallLink, iAtomTableWrapper *_destTable)
	: Rule(_destTable,false,false,"CrispUnificationRule"), ForallLink(_ForallLink)
	{
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(0);
		return Vertex((Handle)NULL);
	}

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{ return Rule::setOfMPs(); }

	bool validate2(MPs& args) const { return true; }

	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};



class CrispUnificationRule : public BaseCrispUnificationRule
{
public:

	CrispUnificationRule(iAtomTableWrapper *_destTable)
	: BaseCrispUnificationRule(_destTable) {}
		
	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

//	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2				(MPs& args) const { return true; }

	NO_DIRECT_PRODUCTION;
};

/// Requires that all subtrees are separately produced; hence requires HypothesisRule.

class StrictCrispUnificationRule : public BaseCrispUnificationRule
{
public:
	StrictCrispUnificationRule(iAtomTableWrapper *_destTable)
	: BaseCrispUnificationRule(_destTable) {}
	bool validate2				(MPs& args) const { return true; }		
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
};

//#if 0

template<typename DeductionFormula, Type InclusionLink> //=IMPLICATION_LINK>
class DeductionRule : public GenericRule<DeductionFormula>
{
    //DeductionFormula f;

	meta i2oType(const vector<Vertex>& h) const
	{
		assert(h.size()==2);
		AtomSpace *nm = CogServer::getAtomSpace();	
		assert(nm->getArity(v2h(h[0]))==2);
		assert(nm->getArity(v2h(h[1]))==2);
		assert(nm->getOutgoing(v2h(h[0]),0));
		assert(nm->getOutgoing(v2h(h[1]),1));
	
		return meta(new tree<Vertex>(mva(InclusionLink, 
						vtree(Vertex(nm->getOutgoing(v2h(h[0]),0))),
						vtree(Vertex(nm->getOutgoing(v2h(h[1]),1)))
				)));
	}
	bool validate2				(Rule::MPs& args) const
	{
		return (args.size() == 2 && !(*args[0] == *args[1]));
	}
	TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[5];

		assert(premiseArray.size()==2);

		std::vector<Handle> nodesAB = CogServer::getAtomSpace()->getOutgoing(v2h(premiseArray[0]));
		std::vector<Handle> nodesBC = CogServer::getAtomSpace()->getOutgoing(v2h(premiseArray[1]));

		//assert(equal(nodesAB[1], nodesBC[0]));

		if (CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE && !equal(nodesAB[1], nodesBC[0]))
		{
			cprintf(0, "Invalid deduction arguments:\n");
#if 0            
			printTree(v2h(premiseArray[0]),0,0);
			printTree(v2h(premiseArray[1]),0,0);
#else 
            NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
            printer.print(v2h(premiseArray[0]));
            printer.print(v2h(premiseArray[1]));
#endif             
			getc(stdin);getc(stdin);
			return NULL;
		}

		AtomSpace *nm = CogServer::getAtomSpace();
		tvs[0] = (TruthValue*) &(nm->getTV(v2h(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(v2h(premiseArray[1])));
		tvs[2] = (TruthValue*) &(nm->getTV(nodesAB[0]));
		tvs[3] = (TruthValue*) &(nm->getTV(nodesAB[1])); //== nodesBC[0]);
		tvs[4] = (TruthValue*) &(nm->getTV(nodesBC[1]));

		return tvs;
	}

public:

	DeductionRule(iAtomTableWrapper *_destTable)
	: GenericRule<DeductionFormula>(_destTable,false,"DeductionRule")
	{
		/// TODO: should use real variable for the other input.
		
		GenericRule<DeductionFormula>::inputFilter.push_back(meta(
				new tree<Vertex>(
				mva((Handle)InclusionLink,
					mva((Handle)ATOM),
					mva((Handle)ATOM)))
			));		
		GenericRule<DeductionFormula>::inputFilter.push_back(meta(
				new tree<Vertex>(
				mva((Handle)InclusionLink,
					mva((Handle)ATOM),
					mva((Handle)ATOM)))
			));		
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (	!inheritsType((Type)(int)v2h(*outh->begin()), InclusionLink))
			return Rule::setOfMPs();

		std::string varname = ("$"+GetRandomString(10));

		Rule::MPs ret;

		tree<Vertex>::iterator top0 = outh->begin();
		
		Vertex var = CreateVar(GenericRule<DeductionFormula>::destTable);
		
		ret.push_back(BBvtree(new BoundVTree(mva(Vertex((Handle)InclusionLink),
			tree<Vertex>(outh->begin(top0)),
			mva(var)))));
		ret.push_back(BBvtree(new BoundVTree(mva(Vertex((Handle)InclusionLink),
			mva(var),
			tree<Vertex>(outh->last_child(top0))		
			))));

		overrideInputFilter = true;

		return makeSingletonSet(ret);
	}

	NO_DIRECT_PRODUCTION;
};

/** @class ScholemFunctionProductionRule
	
*/

class ScholemFunctionProductionRule : public Rule
{
protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(0);

		return Vertex((Handle)NULL);
	}
public:
	ScholemFunctionProductionRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,false,false,"ScholemFunctionProductionRule")
	{
		//inputFilter.push_back(new atom(result));
	}
	bool validate2				(MPs& args) const { return true; }

	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};

#if 0
/// Left side stays constant, RHS is substed
class SimSubstRule1 : public GenericRule<InhSubstFormula>
{
public:
	SimSubstRule1(iAtomTableWrapper *_destTable)
	: GenericRule<InhSubstFormula>(_destTable, false, "SimSubstRule")
	{
		inputFilter.push_back(meta(new tree<Vertex>(mva((Handle)INHERITANCE_LINK,
			mva((Handle)ATOM),
			mva((Handle)ATOM)))));
		inputFilter.push_back(meta(new tree<Vertex>(mva((Handle)ATOM))));
	}
	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	TruthValue** formatTVarray	(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new TruthValue*[1];

		const int N = (int)premiseArray.size();
		assert(N==2);

		tvs[0] = (TruthValue*) &(nm->getTV(v2h(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(v2h(premiseArray[1])));

		return tvs;
	}

	bool validate2				(MPs& args) const { return true; }

	virtual meta i2oType(const vector<Vertex>& h) const;
};

#endif

/*
/// RHS stays constant, LHS is substed
class SimSubstRule2 : public GenericRule<InhSubstFormula>
{
public:
	SimSubstRule2()
	{
		name = "SimSubstRule";
//		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(INHERITANCE_LINK,0))));
	}
	setOfMPs o2iMetaExtra(meta, bool& overrideInputFilter) const;

	TruthValue** formatTVarray	(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new SimpleTruthValue*[1];

		assert(N==1);

		tvs[0] = (TruthValue*) &(nm->getTV(v2h(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(v2h(premiseArray[1])));

		return tvs;
	}

	meta i2oType(const vector<Vertex>& h) const
	{
		const int n = h.size();
		Handle h0 = v2h(h[0]);
		Handle h1 = v2h(h[1]);
		
		assert(2==n);
		assert(getType(h1) == INHERITANCE_LINK);

		// ( any, Inh(a,b) )

		atom ret(h0);

		//assert(ret.hs[0].real == nm->getOutgoing(h[1])[0]);

		ret.hs[0] = (TruthValue*) atom(nm->getOutgoing(h1)[0]);
		
		vector<Handle> hs = nm->getOutgoing(h0);
		ret.substitute(atom(hs[1]), atom(hs[0])); //  parent for child
		
		return meta(new Tree<Vertex>(ret.maketree()));		
		
		meta ret(new Tree<Vertex>(mva(getType(v2h(h[0])),
			mva(nm->getOutgoing(v2h(h[1]))[0]),
			mva(nm->getOutgoing(v2h(h[0]))[1])));

		return ret;
	}
};
*/
template<int N>
class ANDBreakdownRule : public Rule
{
protected:
	ANDBreakdownFormula f;

public:
	ANDBreakdownRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"")
	{
		name = "ANDBreakdownRule/" + i2str(N);
		
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)AND_LINK,
					mva((Handle)ATOM),
					mva((Handle)ATOM)))));
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)HYPOTHETICAL_LINK,
					mva((Handle)ATOM),
					mva((Handle)ATOM)))));
	}

	bool validate2				(MPs& args) const { return true; }

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		/// And parameters aren't ordered. Therefore, the order in which we feed them
		/// here is irrelavent. But we need a hypothetical parameter that will later reming
		/// which kind of atom we need to produce.

		MPs ret;
		BBvtree andlink(new BoundVTree);

		andlink->set_head(Vertex((Handle)AND_LINK));
		
		andlink->append_child(andlink->begin(), outh->begin());
				
		for (int i = 1; i < N; i++)
			andlink->append_child(	andlink->begin(),
									BoundVTree(CreateVar(destTable)).begin());

		ret.push_back(andlink);
		ret.push_back(BBvtree(new BoundVTree(mva((Handle)HYPOTHETICAL_LINK,*outh))));
		
		overrideInputFilter = true;	
	
		return makeSingletonSet(ret);
	}

	NO_DIRECT_PRODUCTION;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
  {
	AtomSpace *nm = CogServer::getAtomSpace();
	std::vector<Handle> hs = nm->getOutgoing(v2h(premiseArray[0]));

	assert(premiseArray.size() == 2);
	assert(hs.size() == N);
	assert(nm->getArity(v2h(premiseArray[1])) == 1);	  	  

	atom topological_model(nm->getOutgoing(v2h(premiseArray[1]))[0]);

	for (uint i = 0; i < hs.size(); i++)
	  if (atom(hs[i]) == topological_model)
			return BoundVertex(hs[i]);
		
	/*LOG(0,"Topo model was:");
	printAtomTree(topological_model,0,0);
	LOG(0,"hs:");
	for (uint i = 0; i < hs.size(); i++)
		printTree(hs[i],0,0);*/
	
	//LOG(0, "ANDBREAKDOWN: NO TOPOLOGICAL MODEL FOUND!");
	assert(0);

	return Vertex((Handle)NULL);
		
/*	std::vector<Handle> hs = nm->getOutgoing(premiseArray[0]);

	assert(premiseArray.size() == 2);
	assert(hs.size() == N);
	assert(nm->getArity(premiseArray[1]) == 1);
	  
	tree<Vertex>::iterator top0 = premiseArray[0].begin();

	tree<Vertex> topological_model(premiseArray[1])[0]
	tree<Vertex>::iterator topological_model = (premiseArray[1])[0];

	for (int i = 0; i < hs.size(); i++)
	for (tree<Vertex>::sibling_iterator i = premiseArray[0].begin();
			i != premiseArray[0].end(); i++)		
		if (*i == topological_model)
			return *i;

	//LOG(0, "ANDBREAKDOWN: NO TOPOLOGICAL MODEL FOUND!");
	assert(0);*/
  }
};

/*
	The basic problem of an individual is distinguishing between _who he "really" is_
	and the "identity" (mental image and related ingrained patterns of thought) that
	society had imposed upon him. The most important fact about the distinction is,
	however, that it is equally fictionary as her individual identity and collective identity.
	In pragmatics, the relevant point is the utility of the distinction, which in this case is 		significant, given the goal structure of any individualist.
*/


/**
\todo The following Rules have not been updated for a long time. Many of them will still be useful and necessary in the future.
*/

#if 0

/** TODO: UPDATE TO Vertex<Handle>
class SubsetEvalRule
{
	Handle domain;
	SubsetEvalFormula f;

	protected:

		MPs inputFilter;
		iAtomTableWrapper *destTable;
public:
	virtual ~SubsetEvalRule() {}
	SubsetEvalRule(iAtomTableWrapper *_destTable);

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return setOfMPs(); //No support (yet)
	}

	meta i2oType(const vector<Vertex>& h) const
	{
		assert(n==1);
		return atomWithNewType(h[0], SUBSET_LINK);
	}
	
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
};


class PrintRule : public Rule
{
public:
	PrintRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,false,true,"PrintRule")
	{
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs();
	}

	NO_DIRECT_PRODUCTION;

  BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
  {
  	for (int i = 0; i < premiseArray.size(); i++)
  		printTree(premiseArray[i], 0, 0);
  }
};*/

class AND2ORRule : public Rule
{
	AND2ORRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(AndLink))));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(new atom(neBoundVertexWithNewType(outh, AndLink))));
		return makeSingletonSet(ret);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(1==n);

		return atomWithNewType(h[0], OR_LINK);
	}
	virtual bool valid(Handle* h, const int n) const
	{
		return isSubType(h[0], AND_LINK);
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(n==1);

		return AND2ORLink(premiseArray[0]);
	}
};

class OR2ANDRule : public Rule
{
	OR2ANDRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(OrLink))));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, OrLink)));
		return makeSingletonSet(ret);
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(1==n);

		return atomWithNewType(h[0], AND_LINK);
	}
	virtual bool valid(Handle* h, const int n) const
	{
		return isSubType( h[0], OR_LINK);
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(n==1);

		return OR2ANDLink(premiseArray[0]);
	}
};
/*
class Exist2ForAllRule : public Rule
{
	Exist2ForAllRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable)
	{
		inputFilter.push_back(new atom(__INSTANCEOF_N, ExistLink));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		MPs ret;
		ret.insert(new atom(NOTLink,
			neBoundVertexWithNewType(outh, AndLink)));
		return ret;
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(n==1);
		return atomWithNewType(h[0], FORALL_LINK);
	}
	virtual bool valid(Handle* h, const int n) const
	{
		assert(n==1);
		return isSubType(h[0], EXIST_LINK);
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(n==1);

		return Exist2ForAllLink(premiseArray[0]);
	}
};*/

class Equi2ImpRule : public Rule
{
	/// "A<=>B" => "AND(A=>B, B=>A)"
	Equi2ImpRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EQUIVALENCE_LINK))));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, EQUIVALENCE_LINK)));
		return makeSingletonSet(ret);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(1 == n);

		return atom(AND_LINK, 2,
					new atom(IMPLICATION_LINK, 2,
						new atom(child(h[0], 0)),
						new atom(child(h[0], 1))),
					new atom(IMPLICATION_LINK, 2,
						new atom(child(h[0], 1)),
						new atom(child(h[0], 0)))
				);
	}
	virtual bool valid(Handle* h, const int n) const
	{
		assert(n==1);

		return isSubType(h[0], EQUIVALENCE_LINK);
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
};

template<Type InclusionLink>
class Inh2SimRule : public GenericRule<Inh2SimFormula>
{
protected:
//	mutable std::vector<Type> ti;

	virtual Type ProductLinkType() const
	{
		return SIMILARITY_LINK;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		std::vector<Handle> ret;
		ret.push_back(premiseArray[0]);
		ret.push_back(premiseArray[1]);

		return ret;
	}

public:
	Inh2SimRule(iAtomTableWrapper *_destTable)
	: GenericRule<Inh2SimFormula>(_destTable, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(INHERITANCE_LINK))));
	}
	virtual bool valid(Handle* premiseArray, const int n) const
	{
		AtomSpace *nm = CogServer::getAtomSpace();
		return (linkInherits(InclusionLink, IMPLICATION_LINK)
			|| linkInherits(InclusionLink,INHERITANCE_LINK))
			&& (nm->getOutgoing(premiseArray[0],0) == nm->getOutgoingng(premiseArray[1],1)
			&& nm->getOutgoing(premiseArray[0],1) == nm->getOutgoing(premiseArray[1],0));
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(1==n);

		return atomWithNewType(h[0], SIMILARITY_LINK);
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(new atomWithNewType(outh, INHERITANCE_LINK)));
		return makeSingletonSet(ret);
	}
};

template<Type LinkType>
class Inclusion2EvalRule : public GenericRule<TautologyFormula>
{
public:
	Inclusion2EvalRule(iAtomTableWrapper *_destTable)
	: GenericRule<TautologyFormula>(_destTable,false)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(LinkType))));
	}

protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs();

		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, LinkType)));
		return makeSingletonSet(ret);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(EVALUATION_LINK, 2,
					new atom(PREDICATE_NODE, "The ?"),
					new atom(child(h[0], 0)));

			//atomWithNewType(h[0], EVALUATION_LINK);
	}

	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;

	virtual std::vector<Type> InputTypes() const
	{
		Type L = LinkType;

		if (ti.empty())
		{
			ti.push_back(L);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return EVALUATION_LINK;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		// Create P:
		Handle  p = addNode(PREDICATE_NODE, "The ?",
				TruthValue::TRUE_TV(),
				false);

		AtomSpace *nm = CogServer::getAtomSpace();
		HandleSeq hs = nm->getOutgoing(premiseArray[0]);

		std::vector<Handle> psat;
		psat.push_back(p);
		psat.push_back(hs[1]); //S

		// Create "EvaluationLink P S" <=> "S = SatisfyingSet P"
		destTable->addLink(EVALUATION_LINK, psat,
				TruthValue::TRUE_TV(),
				RuleResultFreshness);	

		std::vector<Handle> psat2;
		psat2.push_back(p);
		psat2.push_back(hs[0]); //A

		return psat2;
	}
};

typedef Inclusion2EvalRule<MEMBER_LINK> Mem2EvalRule;
typedef Inclusion2EvalRule<SUBSET_LINK> Inh2EvalRule;

class Eval2MemRule : public GenericRule<TautologyFormula>
{
public:
	Eval2MemRule(iAtomTableWrapper *_destTable)
	: GenericRule<TautologyFormula>(_destTable, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
	}
	virtual bool valid(Handle* premiseArray, const int n) const
	{
		return isSubType(premiseArray[0], EVALUATION_LINK);
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(INHERITANCE_LINK, 2,
						new atom(child(h[0],1)),
						new atom(CONCEPT_NODE, "")
					);
		//return //atomWithNewType(h, MEMBER_LINK);
	}

protected:

	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();

		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, LinkType)));
		return makeSingletonSet(ret);
	}

	virtual std::vector<Type> InputTypes() const
	{
		if (ti.empty())
		{
			ti.push_back(EVALUATION_LINK);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return INHERITANCE_LINK;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		HandleSeq hs;
		hs.push_back(premiseArray[1]); //A
		hs.push_back(satisfyingSet(premiseArray[0])); //S

		return hs;

		//return destTable->getHandle(MEMBER_LINK, hs);
	}
};

/** OutputInheritanceLink:
	InheritanceLink, IntensionalInheritanceLink, SubsetLink, ImplicationLink,
	IntensionalImplictionLink, or ExtensionalImplicationLink
*/

template<Type OutputInheritanceLink>
class Sim2InhRule : public Rule
{
	Sim2InhFormula f;
public:
	Sim2InhRule(iAtomTableWrapper *_destTable)
	
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(SIMILARITY_LINK,0))));
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(n==1);
		return atomWithNewType(h[0], OutputInheritanceLink);
	}
	Rule::setOfMPs o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
	{
		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();

		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, OutputInheritanceLink)));
		return makeSingletonSet(ret);
	}

  BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
  {
	assert(n == 1);
	LINKTYPE_ASSERT(premiseArray[0], SIMILARITY_LINK);
	
	CreatePremises;

	FirstPTLTruthValue* tvs[20];

	tvs[0] = (TruthValue*) &(CogServer::getAtomSpace()->getTV(premiseArray[0]));

    TruthValue* retTV = f.compute(tvs, n);

	int i=0;

    for (i = 0; i < n; i++)
      delete tvs[i];

	HandleSeq sab, sba, retlist;

	sab.push_back(premiseArray[0]);
	sab.push_back(premiseArray[1]);

	sba.push_back(premiseArray[1]);
	sba.push_back(premiseArray[0]);

	Handle  ab = destTable->addLink(OutputInheritanceLink, sab,
				retTV->clone(),
				RuleResultFreshness);	

	Handle  ba = destTable->addLink(OutputInheritanceLink, sba,
				retTV->clone(),
				RuleResultFreshness);	

	retlist.push_back(ab);
	retlist.push_back(ba);

	Handle  ret = destTable->addLink(LIST_LINK, retlist,
				retTV->clone(),
				RuleResultFreshness);			

	delete retTV;

	return ret;
  }
};

template<typename DeductionRuleType, Type InheritanceLinkType>
class VarInstantiationRule :	public Sim2InhRule<MEMBER_LINK>, //public Eval2MemRule,
								public SubsetEvalRule<CONCEPT_NODE>,
								public DeductionRule<DeductionRuleType,MEMBER_LINK>, public Inh2EvalRule
{
/*protected:
	Sim2InhFormula f;*/
public:
	VarInstantiationRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(VARIABLE_NODE))));
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(CONCEPT_NODE))));
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(SIMILARITY_LINK))));
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(EVALUATION_LINK, 2,
						new atom(h[0]),
						new atom(h[2]));
	}

public:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs(); //No support (yet)
/*		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();
		MPs ret;
		ret.insert(new atom(atomWithNewType(outh, LinkType)));
		return ret;*/
	}

  BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
  {
	assert(n == 5);
	LINKTYPE_ASSERT(premiseArray[0], EVALUATION_LINK);
	LINKTYPE_ASSERT(premiseArray[1], VARIABLE_NODE);
	LINKTYPE_ASSERT(premiseArray[2], CONCEPT_NODE);
	LINKTYPE_ASSERT(premiseArray[3], SIMILARITY_LINK);
	LINKTYPE_ASSERT(premiseArray[4], EVALUATION_LINK);

	AtomSpace *nm = CogServer::getAtomSpace();
	HandleSeq PforB = nm->getOutgoing(premiseArray[4]);
	Handle hP = PforB[0];

	Handle hC = SatisfyingSet(hP);
	Handle hA = (premiseArray[2]);
	Handle hB = (nm->getOutgoing(premiseArray[3])[1]); //2nd of sim(A,B) std::vector

	Handle inhAB = Sim2InhRule<InheritanceLinkType>::compute(&premiseArray[3], 1);
	//Handle inhBP = Eval2MemRule::compute(&premiseArray[0], 1);
	Handle BsP [] = { hB, hC };
	Handle inhBP = SubsetEvalRule::compute(BsP, 2);

	Handle hAB = (inhAB);
	Handle hBC = (inhBP);

	Handle hargs[5];
	hargs[0] = hAB; hargs[1] = hBC; hargs[2] = hA; hargs[3] = hB; hargs[4] = hC;

	Handle inhAP = DeductionRule<DeductionRuleType>::compute(hargs, 5);

	Handle eval1 = Inh2Eval::compute(&inhAP, 1); //The predicate must still be added

	HandleSeq retlist;
	retlist.push_back(premiseArray[0]); //P
	retlist.push_back(hA);

	const TruthValue& retTV = CogServer::getAtomSpace()->getTV(eval1);

	Handle ret = destTable->addLink(EVALUATION_LINK, retlist,
				retTV,
				RuleResultFreshness);	

	// TODO: Delete the resulting dummy 'eval1' node!

	return ret;
  }
};

/** Calculate either:
A & B
(A=>B) & A
AndLink(...) & AndLink(...) & ...

*/

/**
	@class ArityFreeANDRule
	Shouldn't be used directly. Use AndRule<number of arguments> instead.
*/

class RevisionRule : public GenericRule<RevisionFormula>
{
public:
	RevisionRule(iAtomTableWrapper *_destTable)
	: GenericRule<RevisionFormula>(_destTable, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(ANY, 0)));
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs(); //No support (yet)
/*		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();
		MPs ret;
		ret.insert(neBoundVertexWithNewType(outh, LinkType));

		return ret;*/
	}

	virtual bool valid(Handle* premiseArray, const int n) const
	{
		a1 = destTable->getType(premiseArray[0]);
		a2 = destTable->getType(premiseArray[1]);

		AtomSpace *nm = CogServer::getAtomSpace();
		return	a1 == a2 &&
				nm->getOutgoing(premiseArray[0]) == nm->getOutgoing(premiseArray[1]);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(n==2);

		return atom(h[0]);
	}

protected:
	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;
	mutable Type a1, a2;

	void SortTVs(Handle* premiseArray, const int n, TruthValue*** retTVs, int* retn) const
	{
	    for (int i = 0; i < n; i++)
			(*retTVs)[i] = (TruthValue*) CogServer::getAtomSpace()->getTV(premiseArray[i]);

		*retn = n;
	}

	virtual std::vector<Type> InputTypes() const
	{
		if (ti.empty())
		{
			ti.push_back(a1);
			ti.push_back(a1);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return a1;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		return CogServer::getAtomSpace()->getOutgoing(premiseArray[0]);
	}
};

unsigned int USize(const set<Handle>& triples, const set<Handle>& doubles,
				   const set<Handle>& singles);

template<Type IN_LINK_TYPE, Type OUT_LINK_TYPE>
class Ext2ExtRule : public Rule
{
public:
	Ext2Ext()
	{
		inputFilter.push_back(Btr<atom>(new atom(INSTANCE_OF, 1, new atom(IN_LINK_TYPE))));
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(OUT_LINK_TYPE, 2,
					new atom(CONCEPT_NODE,""),
					new atom(CONCEPT_NODE,""));

	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType(outh.T, OUT_LINK_TYPE))
			return Rule::setOfMPs();
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, IN_LINK_TYPE)));
		return ret;
	}

	virtual bool valid(Handle* h, const int n) const
	{
		assert(1==n);

		return isSubType(h[0], IN_LINK_TYPE);
	}
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(n == 1);
		LINKTYPE_ASSERT(premiseArray[0], IN_LINK_TYPE);

		AtomSpace *nm = CogServer::getAtomSpace();
		std::vector<Handle> in_args = nm->getOutgoing(premiseArray[0]);

		const TruthValue& retTV = nm->getTV(premiseArray[0]);

		std::vector<Handle> out_args;
		out_args.push_back(SatisfyingSet(in_args[0]));
		out_args.push_back(SatisfyingSet(in_args[1]));
		
		Handle p = destTable->addLink(OUT_LINK_TYPE, out_args,
				retTV,
				RuleResultFreshness);	
		
		return ret;
	}
};

typedef Ext2ExtRule<EXTENSIONAL_IMPLICATION_LINK, SUBSET_LINK> ExtImpl2SubsetRule;
typedef Ext2ExtRule<EXTENSIONAL_EQUIVALENCE_LINK, EXTENSIONAL_SIMILARITY_LINK> ExtEqui2ExtSimRule;
typedef Ext2ExtRule<EQUIVALENCE_LINK, SIMILARITY_LINK> Equi2SimRule;

/**
	@class NotEliminationRule
	Removes NOTNOTs.
*/

class NotEliminationRule : public Rule
{
	NotEliminationRule(iAtomTableWrapper *_destTable); 
};

#endif
}


#endif
