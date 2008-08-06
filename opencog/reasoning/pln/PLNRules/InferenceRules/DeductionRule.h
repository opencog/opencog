#ifndef DEDUCTIONRULE_H
#define DEDUCTIONRULE_H

namespace reasoning
{

#define CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE 0

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

} // namespace reasoning
#endif // DEDUCTIONRULE_H