#ifndef SIMPLEANDRULE_H
#define SIMPLEANDRULE_H

using namespace opencog;
namespace reasoning
{

const bool RuleResultFreshness = true;
Handle UnorderedCcompute(iAtomTableWrapper *destTable,
					Type linkT, const ArityFreeFormula<TruthValue,
			       TruthValue*>& fN, Handle* premiseArray, const int n, Handle CX=NULL);

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
		if (!GET_ATW->inheritsType(CogServer::getAtomSpace()->getType(boost::get<Handle>(*outh->begin())), AND_LINK)
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
			      : destTable->addLink(AND_LINK, dummy_outgoing, GET_ATW->getTV(v2h(premiseArray[0])),
				RuleResultFreshness));
		    delete[] hs;

		      //		printf("=> ANDRUle: %s:\n", ret, getTruthValue(ret)->toString().c_str());
		      //		printTree(ret,0,-3);
		      //currentDebugLevel = -3;

		return Vertex(ret);
	}
	
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // SIMPLEANDRULE_H
