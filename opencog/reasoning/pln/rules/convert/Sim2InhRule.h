#ifndef SIM2INHRULE_H
#define SIM2INHRULE_H

namespace reasoning
{

/** OutputInheritanceLink:
	InheritanceLink, IntensionalInheritanceLink, SubsetLink, ImplicationLink,
	IntensionalImplictionLink, or ExtensionalImplicationLink
*/

#if 0
template<Type OutputInheritanceLink>
class Sim2InhRule : public Rule
{
	Sim2InhFormula f;
public:
	Sim2InhRule(iAtomSpaceWrapper *_destTable)
	
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

	tvs[0] = (TruthValue*) &(GET_ATW->getTV(premiseArray[0]));

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
#endif

} // namespace reasoning
#endif // SIM2INHRULE_H
