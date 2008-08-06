#ifndef OR2ANDRULE_H
#define OR2ANDRULE_H

namespace reasoning
{

#if 0
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
#endif

} // namespace reasoning
#endif // OR2ANDRULE_H