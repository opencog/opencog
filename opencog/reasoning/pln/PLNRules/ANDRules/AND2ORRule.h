#ifndef AND2ORRULE_H
#define AND2ORRULE_H

namespace reasoning
{
#if 0
class AND2ORRule : public Rule
{
	AND2ORRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable, false, true, "AND2ORRule")
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

		/*return atomWithNewType(h[0], OR_LINK);*/
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
#endif

} // namespace reasoning
#endif // AND2ORRULE_H