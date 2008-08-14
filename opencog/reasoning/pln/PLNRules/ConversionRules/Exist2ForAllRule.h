#ifndef EXIST2FORALLRULE_H
#define EXIST2FORALLRULE_H

namespace reasoning
{

#if 0
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
}
#endif

} // namespace reasoning
#endif // EXIST2FORALLRULE_H
