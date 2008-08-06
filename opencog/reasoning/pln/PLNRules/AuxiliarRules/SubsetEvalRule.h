#ifndef SUBSETEVALRULE_H
#define SUBSETEVALRULE_H

namespace reasoning
{

/** TODO: UPDATE TO Vertex<Handle> **/
#if 0
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
#endif

} // namespace reasoning
#endif // SUBSETEVALRULE_H