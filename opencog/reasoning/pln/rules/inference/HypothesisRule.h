#ifndef HYPOTHESISRULE_H
#define HYPOTHESISRULE_H

namespace reasoning
{

class HypothesisRule : public Rule
{
protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = Handle::UNDEFINED) const
	{
		return premiseArray[0];
	}
public:
	bool validate2				(MPs& args) const { return true; }
	HypothesisRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable, false, false, "Hypothesis")
	{
		//inputFilter.push_back(new atom(result));
	}
	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};

} // namespace reasoning
#endif // HYPOTHESISRULE_H
