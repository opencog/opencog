#ifndef SIMSUBS1RULE_H
#define SIMSUBS1RULE_H

namespace reasoning
{

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

} // namespace reasoning
#endif // SIMSUBS1RULE_H
