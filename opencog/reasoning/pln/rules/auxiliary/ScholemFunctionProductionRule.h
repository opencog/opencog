#ifndef SCHOLEMPRODRULE_H
#define SCHOLEMPRODRULE_H

namespace reasoning
{

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
	ScholemFunctionProductionRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,false,false,"ScholemFunctionProductionRule")
	{
		//inputFilter.push_back(new atom(result));
	}
	bool validate2				(MPs& args) const { return true; }

	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};

} // namespace reasoning
#endif // SCHOLEMPRODRULE_H
