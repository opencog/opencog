#ifndef LOOKUPRULE_H
#define LOOKUPRULE_H

namespace reasoning
{

/** @class LookupRule	
*/

class LookupRule : public Rule
{
protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		return premiseArray[0];
	}
public:
	bool validate2				(MPs& args) const { return true; }
	LookupRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable, false, false, "Lookup")
	{
		//inputFilter.push_back(new atom(result));
	}
	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};

} // namespace reasoning
#endif // LOOKUPRULE_H
