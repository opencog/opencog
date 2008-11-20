#ifndef ANDRULE_H
#define ANDRULE_H

namespace reasoning
{

class ANDRule : public ArityFreeANDRule
{
public:
	ANDRule(iAtomSpaceWrapper *_destTable)
	: ArityFreeANDRule(_destTable)
	{
		name = "AND Evaluator Rule";
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	//Btr<set<BoundVertex > > attemptDirectProduction(meta outh);

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	
	/// Direct production was used here before. TODO: Check whether this should be resumed!
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // ANDRULE_H
