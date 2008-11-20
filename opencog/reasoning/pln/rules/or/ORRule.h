#ifndef ORRULE_H
#define ORRULE_H

namespace reasoning
{

class ORRule : public GenericRule<ORFormula>
{
public:
	ORRule(iAtomSpaceWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	
	NO_DIRECT_PRODUCTION;
	
	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const;
public:
	bool validate2				(MPs& args) const { return true; }

	virtual meta i2oType(const vector<Vertex>& h) const;
};

} // namespace reasoning
#endif // ORRULE_H
