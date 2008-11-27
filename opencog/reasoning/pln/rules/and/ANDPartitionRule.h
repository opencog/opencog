#ifndef ANDPARTITIONRULE_H
#define ANDPARTITIONRULE_H

namespace reasoning
{

/** @class ANDPartitionRule
	Partitions argument into smaller ANDLinks
*/

class ANDPartitionRule : public Rule
{
	SymmetricANDFormula fN;

public:
	ANDPartitionRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,true,true,"ANDPartitionRule")
	{ }
	
	bool validate2				(MPs& args) const { return true; }
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = Handle::UNDEFINED) const;
	
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // ANDPARTITIONRULE_H
