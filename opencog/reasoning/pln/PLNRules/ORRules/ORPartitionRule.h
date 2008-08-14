#ifndef ORPARTITIONRULE_H
#define ORPARTITIONRULE_H

namespace reasoning
{

/** @class ORPartitionRule
	Partitions argument into, like, OR(A, OR(B, OR(C, D)))
*/

class ORPartitionRule : public Rule
{
	ORRule* regularOR;
public:

	ORPartitionRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable, true, true, "ORPartitionRule") 
	{
		regularOR = new ORRule(_destTable);
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	bool validate2				(MPs& args) const { return true; }
	
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // ORPARTITIONRULE_H
