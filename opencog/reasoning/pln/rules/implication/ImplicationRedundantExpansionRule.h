#ifndef IMPREDUNDANTEXPANSIONRULE_H
#define IMPREDUNDANTEXPANSIONRULE_H

namespace reasoning
{

class ImplicationRedundantExpansionRule : public Rule
{
protected:
	ImplicationRedundantExpansionRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,true,true,"ImplicationRedundantExpansionRule")
	{}
public:
	bool validate2				(MPs& args) const { return true; }

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // IMPREDUNDANTEXPANSIONRULE_H
