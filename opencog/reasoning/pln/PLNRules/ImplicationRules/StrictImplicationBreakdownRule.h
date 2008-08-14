#ifndef STRICTIMPBREAKDOWNRULE_H
#define STRICTIMPBREAKDOWNRULE_H

namespace reasoning
{

/// (x, x->A) => A.
class StrictImplicationBreakdownRule : public Rule
{
public:
	NO_DIRECT_PRODUCTION;

	StrictImplicationBreakdownRule(iAtomTableWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2				(MPs& args) const { return true; }

};

} // namespace reasoning
#endif // STRICTIMPBREAKDOWNRULE_H
