#ifndef IMPTAILEXPANSIONRULE_H
#define IMPTAILEXPANSIONRULE_H

namespace reasoning
{

class ImplicationTailExpansionRule : public Rule
{
public:
	NO_DIRECT_PRODUCTION;

	ImplicationTailExpansionRule(iAtomSpaceWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;

	bool validate2				(MPs& args) const { return true; }
};

} // namespace reasoning
#endif // IMPTAILEXPANSIONRULE_H
