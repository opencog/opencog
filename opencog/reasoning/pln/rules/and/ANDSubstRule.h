#ifndef ANDSUBSTRULE_H
#define ANDSUBSTRULE_H

namespace reasoning
{

class ANDSubstRule : public Rule
{
protected:
	ANDSubstRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"ANDSubstRule")
	{}
public:
	bool validate2				(MPs& args) const { return true; }

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // ANDSUBSTRULE_H

