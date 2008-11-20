#ifndef CRISPTHEOREMRULE_H
#define CRISPTHEOREMRULE_H

namespace reasoning
{

class CrispTheoremRule : public Rule
{
public:
	static map<vtree, vector<vtree> ,less_vtree> thms;
	NO_DIRECT_PRODUCTION;

	CrispTheoremRule(iAtomSpaceWrapper *_destTable);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;

	bool validate2				(MPs& args) const { return true; }
};

} // namespace reasoning
#endif // CRISPTHEOREMRULE_H
