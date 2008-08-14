#ifndef IMPCONSTRUCTIONRULE_H
#define IMPCONSTRUCTIONRULE_H

namespace reasoning
{

#if 0
class ImplicationConstructionRule : public GenericRule<Implication>
{
public:
	ImplicationConstructionRule(iAtomTableWrapper *_destTable)
	: GenericRule<Implication>(_destTable, false, "ImplicationConstructionRule")
	{ }

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	virtual meta i2oType(const vector<Vertex>& h) const;
	bool validate2				(MPs& args) const { return true; }
};
#endif

} // namespace reasoning
#endif // IMPCONSTRUCTIONRULE_H
