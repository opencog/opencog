#ifndef UNORDEREDLINKPERMRULE_H
#define UNORDEREDLINKPERMRULE_H

namespace reasoning
{

#if 0
class UnorderedLinkPermutationRule : public GenericRule<Permutation??>
{
public:
	UnorderedLinkPermutationRule(iAtomSpaceWrapper *_destTable)
	: GenericRule<Permutation?>(_destTable, false, "UnorderedLinkPermutationRule")
	{ }

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	virtual meta i2oType(const vector<Vertex>& h) const;
	bool validate2				(MPs& args) const { return true; }
};
#endif

} // namespace reasoning
#endif // UNORDEREDLINKPERMRULE_H
