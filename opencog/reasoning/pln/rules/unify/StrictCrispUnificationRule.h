#ifndef STRICTCRISPUNIONRULE_H
#define STRICTCRISPUNIONRULE_H

namespace reasoning
{

/// Requires that all subtrees are separately produced; hence requires HypothesisRule.
class StrictCrispUnificationRule : public BaseCrispUnificationRule
{
public:
	StrictCrispUnificationRule(iAtomSpaceWrapper *_destTable)
	: BaseCrispUnificationRule(_destTable) {}
	bool validate2				(MPs& args) const { return true; }		
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
};

} // namespace reasoning
#endif // STRICTCRISPUNIONRULE_H
