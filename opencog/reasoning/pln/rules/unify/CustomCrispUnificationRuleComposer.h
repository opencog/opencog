#ifndef CUSTOMCRISPUNIONCOMPOSERRULE_H
#define CUSTOMCRISPUNIONCOMPOSERRULE_H

namespace reasoning
{

class CustomCrispUnificationRuleComposer : public BaseCrispUnificationRule
{
	Handle ForallLink;
public:

	CustomCrispUnificationRuleComposer(Handle _ForallLink, iAtomSpaceWrapper *_destTable)
	: BaseCrispUnificationRule(_destTable), ForallLink(_ForallLink) {}

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

//	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2(MPs& args) const { return true; }

	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // CUSTOMCRISPUNIONCOMPOSERRULE_H
