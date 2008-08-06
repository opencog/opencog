#ifndef CRISPUNIONRULE_H
#define CRISPUNIONRULE_H

namespace reasoning
{

class CrispUnificationRule : public BaseCrispUnificationRule
{
public:

	CrispUnificationRule(iAtomTableWrapper *_destTable)
	: BaseCrispUnificationRule(_destTable) {}
		
	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

//	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
	bool validate2				(MPs& args) const { return true; }

	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // CRISPUNIONRULE_H