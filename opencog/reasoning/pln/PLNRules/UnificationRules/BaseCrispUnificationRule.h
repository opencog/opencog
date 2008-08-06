#ifndef BASECRISPUNIONRULE_H
#define BASECRISPUNIONRULE_H

namespace reasoning
{

class BaseCrispUnificationRule : public Rule
{
protected:
public:

	BaseCrispUnificationRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable,true,true,"CrispUnificationRule")
	{

		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)HYPOTHETICAL_LINK))
			));
	}

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;

	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // BASECRISPUNIONRULE_H