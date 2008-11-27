#ifndef CUSTOMCRISPUNIONRULE_H
#define CUSTOMCRISPUNIONRULE_H

namespace reasoning
{

class CustomCrispUnificationRule : public Rule
{
protected:
	Handle ForallLink;
public:

	CustomCrispUnificationRule(Handle _ForallLink, iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,false,false,"CrispUnificationRule"), ForallLink(_ForallLink)
	{
		inputFilter.push_back(meta(
				new tree<Vertex>(mva((Handle)ATOM))));
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = Handle::UNDEFINED) const
	{
		assert(0);
		return Vertex((Handle)NULL);
	}

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{ return Rule::setOfMPs(); }

	bool validate2(MPs& args) const { return true; }

	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};


} // namespace reasoning
#endif // CUSTOMCRISPUNIONRULE_H
