#ifndef INH2SIMRULE_H
#define INH2SIMRULE_H

namespace reasoning
{

#if 0
template<Type InclusionLink>
class Inh2SimRule : public GenericRule<Inh2SimFormula>
{
protected:
//	mutable std::vector<Type> ti;

	virtual Type ProductLinkType() const
	{
		return SIMILARITY_LINK;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		std::vector<Handle> ret;
		ret.push_back(premiseArray[0]);
		ret.push_back(premiseArray[1]);

		return ret;
	}

public:
	Inh2SimRule(iAtomTableWrapper *_destTable)
	: GenericRule<Inh2SimFormula>(_destTable, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(INHERITANCE_LINK))));
	}
	virtual bool valid(Handle* premiseArray, const int n) const
	{
		AtomTableWrapper *nm = GET_ATW;
		return (linkInherits(InclusionLink, IMPLICATION_LINK)
			|| linkInherits(InclusionLink,INHERITANCE_LINK))
			&& (nm->getOutgoing(premiseArray[0],0) == nm->getOutgoingng(premiseArray[1],1)
			&& nm->getOutgoing(premiseArray[0],1) == nm->getOutgoing(premiseArray[1],0));
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(1==n);

		return atomWithNewType(h[0], SIMILARITY_LINK);
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(new atomWithNewType(outh, INHERITANCE_LINK)));
		return makeSingletonSet(ret);
	}
};
#endif

} // namespace reasoning
#endif // INH2SIMRULE_H
