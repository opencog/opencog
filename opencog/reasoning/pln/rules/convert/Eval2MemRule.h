#ifndef EVAL2MEMRULE_H
#define EVAL2MEMRULE_H

namespace reasoning
{

#if 0
class Eval2MemRule : public GenericRule<TautologyFormula>
{
public:
	Eval2MemRule(iAtomSpaceWrapper *_destTable)
	: GenericRule<TautologyFormula>(_destTable, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
	}
	virtual bool valid(Handle* premiseArray, const int n) const
	{
		return isSubType(premiseArray[0], EVALUATION_LINK);
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(INHERITANCE_LINK, 2,
						new atom(child(h[0],1)),
						new atom(CONCEPT_NODE, "")
					);
		//return //atomWithNewType(h, MEMBER_LINK);
	}

protected:

	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();

		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, LinkType)));
		return makeSingletonSet(ret);
	}

	virtual std::vector<Type> InputTypes() const
	{
		if (ti.empty())
		{
			ti.push_back(EVALUATION_LINK);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return INHERITANCE_LINK;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		HandleSeq hs;
		hs.push_back(premiseArray[1]); //A
		hs.push_back(satisfyingSet(premiseArray[0])); //S

		return hs;

		//return destTable->getHandle(MEMBER_LINK, hs);
	}
};
#endif 

} // namespace reasoning
#endif // EVAL2MEMRULE_H
