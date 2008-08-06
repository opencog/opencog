#ifndef REVISIONRULE_H
#define REVISIONRULE_H

namespace reasoning
{

#if 0
class RevisionRule : public GenericRule<RevisionFormula>
{
public:
	RevisionRule(iAtomTableWrapper *_destTable)
	: GenericRule<RevisionFormula>(_destTable, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(ANY, 0)));
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs(); //No support (yet)
/*		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();
		MPs ret;
		ret.insert(neBoundVertexWithNewType(outh, LinkType));

		return ret;*/
	}

	virtual bool valid(Handle* premiseArray, const int n) const
	{
		a1 = destTable->getType(premiseArray[0]);
		a2 = destTable->getType(premiseArray[1]);

		AtomSpace *nm = CogServer::getAtomSpace();
		return	a1 == a2 &&
				nm->getOutgoing(premiseArray[0]) == nm->getOutgoing(premiseArray[1]);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(n==2);

		return atom(h[0]);
	}

protected:
	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;
	mutable Type a1, a2;

	void SortTVs(Handle* premiseArray, const int n, TruthValue*** retTVs, int* retn) const
	{
	    for (int i = 0; i < n; i++)
			(*retTVs)[i] = (TruthValue*) CogServer::getAtomSpace()->getTV(premiseArray[i]);

		*retn = n;
	}

	virtual std::vector<Type> InputTypes() const
	{
		if (ti.empty())
		{
			ti.push_back(a1);
			ti.push_back(a1);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return a1;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		return CogServer::getAtomSpace()->getOutgoing(premiseArray[0]);
	}
};
#endif

} // namespace reasoning
#endif // REVISIONRULE_H