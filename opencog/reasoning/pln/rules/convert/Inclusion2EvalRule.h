#ifndef INC2EVALRULE_H
#define INC2EVALRULE_H

namespace reasoning
{

#if 0
template<Type LinkType>
class Inclusion2EvalRule : public GenericRule<TautologyFormula>
{
public:
	Inclusion2EvalRule(iAtomTableWrapper *_destTable)
	: GenericRule<TautologyFormula>(_destTable,false)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(LinkType))));
	}

protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs();

		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, LinkType)));
		return makeSingletonSet(ret);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(EVALUATION_LINK, 2,
					new atom(PREDICATE_NODE, "The ?"),
					new atom(child(h[0], 0)));

			//atomWithNewType(h[0], EVALUATION_LINK);
	}

	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;

	virtual std::vector<Type> InputTypes() const
	{
		Type L = LinkType;

		if (ti.empty())
		{
			ti.push_back(L);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return EVALUATION_LINK;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		// Create P:
		Handle  p = addNode(PREDICATE_NODE, "The ?",
				TruthValue::TRUE_TV(),
				false);

		AtomTableWrapper *nm = GET_ATW;
		HandleSeq hs = nm->getOutgoing(premiseArray[0]);

		std::vector<Handle> psat;
		psat.push_back(p);
		psat.push_back(hs[1]); //S

		// Create "EvaluationLink P S" <=> "S = SatisfyingSet P"
		destTable->addLink(EVALUATION_LINK, psat,
				TruthValue::TRUE_TV(),
				RuleResultFreshness);	

		std::vector<Handle> psat2;
		psat2.push_back(p);
		psat2.push_back(hs[0]); //A

		return psat2;
	}
};

typedef Inclusion2EvalRule<MEMBER_LINK> Mem2EvalRule;
typedef Inclusion2EvalRule<SUBSET_LINK> Inh2EvalRule;
#endif

} // namespace reasoning
#endif // INC2EVALRULE_H
