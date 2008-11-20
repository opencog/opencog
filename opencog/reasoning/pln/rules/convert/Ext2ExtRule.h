#ifndef EXT2EXTRULE_H
#define EXT2EXTRULE_H

namespace reasoning
{

#if 0
template<Type IN_LINK_TYPE, Type OUT_LINK_TYPE>
class Ext2ExtRule : public Rule
{
public:
	Ext2Ext()
	{
		inputFilter.push_back(Btr<atom>(new atom(INSTANCE_OF, 1, new atom(IN_LINK_TYPE))));
	}
	virtual atom i2oType(Handle* h, const int n) const
	{
		return atom(OUT_LINK_TYPE, 2,
					new atom(CONCEPT_NODE,""),
					new atom(CONCEPT_NODE,""));

	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType(outh.T, OUT_LINK_TYPE))
			return Rule::setOfMPs();
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, IN_LINK_TYPE)));
		return ret;
	}

	virtual bool valid(Handle* h, const int n) const
	{
		assert(1==n);

		return isSubType(h[0], IN_LINK_TYPE);
	}
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		assert(n == 1);
		LINKTYPE_ASSERT(premiseArray[0], IN_LINK_TYPE);

		AtomSpaceWrapper *nm = GET_ATW;
		std::vector<Handle> in_args = nm->getOutgoing(premiseArray[0]);

		const TruthValue& retTV = nm->getTV(premiseArray[0]);

		std::vector<Handle> out_args;
		out_args.push_back(SatisfyingSet(in_args[0]));
		out_args.push_back(SatisfyingSet(in_args[1]));
		
		Handle p = destTable->addLink(OUT_LINK_TYPE, out_args,
				retTV,
				RuleResultFreshness);	
		
		return ret;
	}
};

typedef Ext2ExtRule<EXTENSIONAL_IMPLICATION_LINK, SUBSET_LINK> ExtImpl2SubsetRule;
typedef Ext2ExtRule<EXTENSIONAL_EQUIVALENCE_LINK, EXTENSIONAL_SIMILARITY_LINK> ExtEqui2ExtSimRule;
typedef Ext2ExtRule<EQUIVALENCE_LINK, SIMILARITY_LINK> Equi2SimRule;
#endif

} // namespace reasoning
#endif // EXT2EXTRULE_H
