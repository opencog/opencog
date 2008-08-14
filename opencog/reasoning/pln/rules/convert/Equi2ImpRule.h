#ifndef EQUI2IMPRULE_H
#define EQUI2IMPRULE_H

namespace reasoning
{

#if 0
class Equi2ImpRule : public Rule
{
	/// "A<=>B" => "AND(A=>B, B=>A)"
	Equi2ImpRule(iAtomTableWrapper *_destTable)
	: Rule(_destTable)
	{
		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EQUIVALENCE_LINK))));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		Btr<MPs> ret(new MPs);
		ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, EQUIVALENCE_LINK)));
		return makeSingletonSet(ret);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(1 == n);

		return atom(AND_LINK, 2,
					new atom(IMPLICATION_LINK, 2,
						new atom(child(h[0], 0)),
						new atom(child(h[0], 1))),
					new atom(IMPLICATION_LINK, 2,
						new atom(child(h[0], 1)),
						new atom(child(h[0], 0)))
				);
	}
	virtual bool valid(Handle* h, const int n) const
	{
		assert(n==1);

		return isSubType(h[0], EQUIVALENCE_LINK);
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const;
};
#endif

} // namespace reasoning
#endif // EQUI2IMPRULE_H
