#ifndef QUANTIFIERRULE_H
#define QUANTIFIERRULE_H

namespace reasoning
{

template<typename FormulaType, Type OUTPUT_LINK_TYPE>
class QuantifierRule : public Rule
{
	Handle domain;
	FormulaType f;
public:
	bool validate2				(MPs& args) const { return true; }

	QuantifierRule(iAtomTableWrapper *_destTable, const Handle& _domain)
	: Rule(_destTable, false, true, "QuantifierRule"),
	domain(_domain) {
		inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)((OUTPUT_LINK_TYPE==FORALL_LINK) ? EXIST_LINK : FORALL_LINK),
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs(); //No support (yet)
	}

	//Domain should be inferred instead from the premis ConceptNodes!!!

	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		const int n = (int)premiseArray.size();
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[n];
		int i;
		for (i = 0; i < n; i++)
			tvs[i] = (TruthValue*) &(CogServer::getAtomSpace()->getTV(boost::get<Handle>(premiseArray[i])));

		TruthValue* retTV = f.compute(tvs, n);	

		delete[] tvs;

//haxx::
		Handle ret = destTable->addLink(OUTPUT_LINK_TYPE, HandleSeq(),
				*retTV,
				RuleResultFreshness);	
//				false);

        delete retTV;

		return Vertex(ret);
	}
	
	NO_DIRECT_PRODUCTION;
};

typedef QuantifierRule<FORALLFormula, FORALL_LINK> FORALLRule;
typedef QuantifierRule<EXISTFormula, EXIST_LINK> ExistRule;
typedef QuantifierRule<PredicateTVFormula, VARIABLE_SCOPE_LINK> PLNPredicateRule;

} // namespace reasoning
#endif // QUANTIFIERRULE_H