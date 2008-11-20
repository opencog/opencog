#ifndef INH2EVALRULE_H
#define INH2EVALRULE_H

namespace reasoning
{

#if 0
class Inh2EvalRule : public GenericRule<TautologyFormula>
{
protected:
	mutable std::vector<Type> ti;

public:
	bool validate2(Rule::MPs& args) const { return true; }
	Inh2EvalRule(iAtomSpaceWrapper *_destTable)
	: GenericRule<TautologyFormula>(_destTable,false,"")
	{
		GenericRule<FormulaType>::name = "Inh2Eval";
		GenericRule<FormulaType>::inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)INHERITANCE_LINK,
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!inheritsType((Type)boost::get<Handle>(*outh->begin()), EVALUATION_LINK))
			return Rule::setOfMPs();

		Rule::MPs ret;

		BBvtree ret_m(new BoundVTree(*outh));
		*ret_m->begin() = Vertex((Handle)SRC_LINK);
		ret.push_back(ret_m);
		
		overrideInputFilter = true;

		return makeSingletonSet(ret);
	}

	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new SimpleTruthValue*[1];

		assert(premiseArray.size()==1);

		tvs[0] = &(nm->getTV(boost::get<Handle>(premiseArray[0])));

		return tvs;
	}

	virtual meta i2oType(const vector<Vertex>& h) const
	{
		assert(1==h.size());

		meta ret = atomWithNewType(h[0], DEST_LINK);
cprintf(3,"i2otype() outputs: ");
#if 0
rawPrint(*ret, ret->begin(), 3);
#else
  NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME, 
                      NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION, 
                      NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE, 
                      3);
    printer.print(ret->begin());
#endif

		return ret;
	}
	NO_DIRECT_PRODUCTION;
};
#endif

} // namespace reasoning
#endif // INH2EVALRULE_H
