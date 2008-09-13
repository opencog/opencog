#ifndef LINK2LINKRULE_H
#define LINK2LINKRULE_H

namespace reasoning
{

extern std::map<int, string> type2name;
Rule::setOfMPs makeSingletonSet(Rule::MPs& mp);

template<typename FormulaType, Type SRC_LINK, Type DEST_LINK>
class Link2LinkRule : public GenericRule<FormulaType>
{
protected:
//	mutable std::vector<Type> ti;

public:
	bool validate2				(Rule::MPs& args) const { return true; }
	Link2LinkRule(iAtomTableWrapper *_destTable)
	: GenericRule<FormulaType>(_destTable,false,"")
	{
		GenericRule<FormulaType>::name = "Link2Link(" + type2name[SRC_LINK] + "=>" + type2name[DEST_LINK] +")";
		GenericRule<FormulaType>::inputFilter.push_back(meta(
			new tree<Vertex>(
				mva((Handle)SRC_LINK,
					mva((Handle)ATOM),
					mva((Handle)ATOM))
			)));
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if (!GET_ATW->inheritsType((Type)(int)boost::get<Handle>(*outh->begin()), DEST_LINK))
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
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[1];

		assert(premiseArray.size()==1);

		tvs[0] = (TruthValue*) &(CogServer::getAtomSpace()->getTV(boost::get<Handle>(premiseArray[0])));

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

typedef  Link2LinkRule<Mem2InhFormula, MEMBER_LINK, EXTENSIONAL_INHERITANCE_LINK> Mem2InhRule;
typedef  Link2LinkRule<Int2ExtFormula, IMPLICATION_LINK, MIXED_IMPLICATION_LINK>				IntImp2ExtRule;
//typedef  Link2LinkRule<Int2ExtFormula, INHERITANCE_LINK, MIXED_INHERITANCE_LINK>				IntInh2ExtRule;
typedef  Link2LinkRule<Int2ExtFormula, INHERITANCE_LINK, EXTENSIONAL_INHERITANCE_LINK>				IntInh2ExtRule;
typedef  Link2LinkRule<Ext2IntFormula, EXTENSIONAL_IMPLICATION_LINK, MIXED_IMPLICATION_LINK> ExtImp2IntRule;
typedef  Link2LinkRule<Ext2IntFormula, EXTENSIONAL_INHERITANCE_LINK, INHERITANCE_LINK> ExtInh2IntRule;
//typedef  Link2LinkRule<Ext2IntFormula, EXTENSIONAL_INHERITANCE_LINK, MIXED_INHERITANCE_LINK> ExtInh2IntRule;
typedef  Link2LinkRule<Inh2ImpFormula, INHERITANCE_LINK, IMPLICATION_LINK>					Inh2ImpRule;
typedef  Link2LinkRule<Imp2InhFormula, IMPLICATION_LINK, INHERITANCE_LINK>					Imp2InhRule;
typedef  Link2LinkRule<Mem2EvalFormula, MEMBER_LINK, EVALUATION_LINK> Mem2EvalRule;
//typedef  Link2LinkRule<Eval2InhFormula, EVALUATION_LINK, INHERITANCE_LINK> Eval2InhRule;
//typedef  Link2LinkRule<Formula, _LINK, _LINK> Rule;

} // namespace reasoning
#endif // LINK2LINKRULE_H
