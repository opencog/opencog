#ifndef NOTEVALUATORRULE_H
#define NOTEVALUATORRULE_H

namespace reasoning
{

class NotEvaluatorRule : public GenericRule<NotFormula>
{
protected:
	TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new TruthValue*[2];

		const int N = (int)premiseArray.size();
		assert(N==1);

		//std::vector<Handle> real = premiseArray[0];
#if 0
// Welter's comment: this change is waiting for Ari's aproval 
		tvs[0] = (TruthValue*) &(TruthValue::TRIVIAL_TV());
#else
		tvs[0] = new SimpleTruthValue(0,0); //nm->getTV(premiseArray[0]);
		// TODO: create the TrivialTV to use here
#endif
		tvs[1] = (TruthValue*) &(GET_ATW->getTV(boost::get<Handle>(premiseArray[0])));
		return tvs;
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

public:
	NotEvaluatorRule(iAtomSpaceWrapper *_destTable);
	meta i2oType(const vector<Vertex>& h) const;

	bool validate2				(MPs& args) const { return true; }
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // NOTEVALUATORRULE_H
