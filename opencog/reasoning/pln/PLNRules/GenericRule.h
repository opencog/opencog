#ifndef GENERICRULE_H
#define GENERICRULE_H

using namespace opencog;
namespace reasoning
{

template<typename FormulaType>
class GenericRule : public Rule
{
protected:
	mutable FormulaType f;

public:
	virtual set<MPs> o2iMetaExtra(meta outh, bool& overrideInputFilter) const=0;
	virtual meta i2oType						(const vector<Vertex>& h) const=0;

	virtual TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const=0;
	
	/// Always computable
	GenericRule(iAtomTableWrapper *_destTable, bool _FreeInputArity, std::string _name = "")
	: Rule(_destTable, _FreeInputArity, true, _name) {	}
		
	BoundVertex compute(const vector<Vertex>& premiseArray, Handle CX = NULL) const
	{
		const int n = (const int) premiseArray.size();

		printf("<Generic rule args> ");
		for (int j=0;j<n;j++) {
			const Handle *ph = boost::get<Handle>(&premiseArray[j]);
			printf("[%lu] ", (ph?*ph:0));
            //printTree(premiseArray[j],0,3);
		}
		printf(" </Generic rule args>\n");

		assert(validate(premiseArray));

        printf("formatTVarray...\n");
		int TVN = f.TVN;
		TruthValue** tvs = formatTVarray(premiseArray, &TVN);
        printf("formatTVarray OK\n");

		if (!tvs) {
			printf("Warning only: GenericRule: TV array formatting failure.");
			return Vertex((Handle)NULL);
		}

		printf("Computing TV... \n");
		TruthValue* retTV = f.compute(tvs, TVN);
		printf("TV computation ok\n");

		delete[] tvs;
		printf("Res freed.\n");

		/// i2otype gives the atom skeleton (tree) w/o TV. addLink attaches it to Core with TV
		Handle ret = destTable->addAtom(*i2oType(premiseArray),
			*retTV,
			true);	
//			false);
		
        delete retTV;
        
		assert(ret);
		
		printf("Atom added.");

//		printTree(ret,0,3);

		return Vertex(ret);
	}
	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // GENERICRULE_H

