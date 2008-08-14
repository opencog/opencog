#include <platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

ORRule::ORRule(iAtomTableWrapper *_destTable)
: GenericRule<ORFormula>(_destTable, true, "OR Rule")
{
}

Rule::setOfMPs ORRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
        AtomSpace *nm = CogServer::getAtomSpace();
        tree<Vertex>::iterator top = outh->begin();
        
        if (!inheritsType(nm->getType(v2h(*top)), OR_LINK) ||
            top.number_of_children() > 2)
            return Rule::setOfMPs();

        MPs ret;
        
        for (tree<Vertex>::sibling_iterator i = outh->begin(top); i != outh->end(top); i++)
        {
            //ret.push_back(BBvtree(new BoundVTree((ModifiedVTree *) *i)));
			ret.push_back(BBvtree(new BoundVTree(i)));
        }       
            
        overrideInputFilter = true;

        return makeSingletonSet(ret);
}

#define USE_INCLUSION_EXCLUSION_IN_OR_RULE 0
TruthValue** ORRule::formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
{
        const int N = (int)premiseArray.size();
        
cprintf(3, "ORRule::formatTVarray...");
#if USE_INCLUSION_EXCLUSION_IN_OR_RULE
        *newN = N*(N+1)/2; // (N-1)+(N-2)+...+1 = N*(N-1)/2
#else
        *newN = N;
#endif
//printTree(premiseArray[0],0,3);
        TruthValue** tvs = new TruthValue*[*newN];

        int i = 0, ii=0;
        for (i = 0; i < N; i++)
        {
            tvs[ii++] = (TruthValue*) &(getTruthValue(v2h(premiseArray[i])));
cprintf(4,"TV Arg: %s -\n", tvs[i]->toString().c_str());
        }
        
        for (i = 0; i < N-1; i++)
            for (int j = i+1; j < N; j++)
            {
#if USE_INCLUSION_EXCLUSION_IN_OR_RULE

cprintf(4,"Look up ANDLINK for args #%d,%d\n", i,j);
                TableGather comb(mva((Handle)AND_LINK,
                                    mva(premiseArray[i]),
                                    mva(premiseArray[j])
                                ), destTable);
cprintf(4,"Look up %s\n", (comb.empty() ? "success." : "fails."));
                tvs[ii++] = 
                                (!comb.empty()
                                ? getTruthValue(v2h(comb[0]))
#if 0
// Welter's comment: this change is waiting for Ari's aproval 
                                : TruthValue::TRIVIAL_TV());
#else
                                : SimpleTruthValue(0.0,0.0)); // TODO: Use a static variable...
#endif
#else
#endif
            }
cprintf(4, "ORRule::formatTVarray OK.");
        return tvs;
}

meta ORRule::i2oType(const vector<Vertex>& h) const
{
        meta ret(new tree<Vertex>(mva((Handle)OR_LINK)));

        for (uint i=0; i < h.size(); i++)
            ret->append_child(ret->begin(), mva(h[i]).begin());

        return  ret;
}

} // namespace reasoning
