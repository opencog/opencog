#include <platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../Ptlatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

/** ToDo: Update
bool ArityFreeANDRule::asymmetric(Handle* A, Handle* B) const
{
        bool provenAsymmetric = false;
        bool provenSymmetric = false;
        bool swapped = false;
        TruthValue* tvs[2];
        vector<Handle> ImpLinks, EvaLinks;
    
        do
        {
            EvaLinks = nm->getOutgoing(*A); //Eval: P,a
            ImpLinks = nm->getOutgoing(*B); //Impl: P->P2

            tvs[0] = getTruthValue(*A);
            tvs[1] = getTruthValue(*B);
        
            if (nm->getType(*B) == IMPLICATION_LINK
                && nm->getType(*A) == EVALUATION_LINK
                && ImpLinks.size()==2   &&   EvaLinks.size()==2   &&   ImpLinks[0] == EvaLinks[0])
                provenAsymmetric = true;
            else if (!swapped)
            {
                swap<Handle>(*A, *B);
                swapped = true;
            }
            else
                provenSymmetric = true;
        } while (!provenSymmetric && !provenAsymmetric);

        return provenAsymmetric;
}

BoundVertex ArityFreeANDRule::compute(Handle A, Handle B, Handle CX)  const //vector<Handle> vh)
{
        HandleSeq SAB;
        vector<Handle> ImpLinks, EvaLinks;

        TruthValue* t=NULL;
        
        TruthValue* tvs[2];

        bool is_asymmetric = asymmetric(&A, &B); //, ImpLinks, EvaLinks);

        EvaLinks = nm->getOutgoing(A); //Eval: P,a
        ImpLinks = nm->getOutgoing(B); //Impl: P->P2
        tvs[0] = getTruthValue(A);
        tvs[1] = getTruthValue(B);
        
        if (is_asymmetric) //if (isLink(nm->getType(B)))
        {
            t = f2.compute(tvs, 2);
            SAB.push_back(ImpLinks[1]);
            SAB.push_back(EvaLinks[1]);
            
            return destTable->addLink(  EVALUATION_LINK, SAB,
                t,
                                RuleResultFreshness);   

        }
        else if (nm->getType(B) == EVALUATION_LINK
            && nm->getType(A) == EVALUATION_LINK)
        {
            t = fN.compute(tvs, 2);
            SAB.push_back(A);
            SAB.push_back(B);
            
            return destTable->addLink(  AND_LINK, SAB,
                t,
                                RuleResultFreshness);   

        }
        else
        { assert(0); }
        
        return Vertex((Handle)NULL);
}

BoundVertex ArityFreeANDRule::computeSymmetric(vector<Handle> nodes, Handle CX)
{
        const int n = nodes.size();
        TruthValue* t=NULL;
        TruthValue** tvs = new SimpleTruthValue*[nodes.size()];
        int ti=0;
        
        for (ti = 0; ti < nodes.size(); ti++)
            tvs[ti] = getTruthValue(nodes[ti]);

        t = fN.compute(tvs, nodes.size());
        
        Handle h = destTable->addLink(  AND_LINK, nodes,
            t,
                                RuleResultFreshness);   
        
        for (ti = 0; ti < nodes.size(); ti++)
            delete tvs[ti];
        
        //  delete[] tvs;
        
        return h;
}
*/

void ArityFreeANDRule::DistinguishNodes(const vector<Vertex>& premiseArray, set<Handle>& ANDlinks, set<Handle>& nodes) const
{
      const int n = premiseArray.size();
      
    for (int pi = 0; pi < n; pi++)
        ((((Type)(int)v2h(premiseArray[pi])) == AND_LINK)
            ? ANDlinks
            : nodes
            ).insert(v2h(premiseArray[pi]));
}

} // namespace reasoning
