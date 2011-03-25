/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace opencog { namespace pln {

/** ToDo: Update
bool ArityFreeAndRule::asymmetric(Handle* A, Handle* B) const
{
        bool provenAsymmetric = false;
        bool provenSymmetric = false;
        bool swapped = false;
        TruthValuePtr tvs[2];
        vector<Handle> ImpLinks, EvaLinks;
    
        do
        {
            EvaLinks = nm->getOutgoing(*A); //Eval: P,a
            ImpLinks = nm->getOutgoing(*B); //Impl: P->P2

            tvs[0] = nm->getTV(*A);
            tvs[1] = nm->getTV(*B);
        
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

BoundVertex ArityFreeAndRule::compute(Handle A, Handle B, Handle CX)  const //vector<Handle> vh)
{
        HandleSeq SAB;
        vector<Handle> ImpLinks, EvaLinks;

        TruthValue* t=NULL;
        
        TruthValuePtr tvs[2];

        bool is_asymmetric = asymmetric(&A, &B); //, ImpLinks, EvaLinks);

        EvaLinks = nm->getOutgoing(A); //Eval: P,a
        ImpLinks = nm->getOutgoing(B); //Impl: P->P2
        tvs[0] = nm->getTV(A);
        tvs[1] = nm->getTV(B);
        
        if (is_asymmetric) //if (isLink(nm->getType(B)))
        {
            t = f2.compute(tvs, 2);
            SAB.push_back(ImpLinks[1]);
            SAB.push_back(EvaLinks[1]);
            
            return asw->addLink(  EVALUATION_LINK, SAB,
                t,
                                RuleResultFreshness);   

        }
        else if (nm->getType(B) == EVALUATION_LINK
            && nm->getType(A) == EVALUATION_LINK)
        {
            t = fN.compute(tvs, 2);
            SAB.push_back(A);
            SAB.push_back(B);
            
            return asw->addLink(  AND_LINK, SAB,
                t,
                                RuleResultFreshness);   

        }
        else
        { assert(0); }
        
        return Vertex((Handle)NULL);
}

BoundVertex ArityFreeAndRule::computeSymmetric(vector<Handle> nodes, Handle CX)
{
        const int n = nodes.size();
        TruthValue* t=NULL;
        TruthValuePtr* tvs = new TruthValuePtr[nodes.size()];
        int ti=0;
        
        for (ti = 0; ti < nodes.size(); ti++)
            tvs[ti] = nm->getTV(nodes[ti]);

        t = fN.compute(tvs, nodes.size());
        
        Handle h = asw->addLink(  AND_LINK, nodes,
            t,
                                RuleResultFreshness);   
        
        for (ti = 0; ti < nodes.size(); ti++)
            delete tvs[ti];
        
        delete[] tvs;
        
        return h;
}
*/

void ArityFreeAndRule::DistinguishNodes(const VertexSeq& premiseArray,
                                        std::set<pHandle>& Andlinks,
                                        std::set<pHandle>& nodes) const
{
    const int n = premiseArray.size();
      
    for (int pi = 0; pi < n; pi++)
        ((((Type)_v2h(premiseArray[pi])) == AND_LINK)
         ? Andlinks
         : nodes
         ).insert(_v2h(premiseArray[pi]));
}

}} // namespace opencog { namespace pln {
