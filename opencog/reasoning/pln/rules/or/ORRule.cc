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

ORRule::ORRule(AtomSpaceWrapper *_asw)
: GenericRule<ORFormula>(_asw, true, "OR Rule")
{
}

Rule::setOfMPs ORRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
        tree<Vertex>::iterator top = outh->begin();
        
        if (!asw->isSubType(_v2h(*top), OR_LINK) ||
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

// TODO support other numbers of OR arguments besides 2
meta ORRule::targetTemplate() const
{
    return(meta(new vtree(mva((pHandle)OR_LINK, 
                                     mva((pHandle)ATOM),
                                     mva((pHandle)ATOM)
                                     ))));
}

#define USE_INCLUSION_EXCLUSION_IN_OR_RULE 0
TruthValue** ORRule::formatTVarray(const std::vector<Vertex>& premiseArray, int* newN) const
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
            tvs[ii++] = (TruthValue*) &(asw->getTV(_v2h(premiseArray[i])));
cprintf(4,"TV Arg: %s -\n", tvs[i]->toString().c_str());
        }
        
        for (i = 0; i < N-1; i++)
            for (int j = i+1; j < N; j++)
            {
#if USE_INCLUSION_EXCLUSION_IN_OR_RULE

cprintf(4,"Look up ANDLINK for args #%d,%d\n", i,j);
                TableGather comb(mva((pHandle)AND_LINK,
                                    mva(premiseArray[i]),
                                    mva(premiseArray[j])
                                ), asw);
cprintf(4,"Look up %s\n", (comb.empty() ? "success." : "fails."));
                tvs[ii++] = 
                                (!comb.empty()
                                ? getTruthValue(_v2h(comb[0]))
#if 0
// Welter's comment: this change is waiting for Ari's aproval 
                                : TruthValue::TRIVIAL_TV());
#else
                                : SimpleTruthValue(0.0,0.0)); //! @todo Use a static variable...
#endif
#else
#endif
            }
cprintf(4, "ORRule::formatTVarray OK.");
        return tvs;
}

meta ORRule::i2oType(const std::vector<Vertex>& h) const
{
        meta ret(new tree<Vertex>(mva((pHandle)OR_LINK)));

        for (uint i=0; i < h.size(); i++)
            ret->append_child(ret->begin(), mva(h[i]).begin());

        return  ret;
}

}} // namespace opencog { namespace pln {
