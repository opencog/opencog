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

OrRule::OrRule(AtomSpaceWrapper *_asw)
: GenericRule<OrFormula>(_asw, true, "OrRule")
{
}

Rule::setOfMPs OrRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
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

//// Can't support more than 2 inputs.
//meta OrRule::targetTemplate() const
//{
//    return(meta(new vtree(mva((pHandle)OR_LINK,
//                                     mva((pHandle)ATOM),
//                                     mva((pHandle)ATOM)
//                                     ))));
//}

//! @todo uses a somewhat tacky approach to support other numbers of Or
//! arguments besides 2. NOTE: OrRule currently gets an error with >2
//! arguments anyway.
Rule::setOfMPs OrRule::fullInputFilter() const
{
    // The possible inputs are {any 2 Atoms}, {any 3 Atoms}, ...
    int max_arity = 2;

    Rule::setOfMPs ret;

    for (int n = 2; n <= max_arity; n++) {
        MPs args;

        for (int i = 0; i < n; i++)
            args.push_back(BBvtree(new BoundVTree(mva((pHandle)ATOM))));

        ret.insert(args);
    }

    return ret;
}

#define USE_INCLUSION_EXCLUSION_IN_OR_RULE 0
TVSeq OrRule::formatTVarray(const VertexSeq& premiseArray) const
{
    const int N = (int)premiseArray.size();
        
    cprintf(3, "OrRule::formatTVarray...");

    //printTree(premiseArray[0],0,3);
    TVSeq tvs;

    for (int i = 0; i < N; i++) {
        tvs.push_back(asw->getTV(_v2h(premiseArray[i])));
        cprintf(4,"TV Arg: %s -\n", tvs[i]->toString().c_str());
    }
        
    for (int i = 0; i < N-1; i++)
        for (int j = i+1; j < N; j++) {
#if USE_INCLUSION_EXCLUSION_IN_OR_RULE

            cprintf(4,"Look up AndLINK for args #%d,%d\n", i,j);
            TableGather comb(mva((pHandle)AND_LINK,
                                 mva(premiseArray[i]),
                                 mva(premiseArray[j])
                                 ), asw);
            cprintf(4,"Look up %s\n", (comb.empty() ? "success." : "fails."));
            tvs.push_back(!comb.empty()? asw->getTV(_v2h(comb[0]))
                           : TruthValuePtr(new SimpleTruthValue(0.0,0.0)));
#else
#endif
}
cprintf(4, "OrRule::formatTVarray OK.");
return tvs;
}

meta OrRule::i2oType(const VertexSeq& h) const
{
        meta ret(new tree<Vertex>(mva((pHandle)OR_LINK)));

        for (uint i=0; i < h.size(); i++)
            ret->append_child(ret->begin(), mva(h[i]).begin());

        return  ret;
}

}} // namespace opencog { namespace pln {
