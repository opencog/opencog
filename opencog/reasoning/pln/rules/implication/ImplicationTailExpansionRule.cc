/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

ImplicationTailExpansionRule::ImplicationTailExpansionRule(AtomSpaceWrapper *_asw)
: Rule(_asw, false, true, "ImplicationTailExpansionRule")
{
    inputFilter.push_back(meta(
        new tree<Vertex>(mva((pHandle)
            IMPLICATION_LINK,
                mva((pHandle)ATOM),
                mva((pHandle)ATOM)
        ))
    ));
    inputFilter.push_back(meta(
        new tree<Vertex>(mva((pHandle)
            IMPLICATION_LINK,
                mva((pHandle)ATOM),
                mva((pHandle)ATOM)
        ))
    ));
}

Rule::setOfMPs ImplicationTailExpansionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    if (!(asw->isSubType(_v2h(*outh->begin()), IMPLICATION_LINK)))
        return Rule::setOfMPs();
        
    tree<Vertex>::sibling_iterator hs1 = outh->begin(outh->begin());
    tree<Vertex>::sibling_iterator hs0 = hs1++;
    tree<Vertex>::sibling_iterator hs11 = outh->begin(hs1);
    tree<Vertex>::sibling_iterator hs10 =hs11++;

cprintf(0,"T:%d\n", (Type)_v2h(*hs1));
    
    if (hs0  == outh->end(outh->begin()) ||
        hs11 == outh->end(hs1) ||
        !(asw->isSubType(_v2h(*hs1), AND_LINK)) )
        return Rule::setOfMPs();

    /// A => (B&C)
    
    MPs ret;

    ret.push_back(BBvtree(new BoundVTree(mva((pHandle)IMPLICATION_LINK, 
            mva(*hs0),
            vtree(hs10)))));
    ret.push_back(BBvtree(new BoundVTree(mva((pHandle)IMPLICATION_LINK, 
            mva(*hs0),
            vtree(hs11)))));
//      ret.push_back(BBvtree(new BoundVTree(myvar)));

    overrideInputFilter = true;
    
    return makeSingletonSet(ret);
}

BoundVertex ImplicationTailExpansionRule::compute(const VertexSeq& premiseArray, pHandle CX) const
{
/*  for (int i=0;i<premiseArray.size();i++)
        printTree(v2h(premiseArray[i]),0,0);
    getc(stdin);getc(stdin);*/
    
    vtree res(mva((pHandle)IMPLICATION_LINK,
        mva(asw->getOutgoing(_v2h(premiseArray[0]))[0]),
        mva((pHandle)AND_LINK,
            mva(asw->getOutgoing(_v2h(premiseArray[0]))[1]),
            mva(asw->getOutgoing(_v2h(premiseArray[1]))[1])
        )
    ));
        
    /*for (int i=0;i<premiseArray.size();i++)
        res.append_child(res.begin(), premiseArray[i]*/
    TruthValuePtr tv(asw->getTV(_v2h(SimpleAndRule<2>(asw).compute(premiseArray,CX).value)));
    return Vertex(asw->addAtom(res, *tv, true));
}

}} // namespace opencog { namespace pln {
