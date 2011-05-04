/** ContextFreeToSensitiveRule.cc --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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

#include "ContextFreeToSensitiveRule.h"
#include <opencog/util/algorithm.h>
#include "../RuleFunctions.h"

namespace opencog { namespace pln {

meta ContextFreeToSensitiveRule::i2oType(const VertexSeq& h) const {
    OC_ASSERT(h.size() == 2);
    pHandle CX = _v2h(h[0]);
    pHandle A = _v2h(h[1]);
    return meta(new vtree(mva((pHandle)CONTEXT_LINK, mva(CX), mva(A))));
}

TVSeq ContextFreeToSensitiveRule::formatTVarray(const VertexSeq& premiseArray) const {
    OC_ASSERT(premiseArray.size()==2);
    TVSeq res;
    res.push_back(asw->getTV(_v2h(premiseArray[0])));
    res.push_back(asw->getTV(_v2h(premiseArray[1])));
    return res;
}

ContextFreeToSensitiveRule::ContextFreeToSensitiveRule(AtomSpaceWrapper* _asw)
    : super(_asw, false, "ContextFreeToSensitiveRule") {
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)CONCEPT_NODE))));
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
}

Rule::setOfMPs ContextFreeToSensitiveRule::o2iMetaExtra(meta outh,
                                                        bool& overrideInputFilter) const {
    vtree::iterator root = outh->begin();
    pHandle ph = _v2h(*root);

    if(!asw->isSubType(ph, CONTEXT_LINK))
        return Rule::setOfMPs();

    OC_ASSERT(root.number_of_children() == 2);

    pHandle CX = _v2h(*root.begin());
    pHandle A = _v2h(*root.last_child());

    BoundVTree* CX_bv = new BoundVTree(mva(CX));
    BoundVTree* A_bv = new BoundVTree(mva(A));

    overrideInputFilter = true;

    Rule::MPs res;
    res.push_back(BBvtree(CX_bv));
    res.push_back(BBvtree(A_bv));

    return makeSingletonSet(res);
}

meta ContextFreeToSensitiveRule::targetTemplate() const {
    return(meta(new vtree(mva((pHandle)CONTEXT_LINK, 
                              vtree(CreateVar(asw)),
                              vtree(CreateVar(asw))))));
}

}} // namespace opencog { namespace pln {
