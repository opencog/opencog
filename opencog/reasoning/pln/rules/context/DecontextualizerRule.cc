/** DecontextualizerRule.cc --- 
 *
 * Copyright (C) 2010 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@laptop>
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

#include "DecontextualizerRule.h"
#include <opencog/util/algorithm.h>
#include "../RuleFunctions.h"

namespace opencog { namespace pln {

meta DecontextualizerRule::i2oType(const std::vector<Vertex>& h) const {
    OC_ASSERT(h.size() == 1);
    pHandle ph = _v2h(h[0]);
    OC_ASSERT(asw->isSubType(ph, CONTEXT_LINK));
    OC_ASSERT(asw->getArity(ph) == 2);
    pHandle CX = asw->getOutgoing(ph,0);
    pHandle A = asw->getOutgoing(ph,1); // contextualized atom

    // case b) decontextualizing a Node
    // @note it is missing the cases where concepts are AND_LINK and such
    if(asw->isSubType(A, NODE)) {
        return meta(new vtree(mva((pHandle)SUBSET_LINK, mva(CX), mva(A))));
    }
    // case a) decontextualizing a relation
    else {
        OC_ASSERT(asw->getArity(A)==2); // @todo generilize for n-ary
        pHandle AC1 = asw->getOutgoing(A,0);
        pHandle AC2 = asw->getOutgoing(A,1);
        return meta(new vtree(mva((pHandle)asw->getType(A),
                                  mva((pHandle)AND_LINK, mva(CX), mva(AC1)),
                                  mva((pHandle)AND_LINK, mva(CX), mva(AC2)))));
    }
}

TVSeq DecontextualizerRule::formatTVarray(const std::vector<Vertex>& premiseArray) const {
    OC_ASSERT(premiseArray.size()==1);
    return TVSeq(1, &(asw->getTV(_v2h(premiseArray[0]))));
}

DecontextualizerRule::DecontextualizerRule(AtomSpaceWrapper* _asw)
    : GenericRule<IdentityFormula>(_asw, false, "DecontextualizerRule") {
    inputFilter.push_back(meta(new vtree(mva((pHandle)CONTEXT_LINK,
                                             mva((pHandle)NODE),
                                             mva((pHandle)ATOM)))));
}

Rule::setOfMPs DecontextualizerRule::o2iMetaExtra(meta outh,
                                                  bool& overrideInputFilter) const {
    vtree::iterator root = outh->begin();    
    pHandle ph = _v2h(*root);

    if(asw->isSubType(ph, LINK))
        return Rule::setOfMPs();

    // @todo it should probably be better to first try to
    // contextualize a relation and then only if it's not possible try
    // the Node contextualization

    BoundVTree* res;

    // case b) contextualizing a Node
    if(asw->getType(ph) == SUBSET_LINK) {
        OC_ASSERT(root.number_of_children() == 2); // maybe this could be relaxed
        vtree::iterator CX_it = root.begin();
        vtree::iterator A_it = root.last_child();
        res = new BoundVTree(mva((pHandle)CONTEXT_LINK, vtree(CX_it), vtree(A_it)));
    }
    // case a) contextualizing a relation
    else {
        OC_ASSERT(root.number_of_children() == 2); // maybe could be relaxed
        vtree::iterator And1_it = root.begin();
        pHandle And1 = _v2h(*And1_it);
        vtree::iterator And2_it = root.last_child();
        pHandle And2 = _v2h(*And2_it);
        OC_ASSERT(asw->getType(And1) == AND_LINK);
        OC_ASSERT(asw->getType(And2) == AND_LINK);
        OC_ASSERT(And1_it.number_of_children() == 2); // @todo generilize for n-ary
        OC_ASSERT(And2_it.number_of_children() == 2); // @todo generilize for n-ary
        pHandleSeq out1, out2; // set of potential contexts, it
                               // assumes there are Nodes, which might
                               // be too constraining
        for(vtree::sibling_iterator sib = And1_it.begin();
            sib != And1_it.end(); sib++) {
            pHandle on_ph = _v2h(*sib);
            OC_ASSERT(asw->isSubType(on_ph, NODE));
            out1.push_back(on_ph);
        }
        for(vtree::sibling_iterator sib = And2_it.begin();
            sib != And2_it.end(); sib++) {
            pHandle on_ph = _v2h(*sib);
            OC_ASSERT(asw->isSubType(on_ph, NODE));
            out2.push_back(on_ph);
        }
        // the intersection btw out1 and out2 will be the new context
        pHandleSeq inter = set_intersection(out1, out2);
        OC_ASSERT(inter.size() == 1); // @todo generilize for n-ary
        pHandle CX = *inter.begin();
        pHandleSeq new_out1 = set_difference(out1, inter); // can be optimized
        pHandleSeq new_out2 = set_difference(out2, inter); // can be optimized
        OC_ASSERT(new_out1.size() == 1); // @todo generilize for n-ary
        pHandle A = new_out1[0];
        OC_ASSERT(new_out2.size() == 1); // @todo generilize for n-ary
        pHandle B = new_out2[0];
        res = new BoundVTree(mva((pHandle)CONTEXT_LINK, mva(CX),
                                 mva((pHandle)asw->getType(ph),
                                     mva(A), mva(B))));
    }
    
    return makeSingletonSet(Rule::MPs(1, BBvtree(res)));
}
        
meta DecontextualizerRule::targetTemplate() const {
    return(meta(new vtree(mva((pHandle)LINK,
                              vtree(CreateVar(asw)),
                              vtree(CreateVar(asw))))));
}

}} // namespace opencog { namespace pln {
