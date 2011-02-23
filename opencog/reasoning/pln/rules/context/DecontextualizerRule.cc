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

meta DecontextualizerRule::i2oType(const VertexSeq& h) const {
    OC_ASSERT(h.size() == 1);
    pHandle ph = _v2h(h[0]);
    OC_ASSERT(asw->isSubType(ph, CONTEXT_LINK));
    OC_ASSERT(asw->getArity(ph) == 2);
    pHandle CX = asw->getOutgoing(ph,0);
    pHandle A = asw->getOutgoing(ph,1); // contextualized atom

    // this is for case b)
    if(asw->isSubType(A, PREDICATE_NODE) ||
       asw->isSubType(A, EVALUATION_LINK)) {
        /// @todo this might be decomposed into subrule
        A = asw->addLink(SATISFYING_SET_LINK, pHandleSeq(1, A),
                         *asw->getTV(A), false);
    }
    // case a)
    // @note it is missing the cases where concepts are AND_LINK and such
    // for that we need some sort of type checker
    if(asw->isSubType(A, CONCEPT_NODE) ||
       asw->isSubType(A, SATISFYING_SET_LINK)) {
        return meta(new vtree(mva((pHandle)SUBSET_LINK, mva(CX), mva(A))));
    } else { // case c) Relation
        OC_ASSERT(asw->getArity(A)==2); /// @todo generilize for n-ary
        pHandle AC1 = asw->getOutgoing(A,0);
        pHandle AC2 = asw->getOutgoing(A,1);
        return meta(new vtree(mva((pHandle)asw->getType(A),
                                  mva((pHandle)AND_LINK, mva(CX), mva(AC1)),
                                  mva((pHandle)AND_LINK, mva(CX), mva(AC2)))));
    }

}

TVSeq DecontextualizerRule::formatTVarray(const VertexSeq& premiseArray) const {
    OC_ASSERT(premiseArray.size()==1);
    return TVSeq(1, asw->getTV(_v2h(premiseArray[0])));
}

DecontextualizerRule::DecontextualizerRule(AtomSpaceWrapper* _asw)
    : GenericRule<IdentityFormula>(_asw, false, "DecontextualizerRule") {
    inputFilter.push_back(meta(new vtree(mva((pHandle)CONTEXT_LINK,
                                             mva((pHandle)ATOM),
                                             mva((pHandle)ATOM)))));
}

Rule::setOfMPs DecontextualizerRule::o2iMetaExtra(meta outh,
                                                  bool& overrideInputFilter) const {
    vtree::iterator root = outh->begin();
    pHandle ph = _v2h(*root);

    if(asw->isSubType(ph, NODE) ||
       root.number_of_children() != 2) // maybe this should be relaxed
        return Rule::setOfMPs();

    BoundVTree* res;

    vtree::iterator A1_it = root.begin();
    pHandle A1 = _v2h(*A1_it);
    vtree::iterator A2_it = root.last_child();
    pHandle A2 = _v2h(*A2_it);

    // case c)
    if(asw->getType(A1) == AND_LINK && asw->getType(A2) == AND_LINK) {
        // @todo generilize for n-ary
        if (A1_it.number_of_children() != 2 ||
                A2_it.number_of_children() != 2)
            return Rule::setOfMPs();

        pHandleSeq out1, out2; // set of potential contexts
        for(vtree::sibling_iterator sib = A1_it.begin();
            sib != A1_it.end(); sib++) {
            out1.push_back(_v2h(*sib));
        }
        for(vtree::sibling_iterator sib = A2_it.begin();
            sib != A2_it.end(); sib++) {
            out2.push_back(_v2h(*sib));
        }
        // the intersection btw out1 and out2 will be the new context
        pHandleSeq inter = set_intersection(out1, out2);

        if(inter.size() > 0) {
            if(inter.size() != 1) /// @todo generalize for n-ary
                return Rule::setOfMPs();
            pHandle CX = *inter.begin();
            // can be optimized
            pHandleSeq new_out1 = set_difference(out1, inter);
            pHandleSeq new_out2 = set_difference(out2, inter);
            /// @todo generalize for n-ary
            if(new_out1.size() != 1 || new_out2.size())
                return Rule::setOfMPs();
            pHandle A = new_out1[0];
            pHandle B = new_out2[0];
            res = new BoundVTree(mva((pHandle)CONTEXT_LINK, mva(CX),
                                     mva((pHandle)asw->getType(ph),
                                         mva(A), mva(B))));
            makeSingletonSet(Rule::MPs(1, BBvtree(res)));
        }
    }

    // otherwise case a) and b)
    if(asw->isSubType(ph, SUBSET_LINK)) {
        if(asw->isSubType(A2, SATISFYING_SET_LINK)) // case b)
            A2 = _v2h(*A2_it.begin());
        res = new BoundVTree(mva((pHandle)CONTEXT_LINK, mva(A1), mva(A2)));
        return makeSingletonSet(Rule::MPs(1, BBvtree(res)));
    } else // no case applies
        return Rule::setOfMPs();
}
        
meta DecontextualizerRule::targetTemplate() const {
    return(meta(new vtree(mva((pHandle)LINK,
                              vtree(CreateVar(asw)),
                              vtree(CreateVar(asw))))));
}

}} // namespace opencog { namespace pln {
