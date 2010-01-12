/*
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * Author Nil Geisweiller
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

#include "IntensionalInheritanceRule.h"
#include "../../ASSOC.h"
#include "../../BackInferenceTreeNode.h"

#include <boost/assign/list_of.hpp>

namespace opencog { namespace pln {

using boost::assign::list_of;

IntensionalInheritanceRule::IntensionalInheritanceRule(AtomSpaceWrapper* _asw)
    : Rule(_asw, false, true, "IntensionalInheritanceRule"), sser(_asw)
{
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)CONCEPT_NODE))));
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)CONCEPT_NODE))));
}

Rule::setOfMPs IntensionalInheritanceRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const {
    if(!asw->inheritsType((Type)_v2h(*outh->begin()),
                          INTENSIONAL_INHERITANCE_LINK))
        return setOfMPs();

    tree<Vertex>::iterator head_it = outh->begin();

    Rule::MPs ret;
    ret.push_back(BBvtree(new BoundVTree(outh->begin(head_it))));
    ret.push_back(BBvtree(new BoundVTree(outh->last_child(head_it))));
            
    overrideInputFilter = true;

    return makeSingletonSet(ret);
}

meta IntensionalInheritanceRule::i2oType(const vector<Vertex>& h_vec) const
{
    OC_ASSERT(h_vec.size()==2);
    return meta(new tree<Vertex>(mva((pHandle)INTENSIONAL_INHERITANCE_LINK, 
                                     vtree(h_vec[0]),
                                     vtree(h_vec[1])
                                     )));
}

BoundVertex IntensionalInheritanceRule::compute(const vector<Vertex>& premiseArray,
                                                pHandle CX, bool fresh) const
{
    OC_ASSERT(premiseArray.size() == 2);

    pHandle sub_h = _v2h(premiseArray[0]);
    pHandle super_h = _v2h(premiseArray[1]);

    OC_ASSERT(asw->isSubType(sub_h, CONCEPT_NODE));
    OC_ASSERT(asw->isSubType(super_h, CONCEPT_NODE));

    pHandle sub_ASSOC_h = CreateConceptASSOC(asw, sub_h);
    pHandle super_ASSOC_h = CreateConceptASSOC(asw, super_h);

    const vector<Vertex> ASSOC_vv = list_of(sub_ASSOC_h)(super_ASSOC_h);
    BoundVertex bv = sser.compute(ASSOC_vv, fresh);
    pHandle subset_ASSOC_h = _v2h(bv.GetValue());

    pHandle ret = asw->addAtom(*i2oType(premiseArray), 
                               asw->getTV(subset_ASSOC_h),
                               fresh);
    return BoundVertex(ret);
}

}} // namespace opencog { namespace pln {

