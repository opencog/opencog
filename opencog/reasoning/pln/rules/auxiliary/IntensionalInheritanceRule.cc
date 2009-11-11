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

#include <boost/assign/list_of.hpp>

namespace opencog { namespace pln {

using boost::assign::list_of;

IntensionalInheritanceRule::IntensionalInheritanceRule(AtomSpaceWrapper* asw)
    : Rule(asw, false, false, "IntensionalInheritanceRule"),
      _asw(asw), _sser(asw)
{
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
                                                pHandle CX) const
{
    OC_ASSERT(premiseArray.size() == 2);

    pHandle sub_h = _v2h(premiseArray[0]);
    pHandle super_h = _v2h(premiseArray[1]);

    OC_ASSERT(_asw->isSubType(sub_h, CONCEPT_NODE));
    OC_ASSERT(_asw->isSubType(super_h, CONCEPT_NODE));

    pHandle sub_ASSOC_h = CreateConceptASSOC(_asw, sub_h);
    pHandle super_ASSOC_h = CreateConceptASSOC(_asw, super_h);

    const vector<Vertex> ASSOC_vv = list_of(sub_ASSOC_h)(super_ASSOC_h);
    BoundVertex bv = _sser.compute(ASSOC_vv);
    pHandle subset_ASSOC_h = _v2h(bv.GetValue());

    pHandle ret = _asw->addAtom(*i2oType(premiseArray),
                                _asw->getTV(subset_ASSOC_h), true);
    return BoundVertex(ret);
}

}} // namespace opencog { namespace pln {

