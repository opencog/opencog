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

#include "SubsetEvalRule.h"

#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace opencog { namespace pln {

using std::set;
using std::find;
using std::find_if;

const strength_t MIN_MEMBERS_STRENGTH = 0.000001;
const strength_t MIN_MEMBERS_COUNT = 1;

SubsetEvalRule::SubsetEvalRule(AtomSpaceWrapper* asw, Type argType)
    : Rule(asw, false, true, "SubsetEvalRule"), ArgType(argType), _asw(asw)
{
    if (argType != CONCEPT_NODE) {
        // If we are not using ATOM type, then we need to insure a unique name
        name += classserver().getTypeName(argType);
    }
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ArgType))));
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ArgType))));
}


Rule::setOfMPs SubsetEvalRule::o2iMetaExtra(meta outh,
                                            bool& overrideInputFilter) const {
    if(!_asw->isSubType(_v2h(*outh->begin()), SUBSET_LINK))
        return setOfMPs();

    tree<Vertex>::iterator head_it = outh->begin();

    Rule::MPs ret;
    // Return the two arguments; if they're ATOM, make them more specific.
    if (_v2h(*outh->begin(head_it)) == ATOM)
        ret.push_back(BBvtree(new BoundVTree(mva((pHandle)ArgType))));
    else
        ret.push_back(BBvtree(new BoundVTree(outh->begin(head_it))));

    if (_v2h(*outh->last_child(head_it)) == ATOM)
        ret.push_back(BBvtree(new BoundVTree(mva((pHandle)ArgType))));
    else
        ret.push_back(BBvtree(new BoundVTree(outh->last_child(head_it))));

    overrideInputFilter = true;

    return makeSingletonSet(ret);
}

meta SubsetEvalRule::i2oType(const VertexSeq& h_vec) const
{
    OC_ASSERT(h_vec.size()==2);
    return meta(new tree<Vertex>(mva((pHandle)SUBSET_LINK,
                                     vtree(h_vec[0]),
                                     vtree(h_vec[1])
                                     )));
}

TVSeq SubsetEvalRule::formatTVarray(const VertexSeq& premises) const
{
    OC_ASSERT(premises.size() == 2);

    pHandle sub_h = _v2h(premises[0]);
    pHandle super_h = _v2h(premises[1]);

//    OC_ASSERT(_asw->isSubType(sub_h, CONCEPT_NODE));
//    OC_ASSERT(_asw->isSubType(super_h, CONCEPT_NODE));

    pHandleSet used;

    pHandleSet mlSetSub = memberLinkSet(sub_h, MIN_MEMBERS_STRENGTH,
                                        MIN_MEMBERS_COUNT, _asw);
    pHandleSet mlSetSuper = memberLinkSet(super_h, MIN_MEMBERS_STRENGTH,
                                          MIN_MEMBERS_COUNT, _asw);

    TVSeq tvsSub, tvsSuper;

    // We fill tvsSub and tvsSuper by padding the missing TVs with zeros
    // so that each element of tvsSub and tvsSuper are aligned

    foreach(const pHandle& h_ml_sub, mlSetSub) {
        //std::cout << _asw->pHandleToString(h_ml_sub) << std::endl;
        tvsSub.push_back(_asw->getTV(h_ml_sub));

        EqOutgoing eqMember(h_ml_sub, 0, _asw);
        pHandleSetConstIt h_ml_super_cit =
            find_if(mlSetSuper.begin(), mlSetSuper.end(), eqMember);
        if (h_ml_super_cit == mlSetSuper.end())
            tvsSuper.push_back(TruthValuePtr(new SimpleTruthValue(0, 0)));
        else {
            tvsSuper.push_back(_asw->getTV(*h_ml_super_cit));
            used.insert(*h_ml_super_cit);
        }
    }

    foreach(const pHandle& h_ml_super, mlSetSuper) {
        //std::cout << _asw->pHandleToString(h_ml_super) << std::endl;
        if (!STLhas(used, h_ml_super)) {
            tvsSuper.push_back(_asw->getTV(h_ml_super));
            tvsSub.push_back(TruthValuePtr(new SimpleTruthValue(0, 0)));
        }
    }

    assert(tvsSub.size() == tvsSuper.size());

    TVSeq tvs(tvsSub.begin(), tvsSub.end());
    tvs.insert(tvs.end(), tvsSuper.begin(), tvsSuper.end());
    return tvs;
}


BoundVertex SubsetEvalRule::compute(const VertexSeq& premiseArray,
                                    pHandle CX,
                                    bool fresh) const
{
    TVSeq tvs = formatTVarray(premiseArray);

    TruthValue* retTV = formula.compute(tvs);

    pHandle ret = _asw->addAtom(*i2oType(premiseArray), *retTV, fresh);
    return BoundVertex(ret);
}

}} // namespace opencog { namespace pln {
