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

//#include "../Rule.h"
//#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace opencog { namespace pln {

using std::set;
using std::find;

SubsetEvalRule::SubsetEvalRule(iAtomSpaceWrapper *_destTable)
    : Rule(_destTable, false, false, "SubsetEvalRule")
{
    /*inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N,
                                             1,
                                             new atom(CONCEPT_NODE, 0))));*/
}

BoundVertex SubsetEvalRule::compute(const vector<Vertex>& premiseArray,
                                    pHandle CX) const
{
    assert(premiseArray.size() == 2);

    assert(GET_ASW->inheritsType(GET_ASW->getType(_v2h(premiseArray[0])),
                                 CONCEPT_NODE));
    assert(GET_ASW->inheritsType(GET_ASW->getType(_v2h(premiseArray[1])),
                                 CONCEPT_NODE));

    pHandleSet used;

    pHandleSet satSetSub = constitutedSet(_v2h(premiseArray[0]), 0.0f, 1);
    pHandleSet satSetSuper = constitutedSet(_v2h(premiseArray[1]), 0.0f, 1);

    vector<TruthValue*> tvsSub;
    vector<TruthValue*> tvsSuper;

    // We fill tvsSub and tvsSuper by padding the missing TVs with zeros
    // so that each element of tvsSub and tvsSuper are aligned

    foreach(const pHandle& h_sub, satSetSub) {
        tvsSub.push_back(GET_ASW->getTV(h_sub).clone());
        pHandleSetConstIt h_super_cit =
            find<pHandleSetConstIt, pHandle>(satSetSuper.begin(),
                                             satSetSuper.end(),
                                             h_sub);
        if (h_super_cit == satSetSuper.end())
            tvsSuper.push_back(new SimpleTruthValue(0, 0));
        else {
            tvsSuper.push_back(GET_ASW->getTV(*h_super_cit).clone());
            used.insert(*h_super_cit);
        }
    }
    
    foreach(const pHandle& h_super, satSetSuper) {
        if (!STLhas(used, h_super)) {
            tvsSuper.push_back(GET_ASW->getTV(h_super).clone());
            tvsSub.push_back(new SimpleTruthValue(0, 0));
        }
    }

    assert(tvsSub.size() == tvsSuper.size());

    unsigned int N = tvsSub.size();

    TruthValue** tvs1 = new TruthValue*[N];
    TruthValue** tvs2 = new TruthValue*[N];

    std::copy(tvsSub.begin(), tvsSub.end(), tvs1);
    std::copy(tvsSuper.begin(), tvsSuper.end(), tvs2);

    TruthValue* retTV = formula.compute2(tvs1, (int)N, tvs2, (int)N);

    for (unsigned int i = 0; i < N; i++) {
        delete tvs1[i];
        delete tvs2[i];
    }

    delete tvs1;
    delete tvs2;

    vector<pHandle> hs;
    hs.push_back(_v2h(premiseArray[0]));
    hs.push_back(_v2h(premiseArray[1]));

    pHandle ret = NULL;/*destTable->addLink(SUBSET_LINK,
                                     hs,
                                     retTV,
                                     true);*/
    //                              false);

    return BoundVertex(ret);
}

}} // namespace opencog { namespace pln {
