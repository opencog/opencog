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
    asw = static_cast<AtomSpaceWrapper*>(_destTable);
    /*inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N,
                                             1,
                                             new atom(CONCEPT_NODE, 0))));*/
}

BoundVertex SubsetEvalRule::compute(const vector<Vertex>& premiseArray,
                                    pHandle CX) const
{
    OC_ASSERT(premiseArray.size() == 2);

    pHandle h_sub = _v2h(premiseArray[0]);
    pHandle h_super = _v2h(premiseArray[1]);

    OC_ASSERT(asw->isSubType(h_sub, CONCEPT_NODE));
    OC_ASSERT(asw->isSubType(h_super, CONCEPT_NODE));

    pHandleSet used;

    pHandleSet satSetSub = constitutedSet(h_sub, 0.0f, 1, asw);
    pHandleSet satSetSuper = constitutedSet(h_super, 0.0f, 1, asw);

    vector<TruthValue*> tvsSub;
    vector<TruthValue*> tvsSuper;

    // We fill tvsSub and tvsSuper by padding the missing TVs with zeros
    // so that each element of tvsSub and tvsSuper are aligned

    foreach(const pHandle& h_el_sub, satSetSub) {
        std::cout << asw->pHandleToString(h_el_sub) << std::endl;
        tvsSub.push_back(asw->getTV(h_el_sub).clone());
        pHandleSetConstIt h_el_super_cit =
            find<pHandleSetConstIt, pHandle>(satSetSuper.begin(),
                                             satSetSuper.end(),
                                             h_el_sub);
        if (h_el_super_cit == satSetSuper.end())
            tvsSuper.push_back(new SimpleTruthValue(0, 0));
        else {
            tvsSuper.push_back(GET_ASW->getTV(*h_el_super_cit).clone());
            used.insert(*h_el_super_cit);
        }
    }
    
    foreach(const pHandle& h_el_super, satSetSuper) {
        if (!STLhas(used, h_el_super)) {
            tvsSuper.push_back(GET_ASW->getTV(h_el_super).clone());
            tvsSub.push_back(new SimpleTruthValue(0, 0));
        }
    }

    assert(tvsSub.size() == tvsSuper.size());

    unsigned int N = tvsSub.size();

    TruthValue** tvs1 = new TruthValue*[N];
    TruthValue** tvs2 = new TruthValue*[N];

    std::cout << "SUB" << std::endl;
    for(unsigned int i = 0; i < N; i++) {
        std::cout << tvsSub[i]->toString() << std::endl;
    }
    std::cout << "SUPER" << std::endl;
    for(unsigned int i = 0; i < N; i++) {
        std::cout << tvsSuper[i]->toString() << std::endl;
    }

    std::copy(tvsSub.begin(), tvsSub.end(), tvs1);
    std::copy(tvsSuper.begin(), tvsSuper.end(), tvs2);

    TruthValue* retTV = formula.compute2(tvs1, (int)N, tvs2, (int)N);

    for (unsigned int i = 0; i < N; i++) {
        delete tvs1[i];
        delete tvs2[i];
    }

    delete tvs1;
    delete tvs2;

    pHandle ret = asw->addLink(SUBSET_LINK,
                               _v2h(premiseArray[0]),
                               _v2h(premiseArray[1]),
                               *retTV, true);
    return BoundVertex(ret);
}

}} // namespace opencog { namespace pln {
