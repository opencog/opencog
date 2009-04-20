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

namespace reasoning
{
    /*
SubsetEvalRule::SubsetEvalRule(iAtomSpaceWrapper *_destTable)
{
    inputFilter.push_back(boost::shared_ptr<atom>(new atom(__INSTANCEOF_N, 1, new atom(CONCEPT_NODE, 0))));
}

Handle SubsetEvalRule::compute(const vector<Vertex>& premiseArray,
                               Handle CX) const
{
    assert(n == 2);

    assert(inheritsType(nm->getType(premiseArray[0]), CONCEPT_NODE));
    assert(inheritsType(nm->getType(premiseArray[1]), CONCEPT_NODE));

    vector<Handle> set1;
    vector<Handle> set2;
    set<Handle> used;

    set1 = constitutedSet(premiseArray[0], 0.0f, 1);
    set2 = constitutedSet(premiseArray[1], 0.0f, 1);

    TruthValue** tvs1 = new SimpleTruthValue*[set1.size()+set2.size()];
    TruthValue** tvs2 = new SimpleTruthValue*[set1.size()+set2.size()];

    int i = 0, tot = 0;

    // We pad the missing TVs with zeros:

    for (i = 0; i < set1.size(); i++) {
        tvs1[i] = getTruthValue(set1[i]);
        vector<Handle>::iterator h2 =
            find<vector<Handle>::iterator, Handle>(set2.begin(),
                                                   set2.end(),
                                                   set1[i]);
        if (h2 == set2.end())
            tvs2[i] = new SimpleTruthValue(0, 0);
        else {
            tvs2[i] = getTruthValue(*h2)->clone();
            used.insert(*h2);
        }
    }
    tot = i;

    for (i = 0; i < set2.size(); i++)
        if (!STLhas(used, set2[i])) {
            tvs2[tot] = getTruthValue(set2[i])->clone();
            tvs1[tot] = new SimpleTruthValue(0, 0);
            tot++;
        }

    TruthValue* retTV = f.compute(tvs1, tot, tvs2, tot);

    for (i = 0; i < set1.size(); i++)
        delete tvs1[i];

    for (i = 0; i < set2.size(); i++)
        delete tvs2[i];

    delete tvs1;
    delete tvs2;

    vector<Handle> hs;
    hs.push_back(premiseArray[0]);
    hs.push_back(premiseArray[1]);

    Handle ret = destTable->addLink(SUBSET_LINK,
                                    hs,
                                    retTV,
                                    true);
//                              false);

    return ret;
}
    */
} // namespace reasoning
