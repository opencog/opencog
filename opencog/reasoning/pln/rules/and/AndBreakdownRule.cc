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

/*setOfMPs AndBreakdownRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
{
    boost::shared_ptr<MPs> ret(new MPs);

    MPs->push_back(
}*/

Rule::setOfMPs AndBreakdownRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    /// And parameters aren't ordered. Therefore, the order in which we feed them
    /// here is irrelavent. But we need a hypothetical parameter that will later remind
    /// which kind of atom we need to produce.

    ///haxx:: (would also ignore nested AndLinks)
    // Mainly to decrease the combinatorial explosion by not producing infinite
    // series of AndBDRules within the BIT.
    if (asw->isSubType(_v2h(*outh->begin()), AND_LINK) ||
        // this one should probably be done more generally (for most rules)
        asw->isSubType(_v2h(*outh->begin()), HYPOTHETICAL_LINK))
        return Rule::setOfMPs();

    MPs ret;
    BBvtree andlink(new BoundVTree);

    andlink->set_head(Vertex((pHandle)AND_LINK));

    andlink->append_child(andlink->begin(), outh->begin());

    for (uint i = 1; i < N; i++)
        andlink->append_child(andlink->begin(),
                              BoundVTree(CreateVar(asw)).begin());

    ret.push_back(andlink);
    ret.push_back(BBvtree(new BoundVTree(mva((pHandle)HYPOTHETICAL_LINK,*outh))));

    overrideInputFilter = true;

    return makeSingletonSet(ret);
}

BoundVertex AndBreakdownRule::compute(const VertexSeq& premiseArray,
                    pHandle CX, bool fresh) const
{
    std::vector<pHandle> hs = asw->getOutgoing(boost::get<pHandle>(premiseArray[0]));

    assert(premiseArray.size() == 2);
    assert(hs.size() == N);
    assert(asw->getArity(boost::get<pHandle>(premiseArray[1])) == 1);

    atom topological_model(asw->getOutgoing(boost::get<pHandle>(premiseArray[1]))[0]);

    for (uint i = 0; i < hs.size(); i++)
        if (atom(hs[i]) == topological_model) {
            pHandle a = hs[i];
            TruthValue* resultTV;
            TVSeq tvs(1, asw->getTV(_v2h(premiseArray[0]))); /// @todo why?
            resultTV = formula.compute(tvs);

            //! @todo More pHandles messiness.
            Handle realHandle = asw->fakeToRealHandle(a).first;
            asw->getAtomSpace()->setTV(realHandle, *resultTV);
            //asw->addAtom(mva(hs[i]), *tv, false);

            delete resultTV;

            return BoundVertex(a);
        }

/*LOG(0,"Topo model was:");
      printAtomTree(topological_model,0,0);
      LOG(0,"hs:");
      for (uint i = 0; i < hs.size(); i++)
      printTree(hs[i],0,0);*/

//LOG(0, "AndBREAKDOWN: NO TOPOLOGICAL MODEL FOUND!");
    assert(0);

    return Vertex((pHandle)PHANDLE_UNDEFINED);

    /*  std::vector<Handle> hs = asw->getOutgoing(premiseArray[0]);

assert(premiseArray.size() == 2);
assert(hs.size() == N);
assert(asw->getArity(premiseArray[1]) == 1);

tree<Vertex>::iterator top0 = premiseArray[0].begin();

tree<Vertex> topological_model(premiseArray[1])[0]
tree<Vertex>::iterator topological_model = (premiseArray[1])[0];

for (int i = 0; i < hs.size(); i++)
for (tree<Vertex>::sibling_iterator i = premiseArray[0].begin();
        i != premiseArray[0].end(); i++)
    if (*i == topological_model)
        return *i;

//LOG(0, "ANDBREAKDOWN: NO TOPOLOGICAL MODEL FOUND!");
assert(0);*/
}


}} // namespace opencog { namespace pln {
