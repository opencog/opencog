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

Rule::setOfMPs OrPartitionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    return PartitionRule_o2iMetaExtra(outh, overrideInputFilter, OR_LINK, asw);
        
/*      if (!inheritsType(nm->getType(v2h(*outh->begin())), OR_LINK) ||
        outh->begin().number_of_children() <= 2)
        return Rule::setOfMPs();

** @todo Update to BoundVTree. I no longer remember how this was supposed to work!
        MPs ret;

        //vector<atom> hs;
//          hs.push_back(out_hs[0]);

        vector<atom>::iterator bigstart = out_hs.begin();
        vector<atom> hs2(++bigstart, out_hs.end());
//      hs.push_back(atom(OR_LINK, hs2));

        ret.push_back(BBvtree(new BoundVTree(atom(out_hs[0]).maketree()));
        ret.push_back(BBvtree(new BoundVTree(atom(OR_LINK, hs2).maketree()));

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);*/
}

BoundVertex OrPartitionRule::compute(const VertexSeq& premiseArray,
                                     pHandle CX, bool fresh) const
{
/*  Handle *hs = new Handle[premiseArray.size()];
    transform(premiseArray.begin(), premiseArray.end(), hs[0], DropVertexBindings()); //mem_fun(
    const int n = premiseArray.size();*/

    BoundVertex ret = regularOr->compute(premiseArray, CX, fresh);
//  delete[] hs;
    return ret;
}

}} // namespace opencog { namespace pln {
