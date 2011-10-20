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

Rule::setOfMPs AndPartitionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    return PartitionRule_o2iMetaExtra(outh, overrideInputFilter, AND_LINK, asw);
}

BoundVertex AndPartitionRule::compute(const VertexSeq& premiseArray,
                                      pHandle CX,
                                      bool fresh) const
{
    const int N = (int)premiseArray.size();
    pHandle *hs = new pHandle[N];
        
    transform(premiseArray.begin(), premiseArray.end(), &hs[0], GetHandle()); //mem_fun(
    //          Concat<DropVertexBindings, GetHandle, BoundVertex, Handle>());

    BoundVertex ret = Vertex(UnorderedCcompute(asw, AND_LINK, fN,
                                                   hs, N, CX, fresh));
    delete[] hs;
    return ret;
}

}} // namespace opencog { namespace pln {
