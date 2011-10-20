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

BoundVertex BaseCrispUnificationRule::compute(const VertexSeq& premiseArray, pHandle CX, bool fresh) const
{
    const int n = premiseArray.size();
    
	cprintf(4, "BaseCrispUnificationRule::compute:");
#if 0    
    for (int i=0;i<n;i++)
        printTree(v2h(premiseArray[i]), 0, 4);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    for (int i=0;i<n;i++)
        printer.print(_v2h(premiseArray[i]), 4);
#endif
    
    assert(asw->getType(_v2h(premiseArray[0])) == FORALL_LINK);
    assert(asw->getType(_v2h(premiseArray[n-1])) == HYPOTHETICAL_LINK);

    pHandle topologicalStub = asw->getOutgoing(_v2h(premiseArray[n-1]))[0];

    TruthValuePtr tv = asw->getTV(_v2h(premiseArray[0]));
    
    pHandle ret = asw->addLink(asw->getType(topologicalStub),
                              asw->getOutgoing(topologicalStub),
                              *tv, fresh);   

    return Vertex(ret);
}

}} // namespace opencog { namespace pln {
