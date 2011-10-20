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

Rule::setOfMPs CustomCrispUnificationRuleComposer::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    if (asw->isSubType(_v2h(*outh->begin()), FORALL_LINK) ||
        asw->isSubType(_v2h(*outh->begin()), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(vtree(outh->begin()));
#endif

    Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, ForallLink, asw);

    if (!i)
        return Rule::setOfMPs();

    setOfMPs ret;
    
    MPs ret1;
    typedef std::pair<pHandle,vtree> phvt;
    DeclareBtr(bindingsT, pre_binds);

    foreach(phvt vp, *i->bindings)
        (*pre_binds)[vp.first] = make_real(vp.second);

    ret1.push_back(BBvtree(new BoundVTree(mva(i->original_handle), pre_binds)));

    BBvtree rootAtom(new BoundVTree(mva((pHandle)HYPOTHETICAL_LINK, *i), pre_binds));

    ret1.push_back(rootAtom);

    for_each(ret1.begin(), ret1.end(), boost::bind(&bind_Bvtree, _1, *i->bindings));

    ret.insert(ret1);

    overrideInputFilter = true;
    
cprintf(3,"Crispu.o2i: OK! Solution vector size=%u\n", (uint) ret.size());
    
    return ret;
}

}} // namespace opencog { namespace pln {
