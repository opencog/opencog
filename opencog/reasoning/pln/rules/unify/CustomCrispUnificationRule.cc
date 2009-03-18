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

namespace haxx
{
    /// \todo This data must persist even if the BITNodeRoot is deleted.
    extern map<pHandle,vector<pHandle> > inferred_from;
    extern map<pHandle,reasoning::Rule*> inferred_with;
}

namespace reasoning
{

bool UnificationRuleResultFreshness = true; //false;

Btr<set<BoundVertex > > CustomCrispUnificationRule::attemptDirectProduction(meta outh)
{
    if (GET_ATW->inheritsType(GET_ATW->getType(_v2h(*outh->begin())), FORALL_LINK) ||
        GET_ATW->inheritsType(GET_ATW->getType(_v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Btr<set<BoundVertex > >();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(vtree(outh->begin()));
#endif
cprintf(3,"FindMatchingUniversals...\n");
    Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, ForallLink, destTable);
cprintf(3,"FindMatchingUniversals OK!\n");
    if (!i)
        return Btr<set<BoundVertex > >();

    Btr<set<BoundVertex > > ret(new set<BoundVertex >);
    
    MPs ret1;
    typedef pair<pHandle,vtree> phvt;
    DeclareBtr(bindingsT, pre_binds);

    foreach(phvt vp, *i->bindings)
    {
        (*pre_binds)[vp.first] = make_real(vp.second);
#if 0    
        printTree(vp.first,0,0);
        cprintf(0,"=");
        printTree((*pre_binds)[vp.first],0,0);
#else 
        printer.print(vp.first);
        cprintf(0,"=");
        printer.print((*pre_binds)[vp.first]);
#endif
    }

    BBvtree rootAtom(new BoundVTree(*i, pre_binds));
    bind_Bvtree(rootAtom, *i->bindings);
    pHandle topologicalStub = destTable->addAtom(*rootAtom, TruthValue::TRIVIAL_TV(), false, true);

    pHandle ret_h = destTable->addLink(  GET_ATW->getType(topologicalStub),
                                GET_ATW->getOutgoing(topologicalStub),
                                GET_ATW->getTV(i->original_handle),
                                UnificationRuleResultFreshness);    
    
    ret->insert(BoundVertex(ret_h, pre_binds));

/*  haxx::bitnoderoot->inferred_with[ret_h] = (Rule*)(int)this;
    if (haxx::bitnoderoot->inferred_from[ret_h].empty()) //comes here often, we want the link only once
        haxx::bitnoderoot->inferred_from[ret_h].push_back(ForallLink);
*/
    haxx::inferred_with[ret_h] = (Rule*)this;
    if (haxx::inferred_from[ret_h].empty()) //comes here often, we want the link only once
        haxx::inferred_from[ret_h].push_back(ForallLink);

    return ret;
}

} // namespace reasoning
