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
    map<string, map<Handle, Handle> > scholemFunctions;
    extern bool AllowFW_VARIABLENODESinCore;
    reasoning::BITNodeRoot* bitnoderoot;
    
    /// \todo This data must persist even if the BITNodeRoot is deleted.
    //extern map<Handle,vector<Handle> > inferred_from;
    //extern map<Handle,reasoning::Rule*> inferred_with;
}

namespace reasoning
{

Rule::setOfMPs ScholemFunctionProductionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    return Rule::setOfMPs();
}

/** Won't produce EXISTING Scholem function mappings! Thus, run lookupRule 1st!
    Scholem links are not memory-managed.
*/

boost::shared_ptr<set<BoundVertex > > ScholemFunctionProductionRule::attemptDirectProduction(meta outh)
{
    boost::shared_ptr<set<BoundVertex > > ret;
    
    if (!GET_ATW->inheritsType(GET_ATW->getType(_v2h(*outh->begin())), SCHOLEM_LINK))
        return ret;

    //assert(outh->begin().number_of_children() == 2);
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    if (outh->begin().number_of_children() != 2)
    {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) with != 2 args.");
        printer.print(_v2h(*outh->begin()), 2);
    }

    tree<Vertex> old_subst(*outh);
    
    tree<Vertex>::sibling_iterator child2 = old_subst.begin(old_subst.begin());
    tree<Vertex>::sibling_iterator child1 = child2++;

#ifdef _MSC_VER
#pragma warning(Remove this hack ASAP!)
#endif // _MSC_VER
//#warning "Remove this hack ASAP!"
haxx::AllowFW_VARIABLENODESinCore = true;
    
//  assert(!inheritsType(nm->getType(v2h(*child1)), VARIABLE_NODE));
//  assert(!inheritsType(nm->getType(v2h(*child2)), VARIABLE_NODE));
    if (GET_ATW->inheritsType(GET_ATW->getType(_v2h(*child1)), VARIABLE_NODE))
    {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) child1 problem.");
        printer.print(_v2h(*child1), 2);
    }
    
    if (GET_ATW->inheritsType(GET_ATW->getType(_v2h(*child2)), VARIABLE_NODE))
    {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) child2 problem.");
        printer.print(_v2h(*child2), 2);
    }

    
    *child2 = CreateVar(destTable);
    
    TableGather s(old_subst, destTable);

    if (s.empty())
    {
        ret = boost::shared_ptr<set<BoundVertex > >(new set<BoundVertex>);
        ret->insert(BoundVertex(Vertex(destTable->addAtom(*atomWithNewType(*outh, SCHOLEM_LINK),
                TruthValue::TRUE_TV(),
                false, false))));
        
        return ret;
    }

haxx::AllowFW_VARIABLENODESinCore = false;

    /// This Rule shouldn't be used to produce EXISTING Scholem function mappings!
    
	LOG(3, "Tried to re-map a scholem function argument.");
    
    return ret;

/*  map<string, map<Handle, Handle> >::iterator f = haxx::scholemFunctions.find(outh.name);
    if (f != scholemFunctions.end())
    {
        Handle arg1 = outh.hs[0].attach(destTable);

        map<Handle, Handle> old_subst = f->second.find(arg1);

        if (old_subst != haxx::scholemFunctions.end())
        {
            if (atom(old_subst->second) == outh)
                return 
        }
        else
        {
            f->second[arg1] = outh.hs[1].attach(destTable);
        }
    }
    else
        haxx::scholemFunctions*/
}

} // namespace reasoning
