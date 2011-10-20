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

namespace haxx
{
std::map<std::string, std::map<Handle, Handle> > scholemFunctions;
opencog::pln::BITNodeRoot* bitnoderoot;
}

namespace opencog { namespace pln {

Rule::setOfMPs ScholemFunctionProductionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    return Rule::setOfMPs();
}

/** Won't produce EXISTING Scholem function mappings! Thus, run lookupRule 1st!
    Scholem links are not memory-managed.
*/

Btr<std::set<BoundVertex > >
ScholemFunctionProductionRule::attemptDirectProduction(meta outh, bool fresh) const
{
    Btr<std::set<BoundVertex > > ret;

    if (!asw->isSubType(_v2h(*outh->begin()), SCHOLEM_LINK))
        return ret;

    //assert(outh->begin().number_of_children() == 2);
    NMPrinter printer(NMP_HANDLE | NMP_TYPE_NAME);
    if (outh->begin().number_of_children() != 2) {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) with != 2 args.");
        printer.print(_v2h(*outh->begin()), 2);
    }

    tree<Vertex> old_subst(*outh);

    tree<Vertex>::sibling_iterator child2 = old_subst.begin(old_subst.begin());
    tree<Vertex>::sibling_iterator child1 = child2++;

//  assert(!inheritsType(nm->getType(v2h(*child1)), VARIABLE_NODE));
//  assert(!inheritsType(nm->getType(v2h(*child2)), VARIABLE_NODE));
    if (asw->isSubType(_v2h(*child1), VARIABLE_NODE)) {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) child1 problem.");
        printer.print(_v2h(*child1), 2);
    }

    if (asw->isSubType(_v2h(*child2), VARIABLE_NODE)) {
        printer.print(outh->begin(), 2);
        LOG(2, "ScholemFunctionProductionRule::attemptDirectProduction(meta outh) child2 problem.");
        printer.print(_v2h(*child2), 2);
    }

    *child2 = CreateVar(asw);

    TableGather s(old_subst, asw);

    if (s.empty()) {
        ret = Btr<std::set<BoundVertex > >(new std::set<BoundVertex>);
        ret->insert(BoundVertex(Vertex(asw->addAtom(*atomWithNewType(*outh,
                                                                     SCHOLEM_LINK,
                                                                     asw),
                                                    TruthValue::TRUE_TV(),
                                                    fresh))));
        return ret;
    }
    /// This Rule shouldn't be used to produce EXISTING Scholem function mappings!
    LOG(3, "Tried to re-map a scholem function argument.");

    return ret;

    /*  map<string, map<Handle, Handle> >::iterator f = haxx::scholemFunctions.find(outh.name);
        if (f != scholemFunctions.end())
        {
            Handle arg1 = outh.hs[0].attach(asw);

            map<Handle, Handle> old_subst = f->second.find(arg1);

            if (old_subst != haxx::scholemFunctions.end())
            {
                if (atom(old_subst->second) == outh)
                    return
            }
            else
            {
                f->second[arg1] = outh.hs[1].attach(asw);
            }
        }
        else
            haxx::scholemFunctions*/
}

}} // namespace opencog { namespace pln {
