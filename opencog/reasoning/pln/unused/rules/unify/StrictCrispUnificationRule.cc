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

/***

StrictCrispUnificationRule has essentially been obsoleted by CustomCrispUnificationRule,
but we keep it here because its fundamentally different approach.
StrictCrispUnificationRule is a single stand-alone Rule, whereas Custom versions work so that
for each quantified atom struct, there'll be a new associated Custom Rule.

For StrictCrispUnificationRule:

Problem: StrictCrispUnificationRule has to bind FW_VARs, but it uses a design hack
which gathers hints from Atom Table instead of being pure o2i. Hence, it will
have to make var bindings, but it cannot return the information about them in
any way!

Solutions:
a) Do not bind FW_VARs. o2i returns the ForAllLink and the internals as-is,
containing the FW_VARs as they were passed to o2i from above.
BUT: FW_VARs were bound in a specific way in order to make StrictCrispUnificationRule
work. If they are left unbound, then they may be bound elsewhere in a way
which prevents StrictCrispUnificationRule from working, without our being able
to know this.

b) DO bind FW_VARs. 
BUT: Since we use direct production, bindings cannot be returned.
We can apply the bindings immediately to the ForAllLink's internals,
but then we have made local bindings without being able to pass knowledge of
that to other processes. Hence, if $1 (FW_VAR) was bound here in a certain way,
it can be bound on another branch different way, and if these branches are
combined, no conflict will be found!

*/

Rule::setOfMPs StrictCrispUnificationRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    if (asw->isSubType(_v2h(*outh->begin()), FORALL_LINK) ||
        asw->isSubType(_v2h(*outh->begin()), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(outh->begin());
#endif

    Btr< std::set<Btr<ModifiedBoundVTree> > > varforms = FindMatchingUniversals(outh, asw);
    
///return Rule::setOfMPs();

    if (varforms->empty())
        return Rule::setOfMPs();

    setOfMPs ret;
    
    foreach(Btr<ModifiedBoundVTree> i, *varforms)
    {
        MPs ret1;

        typedef std::pair<pHandle,vtree> phvt;
        DeclareBtr(bindingsT, pre_binds);

        foreach(phvt vp, *i->bindings)
        {
            (*pre_binds)[vp.first] = make_real(vp.second);
        }

        ret1.push_back(BBvtree(new BoundVTree(mva(i->original_handle), pre_binds)));

cprintf(3,"And formm:\n");
#if 0
rawPrint(*ret1[0], ret1[0]->begin(),3);
#else 
printer.print(ret1[0]->begin(),3);
#endif

        BBvtree rootAtom(new BoundVTree(mva((pHandle)HYPOTHETICAL_LINK, *i), pre_binds));

cprintf(3,"Root:\n");
#if 0
rawPrint(*rootAtom, rootAtom->begin(),3);
#else 
printer.print(rootAtom->begin(),3);
#endif

        ///Record targets to prevent multiple occurrence of the same target in the arg set.

        //set<vtree, less_vtree> arg_targets;
        std::set<vtree, less_vtree> arg_targets;

        /// rootAtom->hs[0] is the 1st link under the HYPOTHETICAL_LINK
        /// rootAtom->hs[0].hs[0] is the 1st link that this Rule does NOT prove

        for (vtree::sibling_iterator sit = i->begin(i->begin()); sit != i->end(i->begin()); sit++)
        {
            vtree tRes(sit);

            for (vtree::post_order_iterator pit = tRes.begin_post(); pit!=tRes.end_post(); pit++)
			{
                vtree pit_vtree(pit);
                if ( (asw->isType(_v2h(*pit))
                    || asw->isSubType(_v2h(*pit), FW_VARIABLE_NODE))
                    && _v2h(*pit) != (pHandle)LIST_LINK
					&& !STLhas(arg_targets, pit_vtree) )
                {
					ret1.push_back(BBvtree(new BoundVTree(pit_vtree, pre_binds)));

					arg_targets.insert(pit_vtree);
                }
            }
        }

        ret1.push_back(rootAtom);

        for_each(ret1.begin(), ret1.end(),
                 boost::bind(&bind_Bvtree, _1, *i->bindings));

/*MPs::iterator m = ret1.begin();
for(; m != ret1.end(); m++)
    if (*m && (*m)->begin() != (*m)->end())
        printer.print((*m)->begin(),3);*/
        
        ret.insert(ret1);
    }

///     atom* Andform = new atom(AND_LINK, 2,
/// new atom(__INSTANCEOF_N, 1, new atom(FORALL_LINK,0)),
/// new atom(__INDEX2, 1, Orform)

    overrideInputFilter = true;
    
cprintf(3,"Crispu.o2i: OK! Solution vector size=%u\n", (uint) ret.size());
    
    return ret;
}

}} // namespace opencog { namespace pln {
