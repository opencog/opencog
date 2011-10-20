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

#if 0
atom UnorderedLinkPermutationRule::i2oType(Handle* h) const
{
        assert(2==n);
        return  atom(child(h[1],0)); //Child of the topological link gives the topology
}

Rule::setOfMPs UnorderedLinkPermutationRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
{
        if (!outh.matchType(AND_LINK) || outh.hs.size() > MAX_ARITY_FOR_PERMUTATION)
            return Rule::setOfMPs();

printAtomTree(outh,0,1);

        set< vector<atom> > *ps = newCreatePermutations<atom>(outh.hs);

        boost::shared_ptr<MPs> ret(new MPs);

        boost::shared_ptr<MetaPredicate> newa( new atom(OR_LINK, 0) );

        for (set< vector<atom> >::iterator i = ps->begin(); i != ps->end(); i++)
            newa->hs.push_back(atom(outh.T, *i));

        ret->push_back(newa);

        printAtomTree(*(*ret)[0],0,4);

        /// Add the pseudo atom which determines the TOPOLOGY of the desired result

        boost::shared_ptr<atom> pseudoAtom(new atom(HYPOTHETICAL_LINK, 1, new atom(outh)));
        Handle h = pseudoAtom->attach(asw);
        ret->push_back(pseudoAtom);

        TruthValue *tv = getTruthValue(h);


        //printAtomTree(*(*ret)[1],0,4);

        delete ps;

        overrideInputFilter = true;

        return makeSingletonSet(ret);
    }
#endif

}} // namespace opencog { namespace pln {
