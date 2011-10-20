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
Rule::setOfMPs ImplicationConstructionRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
{
        if (!inheritsType(outh.T, IMPLICATION_LINK) || outh.hs.size() != 2)
            return Rule::setOfMPs();

        boost::shared_ptr<MPs> ret(new MPs);

        ret->push_back(boost::shared_ptr<atom>(new atom(outh)));
        (*ret)[0]->T = And_LINK;

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
}

atom ImplicationConstructionRule::i2oType(Handle* h, const int n) const
{
        assert(1==n);

        return  atom(IMPLICATION_LINK, 2,
                        new atom(child(h[0], 0)),
                        new atom(child(h[0], 1))
                );
}
#endif

}} // namespace opencog { namespace pln {
