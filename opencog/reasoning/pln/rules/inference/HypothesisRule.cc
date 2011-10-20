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

#define HYPRULE_MAKES_ZERO_CONFIDENCE_ATOMS false

namespace opencog { namespace pln {

boost::shared_ptr<std::set<BoundVertex> > HypothesisRule::attemptDirectProduction(meta outh, bool fresh) const
{
    std::set<BoundVertex>* ret = new std::set<BoundVertex>;
    
    Type t = asw->getTypeV(*outh);
    bool hyp_link = asw->inheritsType(t, HYPOTHETICAL_LINK);

	if (HYPRULE_MAKES_ZERO_CONFIDENCE_ATOMS || hyp_link)
        if (!hasFW_VAR(*outh))
        {
            cprintf(4,"HYP0:\n");
            NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
            printer.print(outh->begin(), 4);
            
            ret->insert(BoundVertex(asw->addAtom(*outh,
                                                 TruthValue::TRIVIAL_TV(),
                                                 fresh)));
            
            cprintf(4,"HYP:\n");
            printer.print(_v2h(ret->begin()->value), 4);
        }

    return boost::shared_ptr<std::set<BoundVertex> >(ret);
}

Rule::setOfMPs HypothesisRule:: o2iMetaExtra(meta outh,
                                             bool& overrideInputFilter) const
{
    assert(0);
    return Rule::setOfMPs();
}

}} // namespace opencog { namespace pln {
