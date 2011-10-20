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

/*Rule::setOfMPs AndSubstRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    if (!inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK)
        || outh->number_of_children() != 2)
        return Rule::setOfMPs();        

    tree<Vertex>::sibling_iterator hs1 = outh->begin(outh->begin());
    tree<Vertex>::sibling_iterator hs0 = hs1++;
    
    if (!inheritsType(nm->getType(v2h(*hs0)), AND_LINK)
        || (   !inheritsType(nm->getType(v2h(*hs0)), AND_LINK)
            && !inheritsType(nm->getType(v2h(*hs0)), FW_VARIABLE_NODE)
    )
        return Rule::setOfMPs();
    
}

BoundVertex AndSubstRule::compute(const VertexSeq& premiseArray, Handle CX) const
{
} */

}} // namespace opencog { namespace pln {
