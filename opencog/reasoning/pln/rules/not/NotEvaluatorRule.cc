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

namespace opencog { namespace pln {

NotEvaluatorRule::NotEvaluatorRule( opencog::pln::AtomSpaceWrapper *_asw )
: GenericRule<opencog::pln::NotFormula>(_asw, true, "NotEvaluatorRule")
{
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)NOT_LINK,
                                                    mva((pHandle)ATOM)))
                               ));
}

meta NotEvaluatorRule::i2oType(const std::vector<Vertex>& h) const
{
    assert(1 == h.size());
    return meta(new tree<Vertex>(mva((pHandle)NOT_LINK,
                                     tree<Vertex>(h[0])
                                     )));
}

// Private method
Rule::setOfMPs NotEvaluatorRule::o2iMetaExtra(meta outh,
                                              bool& overrideInputFilter) const
{
    if (!asw->isSubType(_v2h(*outh->begin()), NOT_LINK))
        return Rule::setOfMPs();

    //! @todo why not?
//    LOG(-10, "SHOULD NOT BE HERE!"); assert(0);
    MPs ret;

    assert(outh->begin().number_of_children() == 1);
    ret.push_back(BBvtree(new BoundVTree(outh->begin(outh->begin())))); //1st child

//  printAtomTree(*(*ret)[0],0,4);

    overrideInputFilter = true;
    return makeSingletonSet(ret);
}

}} // namespace opencog { namespace pln {
