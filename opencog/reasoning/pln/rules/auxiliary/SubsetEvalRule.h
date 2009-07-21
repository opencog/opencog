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

#ifndef SUBSETEVALRULE_H
#define SUBSETEVALRULE_H

#include "../../AtomLookupProvider.h"
#include "../../formulas/Formulas.h"
#include "../../rules/Rule.h"

namespace opencog { namespace pln {

/*
class SubsetEvalRule
{
    pHandle domain;
    SubsetEvalFormula f;

protected:

    MPs inputFilter;
    iAtomSpaceWrapper *destTable;
public:
    virtual ~SubsetEvalRule() {}
    SubsetEvalRule(iAtomSpaceWrapper *_destTable);

    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const {
        return setOfMPs(); //No support (yet)
    }

    meta i2oType(const vector<Vertex>& h) const {
        assert(n == 1);
        return atomWithNewType(h[0], SUBSET_LINK);
    }

    BoundVertex compute(const vector<Vertex>& premiseArray, pHandle CX = NULL) const
};
*/

}} // namespace opencog { namespace pln {
#endif // SUBSETEVALRULE_H
