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

#ifndef STRICTIMPBREAKDOWNRULE_H
#define STRICTIMPBREAKDOWNRULE_H

#include "../Rule.h"

namespace opencog { namespace pln {

/// (A->B, A) => B.
class StrictImplicationBreakdownRule : public Rule
{
public:
    NO_DIRECT_PRODUCTION;
    
    StrictImplicationBreakdownRule(AtomSpaceWrapper *_asw);
//    meta targetTemplate() const;
    Rule::setOfMPs fullInputFilter() const;

    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
    /**
     * @param premiseArray a vector of premises, (A->B, A)
     * @param CX Context to use for rule computation. Currently unused.
     * @param fresh allows atoms to be added with the same name/outgoing set.
     *              If fresh == false and the atom already exist then the new
     *              truth value is merged (via TruthValue::merge) with the old.
     *              Otherwise (fresh == true) then a new dummy context
     *              is associated to that new truth value.
     *
     * @return the BoundVertex of the conclusion, A
     */
    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;
    bool validate2(MPs& args) const { return true; }
};

}} // namespace opencog { namespace pln {
#endif // STRICTIMPBREAKDOWNRULE_H
