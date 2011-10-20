/*
 * Copyright (C) 2009 by OpenCog Foundation
 * All Rights Reserved
 *
 * Author Nil Geisweiller
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

#ifndef INTENSIONALINHERITANCERULE_H
#define INTENSIONALINHERITANCERULE_H

#include "../../rules/Rule.h"
#include "../../AtomSpaceWrapper.h"

#include "SubsetEvalRule.h"

namespace opencog { namespace pln {

class IntensionalInheritanceRule : public Rule
{
    SubsetEvalRule sser;
    Type ArgType;

public:
    IntensionalInheritanceRule(AtomSpaceWrapper* _asw, Type argType);

    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

    meta i2oType(const VertexSeq& h) const;
    
    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = NULL,
                        bool fresh = true) const;

    //@todo: not sure it is enough
    bool validate2(MPs& args) const {
        return true;
    }

    NO_DIRECT_PRODUCTION;  
};

}} // namespace opencog { namespace pln {

#endif
