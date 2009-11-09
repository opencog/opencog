/*
 * ASSOC.cc
 *
 * Copyright (C) 2009 by Singularity Institute for Artificial Intelligence
 * Written by Nil Geisweiller
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

#include "ASSOC.h"

namespace opencog { namespace pln {

pHandle CreateConceptASSOC(AtomSpaceWrapper* asw, pHandle c_h) {
    OC_ASSERT(asw->isSubType(c_h, CONCEPT_NODE));

    pHandle not_c_h = asw->getHandle(NOT_LINK, c_h);
    // Maybe that assert is too strong
    OC_ASSERT(not_c_h != PHANDLE_UNDEFINED,
              "If the negation of the concept is not defined we cannot calculate ASSOC_c_h");

    // create ASSOC_conceptName
    pHandle ASSOC_c_h = asw->addNode(CONCEPT_NODE, asw->getName(c_h),
                                     TruthValue::DEFAULT_TV());

    // search for all elements 'super_h' such that 'SubSet c_h super_h'
    vtree mp(static_cast<pHandle>(SUBSET_LINK));
    TableGather table(mp, asw);
    for(TableGatherConstIt tgci = table.begin(); tgci != table.end(); ++tgci) {
        pHandle subset_h = boost::get<pHandle>(tgci->GetValue());
        pHandle sub_h = asw->getOutgoing(subset_h, 0);
        if(sub_h == c_h) {
            pHandle super_h = asw->getOutgoing(subset_h, 1);
            // look for 'SubSet NOT c_h super_h', if it does not
            // exist consider 'SubSet NOT c_h super_h' as null
            SimpleTruthValue subset_not_c_TV = TruthValue::NULL_TV();
            pHandle subset_not_c_h = asw->getHandle(SUBSET_LINK,
                                                    not_c_h, super_h);
            if(subset_not_c_h != PHANDLE_UNDEFINED)
                subset_not_c_TV = asw->getTV(subset_not_c_h);
            // create memberLink super_h ASSOC_c_h_h
            // with tv according to ASSOC_formula
            asw->addLink(MEMBER_LINK, super_h, ASSOC_c_h,
                         ASSOC(asw->getTV(subset_h), subset_not_c_TV));
        }
    }
    return ASSOC_c_h;
}

SimpleTruthValue ASSOC(const TruthValue& tv1, const TruthValue& tv2) {
    // Note that for the moment confidence is ignored, this should be fixed
    return SimpleTruthValue(std::max(tv1.getMean() - tv2.getMean(),
                                     static_cast<strength_t>(0.0)), 1);
}

}}
