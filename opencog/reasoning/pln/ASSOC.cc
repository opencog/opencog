/*
 * ASSOC.cc
 *
 * Copyright (C) 2009 by OpenCog Foundation
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
    // Not possible now that IntensionalInheritanceRule is generic wrt input
    // Atom Type.
    // OC_ASSERT(asw->isSubType(c_h, CONCEPT_NODE));

    std::string c_name = asw->getName(c_h);
    //! @todo Since this Atom stores the result, just look it up if this is
    //! called again.
    std::string c_ASSOC_name = c_name + ASSOC_suffix;

    pHandle not_c_h = asw->getHandle(NOT_LINK, c_h);
//    OC_ASSERT(not_c_h != PHANDLE_UNDEFINED,
//              "NOT_LINK %s is not defined so we cannot create %s",
//              c_name.c_str(), c_ASSOC_name.c_str());
    if (not_c_h == PHANDLE_UNDEFINED) {
        cprintf(0, "NOT_LINK %s is not defined so we cannot create %s",
                   c_name.c_str(), c_ASSOC_name.c_str());
        return PHANDLE_UNDEFINED;
    }

    // create ASSOC_conceptName
    pHandle c_ASSOC_h = asw->addNode(CONCEPT_NODE, c_ASSOC_name,
                                     TruthValue::DEFAULT_TV());

    // search for all elements 'super_h' such that 'SubSet c_h super_h'
    vtree mp(static_cast<pHandle>(SUBSET_LINK));
    TableGather table(mp, asw);
    for (TableGatherConstIt tgci = table.begin(); tgci != table.end(); ++tgci) {
        pHandle subset_h = boost::get<pHandle>(tgci->GetValue());
        pHandle sub_h = asw->getOutgoing(subset_h, 0);
        if (sub_h == c_h) {
            pHandle super_h = asw->getOutgoing(subset_h, 1);
            pHandle subset_not_c_h = asw->getHandle(SUBSET_LINK,
                                                    not_c_h, super_h);
//            OC_ASSERT(subset_not_c_h != PHANDLE_UNDEFINED,
//                      "SUBSET_LINK NOT %s %s is not defined",
//                      c_name.c_str(), asw->getName(super_h).c_str());
            if (subset_not_c_h == PHANDLE_UNDEFINED) {
                cprintf(0, "SUBSET_LINK NOT %s %s is not defined",
                            c_name.c_str(), asw->getName(super_h).c_str());
                return PHANDLE_UNDEFINED;
            }
            TruthValuePtr subset_not_c_TV(asw->getTV(subset_not_c_h));
            asw->addLink(MEMBER_LINK, super_h, c_ASSOC_h,
                         ASSOC(*asw->getTV(subset_h), *subset_not_c_TV));
        }
    }
    return c_ASSOC_h;
}

SimpleTruthValue ASSOC(const TruthValue& tv1, const TruthValue& tv2) {
    // Note that for the moment confidence is ignored, this should be fixed
    return SimpleTruthValue(std::max(tv1.getMean() - tv2.getMean(),
                                     static_cast<strength_t>(0.0)), 1);
}

}}
