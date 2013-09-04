/*
 * opencog/embodiment/Learning/behavior/BDRetriever.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>

#include <opencog/spacetime/atom_types.h>
#include <opencog/spacetime/Temporal.h>
#include <opencog/spacetime/TemporalTable.h>
#include <opencog/spacetime/TimeServer.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>

#include "BDRetriever.h"
namespace behavior
{

using namespace opencog::pai;
using namespace opencog;

void BDRetriever::retrieveExemplar(CompositeBehaviorDescription& bd,
                                   const WorldProvider& wp,
                                   const std::string& name,
                                   const Temporal& temp)
{
    Handle h = wp.getAtomSpace().getHandle(CONCEPT_NODE, name);
    if (h != Handle::UNDEFINED)
        retrieveExemplar(bd, wp, h, temp);
}

void BDRetriever::retrieveExemplar(CompositeBehaviorDescription& bd,
                                   const WorldProvider& wp,
                                   Handle trickConceptNode,
                                   const Temporal& temp)
{
    std::list<HandleTemporalPair> retP;
    timeServer().getTimeInfo(std::back_inserter(retP),
                                  trickConceptNode, temp,
                                  TemporalTable::EXACT);
    if (!retP.empty()) {
        OC_ASSERT(retP.size() == 1,
                         "retP std::list should have exactly one 'HandleTemporal Pair'.");
        Handle h = timeServer().getAtTimeLink(*(retP.begin()));
        OC_ASSERT(h != Handle::UNDEFINED,
                         "Handle h should not be an 'Handle::UNDEFINED'.");
        std::list<Handle> result;
        std::vector<Handle> outputs;
        outputs.push_back(Handle::UNDEFINED);
        outputs.push_back(h);
        Type types[2];
        types[0] = AT_TIME_LINK;
        types[1] = AT_TIME_LINK;

        wp.getAtomSpace().getHandleSet(back_inserter(result), outputs,
                                       types, NULL, 2, MEMBER_LINK, false);
        for (std::list<Handle>::iterator ih = result.begin(); ih != result.end(); ++ih) {

            OC_ASSERT((*ih) != Handle::UNDEFINED && wp.getAtomSpace().getArity(*ih) == 2,
                             "Handle should not be an 'Handle::UNDEFINED' and should have arity 2.");

            Handle h_at_time = wp.getAtomSpace().getOutgoing(*ih, 0);
            OC_ASSERT(h_at_time != Handle::UNDEFINED && wp.getAtomSpace().getType(h_at_time) == AT_TIME_LINK,
                             "Handle h_at_time should not be an 'Handle::UNDEFINED' and should be an 'AT_TIME_LINK'.");

            OC_ASSERT(wp.getAtomSpace().getArity(h_at_time) == 2, "Handle h_at_time should have arity 2.");

            Temporal t = Temporal::getFromTimeNodeName(wp.getAtomSpace().getName(wp.getAtomSpace().getOutgoing(h_at_time, 0)).c_str());
            Handle h_bd = wp.getAtomSpace().getOutgoing(h_at_time, 1);
            bd.addPredicate(h_bd, t);
        }
    }
}

void BDRetriever::retrieveLastExemplar(CompositeBehaviorDescription& bd,
                                       Temporal& et,
                                       const WorldProvider& wp,
                                       const std::string& tn)
{
    Handle h = wp.getAtomSpace().getHandle(CONCEPT_NODE, tn);
    if (h != Handle::UNDEFINED) {
        std::list<HandleTemporalPair> retP;
        timeServer().getTimeInfo(std::back_inserter(retP), h,
                                      Temporal(wp.getLatestSimWorldTimestamp()),
                                      TemporalTable::PREVIOUS_BEFORE_START_OF);
        if (!retP.empty()) {
            OC_ASSERT(retP.size() == 1,
                             "retP std::list should have exactly one 'HandleTemporal Pair'.");
            et = *retP.begin()->getTemporal();
            retrieveExemplar(bd, wp, h, et);
        }
    }
}

void BDRetriever::addLastExemplar(BehaviorCategory& bc,
                                  std::vector<Temporal>& est,
                                  const WorldProvider& wp,
                                  const std::string& tn)
{
    OC_ASSERT(bc.getSize() == (int)est.size(),
                     "bc and est should have the same size");
    CompositeBehaviorDescription bd(&wp.getAtomSpace());
    Temporal et(0);
    retrieveLastExemplar(bd, et, wp, tn);
    if (bd.empty()) { //if the exemplar is empty it is simply ignored
        logger().info("BDRetriever - The last exemplar is empty it will simply be ignored.");
    } else {
        bc.addCompositeBehaviorDescription(bd);
        est.push_back(et);
        logger().info("BDRetriever - The last exemplar %s has been retrieved successfully.", bd.toString().c_str());
    }
}

void BDRetriever::retrieveAllExemplars(BehaviorCategory& bc,
                                       std::vector<Temporal>& est,
                                       const WorldProvider& wp,
                                       const std::string& tn)
{
    Handle h = wp.getAtomSpace().getHandle(CONCEPT_NODE, tn);
    if (h != Handle::UNDEFINED) {
        std::list<HandleTemporalPair> retP;
        timeServer().getTimeInfo(std::back_inserter(retP), h,
                                      Temporal(wp.getLatestSimWorldTimestamp()),
                                      TemporalTable::STARTS_BEFORE);
        for (std::list<HandleTemporalPair>::iterator ip = retP.begin(); ip != retP.end(); ++ip) {
            CompositeBehaviorDescription bd(&wp.getAtomSpace());
            Temporal temp = *(ip->getTemporal());
            retrieveExemplar(bd, wp, h, temp);
            if (bd.empty()) {
                logger().info("BDRetriever - The exemplar of interval [%u, %u] is empty it will simply be ignored.", temp.getLowerBound(), temp.getUpperBound());
            } else {
                bc.addCompositeBehaviorDescription(bd);
                est.push_back(temp);
                logger().info("BDRetriever - The following exemplar %s has been retrieved successfully.", bd.toString().c_str());
            }
        }
    }
}

}//~namespace behavior
