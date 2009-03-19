/**
 * BDRetriever.cc
 *
 * Author(s):
 *   Nil Geisweiller
 * Creation: Thu Sep 6 2007
 */

#include "BDRetriever.h"
#include <opencog/atomspace/Temporal.h>
#include <opencog/atomspace/TemporalTable.h>
#include "util/exceptions.h"
#include "PAI.h"

namespace behavior
{

using namespace PerceptionActionInterface;
using namespace opencog;

void BDRetriever::retrieveExemplar(CompositeBehaviorDescription& bd,
                                   const WorldProvider& wp,
                                   const std::string name,
                                   const Temporal& temp)
{
    Handle h = wp.getSpaceServer().getAtomSpace().getHandle(CONCEPT_NODE, name);
    if (h != Handle::UNDEFINED)
        retrieveExemplar(bd, wp, h, temp);
}

void BDRetriever::retrieveExemplar(CompositeBehaviorDescription& bd,
                                   const WorldProvider& wp,
                                   Handle trickConceptNode,
                                   const Temporal& temp)
{
    std::list<HandleTemporalPair> retP;
    wp.getSpaceServer().getAtomSpace().getTimeInfo(std::back_inserter(retP),
            trickConceptNode, temp,
            TemporalTable::EXACT);
    if (!retP.empty()) {
        opencog::cassert(TRACE_INFO, retP.size() == 1,
                         "retP std::list should have exactly one 'HandleTemporal Pair'.");
        Handle h = wp.getSpaceServer().getAtomSpace().getAtTimeLink(*(retP.begin()));
        opencog::cassert(TRACE_INFO, h != Handle::UNDEFINED,
                         "Handle h should not be an 'Handle::UNDEFINED'.");
        std::list<Handle> result;
        std::vector<Handle> outputs;
        outputs.push_back(Handle::UNDEFINED);
        outputs.push_back(h);
        Type types[2];
        types[0] = AT_TIME_LINK;
        types[1] = AT_TIME_LINK;

        wp.getSpaceServer().getAtomSpace().getHandleSet(back_inserter(result), outputs,
                types, NULL, 2, MEMBER_LINK, false);
        for (std::list<Handle>::iterator ih = result.begin(); ih != result.end(); ++ih) {

            opencog::cassert(TRACE_INFO, (*ih) != Handle::UNDEFINED && wp.getSpaceServer().getAtomSpace().getArity(*ih) == 2,
                             "Handle should not be an 'Handle::UNDEFINED' and should have arity 2.");

            Handle h_at_time = wp.getSpaceServer().getAtomSpace().getOutgoing(*ih, 0);
            opencog::cassert(TRACE_INFO, h_at_time != Handle::UNDEFINED && wp.getSpaceServer().getAtomSpace().getType(h_at_time) == AT_TIME_LINK,
                             "Handle h_at_time should not be an 'Handle::UNDEFINED' and should be an 'AT_TIME_LINK'.");

            opencog::cassert(TRACE_INFO, wp.getSpaceServer().getAtomSpace().getArity(h_at_time) == 2, "Handle h_at_time should have arity 2.");

            Temporal t = Temporal::getFromTimeNodeName(wp.getSpaceServer().getAtomSpace().getName(wp.getSpaceServer().getAtomSpace().getOutgoing(h_at_time, 0)).c_str());
            Handle h_bd = wp.getSpaceServer().getAtomSpace().getOutgoing(h_at_time, 1);
            bd.addPredicate(h_bd, t);
        }
    }
}

void BDRetriever::retrieveLastExemplar(CompositeBehaviorDescription& bd,
                                       Temporal& et,
                                       const WorldProvider& wp,
                                       const std::string& tn)
{
    Handle h = wp.getSpaceServer().getAtomSpace().getHandle(CONCEPT_NODE, tn);
    if (h != Handle::UNDEFINED) {
        std::list<HandleTemporalPair> retP;
        wp.getSpaceServer().getAtomSpace().getTimeInfo(std::back_inserter(retP), h,
                Temporal(wp.getLatestSimWorldTimestamp()),
                TemporalTable::PREVIOUS_BEFORE_START_OF);
        if (!retP.empty()) {
            opencog::cassert(TRACE_INFO, retP.size() == 1,
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
    opencog::cassert(TRACE_INFO, bc.getSize() == (int)est.size(),
                     "bc and est should have the same size");
    CompositeBehaviorDescription bd;
    Temporal et(0);
    retrieveLastExemplar(bd, et, wp, tn);
    if (bd.empty()) { //if the exemplar is empty it is simply ignored
        logger().log(opencog::Logger::INFO, "BDRetriever - The last exemplar is empty it will simply be ignored.");
    } else {
        bc.addCompositeBehaviorDescription(bd);
        est.push_back(et);
        logger().log(opencog::Logger::INFO, "BDRetriever - The last exemplar %s has been retrieved successfully.", bd.toString().c_str());
    }
}

void BDRetriever::retrieveAllExemplars(BehaviorCategory& bc,
                                       std::vector<Temporal>& est,
                                       const WorldProvider& wp,
                                       const std::string& tn)
{
    Handle h = wp.getSpaceServer().getAtomSpace().getHandle(CONCEPT_NODE, tn);
    if (h != Handle::UNDEFINED) {
        std::list<HandleTemporalPair> retP;
        wp.getSpaceServer().getAtomSpace().getTimeInfo(std::back_inserter(retP), h,
                Temporal(wp.getLatestSimWorldTimestamp()),
                TemporalTable::STARTS_BEFORE);
        for (std::list<HandleTemporalPair>::iterator ip = retP.begin(); ip != retP.end(); ++ip) {
            CompositeBehaviorDescription bd;
            Temporal temp = *(ip->getTemporal());
            retrieveExemplar(bd, wp, h, temp);
            if (bd.empty()) {
                logger().log(opencog::Logger::INFO, "BDRetriever - The exemplar of interval [%u, %u] is empty it will simply be ignored.", temp.getLowerBound(), temp.getUpperBound());
            } else {
                bc.addCompositeBehaviorDescription(bd);
                est.push_back(temp);
                logger().log(opencog::Logger::INFO, "BDRetriever - The following exemplar %s has been retrieved successfully.", bd.toString().c_str());
            }
        }
    }
}

}//~namespace behavior
