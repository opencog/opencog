/*
 * opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.cc
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 *
 * Updated: by Jinhua Chua, on 2011-11-22
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

#include <algorithm>
#include <cctype>
#include <stack>

#include <boost/algorithm/string.hpp>

#include <opencog/util/misc.h>
#include <opencog/util/Logger.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/random.h>

#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/nlp/types/atom_types.h>
#include <opencog/spacetime/atom_types.h>
#include <opencog/query/BindLink.h>

#include <opencog/spacetime/HandleTemporalPairEntry.h>
#include <opencog/spacetime/SpaceServer.h>
#include <opencog/spacetime/Temporal.h>
#include <opencog/spacetime/TemporalTable.h>
#include <opencog/spacetime/TimeServer.h>
#include <opencog/spacetime/SpaceTime.h>

#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>

#include "AtomSpaceUtil.h"
#include "PredefinedProcedureNames.h"
#include "CompareAtomTreeTemplate.h"

using std::string;
using std::list;
using namespace opencog; /// @todo make it under the namespace opencog

#define IS_HOLDING_PREDICATE_NAME "isHolding"
#define IS_HOLDING_SOMETHING_PREDICATE_NAME "isHoldingSomething"

// Initialize static variables that stores the LatestLinks for each type of information. 
// We cache these LatestLinks to accelerate updating. 
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestAgentActionDone;
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestPhysiologicalFeeling;
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestAvatarSayActionDone;
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestAvatarActionDone;
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestAvatarActionPredicate;

AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestModulators; 
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestDemands; 
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestFeelings; 
AtomSpaceUtil::HandleToHandleMap AtomSpaceUtil::latestStimulus; 

const double AtomSpaceUtil::highLongTermImportance = 0.7;
std::map<Handle, AtomSpaceUtil::HandleToHandleMap > AtomSpaceUtil::latestSpatialPredicate;
std::map<Handle, Handle> AtomSpaceUtil::latestSchemaPredicate;
boost::unordered_map<std::string, HandleSeq> AtomSpaceUtil::frameElementsCache;
Handle AtomSpaceUtil::latestIsExemplarAvatar = Handle::UNDEFINED;

Handle AtomSpaceUtil::addNode(AtomSpace& atomSpace,
                              Type nodeType,
                              const std::string& nodeName,
                              bool permanent, bool renew_sti)
{
    Handle result = atomSpace.getHandle(nodeType, nodeName);
    if (result == Handle::UNDEFINED) {
        result = atomSpace.addNode(nodeType, nodeName);
        if (permanent) {
            atomSpace.setLTI(result, 1);
        }
    } else if (permanent) {
        if (atomSpace.getLTI(result) < 1) {
            atomSpace.setLTI(result, 1);
        }
    } else if (renew_sti) {
        result = atomSpace.addNode(nodeType, nodeName);
    }
    return result;
}

Handle AtomSpaceUtil::addLink(AtomSpace& atomSpace,
                              Type linkType,
                              const HandleSeq& outgoing,
                              bool permanent, bool renew_sti, TruthValuePtr tv)
{
    Handle result = atomSpace.getHandle(linkType, outgoing);
    if (result == Handle::UNDEFINED) {
        result = atomSpace.addLink(linkType, outgoing,tv);
        if (permanent) {
            atomSpace.setLTI(result, 1);
        }
    } else if (permanent) {
        if (atomSpace.getLTI(result) < 1) {
            atomSpace.setLTI(result, 1);
        }
    } else if (renew_sti) {
        result = atomSpace.addLink(linkType, outgoing,tv);
    }
    return result;
}

// TODO: DEPRECATED
Handle AtomSpaceUtil::addRewardPredicate(AtomSpace& atomSpace,
        const char* petId,
        unsigned long timestamp)
{
    HandleSeq evalLinkOutgoing;

    string predicateName = petId;
    predicateName += ".+++";
    evalLinkOutgoing.push_back(addNode(atomSpace, PREDICATE_NODE, predicateName.c_str()));
    HandleSeq emptyOutgoing;
    evalLinkOutgoing.push_back(addLink(atomSpace, LIST_LINK, emptyOutgoing));

    Handle evalLink = addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);

    timeServer().addTimeInfo(evalLink, timestamp);

    return evalLink;
}

// TODO: DEPRECATED
Handle AtomSpaceUtil::addPunishmentPredicate(AtomSpace& atomSpace,
        const char* petId,
        unsigned long timestamp)
{
    HandleSeq evalLinkOutgoing;

    string predicateName = petId;
    predicateName += ".---";
    evalLinkOutgoing.push_back(addNode(atomSpace, PREDICATE_NODE, predicateName.c_str()));
    HandleSeq emptyOutgoing;
    evalLinkOutgoing.push_back(addLink(atomSpace, LIST_LINK, emptyOutgoing));

    Handle evalLink = addLink(atomSpace, EVALUATION_LINK, evalLinkOutgoing);

    timeServer().addTimeInfo(evalLink, timestamp);

    return evalLink;
}

bool AtomSpaceUtil::isActionPredicatePresent(AtomSpace& atomSpace,
        const char* actionPredicateName,
        Handle actionExecLink,
        unsigned long sinceTimestamp)
{

    //cout << "Looking for action predicate '" << actionPredicateName << "' after timestamp '" << sinceTimestamp << "' for action: " << atomSpace.atomAsString(actionExecLink) << endl;
    bool result = false;
    HandleSeq evalListLinkOutgoing;
    evalListLinkOutgoing.push_back(actionExecLink);
    Handle evalListLink = atomSpace.getHandle(LIST_LINK, evalListLinkOutgoing);
    if (evalListLink != Handle::UNDEFINED) {
        //cout << "Found the ListLink with the ExecLink inside" << endl;
        Handle predicateNode = atomSpace.getHandle(PREDICATE_NODE,
                               actionPredicateName);
        if (predicateNode != Handle::UNDEFINED) {
            //cout << "Found the PredicateNode" << endl;
            HandleSeq evalLinkOutgoing;
            evalLinkOutgoing.push_back(predicateNode);
            evalLinkOutgoing.push_back(evalListLink);
            Handle evalLink = atomSpace.getHandle(EVALUATION_LINK,
                                                  evalLinkOutgoing);
            //cout << "evalLink = " << evalLink << endl;
            if (evalLink != Handle::UNDEFINED) {
                //cout << "Found the EvalLink with the PredicateNode and the ListLink in its " << endl;
                list<HandleTemporalPair> ocurrences;
                timeServer().getTimeInfo(back_inserter(ocurrences),
                                      evalLink,
                                      Temporal(sinceTimestamp),
                                      TemporalTable::NEXT_AFTER_END_OF);
                //if (!ocurrences.empty()) {
                //    cout << "Got the following TimeServer entry: " << ocurrences.front().toString() << " for " << atomSpace.atomAsString(evalLink) << endl;
                //} else {
                //    cout << "Got no TimeServer entry for " << atomSpace.atomAsString(evalLink) << endl;
                //}
                result = !ocurrences.empty();
            }
        }
    }
    return result;
}

bool AtomSpaceUtil::getXYZOFromPositionEvalLink(const AtomSpace& atomspace,
        Handle evalLink,
        double &x,
        double &y,
        double &z,
        Handle &o)
{
    if (atomspace.getType(evalLink) != EVALUATION_LINK
            ||  atomspace.getArity(evalLink) != 2)
        return false;

    Handle eval_pred = atomspace.getOutgoing(evalLink, 0);
    Handle pred_args = atomspace.getOutgoing(evalLink, 1);

    if (atomspace.getName(eval_pred) != AGISIM_POSITION_PREDICATE_NAME
            ||  atomspace.getArity(pred_args) != 4)
        return false;

    Handle xh = atomspace.getOutgoing(pred_args, 1);
    Handle yh = atomspace.getOutgoing(pred_args, 2);
    Handle zh = atomspace.getOutgoing(pred_args, 3);
    o = atomspace.getOutgoing(pred_args, 0);

    x = atof(atomspace.getName(xh).c_str());
    y = atof(atomspace.getName(yh).c_str());
    z = atof(atomspace.getName(zh).c_str());

    //      printf("%.3f %.3f %.3f found for %s\n", x, y, z, atomspace.getName(o).c_str());

    return true;
}

Handle AtomSpaceUtil::getCurrentSpaceMapHandle(const AtomSpace &atomSpace)
{
    return spaceServer().getLatestMapHandle();
}

/*
Handle AtomSpaceUtil::getSpaceMapHandleAtTimestamp(const AtomSpace &atomSpace,
        unsigned long t)
{
    Temporal temporal(t);
    std::vector<HandleTemporalPair> temporalPairs;

    Handle spaceMapNode = atomSpace.getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME);

    Handle spaceMapHandle = Handle::UNDEFINED;

    if (spaceMapNode == Handle::UNDEFINED) {
        return spaceMapHandle;
    }

    const SpaceServer& spaceServer = atomSpace.getSpaceServer();
    TimeServer& timeserver = timeServer();

    // get temporal pair for the EXACT timestamp. Only spaceMapNodes are
    // considered
    timeserver.getTimeInfo(back_inserter(temporalPairs),
                          spaceMapNode, temporal,
                          TemporalTable::EXACT);
    if (!temporalPairs.empty()) {

        // found at least one temporalPair. There should be at most one
        spaceMapHandle = timeserver.getAtTimeLink(temporalPairs[0]);
        if (!spaceServer.containsMap(spaceMapHandle)) {
            spaceMapHandle = Handle::UNDEFINED;
        }
    }

    if (spaceMapHandle == Handle::UNDEFINED) {
        // found none temporalPair with EXACT timestamp. Now looking for
        // timestamps before the one used.
        temporalPairs.clear();
        timeserver.getTimeInfo(back_inserter(temporalPairs),
                              spaceMapNode,
                              temporal,
                              TemporalTable::PREVIOUS_BEFORE_START_OF);
        while (!temporalPairs.empty()) {
            spaceMapHandle = timeserver.getAtTimeLink(temporalPairs[0]);
            Temporal* nextTemporal = temporalPairs[0].getTemporal();
            if (spaceServer.containsMap(spaceMapHandle)) {
                break;
            } else {
                spaceMapHandle = Handle::UNDEFINED;
                temporalPairs.clear();
                timeserver.getTimeInfo(back_inserter(temporalPairs),
                                      spaceMapNode,
                                      *nextTemporal,
                                      TemporalTable::PREVIOUS_BEFORE_START_OF);

            }
        }
    }

    return spaceMapHandle;
}
*/
bool AtomSpaceUtil::getPredicateValueAtSpaceMap(AtomSpace& atomSpace,
        const std::string predicate,
        const SpaceServer::SpaceMap& sm,
        Handle obj1, Handle obj2)
{
    if (!sm.containsObject(obj1) ||
            !sm.containsObject(obj2) ) {
        logger().error("AtomSpaceUtil - One or both objects were not present in Space Map");
        return false;
    }

    if (predicate == "near") {

        try {
            //const SpaceServer::ObjectMetadata&
            //    md1 = sm.getMetaData(atomSpace.getName(obj1));
            const spatial::Entity3D* entity1 = sm.getEntity( obj1 );
            //SpaceServer::SpaceMapPoint obj1Center( entity1.getPosition( ).x, entity1.getPosition( ).y );
            //const SpaceServer::ObjectMetadata&
            //    md2 = sm.getMetaData(atomSpace.getName(obj2));

            const spatial::Entity3D* entity2 = sm.getEntity( obj2);
            //SpaceServer::SpaceMapPoint obj2Center( entity2.getPosition( ).x, entity2.getPosition( ).y );

            //std::cout << "MD1 X : " << md1.centerX << " Y : " << md1.centerY
            //    << " MD2 X : " << md2.centerX << " Y : " << md2.centerY
            //    << std::endl;
            double dist =  entity1->getPosition( )
                            - entity2->getPosition( ) ;
            //SpaceServer::SpaceMap::eucDist(obj1Center, obj2Center);

            double maxDistance = ( sm.xMax( ) - sm.xMin( ) ) / 50;
            double distStrength = 0;
            if ( dist < maxDistance ) {
                distStrength = std::max( 0.0, 1.0 - dist / maxDistance );
            } // if

            return ( distStrength > 0.5 );

        } catch (opencog::AssertionException& e) {
            return false;
        }

    } else if (predicate == "above") {
        // need 3D info
        return false;
    } else if (predicate == "below") {
        // need 3D info
        return false;
    } else if (predicate == "inside") {
        // need 3D info
        return false;
    } else return false;
}
/*
bool AtomSpaceUtil::getPredicateValueAtTimestamp(const AtomSpace &atomSpace,
        const std::string& predicate,
        unsigned long timestamp,
        Handle obj1, Handle obj2)
{
    // get the SpaceMap to do the test
    Handle spaceMapHandle = getSpaceMapHandleAtTimestamp(atomSpace,
                            timestamp);
    if (spaceMapHandle == Handle::UNDEFINED) {
        return false;
    } else {
        const SpaceServer::SpaceMap&
        sm = atomSpace.getSpaceServer().getMap(spaceMapHandle);
        return getPredicateValueAtSpaceMap(atomSpace,
                                           predicate, sm, obj1, obj2);
    }
}
*/
bool AtomSpaceUtil::getHasSaidValueAtTime(AtomSpace &atomSpace,
        unsigned long timestamp,
        unsigned long delay,
        Handle from_h,
        Handle to_h,
        const std::string& message,
        bool include_to)
{
    OC_ASSERT(delay < timestamp,
                     "timestamp - delay must be positive");
    unsigned long tl = timestamp - delay;
    unsigned long tu = timestamp;
    Temporal temp(tl, tu);
    std::list<HandleTemporalPair> ret;
    timeServer().getTimeInfo(back_inserter(ret), Handle::UNDEFINED, temp,
                          TemporalTable::STARTS_WITHIN);

    if (ret.empty()) {
        return false;
    } else {
        //check if all atoms of the structure to find are present
        Handle action_done_h = atomSpace.getHandle(PREDICATE_NODE,
                               ACTION_DONE_PREDICATE_NAME);

        if (action_done_h == Handle::UNDEFINED)
            return false;
        Handle say_h = atomSpace.getHandle(GROUNDED_SCHEMA_NODE,
                                           SAY_SCHEMA_NAME);

        if (say_h == Handle::UNDEFINED)
            return false;
        //create the sentence atom
        string atom_message_name;
        if (include_to) {
            OC_ASSERT(atomSpace.isNode(to_h),
                       "Handle to_h should be a 'Node'.");
            atom_message_name = string("to:") + atomSpace.getName(to_h)
                                + string(": ") + message;
        } else atom_message_name = message;
        Handle sentence_h = atomSpace.getHandle(SENTENCE_NODE,
                                                atom_message_name);

        if (sentence_h == Handle::UNDEFINED) {
            return false;
        }

        //define template
        atom_tree *say_template =
            makeVirtualAtom(EVALUATION_LINK,
                            makeVirtualAtom(action_done_h, NULL),
                            makeVirtualAtom(LIST_LINK,
                                            makeVirtualAtom(EXECUTION_LINK,
                                                            makeVirtualAtom(say_h, NULL),
                                                            makeVirtualAtom(LIST_LINK,
                                                                            makeVirtualAtom(from_h, NULL),
                                                                            makeVirtualAtom(sentence_h, NULL),
                                                                            NULL
                                                                           ),
                                                            NULL
                                                           ),
                                            NULL
                                           ),
                            NULL
                           );

        does_fit_template dft(*say_template, &atomSpace, true);
        //iterate over the atom list to see if such a message has been said
        for (std::list<HandleTemporalPair>::const_iterator ret_it = ret.begin();
                ret_it != ret.end(); ++ret_it) {
            if (dft(ret_it->getHandle()))
                return true;
        }
        return false;
    }
}


bool AtomSpaceUtil::isMovingBtwSpaceMap(const AtomSpace& atomSpace,
                                        const SpaceServer::SpaceMap& sm1,
                                        const SpaceServer::SpaceMap& sm2,
                                        Handle obj)
{
    //const std::string& obj_str = atomSpace.getName(obj);
    bool insm1 = sm1.containsObject(obj);
    bool insm2 = sm2.containsObject(obj);
    //we consider that :
    //1)if the object appears or disappears from the spaceMaps it has moved.
    //2)if it is in neither those spaceMap it hasn't
    if (insm1 && insm2) {
        //check if has moved
        //const SpaceServer::ObjectMetadata& md1 = sm1.getMetaData(obj_str);
        //const SpaceServer::ObjectMetadata& md2 = sm2.getMetaData(obj_str);
        const spatial::Entity3D* entity1 = sm1.getEntity( obj );
        const spatial::Entity3D* entity2 = sm2.getEntity( obj );
        //return md1==md2;
        return ( entity1 == entity2 );
    } else if (!insm1 && !insm2)
        return false; //case 2)
    else return true; //case 1)
}

bool AtomSpaceUtil::isMovingBtwSpaceMap(const AtomSpace& atomSpace,
                                        const SpaceServer::SpaceMap& sm,
                                        Handle obj)
{
    //TODO
    OC_ASSERT(false);
    return false;
}



Handle AtomSpaceUtil::getLatestHandle(const AtomSpace &atomSpace,HandleSeq& handles)
{
    if (handles.size() < 1)
        return Handle::UNDEFINED;

    if ( handles.size() < 2)
        return handles.front();

    std::vector<HandleTemporalPair> handleTemporalPairs;

    foreach(Handle h, handles)
    {
        timeServer().getTimeInfo( back_inserter(handleTemporalPairs), h);
    }

    std::vector<HandleTemporalPair>::iterator iHandleTemporalPair;
    std::vector<HandleTemporalPair>::iterator iLatestHandleTemporalPair;

    iHandleTemporalPair = handleTemporalPairs.begin();
    iLatestHandleTemporalPair = handleTemporalPairs.begin();

    while ( iHandleTemporalPair != handleTemporalPairs.end() )
    {

        if ( HandleTemporalPairEntry::handleTemporalPairCompare
                (&*iHandleTemporalPair,
                 &*iLatestHandleTemporalPair) > 0 ) {
            iLatestHandleTemporalPair = iHandleTemporalPair;
        }

        iHandleTemporalPair ++;
    }

    if ( iLatestHandleTemporalPair == handleTemporalPairs.end() )
    {
        logger().warn("AtomSpaceUtil::getLatestHandle: - Failed to find the latest handle!");
        return Handle::UNDEFINED;
    }

    // return the latest handle


    return iLatestHandleTemporalPair->getHandle();
}

Handle AtomSpaceUtil::getLatestEvaluationLink(AtomSpace &atomSpace,
                             std::string predicateName,
                             Handle a,
                             Handle b,
                             Handle c,
                             bool getPositiveResult)
throw(opencog::NotFoundException)
{
    HandleSeq seq0;
    seq0.push_back(a);

    // used for binary predicates like near, inside, above and below
    if (b != Handle::UNDEFINED)
    {
        seq0.push_back(b);
        if (c != Handle::UNDEFINED)
            seq0.push_back(c);
    } // if

    // testing if there is a predicate already
    Handle predicateHandle = atomSpace.getHandle(PREDICATE_NODE,
                             predicateName);
    if (predicateHandle == Handle::UNDEFINED) {
        return Handle::UNDEFINED;
    }

    // some EvalLink has just predicate and the first outgoing
    // e.g.:
//    (EvaluationLink (stv 1 1)
//       (PredicateNode "is_food")
//       (ListLink
//          (ConceptNode "apple")
//       )
//    )
    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
    HandleSeq evalLinkHandleset;
    if (listLinkHandle != Handle::UNDEFINED)
    {
        HandleSeq seq;
        seq.push_back(predicateHandle);
        seq.push_back(listLinkHandle);

        atomSpace.getHandlesByOutgoing(back_inserter(evalLinkHandleset),
                               seq, NULL, NULL, 2, EVALUATION_LINK, false);

    }

    // some EvalLink has its value node blow the first outgoings
    // in this case, need to call pattern matcher to get the value node given the predicate and first outgoings
    // e.g.:
//    (EvaluationLink (stv 1 1)
//       (PredicateNode "color")
//       (ListLink
//          (ConceptNode "sky")
//          (ConceptNode "blue")
//       )
//    )

    if (evalLinkHandleset.size() == 0)
    {
        evalLinkHandleset = getEvaluationLinks(atomSpace,predicateName,seq0);
    }


    //  try to get the EvaluationLink with truth value >= 0.5 if any, if not, return the one < 0.5. vice versa
    HandleSeq handleset;
    foreach (Handle eh, evalLinkHandleset)
    {
        if ( atomSpace.getMean(eh) >= 0.5)
        {
            if (getPositiveResult)
                handleset.push_back(eh);
        }
        else if (! getPositiveResult)
            handleset.push_back(eh);
    }

    // and then get the lastest one
    if (handleset.size() == 0)
        return getLatestHandle(atomSpace,evalLinkHandleset);
    else
        return getLatestHandle(atomSpace,handleset);

}


float AtomSpaceUtil::getPredicateValue(AtomSpace &atomSpace,
                                       std::string predicateName,
                                       Handle a,
                                       Handle b)
throw(opencog::NotFoundException)
{

    HandleSeq seq0;
    seq0.push_back(a);

    // used for binary predicates like near, inside, above and below
    if (b != Handle::UNDEFINED) {
        seq0.push_back(b);
    } // if

    // testing if there is a predicate already
    Handle predicateHandle = atomSpace.getHandle(PREDICATE_NODE,
                             predicateName);
    if (predicateHandle == Handle::UNDEFINED) {
        throw opencog::NotFoundException( TRACE_INFO,
                  (std::string("AtomSpaceUtil - Predicate not found: ")
                   + predicateName ).c_str( ) );
    }

    // testing if there is a list link already
    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
    if (listLinkHandle == Handle::UNDEFINED) {
        throw opencog::NotFoundException( TRACE_INFO,
                ( "AtomSpaceUtil - List link not found. "
                  "predicateName[" + predicateName + "]").c_str( ) );
    }

    HandleSeq seq;
    seq.push_back(predicateHandle);
    seq.push_back(listLinkHandle);

    Handle evalLinkHandle = atomSpace.getHandle(EVALUATION_LINK, seq);
    if (evalLinkHandle == Handle::UNDEFINED) {
        throw opencog::NotFoundException(TRACE_INFO,
               ("AtomSpaceUtil - There is no evaluation link for predicate: "
                 + predicateName).c_str() );
    }
    return atomSpace.getMean(evalLinkHandle);
}


bool AtomSpaceUtil::isPredicateTrue(AtomSpace &atomSpace,
                                    std::string predicateName,
                                    Handle a, Handle b)
{
    try {
        return ( getPredicateValue( atomSpace, predicateName, a, b ) > 0.5 );
    } catch ( opencog::NotFoundException& ex ) {
        return false;
    }
}

bool AtomSpaceUtil::isPetOwner( AtomSpace& atomSpace,
                                Handle avatar, Handle pet )
{
    HandleSeq seq0;
    seq0.push_back(avatar);
    seq0.push_back(pet);

    // testing if there is a predicate already
    Handle predicateHandle = atomSpace.getHandle(PREDICATE_NODE,
                             OWNERSHIP_PREDICATE_NAME );
    if (predicateHandle == Handle::UNDEFINED) {
        logger().fine("IsFriendly - Found no \"owns\" predicate.");
        return false;
    } // if

    // testing if there is a list link already
    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
    if (listLinkHandle == Handle::UNDEFINED) {
        logger().fine("IsFriendly - Obj %s and %s have no ListLink.",
                     atomSpace.getName(avatar).c_str(),
                     atomSpace.getName(pet).c_str());
        return false;
    } // if

    HandleSeq seq;
    seq.push_back(predicateHandle);
    seq.push_back(listLinkHandle);

    Handle evalLinkHandle = atomSpace.getHandle(EVALUATION_LINK, seq);
    if (evalLinkHandle == Handle::UNDEFINED) {
        logger().warn("IsFriendly - Found no EvalLink.");
        return false;
    }

    return true;
}

bool AtomSpaceUtil::getSizeInfo(AtomSpace& atomSpace,
                                Handle object,
                                double& length, double& width, double &height)
{

    Handle sizePredicate = atomSpace.getHandle(PREDICATE_NODE,
                           SIZE_PREDICATE_NAME);
    if (sizePredicate == Handle::UNDEFINED) {
        logger().fine("AtomSpaceUtil - No size predicate found.");
        return false;
    }

#ifdef USE_GET_HANDLE_SET
    HandleSeq seq;
    seq.push_back(sizePredicate);
    seq.push_back(Handle::UNDEFINED);

    std::vector<Handle> allHandles;
    atomSpace.getHandleSet(back_inserter(allHandles),
                           seq, NULL, NULL, 2, EVALUATION_LINK, false);

    foreach(Handle evalLink, allHandles) {
        // getting data from evalLink
        Handle listLink = atomSpace.getOutgoing(evalLink, 1);
        if (atomSpace.getType(listLink) == LIST_LINK
                && atomSpace.getArity(listLink) == 4) {
            // ensure that the size predicate relats to the object
            HandleSeq outgoing = atomSpace.getOutgoing(listLink);
            if (outgoing[0] == object) {
//TODO: elvys remove logs comment
                length = atof(atomSpace.getName(outgoing[1]).c_str());
                width = atof(atomSpace.getName(outgoing[2]).c_str());
                height = atof(atomSpace.getName(outgoing[3]).c_str());
//                logger().info("AtomSpaceUtil - Obj %s (width: %.2lf length: %.2lf height: %.2lf)",
//                            atomSpace.getName(object).c_str(), width, length, height);
                return true;
            }
        }
    }
#else
    HandleSeq incomingSet = atomSpace.getIncoming(sizePredicate);
    foreach(Handle incomingHandle, incomingSet) {
        AtomPtr a(incomingHandle);
        LinkPtr incomingLink(LinkCast(a));
        if (incomingLink->getType() == EVALUATION_LINK &&  incomingLink->getArity() == 2 && 
                incomingLink->getOutgoingAtom(0) == sizePredicate) {
            Handle targetHandle = incomingLink->getOutgoingAtom(1);
            AtomPtr targetAtom(targetHandle);
            if (targetAtom->getType() == LIST_LINK) {
                LinkPtr listLink(LinkCast(targetAtom));
                if (listLink->getArity() == 4 && listLink->getOutgoingAtom(0) == object) {
                    length = atof(atomSpace.getName(listLink->getOutgoingAtom(1)).c_str());
                    width = atof(atomSpace.getName(listLink->getOutgoingAtom(2)).c_str());
                    height = atof(atomSpace.getName(listLink->getOutgoingAtom(3)).c_str());
                    return true;
                }
            }
        }
    } 
#endif 

//TODO: elvys remove logs comment
//    logger().fine("AtomSpaceUtil - No size pred for obj %s found.",
//                    atomSpace.getName(object).c_str());
    return false;
}

Handle AtomSpaceUtil::addGenericPropertyPred(AtomSpace& atomSpace,
        std::string predicateName,
        const HandleSeq& ll_out,
        TruthValuePtr tv, bool permanent, const Temporal &t)
{
    bool predBool = true;
    if (tv->getMean() >= 0.5) {
        predBool = false;
    }

    Handle ph = atomSpace.getHandle(PREDICATE_NODE, predicateName);
    // if predicate handle not defined and TV equals < 0.5 just return
    if (ph == Handle::UNDEFINED && predBool) {
        logger().fine("AtomSpaceUtil - %s not added (no pred handle and TV less than 0.5).",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    } else {
        ph = AtomSpaceUtil::addNode(atomSpace, PREDICATE_NODE, predicateName, true);
    }

    Handle ll = atomSpace.getHandle(LIST_LINK, ll_out);
    // if list link handle not defined and TV equals < 0.5 just return
    if (ll == Handle::UNDEFINED && predBool) {
        logger().fine("AtomSpaceUtil - %s not added (no ListLink and TV less than 0.5)",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    } else {
        ll = atomSpace.addLink(LIST_LINK, ll_out,TruthValue::TRUE_TV());
    }

    HandleSeq hs2;
    hs2.push_back(ph);
    hs2.push_back(ll);
    Handle el = atomSpace.getHandle(EVALUATION_LINK, hs2);

    // if evaluation link handle not defined and TV equals < 0.5 just return
    if (el == Handle::UNDEFINED && predBool) {
        logger().fine("AtomSpaceUtil - %s not added (no EvalLink and TV less than 0.5).",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    } else {
        el = atomSpace.addLink(EVALUATION_LINK, hs2,TruthValue::TRUE_TV());
        logger().fine("AtomSpaceUtil - %s added with TV %f.",
                     predicateName.c_str(), tv->getMean());
    }
    atomSpace.setTV(el, tv);

    Handle result;
    // if not undefined temporal then  a time information should be inserted
    // inserted into AtomSpace.
    if (t != UNDEFINED_TEMPORAL) {
        result = timeServer().addTimeInfo(el, t);
    } else {
        result = el;
    }
    if (permanent) { atomSpace.setLTI(result, 1); }
    return result;
}

Handle AtomSpaceUtil::getMostRecentEvaluationLink(const AtomSpace& atomSpace,
        const std::string& predicateNodeName )
{
    std::vector<HandleTemporalPair> timestamps;
    getAllEvaluationLinks( atomSpace, timestamps, predicateNodeName );

    if ( timestamps.size() == 0 ) {
        logger().debug(
                     "AtomSpaceUtil - Found no entries for PredicateNode '%s' in TimeServer.",
                     predicateNodeName.c_str());
        return Handle::UNDEFINED;
    }

    int mostRecentIndex = 0;
    Temporal *mostRecent = timestamps[mostRecentIndex].getTemporal();
    for (unsigned int i = 1; i < timestamps.size(); i++) {
        if (*(timestamps[i].getTemporal()) > *mostRecent) {
            mostRecent = timestamps[i].getTemporal();
            mostRecentIndex = i;
        } // if
    } // for

    Handle selectedHandle = timestamps[mostRecentIndex].getHandle();
    if ( selectedHandle == Handle::UNDEFINED) {
        logger().debug("AtomSpaceUtil - Timeserver returned NULL handle "
                "for PredicateNode '%s'", predicateNodeName.c_str());
        return Handle::UNDEFINED;
    }

    return selectedHandle;
}
std::vector<Handle> AtomSpaceUtil::getInheritanceLinks(AtomSpace & atomSpace, Handle hFirstOutgoing)
{
    // Create BindLink used by pattern matcher
    std::vector<Handle> inheritanceLinkOutgoings, implicationLinkOutgoings, bindLinkOutgoings;

    Handle hVariableNode = atomSpace.addNode(VARIABLE_NODE, "$var_any");

    inheritanceLinkOutgoings.push_back(hFirstOutgoing);
    inheritanceLinkOutgoings.push_back(hVariableNode);
    Handle hinheritanceLink = atomSpace.addLink(INHERITANCE_LINK, inheritanceLinkOutgoings,TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hinheritanceLink);
    implicationLinkOutgoings.push_back(hinheritanceLink);
    Handle hImplicationLink = atomSpace.addLink(IMPLICATION_LINK, implicationLinkOutgoings,TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariableNode);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = atomSpace.addLink(BIND_LINK, bindLinkOutgoings,TruthValue::TRUE_TV());

    // Run pattern matcher
    Handle hResultListLink = bindlink(&atomSpace, hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink, otherwise some scheme script
    //       may fail to remove the inheritanceLink when necessary.
    //       Because the inheritanceLink would have an incoming (i.e. hResultListLink here),
    //       which would make cog-delete scheme function fail.
    std::vector<Handle> resultSet = atomSpace.getOutgoing(hResultListLink);
    atomSpace.removeAtom(hResultListLink);

    return resultSet;
}

std::vector<Handle> AtomSpaceUtil::getNodesByInheritanceLink(AtomSpace & atomSpace, Handle& hSecondOutgoing)
{
    // Create BindLink used by pattern matcher
    std::vector<Handle> inheritanceLinkOutgoings, implicationLinkOutgoings, bindLinkOutgoings;

    Handle hVariableNode = atomSpace.addNode(VARIABLE_NODE, "$var_any");

    inheritanceLinkOutgoings.push_back(hVariableNode);
    inheritanceLinkOutgoings.push_back(hSecondOutgoing);
    Handle hinheritanceLink = atomSpace.addLink(INHERITANCE_LINK, inheritanceLinkOutgoings,TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hinheritanceLink);
    implicationLinkOutgoings.push_back(hVariableNode);
    Handle hImplicationLink = atomSpace.addLink(IMPLICATION_LINK, implicationLinkOutgoings,TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariableNode);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = atomSpace.addLink(BIND_LINK, bindLinkOutgoings,TruthValue::TRUE_TV());

    // Run pattern matcher
    Handle hResultListLink = bindlink(&atomSpace, hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink, otherwise some scheme script
    //       may fail to remove the inheritanceLink when necessary.
    //       Because the inheritanceLink would have an incoming (i.e. hResultListLink here),
    //       which would make cog-delete scheme function fail.
    std::vector<Handle> resultSet = atomSpace.getOutgoing(hResultListLink);
    atomSpace.removeAtom(hResultListLink);

    return resultSet;
}

std::vector<Handle> AtomSpaceUtil::getEvaluationLinks(AtomSpace &atomSpace, string predicate, HandleSeq &hfirstOutgoings)
{
    // Create BindLink used by pattern matcher
    std::vector<Handle> implicationLinkOutgoings, bindLinkOutgoings;

    Handle hVariableNode = atomSpace.addNode(VARIABLE_NODE, "$var_any");

    Handle predicateNode = atomSpace.addNode(PREDICATE_NODE, predicate);

    HandleSeq predicateListLinkOutgoings;

    foreach (Handle h, hfirstOutgoings)
    {
        predicateListLinkOutgoings.push_back(h);
    }

    predicateListLinkOutgoings.push_back(hVariableNode);

    Handle predicateListLink = atomSpace.addLink(LIST_LINK, predicateListLinkOutgoings,TruthValue::TRUE_TV());

    HandleSeq evalLinkOutgoings;
    evalLinkOutgoings.push_back(predicateNode);
    evalLinkOutgoings.push_back(predicateListLink);
    Handle hEvalLink = atomSpace.addLink(EVALUATION_LINK, evalLinkOutgoings,TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hEvalLink);

    // return the EvaluationLinks
    implicationLinkOutgoings.push_back(hEvalLink);

    Handle hImplicationLink = atomSpace.addLink(IMPLICATION_LINK, implicationLinkOutgoings,TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariableNode);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = atomSpace.addLink(BIND_LINK, bindLinkOutgoings,TruthValue::TRUE_TV());

//            cout<< "hBindLink: \n" << atomSpace.atomAsString(hBindLink) << std::endl;


    // Run pattern matcher
    Handle hResultListLink = bindlink(&atomSpace, hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink, otherwise some scheme script
    //       may fail to remove the inheritanceLink when necessary.
    //       Because the inheritanceLink would have an incoming (i.e. hResultListLink here),
    //       which would make cog-delete scheme function fail.
    std::vector<Handle> resultSet = atomSpace.getOutgoing(hResultListLink);
    atomSpace.removeAtom(hResultListLink);

    return resultSet;
}

std::vector<Handle> AtomSpaceUtil::getNodesByEvaluationLink(AtomSpace &atomSpace, string predicate, HandleSeq &hNonFirstOutgoings)
{
    // Create BindLink used by pattern matcher
    std::vector<Handle> implicationLinkOutgoings, bindLinkOutgoings;

    Handle hVariableNode = atomSpace.addNode(VARIABLE_NODE, "$var_any");

    Handle predicateNode = atomSpace.addNode(PREDICATE_NODE, predicate);

    HandleSeq predicateListLinkOutgoings;
    predicateListLinkOutgoings.push_back(hVariableNode);

    foreach (Handle h, hNonFirstOutgoings)
    {
        predicateListLinkOutgoings.push_back(h);
    }

    Handle predicateListLink = atomSpace.addLink(LIST_LINK, predicateListLinkOutgoings,TruthValue::TRUE_TV());

    HandleSeq evalLinkOutgoings;
    evalLinkOutgoings.push_back(predicateNode);
    evalLinkOutgoings.push_back(predicateListLink);
    Handle hEvalLink = atomSpace.addLink(EVALUATION_LINK, evalLinkOutgoings,TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hEvalLink);

    implicationLinkOutgoings.push_back(hVariableNode);

    Handle hImplicationLink = atomSpace.addLink(IMPLICATION_LINK, implicationLinkOutgoings,TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariableNode);
    bindLinkOutgoings.push_back(hImplicationLink);
    Handle hBindLink = atomSpace.addLink(BIND_LINK, bindLinkOutgoings,TruthValue::TRUE_TV());

//            cout<< "hBindLink: \n" << atomSpace.atomAsString(hBindLink) << std::endl;


    // Run pattern matcher
    Handle hResultListLink = bindlink(&atomSpace, hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink, otherwise some scheme script
    //       may fail to remove the inheritanceLink when necessary.
    //       Because the inheritanceLink would have an incoming (i.e. hResultListLink here),
    //       which would make cog-delete scheme function fail.
    std::vector<Handle> resultSet = atomSpace.getOutgoing(hResultListLink);
    atomSpace.removeAtom(hResultListLink);

    return resultSet;
}

Handle AtomSpaceUtil::getReferenceLink(AtomSpace & atomSpace, Handle hFirstOutgoing) 
{
    // Create BindLink used by pattern matcher
    std::vector<Handle> referenceLinkOutgoings, implicationLinkOutgoings, bindLinkOutgoings; 

    Handle hVariableNode = atomSpace.addNode(VARIABLE_NODE, "$var_any");

    referenceLinkOutgoings.push_back(hFirstOutgoing); 
    referenceLinkOutgoings.push_back(hVariableNode); 
    Handle hReferenceLink = atomSpace.addLink(REFERENCE_LINK, referenceLinkOutgoings,TruthValue::TRUE_TV());

    implicationLinkOutgoings.push_back(hReferenceLink); 
    implicationLinkOutgoings.push_back(hReferenceLink); 
    Handle hImplicationLink = atomSpace.addLink(IMPLICATION_LINK, implicationLinkOutgoings,TruthValue::TRUE_TV());

    bindLinkOutgoings.push_back(hVariableNode);  
    bindLinkOutgoings.push_back(hImplicationLink); 
    Handle hBindLink = atomSpace.addLink(BIND_LINK, bindLinkOutgoings,TruthValue::TRUE_TV());

    // Run pattern matcher
    Handle hResultListLink = bindlink(&atomSpace, hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink, otherwise some scheme script
    //       may fail to remove the ReferenceLink when necessary. 
    //       Because the ReferenceLink would have an incoming (i.e. hResultListLink here), 
    //       which would make cog-delete scheme function fail.  
    std::vector<Handle> resultSet = atomSpace.getOutgoing(hResultListLink); 
    atomSpace.removeAtom(hResultListLink); 

    // Check and return the result
//    foreach(Handle hResult, resultSet) {
//        std::cout<<"Found a ReferenceLink: "<<atomSpace.atomAsString(hResult)<<std::endl; 
//
//        std::vector<Handle> incomingSet = atomSpace.getIncoming(hResult);
//        foreach(Handle hIncoming, incomingSet) {
//            std::cout<<"An incoming: "<< atomSpace.atomAsString(hIncoming)<<std::endl;
//        }
//
//        std::cout<<std::endl;
//    }

    if ( resultSet.size() != 1 ) {
        logger().warn( "AtomSpaceUtil::%s - The number of ReferenceLink containing '%s' should be exactly 1, but got %d", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hFirstOutgoing).c_str(), 
                       resultSet.size()
                     );

        return Handle::UNDEFINED; 
    }
    
   
    return resultSet[0];  
}

Handle AtomSpaceUtil::getReference(AtomSpace & atomSpace, Handle hFirstOutgoing)
{
    Handle hReferenceLink = AtomSpaceUtil::getReferenceLink(atomSpace, hFirstOutgoing); 

    if (hReferenceLink==Handle::UNDEFINED)
        return Handle::UNDEFINED; 
    else 
        return atomSpace.getOutgoing(hReferenceLink, 1); 
}

float AtomSpaceUtil::getCurrentPetFeelingLevel( AtomSpace& atomSpace,
        const std::string& petId,
        const std::string& feeling)
{
    //! @todo This code was copypasted from PAI::addPhysiologicalFeeling. It should use getHandleSet etc
    Handle feelingNode = atomSpace.addNode(PREDICATE_NODE, feeling);

    //! @todo Can only be a "Pet". But we're going to merge the Pet and Humanoid agent-types anyway
    Handle agentNode = atomSpace.addNode(PET_NODE, petId);

    // Add EvaluationLink
    HandleSeq evalLinkOutgoing;
    evalLinkOutgoing.push_back(feelingNode);
    evalLinkOutgoing.push_back(atomSpace.addLink(LIST_LINK, agentNode,TruthValue::TRUE_TV()));
    Handle evalLink = atomSpace.addLink(EVALUATION_LINK, evalLinkOutgoing,TruthValue::TRUE_TV());

    TimeServer& ts = timeServer();
    unsigned long latest = ts.getLatestTimestamp();

    // An error could occur here, but that should never happen
    Temporal t(latest);
    Handle atTime = ts.getAtTimeLink(HandleTemporalPair(evalLink, &t));

    return atomSpace.getTV(atTime)->getMean();
}

float AtomSpaceUtil::getCurrentModulatorLevel(AtomSpace & atomSpace,
                                              const std::string & modulatorName)

{
    float errorValue = randGen().randfloat();   // If error happens, return this value anyway.

    // Get the Handle to GroundSchemaNode
    std::string modulatorUpdater = modulatorName + "ModulatorUpdater";

    Handle hGroundedSchemaNode = atomSpace.getHandle
                                        ( GROUNDED_SCHEMA_NODE, // Type of the Atom wanted
                                          modulatorUpdater      // Name of the Atom wanted
                                        );

    if ( hGroundedSchemaNode == Handle::UNDEFINED ||
         atomSpace.getType(hGroundedSchemaNode) != GROUNDED_SCHEMA_NODE ) {

        logger().warn( "AtomSpaceUtil::%s - Found no GroundSchemaNode named '%s'. Return random value: %f",
                       __FUNCTION__, 
                       modulatorUpdater.c_str(), 
                       errorValue
                     );
        return errorValue; 
    }

    // Get the  HandleSet to ExecutionOutputLink
    std::vector<Handle> executionOutputLinkSet;

    atomSpace.getHandleSet
                  ( back_inserter(executionOutputLinkSet), // return value
                    hGroundedSchemaNode,      // returned link should contain this node
                    EXECUTION_OUTPUT_LINK,    // type of the returned link 
                    false                     // subclass is not acceptable, 
                                              // i.e. returned link should be exactly of 
                  );                          // type EXECUTION_OUTPUT_LINK

    // Get ExecutionOutputLink
    if (executionOutputLinkSet.size() !=1 ) {
        logger().error("AtomSpaceUtil::%s - Number of ExecutionOutputLink should be exactly 1, but got %d.", 
                       executionOutputLinkSet.size(), 
                       __FUNCTION__
                      ); 
        return false; 
    }

    Handle hExecutionOutputLink = executionOutputLinkSet[0];

//    // Pick up the ExecutionOutputLink containing ListLink
//    std::vector<Handle>::iterator iExecutionOutputLink;
//
//    for(iExecutionOutputLink = executionOutputLinkSet.begin(); 
//        iExecutionOutputLink != executionOutputLinkSet.end(); 
//        ++ iExecutionOutputLink ) {
//
//        if ( atomSpace.getArity(*iExecutionOutputLink) == 2 &&
//             atomSpace.getType( atomSpace.getOutgoing(*iExecutionOutputLink, 1) ) == LIST_LINK
//           )
//            break; 
//    }
//
//    if ( iExecutionOutputLink == executionOutputLinkSet.end() ) {
//        logger().error("AtomSpaceUtil::%s - Failed to find a ExecutionOutputLink that takes ListLink as its second outgoing.", 
//                       __FUNCTION__
//                      ); 
//        return false; 
//    }
//
//    Handle hExecutionOutputLink = *iExecutionOutputLink;

    // Get all the SimilarityLink that contains the ExecutionOutputLink
    //
    // AtTimeLink
    //     TimeNode "timestamp"
    //     SimilarityLink
    //         NumberNode "modulator_value"
    //         ExecutionOutputLink
    //            ...
    
    std::vector<Handle> similarityLinkSet;

    atomSpace.getHandleSet(back_inserter(similarityLinkSet), hExecutionOutputLink, SIMILARITY_LINK, false);

    logger().debug("AtomSpaceUtil::%s - Get %d SimilarityLink that holds modulator updater (ExecutionOutputLink)", 
                   __FUNCTION__, 
                   similarityLinkSet.size()
                  );

    // Get all the HandleTemporalPairs of SimilarityLink
    std::vector<HandleTemporalPair> handleTemporalPairs;

    foreach(Handle hSimilarityLink, similarityLinkSet) {
        timeServer().getTimeInfo( back_inserter(handleTemporalPairs),
                                               hSimilarityLink
                                             );
    }

    logger().debug("AtomSpaceUtil::%s - Get %d HandleTemporalPairs of SimilarityLink that holds the modulator updater (ExecutionOutputLink)", 
                   __FUNCTION__, 
                   handleTemporalPairs.size()
                  );

    // Pick up the HandleTemporalPair with latest temporal (greatest temporal)
    // Note: 
    //     1. We assume the SimilarityLink contains a NumberNode without checking, 
    //        if not there's a chance that it would find a latest SimilarityLink without a NUmberNode, 
    //        this might cause some problem
    //     2. you can use std::sort and HandleTemporalPairEntry::SortComparison to sort the whole HandleTemporalPairs
    std::vector<HandleTemporalPair>::iterator iHandleTemporalPair;
    std::vector<HandleTemporalPair>::iterator iLatestHandleTemporalPair;

    iHandleTemporalPair = handleTemporalPairs.begin();
    iLatestHandleTemporalPair = handleTemporalPairs.begin(); 

    while ( iHandleTemporalPair != handleTemporalPairs.end() ) {

        if ( HandleTemporalPairEntry::handleTemporalPairCompare
                (&*iHandleTemporalPair,
                 &*iLatestHandleTemporalPair) > 0 ) {
            iLatestHandleTemporalPair = iHandleTemporalPair;
        }

        iHandleTemporalPair ++; 
    }

    if ( iLatestHandleTemporalPair == handleTemporalPairs.end() ) {
        logger().warn("AtomSpaceUtil::%s - Failed to find the latest HandleTemporalPair that contains '%s'. Return random value: %f", 
                      __FUNCTION__, 
                      atomSpace.atomAsString(hExecutionOutputLink).c_str(), 
                      errorValue
                     );
        return errorValue;
    }

    // Get the latest NumberNode
    Handle hLatestSimilarityLink = iLatestHandleTemporalPair->getHandle();

    if ( atomSpace.getArity(hLatestSimilarityLink) != 2 ) {
        logger().warn("AtomSpaceUtil::%s - The arity of SimilarityLink holding the modulator value (NumberNode) and modulator updater (ExecutionOutputLink) should be exactly 2. But Got %d.", 
                      __FUNCTION__, 
                      atomSpace.getArity(hLatestSimilarityLink)
                     );
        return errorValue; 
    }

    logger().debug("AtomSpaceUtil::%s - Get the latest SimilarityLink '%s' for modulator '%s'", 
                   __FUNCTION__, 
                   atomSpace.atomAsString(hLatestSimilarityLink).c_str(), 
                   modulatorName.c_str()
                  );

    Handle hNumberNode = atomSpace.getOutgoing(hLatestSimilarityLink, 0);

    if ( atomSpace.getType(hNumberNode) != NUMBER_NODE )
        hNumberNode = atomSpace.getOutgoing(hLatestSimilarityLink, 1);

    if ( atomSpace.getType(hNumberNode) != NUMBER_NODE ) {
        logger().warn("AtomSpaceUtil::%s - Failed to find the NumberNode containing the latest modulator value. Return random value: %f.",
                       __FUNCTION__, 
                       errorValue
                     );
        return errorValue; 
    }

    // Return the latest modulator value
    return boost::lexical_cast<float> ( atomSpace.getName(hNumberNode) );
}

float AtomSpaceUtil::getCurrentDemandLevel(AtomSpace & atomSpace,
                                           const std::string & demandName)
{
    float errorValue = randGen().randfloat();   // If error happens, return this value anyway.

    // Get the Handle to GroundSchemaNode
    std::string demandUpdater = demandName + "DemandUpdater";

    Handle hGroundedSchemaNode = atomSpace.getHandle
                                        ( GROUNDED_SCHEMA_NODE, // Type of the Atom wanted
                                          demandUpdater         // Name of the Atom wanted
                                        );

    if ( hGroundedSchemaNode == Handle::UNDEFINED ||
         atomSpace.getType(hGroundedSchemaNode) != GROUNDED_SCHEMA_NODE ) {

        logger().warn( "AtomSpaceUtil::%s - Found no GroundSchemaNode named '%s'. Return random value: %f",
                       __FUNCTION__, 
                       demandUpdater.c_str(), 
                       errorValue
                     );
        return errorValue; 
    }

    // Get the  HandleSet to ExecutionOutputLink
    std::vector<Handle> executionOutputLinkSet;

    atomSpace.getHandleSet
                  ( back_inserter(executionOutputLinkSet), // return value
                    hGroundedSchemaNode,      // returned link should contain this node
                    EXECUTION_OUTPUT_LINK,    // type of the returned link 
                    false                     // subclass is not acceptable, 
                                              // i.e. returned link should be exactly of 
                  );                          // type EXECUTION_OUTPUT_LINK

    // Pick up the ExecutionOutputLink containing ListLink
    std::vector<Handle>::iterator iExecutionOutputLink;

    for(iExecutionOutputLink = executionOutputLinkSet.begin(); 
        iExecutionOutputLink != executionOutputLinkSet.end(); 
        ++ iExecutionOutputLink ) {

        if ( atomSpace.getArity(*iExecutionOutputLink) == 2 &&
             atomSpace.getType( atomSpace.getOutgoing(*iExecutionOutputLink, 1) ) == LIST_LINK
           )
            break; 
    }

    if ( iExecutionOutputLink == executionOutputLinkSet.end() ) {
        logger().error("AtomSpaceUtil::%s - Failed to find a ExecutionOutputLink that takes ListLink as its second outgoing.", 
                       __FUNCTION__
                      ); 
        return false; 
    }

    Handle hExecutionOutputLink = *iExecutionOutputLink;

    // Get all the SimilarityLink that contains the ExecutionOutputLink
    //
    // AtTimeLink
    //     TimeNode "timestamp"
    //     SimilarityLink
    //         NumberNode "demand_level"
    //         ExecutionOutputLink
    //            ...
    
    std::vector<Handle> similarityLinkSet;

    atomSpace.getHandleSet(back_inserter(similarityLinkSet), hExecutionOutputLink, SIMILARITY_LINK, false);

    logger().debug("AtomSpaceUtil::%s - Get %d SimilarityLink that holds demand updater (ExecutionOutputLink)", 
                   __FUNCTION__, 
                   similarityLinkSet.size()
                  );

    // Get all the HandleTemporalPairs of SimilarityLink
    std::vector<HandleTemporalPair> handleTemporalPairs;

    foreach(Handle hSimilarityLink, similarityLinkSet) {
        timeServer().getTimeInfo( back_inserter(handleTemporalPairs),
                                               hSimilarityLink
                                             );
    }

    logger().debug("AtomSpaceUtil::%s - Get %d HandleTemporalPairs of SimilarityLink that holds the demand updater (ExecutionOutputLink)", 
                   __FUNCTION__, 
                   handleTemporalPairs.size()
                  );

    // Pick up the HandleTemporalPair with latest temporal (greatest temporal)
    // Note: 
    //     1. We assume the SimilarityLink contains a NumberNode without checking, 
    //        if not there's a chance that it would find a latest SimilarityLink without a NUmberNode, 
    //        this might cause some problem
    //     2. you can use std::sort and HandleTemporalPairEntry::SortComparison to sort the whole HandleTemporalPairs
    std::vector<HandleTemporalPair>::iterator iHandleTemporalPair;
    std::vector<HandleTemporalPair>::iterator iLatestHandleTemporalPair;

    iHandleTemporalPair = handleTemporalPairs.begin();
    iLatestHandleTemporalPair = handleTemporalPairs.begin(); 

    while ( iHandleTemporalPair != handleTemporalPairs.end() ) {

        if ( HandleTemporalPairEntry::handleTemporalPairCompare
                (&*iHandleTemporalPair,
                 &*iLatestHandleTemporalPair) > 0 ) {
            iLatestHandleTemporalPair = iHandleTemporalPair;
        }

        iHandleTemporalPair ++; 
    }

    if ( iLatestHandleTemporalPair == handleTemporalPairs.end() ) {
        logger().warn("AtomSpaceUtil::%s - Failed to find the latest HandleTemporalPair that contains '%s'. Return random value: %f", 
                      __FUNCTION__, 
                      atomSpace.atomAsString(hExecutionOutputLink).c_str(), 
                      errorValue
                     );
        return errorValue;
    }

    // Get the latest NumberNode
    Handle hLatestSimilarityLink = iLatestHandleTemporalPair->getHandle();

    if ( atomSpace.getArity(hLatestSimilarityLink) != 2 ) {
        logger().warn("AtomSpaceUtil::%s - The arity of SimilarityLink holding the demand level (NumberNode) and demand updater (ExecutionOutputLink) should be exactly 2. But Got %d.", 
                      __FUNCTION__, 
                      atomSpace.getArity(hLatestSimilarityLink)
                     );
        return errorValue; 
    }

    logger().debug("AtomSpaceUtil::%s - Get the latest SimilarityLink '%s' for demand '%s'", 
                   __FUNCTION__, 
                   atomSpace.atomAsString(hLatestSimilarityLink).c_str(), 
                   demandName.c_str()
                  );

    Handle hNumberNode = atomSpace.getOutgoing(hLatestSimilarityLink, 0);

    if ( atomSpace.getType(hNumberNode) != NUMBER_NODE )
        hNumberNode = atomSpace.getOutgoing(hLatestSimilarityLink, 1);

    if ( atomSpace.getType(hNumberNode) != NUMBER_NODE ) {
        logger().warn("AtomSpaceUtil::%s - Failed to find the NumberNode containing the latest demand level. Return random value: %f instead.", 
                       __FUNCTION__, 
                       errorValue
                     );
        return errorValue; 
    }

    // Return the latest demand value
    return boost::lexical_cast<float> ( atomSpace.getName(hNumberNode) );
}

Handle AtomSpaceUtil::getDemandGoalEvaluationLink(AtomSpace & atomSpace, 
                                                  const std::string & demand 
                                                 )
{
    // Get the PredicateNode
    Handle predicateNode = atomSpace.getHandle(PREDICATE_NODE, demand+"DemandGoal");

    if ( predicateNode == opencog::Handle::UNDEFINED ) {
        logger().error("AtomSpaceUtil::%s - Failed to get the PredicateNode for demand goal '%s'", 
                       __FUNCTION__, 
                       demand.c_str()
                      );
        return opencog::Handle::UNDEFINED;  
    }

    // Get the ListLink
    // Note: The ListLink of Demand Goal is empty currently, but this may changes in future. 
    std::vector<Handle> listLinkOutgoing; 

    Handle listLink = atomSpace.getHandle(LIST_LINK, listLinkOutgoing); 

    if ( listLink == opencog::Handle::UNDEFINED ) {
        logger().error("AtomSpaceUtil::%s - Failed to get the ListLink for demand goal '%s'", 
                        __FUNCTION__, 
                        demand.c_str()
                      );
        return opencog::Handle::UNDEFINED; 
    }

    // Get the EvaluationLink
    std::vector<Handle> evaluationLinkOutgoing;

    evaluationLinkOutgoing.push_back(predicateNode); 
    evaluationLinkOutgoing.push_back(listLink); 

    Handle evaluationLink = atomSpace.getHandle(EVALUATION_LINK, evaluationLinkOutgoing);

    if ( evaluationLink == opencog::Handle::UNDEFINED ) {
        logger().error("AtomSpaceUtil::%s - Failed to get the EvaluationLink for demand goal '%s'", 
                       __FUNCTION__, 
                       demand.c_str()
                      ); 
        return opencog::Handle::UNDEFINED;  
    }

    // Return the Handle to the demand goal (EvaluationLink)
    return evaluationLink; 
}

void AtomSpaceUtil::getAllEvaluationLinks(const AtomSpace& atomSpace,
        std::vector<HandleTemporalPair>& timestamps,
        const std::string& predicateNodeName,
        const Temporal& temporal,
        TemporalTable::TemporalRelationship criterion,
        bool needSort)
{

    logger().fine("AtomSpaceUtil - getAllEvaluationLinks - "
            "Searching for PredicateNode '%s'.",
            predicateNodeName.c_str());

    std::vector<Handle> handles;
    atomSpace.getHandleSet(back_inserter(handles),
                           predicateNodeName.c_str(),
                           PREDICATE_NODE,
                           EVALUATION_LINK, true);

    if (handles.empty()) {
        logger().debug("AtomSpaceUtil - getAllEvaluationLinks - "
                "Found no EvaluationLink for PredicateNode '%s'",
                predicateNodeName.c_str());
        return;
    }

    for ( unsigned int i = 0; i < handles.size(); ++i ) {
        timeServer().getTimeInfo(back_inserter(timestamps),
                              handles[i], temporal, criterion);
    }

    if (needSort) {
        std::sort(timestamps.begin(),
                  timestamps.end(),
                  HandleTemporalPairEntry::SortComparison());
    }

    logger().fine("AtomSpaceUtil - getAllEvaluationLinks - end.");
}

Handle AtomSpaceUtil::setPredicateValue( AtomSpace& atomSpace,
        std::string predicateName,
        TruthValuePtr tv,
        Handle object1,
        Handle object2,
        Handle object3 )
{
    HandleSeq listLink;

    if ( object1 != Handle::UNDEFINED ) {
        listLink.push_back( object1 ); 
    }
    if ( object2 != Handle::UNDEFINED ) {
        listLink.push_back( object2 );
    }
    if ( object3 != Handle::UNDEFINED ) {
        listLink.push_back( object3 );
    }

    Handle listLinkHandle = AtomSpaceUtil::addLink(atomSpace, LIST_LINK, listLink);

    Handle predicateHandle = AtomSpaceUtil::addNode(atomSpace,
                                                    PREDICATE_NODE,
                                                    predicateName, 
                                                    true
                                                   );
    HandleSeq evalLink;

    evalLink.push_back(predicateHandle);

    if ( !listLink.empty() )
        evalLink.push_back(listLinkHandle);

    Handle evalLinkHandle = AtomSpaceUtil::addLink(atomSpace,
                                                   EVALUATION_LINK,
                                                   evalLink,
                                                   true
                                                  );
    atomSpace.setTV(evalLinkHandle, tv);

    return evalLinkHandle;
}

Handle AtomSpaceUtil::addPropertyPredicate(AtomSpace& atomSpace,
        std::string predicateName,
        Handle object,
        TruthValuePtr tv,
        bool permanent,
        const Temporal &t)
{
    HandleSeq ll_out;
    ll_out.push_back(object);
    return addGenericPropertyPred(atomSpace, predicateName,
                                  ll_out, tv, permanent, t);
}

Handle AtomSpaceUtil::addPropertyPredicate(AtomSpace& atomSpace,
        std::string predicateName,
        Handle a,
        Handle b,
        TruthValuePtr tv,
        const Temporal& t)
{

    HandleSeq ll_out;
    ll_out.push_back(a);
    ll_out.push_back(b);
    return addGenericPropertyPred(atomSpace,
                                  predicateName,
                                  ll_out, tv, false, t);
}

void AtomSpaceUtil::setupHoldingObject( AtomSpace& atomSpace,
                                        const std::string& holderId,
                                        const std::string& objectId,
                                        long unsigned currentTimestamp )
{

    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) return;

    Handle objectHandle = Handle::UNDEFINED;
    if ( objectId != "" ) {
        objectHandle = atomSpace.getHandle( OBJECT_NODE, objectId );
        if ( objectHandle != Handle::UNDEFINED ) {
            logger().debug("AtomSpaceUtil - Object is a OBJECT_NODE");
        } else {
            objectHandle = atomSpace.getHandle( ACCESSORY_NODE, objectId );
            if ( objectHandle != Handle::UNDEFINED ) {
                logger().debug("AtomSpaceUtil - Object is a ACCESSORY_NODE");
            } else {
                logger().error("AtomSpaceUtil - Object cannot be identified");
                return;
            }
        }
    }

    if (objectHandle == Handle::UNDEFINED) {
        // drop operation
        logger().debug("AtomSpaceUtil - The object's handle is undefined. "
                "Holder dropped the object");
        Handle isHoldingOldAtTimeLink = getMostRecentIsHoldingAtTimeLink(atomSpace, holderId);
        if (isHoldingOldAtTimeLink != Handle::UNDEFINED) {
            // Reconfigure the timestamp of the last grabbed object
            Handle isHoldingEvalLink = getTimedHandle(atomSpace,
                                       isHoldingOldAtTimeLink);
            if ( isHoldingEvalLink != Handle::UNDEFINED ) {
                Temporal t = getTemporal(atomSpace, isHoldingOldAtTimeLink);
                // TODO: What if it was not holding anything anymore (this
                // happens if it gets 2 or more consecutive drop operations)
                // Shouldn't the old isHoldingatTimeLink be updated?

                // Remove the old AtTimeLink
                atomSpace.removeAtom(isHoldingOldAtTimeLink);
                // Add the new one
                long unsigned tl = t.getLowerBound();
                Temporal new_temp(tl, std::max(currentTimestamp - 20, tl));
                logger().debug("AtomSpaceUtil - setupHoldingObject: "
                        "new time = '%s'", new_temp.toString().c_str());
                 // Now, it can be forgotten.
                timeServer().addTimeInfo(isHoldingEvalLink, new_temp);
            }
        }
        AtomSpaceUtil::setPredicateValue( atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          SimpleTruthValue::createTV( 0.0, 1.0 ),
                                          holderHandle );
    } else {
        // grab: it is now holding an object
        // TODO: What if it is already holding another thing? Shouldn't the old
        // isHoldingatTimeLink be updated?
        logger().debug("AtomSpaceUtil - Now '%s' is holding '%s' at '%ul'",
                     atomSpace.getName(holderHandle).c_str(),
                     atomSpace.getName(objectHandle).c_str(),
                     currentTimestamp);
        AtomSpaceUtil::setPredicateValue( atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          SimpleTruthValue::createTV( 1.0, 1.0 ),
                                          holderHandle );
        Handle isHoldingAtTimeLink =
            AtomSpaceUtil::addPropertyPredicate( atomSpace,
                                                 IS_HOLDING_PREDICATE_NAME,
                                                 holderHandle,
                                                 objectHandle,
                                                 SimpleTruthValue::createTV( 1.0, 1.0 ),
                                                 Temporal(currentTimestamp) );
        // Now, it cannot be forgotten (until the agent drop the object)
        // TODO: this is untrue, it should use VLTI
        atomSpace.setLTI(isHoldingAtTimeLink, 1);
    }
}

Handle AtomSpaceUtil::getLatestHoldingObjectHandle(AtomSpace& atomSpace,
        const std::string& holderId )
{

    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) {
        return Handle::UNDEFINED;
    }

    Handle isHoldingLink = getMostRecentIsHoldingLink( atomSpace, holderId );
    if ( isHoldingLink != Handle::UNDEFINED ) {
        Handle listLink = atomSpace.getOutgoing(isHoldingLink, 1);
        if ( listLink != Handle::UNDEFINED ) {
            Handle objectHandle = atomSpace.getOutgoing(listLink, 1);
            if ( objectHandle != Handle::UNDEFINED ) return objectHandle;

            logger().error("AtomSpaceUtil - There is no object on list link");
        }
        logger().error("AtomSpaceUtil - There is no listlink on isHoldingLink");
    }

    logger().debug("AtomSpaceUtil - There is no isHoldingLink for %s",
                  holderId.c_str() );
    return Handle::UNDEFINED;
}

bool AtomSpaceUtil::isObjectBeingHolded( AtomSpace& atomSpace,
        const std::string& objectId )
{
    return ( getObjectHolderHandle( atomSpace, objectId ) != Handle::UNDEFINED );
}

Handle AtomSpaceUtil::getObjectHolderHandle( AtomSpace& atomSpace,
        const std::string& objectId )
{
    // TODO: try to optimize this method. It is using getHandleSet twice for
    // isHolding (below and through getLatestHoldingObjectHandle)
    std::vector<Handle> handles;
    atomSpace.getHandlesByName( back_inserter(handles), objectId, OBJECT_NODE, true );

    if (handles.size() != 1) {
        logger().debug("AtomSpaceUtil - No agent is holding object[%s]",
                     objectId.c_str() );
        return Handle::UNDEFINED;
    }
    Handle holdedObjectHandle = handles[0];

    handles.clear( );
    atomSpace.getHandleSet( back_inserter(handles),
                            "isHolding",
                            PREDICATE_NODE,
                            EVALUATION_LINK, true );

    std::vector<HandleTemporalPair> timestamps;
    for ( unsigned int i = 0; i < handles.size(); ++i ) {
        Handle listLink = atomSpace.getOutgoing(handles[i], 1);
        if ( listLink != Handle::UNDEFINED ) {
            if ( atomSpace.getOutgoing(listLink, 1) == holdedObjectHandle ) {
                // skip other holded objects which aren't that we're seeking
                continue;
            } // if

            Handle holderHandle = atomSpace.getOutgoing(listLink, 0 );
            if ( !AtomSpaceUtil::isPredicateTrue( atomSpace,
                                                  IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                                  holderHandle ) ) {
                // skip links which the holder isn't holding something now
                continue;
            } // if

            if ( getLatestHoldingObjectHandle(atomSpace,
                                              atomSpace.getName(holderHandle))
                    != holdedObjectHandle ) {
                // skip objects that are holding by the same holder of the target object
                // but arent't the target
                continue;
            } // if

            timeServer().getTimeInfo( back_inserter( timestamps ), handles[i]);


        } // if
    } // for

    // get the most recent eval link
    if ( timestamps.size( ) > 0 ) {
        int mostRecentIndex = 0;
        Temporal *mostRecent = timestamps[mostRecentIndex].getTemporal();
        for (unsigned int i = 1; i < timestamps.size(); i++) {
            if (*(timestamps[i].getTemporal()) > *mostRecent) {
                mostRecent = timestamps[i].getTemporal();
                mostRecentIndex = i;
            } // if
        } // for

        Handle listLink = atomSpace.getOutgoing(timestamps[mostRecentIndex].getHandle( ), 1);
        return atomSpace.getOutgoing(listLink, 0);
    } // if

    return Handle::UNDEFINED;
}

std::string AtomSpaceUtil::getObjectHolderId( AtomSpace& atomSpace,
        const std::string& objectId )
{
    Handle objectHandle = getObjectHolderHandle( atomSpace, objectId );
    if ( objectHandle != Handle::UNDEFINED ) {
        return atomSpace.getName( objectHandle );
    } // if
    return "";
}


Handle AtomSpaceUtil::getMostRecentIsHoldingAtTimeLink(AtomSpace& atomSpace,
        const std::string& holderId )
{
    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) {
        return Handle::UNDEFINED;
    } // if

    std::vector<Handle> handles;
    atomSpace.getHandleSet( back_inserter(handles),
                            IS_HOLDING_PREDICATE_NAME,
                            PREDICATE_NODE, EVALUATION_LINK, true );

    std::vector<HandleTemporalPair> timestamps;
    for ( unsigned int i = 0; i < handles.size(); ++i ) {
        Handle listLink = atomSpace.getOutgoing(handles[i], 1);
        if ( listLink != Handle::UNDEFINED ) {
            // get only holder's eval-links
            if ( atomSpace.getOutgoing(listLink, 0) == holderHandle ) {
                timeServer().getTimeInfo( back_inserter( timestamps ),
                                       handles[i]);
            }
        }
    }

    // get the most recent eval link
    if ( timestamps.size( ) > 0 ) {
        int mostRecentIndex = 0;
        Temporal *mostRecent = timestamps[mostRecentIndex].getTemporal();
        for (unsigned int i = 1; i < timestamps.size(); i++) {
            if (*(timestamps[i].getTemporal()) > *mostRecent) {
                mostRecent = timestamps[i].getTemporal();
                mostRecentIndex = i;
            }
        }

        logger().debug("AtomSpaceUtil - The most recent holded object %ul",
                      timestamps[mostRecentIndex].getTemporal()->getUpperBound() );

        return timeServer().getAtTimeLink(timestamps[mostRecentIndex]);
    }
    return Handle::UNDEFINED;
}

Handle AtomSpaceUtil::getMostRecentIsHoldingLink(AtomSpace& atomSpace,
        const std::string& holderId )
{
    Handle h = getMostRecentIsHoldingAtTimeLink(atomSpace, holderId);
    if (h != Handle::UNDEFINED)
        return atomSpace.getOutgoing(h, 1);
    else return Handle::UNDEFINED;
}

std::string AtomSpaceUtil::getHoldingObjectId(AtomSpace& atomSpace,
        const std::string& holderId )
{
    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) return "";

    if ( !AtomSpaceUtil::isPredicateTrue( atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          holderHandle ) ) {
        return "";
    }

    Handle objectHandle = getLatestHoldingObjectHandle( atomSpace, holderId );
    if ( objectHandle == Handle::UNDEFINED ) return "";

    return atomSpace.getName(objectHandle);
}

bool AtomSpaceUtil::isHoldingSomething(AtomSpace& atomSpace,
                                       const std::string& holderId)
{
    Handle holderHandle = getAgentHandle(atomSpace, holderId);
    if (holderHandle == Handle::UNDEFINED) return false;

    return AtomSpaceUtil::isPredicateTrue(atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          holderHandle);
}

Handle AtomSpaceUtil::getHoldingObjectHandleAtTime(AtomSpace& atomSpace,
        const std::string& holderId,
        unsigned long time)
{
    Handle isHoldingLink = getIsHoldingLinkAtTime(atomSpace,
                           holderId,
                           time);
    if ( isHoldingLink != Handle::UNDEFINED ) {
        Handle listLink = atomSpace.getOutgoing(isHoldingLink, 1);
        if ( listLink != Handle::UNDEFINED ) {
            Handle objectHandle = atomSpace.getOutgoing(listLink, 1);
            if ( objectHandle != Handle::UNDEFINED ) {
                return objectHandle;
            }
            logger().error("AtomSpaceUtil - There is no object on list link");
        }
        logger().error("AtomSpaceUtil - There is no listlink on isHoldingLink");
    }

    logger().debug("AtomSpaceUtil - There is no isHoldingLink for %s",
                  holderId.c_str() );
    return Handle::UNDEFINED;
}

Handle AtomSpaceUtil::getIsHoldingLinkAtTime(AtomSpace& atomSpace,
        const std::string& holderId,
        unsigned long time)
{
    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) return Handle::UNDEFINED;

    std::vector<Handle> handles;
    atomSpace.getHandleSet(back_inserter(handles), IS_HOLDING_PREDICATE_NAME,
                           PREDICATE_NODE, EVALUATION_LINK, true);

    std::vector<HandleTemporalPair> timestamps;
    for (std::vector<Handle>::const_iterator h_i = handles.begin();
            h_i != handles.end(); ++h_i) {
        Handle listLink = atomSpace.getOutgoing(*h_i, 1);
        OC_ASSERT(listLink != Handle::UNDEFINED,
                "ListLink must be defined");
        OC_ASSERT(atomSpace.getArity(listLink) == 2,
                "IsHolding predicate must have 2 arguments");
        // get only holder's eval-links occuring within 'time'

        if (atomSpace.getOutgoing(listLink, 0) == holderHandle) {
            logger().debug(
                         "AtomSpaceUtil - before '%d' timestamps for isHolding pred for '%s'.",
                         timestamps.size(), holderId.c_str());

            timeServer().getTimeInfo(back_inserter(timestamps), *h_i,
                                  Temporal(time), TemporalTable::INCLUDES);

            logger().debug(
                         "AtomSpaceUtil - after '%d' timestamps for isHolding pred for '%s'.",
                         timestamps.size(), holderId.c_str());

        } // if
    } // for


    if (!timestamps.empty()) {
        unsigned int ts = timestamps.size();
        //there should be only one element because someone can only carry
        //one object at a time
        if (ts != 1) {
            std::string s("It is assumed that someone can only carry 1 object at a time and it seems that ");
            s += holderId;
            s += std::string(" carries ") + boost::lexical_cast<std::string>(ts) + std::string(" objects ");
            s += std::string("at time ") + boost::lexical_cast<std::string>(time);
            s += std::string(" listed as ");
            for (std::vector<HandleTemporalPair>::iterator tsi = timestamps.begin();
                    tsi != timestamps.end(); ++tsi) {
                s +=  std::string("'") + tsi->toString() + std::string("' ");
            }
            OC_ASSERT(false, s.c_str());
        }
        return timestamps[0].getHandle();
    } else return Handle::UNDEFINED;
}

std::string AtomSpaceUtil::getHoldingObjectIdAtTime(AtomSpace& as,
        const std::string& holderId,
        unsigned long time)
{
    Handle holderHandle = getAgentHandle( as, holderId );
    if ( holderHandle == Handle::UNDEFINED ) return "";

    Handle objectHandle = getHoldingObjectHandleAtTime(as, holderId, time);
    if ( objectHandle == Handle::UNDEFINED ) return "";

    return as.getName(objectHandle);
}

std::string AtomSpaceUtil::getObjectName( const AtomSpace& atomSpace, 
                                          Handle object )
{
    std::string name("");

    if ( object != Handle::UNDEFINED ) {
        HandleSeq objectName(2);
        objectName[0] = Handle::UNDEFINED;
        objectName[1] = object;
        Type types[] = { WORD_NODE, atomSpace.getType( object ) };
        HandleSeq wrLinks;
        atomSpace.getHandlesByOutgoing( back_inserter(wrLinks), objectName,
                                &types[0], NULL, 2, WR_LINK, false );
        if ( wrLinks.size( ) > 0 ) {
            name = atomSpace.getName( atomSpace.getOutgoing( wrLinks[0], 0 ) );
        } // if
    } // if

    return name;
}

std::string AtomSpaceUtil::getObjIdFromName( AtomSpace& atomSpace,
        const std::string& objName )
{
    std::string result;
    Handle objIdHandle = Handle::UNDEFINED;
    Handle objNameHandle = atomSpace.getHandle(WORD_NODE, objName);
    if (objNameHandle != Handle::UNDEFINED) {
        HandleSeq outgoing;
        outgoing.push_back(objNameHandle);
        outgoing.push_back(Handle::UNDEFINED);
        HandleSeq wrLinks;
        atomSpace.getHandlesByOutgoing(back_inserter(wrLinks), outgoing,
                               NULL, NULL, 2, WR_LINK, false);
        if (!wrLinks.empty()) {
            objIdHandle = atomSpace.getOutgoing(wrLinks[0], 1);
            // TODO: check for multiple answers...
        } else {
            logger().debug("AtomSpaceUtil::getObjIdFromName: "
                    "Found avatar name '%s' but no avatar id associated to this name.",
                    objName.c_str());
        }
    } else {
        logger().debug("AtomSpaceUtil::getObjIdFromName: "
                "didn't find avatar name '%s'", objName.c_str());
    }
    if (objIdHandle == Handle::UNDEFINED) {
        // try all letters in lowercase
        std::string lcObjName = objName;
        std::transform(lcObjName.begin(), lcObjName.end(),
                       lcObjName.begin(), (int(*)(int)) std::tolower);
        objNameHandle = atomSpace.getHandle(WORD_NODE, lcObjName);
        if (objNameHandle != Handle::UNDEFINED) {
            HandleSeq outgoing;
            outgoing.push_back(objNameHandle);
            outgoing.push_back(Handle::UNDEFINED);
            HandleSeq wrLinks;
            atomSpace.getHandlesByOutgoing(back_inserter(wrLinks), outgoing,
                                   NULL, NULL, 2, WR_LINK, false);
            if (!wrLinks.empty()) {
                objIdHandle = atomSpace.getOutgoing(wrLinks[0], 1);
                // TODO: check for multiple answers...
            } else {
                logger().debug("AtomSpaceUtil::getObjIdFromName: "
                        "Found avatar name '%s' but no avatar id associated "
                        "with this name.", lcObjName.c_str());
            }
        } else {
            logger().debug("AtomSpaceUtil::getObjIdFromName: "
                    "didn't find avatar name '%s'", lcObjName.c_str());
        }
    }
    if (objIdHandle == Handle::UNDEFINED) {
        // try only the first letter in uppercase, the remaining ones in lowercase
        std::string firstCapObjName = objName;
        std::transform(firstCapObjName.begin(), firstCapObjName.end(),
                       firstCapObjName.begin(), (int(*)(int)) std::tolower);
        firstCapObjName[0] = toupper(firstCapObjName[0]);
        objNameHandle = atomSpace.getHandle(WORD_NODE, firstCapObjName);
        if (objNameHandle != Handle::UNDEFINED) {
            HandleSeq outgoing;
            outgoing.push_back(objNameHandle);
            outgoing.push_back(Handle::UNDEFINED);
            HandleSeq wrLinks;
            atomSpace.getHandlesByOutgoing(back_inserter(wrLinks), outgoing,
                                   NULL, NULL, 2, WR_LINK, false);
            if (!wrLinks.empty()) {
                objIdHandle = atomSpace.getOutgoing(wrLinks[0], 1);
                // TODO: check for multiple answers...
            } else {
                logger().debug("AtomSpaceUtil::getObjIdFromName: "
                        "Found avatar name '%s' but no avatar id associated to this name.",
                        firstCapObjName.c_str());
            }
        } else {
            logger().debug("AtomSpaceUtil::getObjIdFromName: "
                    "didn't find avatar name '%s'",
                    firstCapObjName.c_str());
        }
    }
    if (objIdHandle != Handle::UNDEFINED) {
        result = atomSpace.getName(objIdHandle);
    }
    logger().debug("AtomSpaceUtil::getObjIdFromName: returning '%s'",
                  result.c_str());
    return result;
}

Handle AtomSpaceUtil::getMostRecentPetSchemaExecLink(const AtomSpace& atomSpace,
        unsigned long timestamp,
        bool schemaSuccessful)
{
    logger().debug("AtomSpaceUtil - getMostRecentPetSchemaExecLink");
    std::vector<HandleTemporalPair> timestamps;
    Temporal t(timestamp);

    if (schemaSuccessful) {
        AtomSpaceUtil::getAllEvaluationLinks(atomSpace,
                                             timestamps,
                                             "SchemaDone",
                                             t, TemporalTable::STARTS_AFTER,
                                             true);
    } else {
        AtomSpaceUtil::getAllEvaluationLinks(atomSpace,
                                             timestamps,
                                             "SchemaFailure",
                                             t, TemporalTable::STARTS_AFTER,
                                             true);
    }
    if (timestamps.empty()) return Handle::UNDEFINED;

    for (int i = timestamps.size() - 1; i > 0; --i) {
        Handle evalLink = timestamps[i].getHandle();
        if (evalLink != Handle::UNDEFINED) {
            Handle listLink = atomSpace.getOutgoing(evalLink, 1);
            if (listLink != Handle::UNDEFINED) {
                Handle execLink = atomSpace.getOutgoing(listLink, 0);
                if (execLink != Handle::UNDEFINED
                        && atomSpace.getType(execLink) == EXECUTION_LINK) {
                    return execLink;
                }
            }
        }
    }
    return Handle::UNDEFINED;
}

std::string AtomSpaceUtil::convertPetExecLinkParametersToString(const AtomSpace& atomSpace,
        Handle execLink)
{

    if (execLink != Handle::UNDEFINED) {
        Handle listLink = atomSpace.getOutgoing(execLink, 1);
        if (listLink == Handle::UNDEFINED) return "";

        std::stringstream parameters;
        for (int i = 0; i < atomSpace.getArity(listLink); i++ ) {
            Handle schemaParam = atomSpace.getOutgoing(listLink, i);

            if (schemaParam == Handle::UNDEFINED) {
                logger().error("AtomSpaceUtil - Found no param for schema");
                return "";
            }

            if (i > 0) parameters << ", ";

            if (atomSpace.getType(schemaParam) == LIST_LINK) {
                // rotation or vector
                parameters << atomSpace.getName(atomSpace.getOutgoing(schemaParam, 0));
                parameters << ", ";
                parameters << atomSpace.getName(atomSpace.getOutgoing(schemaParam, 1));
                parameters << ", ";
                parameters << atomSpace.getName(atomSpace.getOutgoing(schemaParam, 2));
            } else {
                // entity
                parameters << atomSpace.getName(schemaParam);
            }
        } // for
        return parameters.str( );
    } // if
    return "";
}

Handle AtomSpaceUtil::getMostRecentAgentActionLink(const AtomSpace& atomSpace,
        const std::string& agentId,
        const Temporal& temporal,
        TemporalTable::TemporalRelationship criterion)
{
    // reference: http://wiki.opencog.org/w/PerceptionActionInterface
    
    Handle latestActionDoneLink = Handle::UNDEFINED;
    std::string agentType = "unknown";

    // get eval links for all agents actions done
    std::vector<HandleTemporalPair> timestamps;
    AtomSpaceUtil::getAllEvaluationLinks( atomSpace, timestamps,
                                          ACTION_DONE_PREDICATE_NAME,
                                          temporal, criterion );

    Temporal previousTemporal(0);

    // filter eval links by agent name and most recent temporal
    unsigned int i = 0;
    for (; i < timestamps.size( ); ++i ) {
        Temporal& temporal = *( timestamps[i].getTemporal( ) );

        Handle evalLink = timestamps[i].getHandle( );

        if ( evalLink == Handle::UNDEFINED) {
            logger().error("AtomSpaceUtil - AtomSpace returned undefined "
                    "handle for evaluation link (agent action done predicate)" );
            continue;
        }

        Handle agentActionLink = atomSpace.getOutgoing(evalLink, 1);

        if ( agentActionLink == Handle::UNDEFINED) {
            logger().error("AtomSpaceUtil - Found no agent action for "
                    "actionDone predicate.");
            continue;
        }

        Handle agentIdNode = atomSpace.getOutgoing(agentActionLink, 0);
        if (agentIdNode == Handle::UNDEFINED ) {
            logger().error("AtomSpaceUtil - Found no agent name for "
                    "actionDone predicate" );
            continue;
        }

        Type inspectedAgentTypeCode = atomSpace.getType(agentIdNode);
        if ( !classserver().isNode( inspectedAgentTypeCode ) ) {
            logger().fine("AtomSpaceUtil - Skipping non-node handle type: %d",
                    inspectedAgentTypeCode );
            continue;
        }

        const std::string& inspectedAgentId = atomSpace.getName(agentIdNode);

        if ( inspectedAgentId != agentId ) {
            logger().fine("AtomSpaceUtil - "
                    "Inspected agent id is [%s; type=%d], but required is %s",
                     inspectedAgentId.c_str(),
                     inspectedAgentTypeCode,
                     agentId.c_str() );
            // it is the wrong agent, then skip it
            continue;
        }

        Handle agentActionNode = atomSpace.getOutgoing(agentActionLink, 1);
        if (agentActionNode == Handle::UNDEFINED) {
            logger().error("AtomSpaceUtil - "
                    "Found no agent action name for actionDone predicate");
            continue;
        }

        logger().fine("AtomSpaceUtil - "
                "Previous temporal[%lu %lu], Inspected temporal[%lu %lu]",
                 previousTemporal.getA(),
                 previousTemporal.getB(),
                 temporal.getA(),
                 temporal.getB() );
        if ( temporal > previousTemporal ) {
            latestActionDoneLink = agentActionLink;
            previousTemporal = temporal;

            if ( inspectedAgentTypeCode == AVATAR_NODE ) {
                agentType = "an avatar";
            } else if ( inspectedAgentTypeCode == PET_NODE ) {
                agentType = "a pet";
            } else if ( inspectedAgentTypeCode == HUMANOID_NODE ) {
                agentType = "an humanoid";
            } else {
                logger().error(
                             "AtomSpaceUtil - Invalid agentIdNode type: %i",
                             inspectedAgentTypeCode );
                continue;
            }
        }
    }

    logger().debug("AtomSpaceUtil::getMostRecentAgentActionLink - "
            "Agent %s is %s", agentId.c_str(), agentType.c_str() );

    return latestActionDoneLink;
}

Handle AtomSpaceUtil::getMostRecentAgentActionLink( AtomSpace& atomSpace,
        const std::string& agentId,
        const std::string& actionName,
        const Temporal& temporal,
        TemporalTable::TemporalRelationship criterion )
{

    Handle agentHandle = getAgentHandle( atomSpace, agentId );
    if ( agentHandle == Handle::UNDEFINED ) {
        logger().debug(
                     "AtomSpaceUtil - Found no agent identified by %s",
                     agentId.c_str( ) );
        return Handle::UNDEFINED;
    } // if

    Handle predicateNodeHandle = atomSpace.getHandle(PREDICATE_NODE,
                                 ACTION_DONE_PREDICATE_NAME);
    if ( predicateNodeHandle == Handle::UNDEFINED ) {
        logger().debug(
                     "AtomSpaceUtil - Found no predicate node named %s",
                     ACTION_DONE_PREDICATE_NAME )
        ;
        return Handle::UNDEFINED;
    } // if

    Handle actionNodeHandle = atomSpace.getHandle( NODE, actionName );
    if ( actionNodeHandle == Handle::UNDEFINED ) {
        logger().debug(
                     "AtomSpaceUtil - Found no NODE for action named %s",
                     actionName.c_str( ) );
        return Handle::UNDEFINED;
    } // if

#ifdef USE_GET_HANDLE_SET
    // retrieve all list_links which describes the given action
    HandleSeq listLinkDescription;
    listLinkDescription.push_back( agentHandle );
    listLinkDescription.push_back( actionNodeHandle );
    listLinkDescription.push_back( Handle::UNDEFINED ); // action parameters at a link list

    std::vector<Handle> handles;
    atomSpace.getHandleSet( back_inserter(handles),
                            listLinkDescription,
                            NULL, NULL, 3, LIST_LINK, false );

    if ( handles.size( ) == 0 ) {
        logger().debug(
                     "AtomSpaceUtil - the given agent wasn't executed group_command yet" );
        // there is no group command for the given agent
        return Handle::UNDEFINED;
    } // if
#else
    std::vector<Handle> handles;
    HandleSeq incomingSet = atomSpace.getIncoming(agentHandle);
    foreach(Handle incomingHandle, incomingSet) {
        AtomPtr a(incomingHandle);
        LinkPtr incomingLink(LinkCast(a));
        if (incomingLink->getType() == LIST_LINK &&
                incomingLink->getArity() == 3 && 
                incomingLink->getOutgoingAtom(0) == agentHandle && 
                incomingLink->getOutgoingAtom(1) == actionNodeHandle) {
            handles.push_back(incomingHandle);
        }
    }
#endif


    std::vector<HandleTemporalPair> timestamps;

    // filter and remove handles which isn't linked by an evaluation_link that has, as the first outgoing,
    // the predicate node handle ACTION_DONE_PREDICATE_NAME
    //std::vector<Handle> filteredHandles;
    unsigned int i;
    for ( i = 0; i < handles.size( ); ++i ) {
        HandleSeq incomingLinks = atomSpace.getIncoming(handles[i]);
        logger().debug(
                     "AtomSpaceUtil - %d incoming links were identified",
                     incomingLinks.size( ) );
        unsigned int j;
        for ( j = 0; j < incomingLinks.size( ); ++j ) {
            logger().debug(
                         "AtomSpaceUtil - %d) type[%s] EVAL_LINK[%lu], outgoing0[%lu] predicateNode[%lu]",
                         j,
                         classserver().getTypeName( atomSpace.getType( incomingLinks[j] ) ).c_str( ),
                         classserver().getTypeName( EVALUATION_LINK ).c_str( ),
                         atomSpace.getOutgoing( incomingLinks[j], 0 ).value(),
                         predicateNodeHandle.value(),
                         atomSpace.getOutgoing( incomingLinks[j], 1 ).value() );
            if ( atomSpace.getType( incomingLinks[j] ) == EVALUATION_LINK &&
                    atomSpace.getOutgoing( incomingLinks[j], 0 ) == predicateNodeHandle ) {
                timeServer().getTimeInfo( back_inserter(timestamps),
                                       incomingLinks[j], temporal, criterion );

                //filteredHandles.push_back( incomingLinks[j] );
            } // if
        } // for
    } // for

    logger().debug(
                 "AtomSpaceUtil - %d handles (was)were identified after filter",
                 timestamps.size( ) );

    Temporal previousTemporal(0);
    Handle mostRecentGroupCommand = Handle::UNDEFINED;
    // filter eval links by agent name and most recent temporal
    for ( i = 0; i < timestamps.size( ); ++i ) {
        Temporal& temporal = *( timestamps[i].getTemporal( ) );
        if ( temporal > previousTemporal ) {
            mostRecentGroupCommand = timestamps[i].getHandle( );
            previousTemporal = temporal;
        } // if
    } // for

    logger().debug(
                 "AtomSpaceUtil - handle %s",
                 ( mostRecentGroupCommand != Handle::UNDEFINED ? "found" : "not found" ) );
    return mostRecentGroupCommand;
}


Handle AtomSpaceUtil::getMostRecentAgentActionLinkWithinTime( const AtomSpace& atomSpace,
        const std::string& agentId,
        unsigned long t1,
        unsigned long t2 )
{
    Temporal t(t1, t2);
    return getMostRecentAgentActionLink(atomSpace,
                                        agentId,
                                        t, TemporalTable::ENDS_WITHIN);
}

Handle AtomSpaceUtil::getMostRecentAgentActionLinkAfterTime( const AtomSpace& atomSpace,
        const std::string& agentId,
        unsigned long timestamp )
{
    Temporal t(timestamp);
    return getMostRecentAgentActionLink(atomSpace,
                                        agentId,
                                        t, TemporalTable::ENDS_AFTER);
}

std::string AtomSpaceUtil::convertAgentActionParametersToString( const AtomSpace& atomSpace,
        Handle agentActionLink )
{
    if ( agentActionLink != Handle::UNDEFINED ) {
        try {
            Handle predicateListLink = atomSpace.getOutgoing( agentActionLink,
                                       1 );
            if ( predicateListLink == Handle::UNDEFINED ) {
                logger().error(
                              "AtomSpaceUtil - There is no description for this action" );
                return "";
            } // if

            if ( atomSpace.getArity( predicateListLink ) <= 2 ) {
                logger().error(
                              "AtomSpaceUtil - There is no parameters on the given action" );
                return "";
            } // if

            Handle actionParametersLink = atomSpace.getOutgoing( predicateListLink, 2 );
            if ( actionParametersLink == Handle::UNDEFINED ) {
                logger().error(
                              "AtomSpaceUtil - There is no parameters on the given action" );
                return "";
            } // if

            int i;
            std::stringstream parameters;
            for ( i = 0; i < atomSpace.getArity(actionParametersLink); ++i ) {
                Handle actionParam = atomSpace.getOutgoing(actionParametersLink, i);
                if ( actionParam == Handle::UNDEFINED) {
                    logger().error(
                                 "AtomSpaceUtil - Found no param for action" );
                    return "";
                } // if

                if ( i > 0 ) {
                    parameters << "; ";
                } // if

                Type t = atomSpace.getType( actionParam );
                if (  (t == CONCEPT_NODE) //boolean or string
                        || (t == NUMBER_NODE) // int or float
                   ) {
                    parameters << atomSpace.getName( actionParam );
                } else if (t == LIST_LINK) { // rotation or vector
                    parameters << atomSpace.getName( atomSpace.getOutgoing( actionParam, 0 ) );
                    parameters << "; ";
                    parameters << atomSpace.getName( atomSpace.getOutgoing( actionParam, 1 ) );
                    parameters << "; ";
                    parameters << atomSpace.getName( atomSpace.getOutgoing( actionParam, 2 ) );
                } else {
                    // entity
                    parameters << atomSpace.getName( actionParam );
                }
            } // for
            return parameters.str( );
        } catch ( opencog::IndexErrorException& ex ) {
            logger().error("AtomSpaceUtil - Invalid outgoing: %s", ex.getMessage( ) );
        } // catch
    } // if
    return "";
}

Handle AtomSpaceUtil::getModulatorSimilarityLink(AtomSpace & atomSpace,  
                                  const std::string & modulator, 
                                  const std::string & petId)
{
    // Format of Modulator
    //
    // SimilarityLink (stv 1.0 1.0)
    //     NumberNode: "modulator_value"
    //     ExecutionOutputLink
    //         GroundedSchemaNode: "modulator_schema_name"
    //         ListLink
    //             PET_HANDLE
    //

    // Get the Handle to Pet
    Handle petHandle = AtomSpaceUtil::getAgentHandle(atomSpace, petId);

    if (petHandle == Handle::UNDEFINED) {
        logger().error( "AtomSpaceUtil::%s - Found no Pet named '%s'.",
                        __FUNCTION__, petId.c_str()
                      );
        return Handle::UNDEFINED;
    }

    // Get the Handle to GroundSchemaNode
    std::string modulatorUpdater = modulator + "ModulatorUpdater";

    Handle groundedSchemaNode = atomSpace.getHandle
                                        ( GROUNDED_SCHEMA_NODE, // Type of the Atom wanted
                                          modulatorUpdater      // Name of the Atom wanted
                                        );

    if (groundedSchemaNode == Handle::UNDEFINED) {
        logger().error( "AtomSpaceUtil::%s - Found no GroundSchemaNode named '%s'.",
                         __FUNCTION__, modulatorUpdater.c_str()
                      );
        return Handle::UNDEFINED;
    }

    // Get the  HandleSet to ExecutionOutputLink
    std::vector<Handle> executionOutputLinkSet;

    atomSpace.getHandleSet
                  ( back_inserter(executionOutputLinkSet), // return value
                    groundedSchemaNode,       // returned link should contain this node
                    EXECUTION_OUTPUT_LINK,    // type of the returned link 
                    false                     // subclass is not acceptable, 
                                              // i.e. returned link should be exactly of 
                  );                          // type EXECUTION_OUTPUT_LINK


    // Pick up the ExecutionOutputLink that contains a ListLink holding petHandle
    std::vector<Handle>::iterator itrHandleSet;

    for ( itrHandleSet = executionOutputLinkSet.begin(); 
          itrHandleSet != executionOutputLinkSet.end(); 
          itrHandleSet ++ ) {

        // Get Handle to ListLink
        Handle listLink = atomSpace.getOutgoing
                                    ( *itrHandleSet, // Handle of the Link to be searched
                                      1              // Index of the Handle in Outgoing set 
                                    ); 

        if ( atomSpace.getType(listLink) == LIST_LINK && 
             atomSpace.getArity(listLink) == 1 &&
             atomSpace.getOutgoing(listLink, 0) == petHandle ) {
            break;
        }// if
    
    }// for
        
    if ( itrHandleSet == executionOutputLinkSet.end() ) {
         logger().error( 
             "AtomSpaceUtil::%s - Found no ExecutionOutputLink for '%s' with petId '%s'.",
              __FUNCTION__, modulatorUpdater.c_str(), petId.c_str()
                       );
        return Handle::UNDEFINED;
    }

    Handle executionOutputLink = *itrHandleSet;

    // Get the Handle to SimilarityLink
    std::vector<Handle> similarityLinkSet;

    atomSpace.getHandleSet
        ( back_inserter(similarityLinkSet), // return value
          executionOutputLink,              // returned link should contain this link
          SIMILARITY_LINK,                  // type of the returned link 
          false                             // subclass is not acceptable, i.e. returned 
        );                                  // link should be exactly of 
                                            // type SIMILARITY_LINK

    if (similarityLinkSet.size() != 1) {
        logger().error( "AtomSpaceUtil::%s - There should be exactly one SimilarityLink to '%s' with petId '%s', found '%d'.",
                        __FUNCTION__, 
                        modulatorUpdater.c_str(),
                        petId.c_str(),
                        similarityLinkSet.size()
                      );

        return Handle::UNDEFINED;
    }

    Handle similarityLink = similarityLinkSet[0];

    if ( atomSpace.getArity(similarityLink) != 2 ) {
        logger().error( "AtomSpaceUtil::%s - The size of Outgoing set for SimilarityLink to '%s' with petId '%s' should be exactly 2, get '%d'", 
                        __FUNCTION__, 
                        modulatorUpdater.c_str(),
                        petId.c_str(), 
                        atomSpace.getArity(similarityLink)
                      );

        return Handle::UNDEFINED;
    }

    // Get the Handle to NumberNode
    //
    // Since SimilarityLink inherits from UnorderedLink, you can NOT use the following 
    // code to get the NumberNode. 
    // Because when you creating an UnorderedLink, it will sort its Outgoing set
    // automatically ("./atomspace/Link.cc", Link::setOutgoingSet method)
    //
    // Handle numberNode = atomSpace.getOutgoing(similarityLink, 0); // Wrong!
    
    Handle numberNode = atomSpace.getOutgoing(similarityLink, 0); 
    
    if ( atomSpace.getType(numberNode) != NUMBER_NODE) {
        
        numberNode = atomSpace.getOutgoing(similarityLink, 1);    

        if ( atomSpace.getType(numberNode) != NUMBER_NODE) {

            logger().error( "AtomSpaceUtil::%s - Could not find any NumberNode in the outgoing set of SimilarityLink for '%s'",
                        __FUNCTION__, 
                        modulatorUpdater.c_str()
                          );

            return Handle::UNDEFINED;
        }
    }// if

    return similarityLink;
}

bool AtomSpaceUtil::getDemandEvaluationLinks (AtomSpace & atomSpace, 
                                              const std::string & demandName, 
                                              Handle & hDemandGoal, 
                                              Handle & hFuzzyWithin)
{
    // Create BindLink used by pattern matcher
    std::vector<Handle> variableListLinkOutgoings, tempOutgoings; 

    Handle hVariableTypeNodeListLink = atomSpace.addNode(VARIABLE_TYPE_NODE, "ListLink"); 

    tempOutgoings.clear(); 
    Handle hVariableNodeListLink = atomSpace.addNode(VARIABLE_NODE, "$var_list_link");  
    tempOutgoings.push_back(hVariableNodeListLink); 
    tempOutgoings.push_back(hVariableTypeNodeListLink); 
    variableListLinkOutgoings.push_back( atomSpace.addLink(TYPED_VARIABLE_LINK, tempOutgoings,TruthValue::TRUE_TV()) );

    Handle hVariableListLink = atomSpace.addLink(LIST_LINK, variableListLinkOutgoings,TruthValue::TRUE_TV());

    tempOutgoings.clear();
    tempOutgoings.push_back( atomSpace.addNode(PREDICATE_NODE, demandName+"DemandGoal") ); 
    Handle hEvaluationLinkDemandGoal = atomSpace.addLink(EVALUATION_LINK, tempOutgoings,TruthValue::TRUE_TV());

    tempOutgoings.clear(); 
    tempOutgoings.push_back( 
        // Must quote the GPN, else the pattern matcher tries to evaluate it!
        atomSpace.addLink(QUOTE_LINK,
                          atomSpace.addNode(GROUNDED_PREDICATE_NODE, "fuzzy_within")) );
    tempOutgoings.push_back(hVariableNodeListLink); 
    Handle hEvaluationLinkFuzzyWithin = atomSpace.addLink(EVALUATION_LINK, tempOutgoings,TruthValue::TRUE_TV());

    tempOutgoings.clear(); 
    tempOutgoings.push_back(hEvaluationLinkDemandGoal); 
    tempOutgoings.push_back(hEvaluationLinkFuzzyWithin); 
    Handle hSimultaneousEquivalenceLink = atomSpace.addLink(SIMULTANEOUS_EQUIVALENCE_LINK, tempOutgoings,TruthValue::TRUE_TV());
    Handle hReturnListLink = atomSpace.addLink(LIST_LINK, tempOutgoings,TruthValue::TRUE_TV());

    tempOutgoings.clear(); 
    tempOutgoings.push_back(hSimultaneousEquivalenceLink); 
    tempOutgoings.push_back(hReturnListLink); 
    Handle hImplicationLink = atomSpace.addLink(IMPLICATION_LINK, tempOutgoings,TruthValue::TRUE_TV());

    tempOutgoings.clear(); 
    tempOutgoings.push_back(hVariableListLink); 
    tempOutgoings.push_back(hImplicationLink); 
    Handle hBindLink = atomSpace.addLink(BIND_LINK, tempOutgoings,TruthValue::TRUE_TV());

    // Run pattern matcher
    Handle hResultListLink = bindlink(&atomSpace, hBindLink);

    // Get result
    // Note: Don't forget remove the hResultListLink, otherwise some scheme script
    //       may fail to remove the ReferenceLink when necessary. 
    //       Because the ReferenceLink would have an incoming (i.e. hResultListLink here), 
    //       which would make cog-delete scheme function fail.  
    std::vector<Handle> resultSet = atomSpace.getOutgoing(hResultListLink); 
    atomSpace.removeAtom(hResultListLink);

    // Check and return the result
//    foreach(Handle hResult, resultSet) {
//        std::cout<<atomSpace.atomAsString(hResult)<<std::endl; 
//    }

    if ( resultSet.size() != 1 ) {
        logger().error( "AtomSpaceUtil::%s - The number of SimultaneousEquivalenceLink containing '%s' and '%s' should be exactly 1, but got %d", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hEvaluationLinkDemandGoal).c_str(), 
                       atomSpace.atomAsString(hEvaluationLinkFuzzyWithin).c_str(), 
                       resultSet.size()
                      );

        return false; 
    }
    
    hResultListLink = resultSet[0]; 
    resultSet = atomSpace.getOutgoing(hResultListLink); 

    hDemandGoal = resultSet[0]; 
    hFuzzyWithin = resultSet[1]; 

    return true; 



/**
    // Get the Handle to GroundSchemaNode
    std::string demandUpdater = demandName + "DemandUpdater";

    Handle hGroundedSchemaNode = atomSpace.getHandle
                                        ( GROUNDED_SCHEMA_NODE, // Type of the Atom wanted
                                          demandUpdater         // Name of the Atom wanted
                                        );

    if ( hGroundedSchemaNode == Handle::UNDEFINED ||
         atomSpace.getType(hGroundedSchemaNode) != GROUNDED_SCHEMA_NODE ) {

        logger().error( "AtomSpaceUtil::%s - Found no GroundSchemaNode named '%s'.",
                         __FUNCTION__, 
                         demandUpdater.c_str()
                      );
        return false;
    }

    // Get the  HandleSet to ExecutionOutputLink
    std::vector<Handle> executionOutputLinkSet;

    atomSpace.getHandleSet
                  ( back_inserter(executionOutputLinkSet), // return value
                    hGroundedSchemaNode,      // returned link should contain this node
                    EXECUTION_OUTPUT_LINK,    // type of the returned link 
                    false                     // subclass is not acceptable, 
                                              // i.e. returned link should be exactly of 
                  );                          // type EXECUTION_OUTPUT_LINK

    // Pick up the ExecutionOutputLink containing ListLink
    std::vector<Handle>::iterator iExecutionOutputLink;

    for(iExecutionOutputLink = executionOutputLinkSet.begin(); 
        iExecutionOutputLink != executionOutputLinkSet.end(); 
        ++ iExecutionOutputLink ) {

        if ( atomSpace.getArity(*iExecutionOutputLink) == 2 &&
             atomSpace.getType( atomSpace.getOutgoing(*iExecutionOutputLink, 1) ) == LIST_LINK
           )
            break; 
    }

    if ( iExecutionOutputLink == executionOutputLinkSet.end() ) {
        logger().error("AtomSpaceUtil::%s - Failed to find a ExecutionOutputLink that takes ListLink as its second outgoing.", 
                       __FUNCTION__
                      ); 
        return false; 
    }

    Handle hExecutionOutputLink = *iExecutionOutputLink;

    // Get the Handle to ListLink that contains ExecutionOutputLink
    std::vector<Handle> listLinkSet;

    atomSpace.getHandleSet
        ( back_inserter(listLinkSet), // return value
          hExecutionOutputLink,       // returned link should contain this link
          LIST_LINK,                  // type of the returned link 
          false                       // subclass is not acceptable, i.e. returned 
        );                            // link should be exactly of type LIST_LINK

    // Pick up the ListLink that contains two NumberNodes and the ExecutionOutputLink above
    std::vector<Handle>::iterator iListLink;

    for (iListLink = listLinkSet.begin(); iListLink != listLinkSet.end(); iListLink ++) {

        if ( atomSpace.getType(*iListLink) == LIST_LINK &&
             atomSpace.getArity(*iListLink) == 3 &&
             atomSpace.getType( atomSpace.getOutgoing(*iListLink, 0) ) == NUMBER_NODE &&
             atomSpace.getType( atomSpace.getOutgoing(*iListLink, 1) ) == NUMBER_NODE
           )
            break;
    }

    if ( iListLink == listLinkSet.end() ) {
        logger().error("AtomSpaceUtil::%s - Failed to find a ListLink that contains two NumberNodes and the ExecutionOutputLink: '%s'", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hExecutionOutputLink).c_str()
                      );
        return false; 
    }

    Handle hListLink = *iListLink;

    // Get GroundedPredicateNode ("fuzzy_within")
    Handle hGroundedPredicateNode = atomSpace.getHandle(GROUNDED_PREDICATE_NODE, "fuzzy_within");

    if ( hGroundedPredicateNode == opencog::Handle::UNDEFINED ||
         atomSpace.getType(hGroundedPredicateNode) != GROUNDED_PREDICATE_NODE ) {

        logger().error("AtomSpaceUtil::%s - Failed to get GROUNDED_PREDICATE_NODE named 'fuzzy_within'", 
                       __FUNCTION__
                      );

        return false; 
    }

    // Get the EvaluationLink that contains both FuzzyWithin GroundedPredicateNode and ListLink above
    std::vector<Handle> evaluationLinkOutgoing;

    evaluationLinkOutgoing.push_back(hGroundedPredicateNode);
    evaluationLinkOutgoing.push_back(hListLink);

    hFuzzyWithin = atomSpace.getHandle(EVALUATION_LINK, evaluationLinkOutgoing);

    if ( hFuzzyWithin == opencog::Handle::UNDEFINED ) {
        logger().error("AtomSpaceUtil::%s - Failed to get EvaluationLink that contains both FuzzyWithin GroundedPredicateNode and ListLink", 
                       __FUNCTION__
                      );
        return false; 
    }

    // Get the Handle to PredicateNode
    std::string demandGoal = demandName + "DemandGoal";

    Handle hPredicateNode = atomSpace.getHandle(PREDICATE_NODE, demandGoal);

    if ( hPredicateNode == Handle::UNDEFINED ||
         atomSpace.getType(hPredicateNode) != PREDICATE_NODE ) {

        logger().error( "AtomSpaceUtil::%s - Found no PredicateNode named '%s'.",
                         __FUNCTION__, 
                         demandGoal.c_str()
                      );
        return false;
    }

    // Get the HandleSet to EvaluationLink that contains the PredicateNode above
    std::vector<Handle> evaluationLinkSet;

    atomSpace.getHandleSet
        ( back_inserter(evaluationLinkSet), // return value
          hPredicateNode,                             // returned link should contain this link
          EVALUATION_LINK,                            // type of the returned link 
          false                                       // subclass is not acceptable, i.e. returned 
        );                                            // link should be exactly of type EVALUATION_LINK

    // Pick up the EvaluationLink that is grouped with hFuzzyWithin inside an SimultaneousEquivalenceLink
    std::vector<Handle>::iterator iEvaluationLink;
    std::vector<Handle> simultaneousEquivalenceLinkOutgoing;

    for ( iEvaluationLink = evaluationLinkSet.begin(); 
          iEvaluationLink != evaluationLinkSet.end();
          iEvaluationLink ++ ) {
        simultaneousEquivalenceLinkOutgoing.clear();
        simultaneousEquivalenceLinkOutgoing.push_back(*iEvaluationLink);
        simultaneousEquivalenceLinkOutgoing.push_back(hFuzzyWithin);

        Handle hSimultaneousEquivalenceLink = atomSpace.getHandle( SIMULTANEOUS_EQUIVALENCE_LINK, 
                                                                  simultaneousEquivalenceLinkOutgoing);

        if ( hSimultaneousEquivalenceLink != opencog::Handle::UNDEFINED )
            break; 
    }

    if ( iEvaluationLink == evaluationLinkSet.end() ) {
        logger().error("AtomSpaceUtil::%s - Failed to get EvaluationLink that is grouped with '%s' inside an SimultaneousEquivalenceLink", 
                       __FUNCTION__, 
                       atomSpace.atomAsString(hFuzzyWithin).c_str()
                      );
        return false; 
    }

    hDemandGoal = *iEvaluationLink;

    return true; 
*/
}

Handle AtomSpaceUtil::getRuleImplicationLink(AtomSpace& atomSpace,
        const std::string& rule)
{

    Handle rulePhraseNode = atomSpace.getHandle(PHRASE_NODE, rule);
    if (rulePhraseNode == Handle::UNDEFINED) {
        logger().error(
                     "AtomSpaceUtil - Found no PhraseNode for rule '%s'.",
                     rule.c_str());
        return Handle::UNDEFINED;
    }

    std::vector<Handle> ruleReferenceLink;
    atomSpace.getHandleSet(back_inserter(ruleReferenceLink),
                           rulePhraseNode, REFERENCE_LINK, false);
    if (ruleReferenceLink.size() != 1) {
        logger().error(
                     "AtomSpaceUtil - There should be exactly one ReferenceLink to rule '%s', found '%d'.",
                     rule.c_str(),
                     ruleReferenceLink.size());
        return Handle::UNDEFINED;
    }

    // handle to ImplicationLink
    Handle implicationLink = atomSpace.getOutgoing(ruleReferenceLink[0], 1);
    if (atomSpace.getType(implicationLink) != IMPLICATION_LINK) {
        logger().error(
                     "AtomSpaceUtil - Outgoing atom index [1] should be an ImplicationLink. Got '%s'.",
                     classserver().getTypeName(atomSpace.getType(implicationLink)).c_str());
        return Handle::UNDEFINED;
    }

    return implicationLink;
}

float AtomSpaceUtil::getRuleImplicationLinkStrength(AtomSpace& atomSpace,
        const std::string& rule,
        const std::string& agentModeName )
{

    Handle implicationLink = AtomSpaceUtil::getRuleImplicationLink(atomSpace,
                             rule);
    if (implicationLink == Handle::UNDEFINED) {
        logger().error(
                     "AtomSpaceUtil - Found no ImplicationLink for rule '%s'.",
                     rule.c_str());
        return (-1.0f);
    }
    Handle agentModeNode = atomSpace.getHandle( CONCEPT_NODE, agentModeName );
    if ( agentModeNode == Handle::UNDEFINED ) {
        logger().error(
                     "AtomSpaceUtil - Found no Handle for the given agent mode '%s'.",
                     agentModeName.c_str());
        return (-1.0f);
    } // if

    // strength is given by link TruthValue
    return (atomSpace.getTV(implicationLink)->getMean());
}

spatial::math::Vector3 AtomSpaceUtil::getMostRecentObjectVelocity( const AtomSpace& atomSpace, const std::string& objectId, unsigned long afterTimestamp )
{
    // look for a velocity predicate at 2 RuleEngine cycles before the current cycle
    std::vector<HandleTemporalPair> timestamps;
    getAllEvaluationLinks( atomSpace, timestamps,
                           AGISIM_VELOCITY_PREDICATE_NAME,
                           Temporal( afterTimestamp ),
                           TemporalTable::ENDS_AFTER, true );

    // inspect velocities to find a target candidate velocity
    int i;
    for (i = timestamps.size() - 1; i > 0; --i) {
        Handle evalLink = timestamps[i].getHandle( );
        if ( evalLink != Handle::UNDEFINED
                && atomSpace.getOutgoing(evalLink, 1) != Handle::UNDEFINED ) {
            Handle listLink = atomSpace.getOutgoing(evalLink, 1);
            Handle currentHandle = atomSpace.getOutgoing(listLink, 0);
            if ( currentHandle == Handle::UNDEFINED
                    || atomSpace.getName(currentHandle) != objectId ) {
                continue;
            } // if
            // a velocity node was found to the given target
            return spatial::math::Vector3( atof( atomSpace.getName(atomSpace.getOutgoing(listLink, 1)).c_str() ),  // x
                                           atof( atomSpace.getName(atomSpace.getOutgoing(listLink, 2)).c_str() ),  // y
                                           atof( atomSpace.getName(atomSpace.getOutgoing(listLink, 3)).c_str() )); // z
        } // if
    } // for
    return spatial::math::Vector3( 0, 0, 0 );
}


Handle AtomSpaceUtil::getObjectHandle( AtomSpace& atomSpace,
                                       const std::string& objectId )
{
    // concept node objects
    if (objectId == "food_bowl" || objectId == "water_bowl"
            || objectId == "pet_home") {
        // TODO: The concept nodes may not be present inside AtomSpace
        // if pet/agent do not perceive
        // any of these objects in the world.
        // For now, whenever it perceives any of them,
        // the concept node is created
        // and its LTI is set to 1 so that it is not forgotten
        // by STI decayment. This may not be the best approach though.
        //OC_ASSERT(as.getHandle(CONCEPT_NODE, objectId) != Handle::UNDEFINED);
        return atomSpace.getHandle(CONCEPT_NODE, objectId);
    } else { //Now let's deal with the default case
        HandleSeq tmp;
        atomSpace.getHandlesByName(std::back_inserter(tmp), objectId, ACCESSORY_NODE);
        if (tmp.empty()) { //it is not an accessory, let's try a structure
            atomSpace.getHandlesByName(std::back_inserter(tmp), objectId, STRUCTURE_NODE);
        }

        if (tmp.empty()) { //it is not an structure, let's try a ordinary object
            atomSpace.getHandlesByName(std::back_inserter(tmp), objectId, OBJECT_NODE);
        }

        //assume that structure and accessories have distinct id
        OC_ASSERT(tmp.size() <= 1);

        return tmp.empty() ? Handle::UNDEFINED : tmp.front();
    }
}

Handle AtomSpaceUtil::getAgentHandle( AtomSpace& atomSpace,
                                      const std::string& agentId )
{
    Handle agentHandle = atomSpace.getHandle( PET_NODE, agentId );
    if ( agentHandle != Handle::UNDEFINED ) {
        logger().debug(
                      "AtomSpaceUtil - Agent is a pet" );
    } else {
        agentHandle = atomSpace.getHandle( AVATAR_NODE, agentId );
        if ( agentHandle != Handle::UNDEFINED ) {
            logger().debug(
                          "AtomSpaceUtil - Agent is an avatar" );
        } else {
            agentHandle = atomSpace.getHandle( HUMANOID_NODE, agentId );
            if ( agentHandle != Handle::UNDEFINED ) {
                logger().debug(
                              "AtomSpaceUtil - Agent is an humanoid" );
            } // if
        } // if
    } // if
    return agentHandle;
}

Handle AtomSpaceUtil::getEntityHandle( AtomSpace& atomSpace,
                                       const std::string& entityId )
{
    // try maybe it's an object
    Handle h = getObjectHandle(atomSpace,entityId);
    if (h == Handle::UNDEFINED)// it's not an object, try maybe it's an avatar
    {
        h = getAgentHandle(atomSpace,entityId);
    }

    return h;

}

Temporal AtomSpaceUtil::getTemporal(AtomSpace& as, Handle atTimeLink)
{
    OC_ASSERT(atTimeLink != Handle::UNDEFINED,
            "No HandleTemporalPair correspond to Handle::UNDEFINED");
    OC_ASSERT(as.getType(atTimeLink) == AT_TIME_LINK,
            "The Atom %s must be an atTimeLink",
            as.atomAsString(atTimeLink).c_str());
    Handle timeNode = as.getOutgoing(atTimeLink, 0);
    OC_ASSERT(as.getType(timeNode) == TIME_NODE,
            "The Atom %s must be a TimeNode",
            as.atomAsString(timeNode).c_str());

    return Temporal::getFromTimeNodeName(as.getName(timeNode).c_str());
}

Handle AtomSpaceUtil::getTimedHandle(AtomSpace& as, Handle atTimeLink)
{
    OC_ASSERT(atTimeLink != Handle::UNDEFINED,
            "No HandleTemporalPair correspond to Handle::UNDEFINED");
    OC_ASSERT(as.getType(atTimeLink) == AT_TIME_LINK,
            "The Atom %s must be an atTimeLink", as.atomAsString(atTimeLink).c_str());

    return as.getOutgoing(atTimeLink, 1);
}

void AtomSpaceUtil::updateGenericLatestInfoMap(std::map<Handle, Handle> & infoMap,
                                               AtomSpace & as,
                                               Handle atTimeLink,
                                               Handle key)
{
    std::map<Handle, Handle>::iterator itr = infoMap.find(key);
    if (itr != infoMap.end()) {
        as.removeAtom(itr->second);
    }
    HandleSeq hs;
    hs.push_back(atTimeLink);
    Handle latestLink = addLink(as, LATEST_LINK, hs, true);
    infoMap[key] = latestLink;
}

void AtomSpaceUtil::updateLatestAgentActionDone(AtomSpace& as,
                                                Handle atTimeLink,
                                                Handle agentNode)
{
    updateGenericLatestInfoMap(latestAgentActionDone,
                               as, atTimeLink, agentNode);
}

void AtomSpaceUtil::updateLatestPhysiologicalFeeling(AtomSpace& as,
                                                     Handle atTimeLink,
                                                     Handle predicateNode)
{
    updateGenericLatestInfoMap(latestPhysiologicalFeeling,
                               as, 
                               atTimeLink, 
                               predicateNode
                              );
}

void AtomSpaceUtil::updateLatestAvatarSayActionDone(AtomSpace& as,
                                                    Handle atTimeLink,
                                                    Handle avatarNode)
{
    updateGenericLatestInfoMap(latestAvatarSayActionDone,
                               as, atTimeLink, avatarNode);
}

void AtomSpaceUtil::updateLatestAvatarActionDone(AtomSpace& as,
                                                 Handle atTimeLink,
                                                 Handle avatarNode)
{
    updateGenericLatestInfoMap(latestAvatarActionDone,
                               as, atTimeLink, avatarNode);
}

void AtomSpaceUtil::updateLatestAvatarActionPredicate(AtomSpace& as,
                                                   Handle atTimeLink,
                                                   Handle predicateNode) 
{
    updateGenericLatestInfoMap(latestAvatarActionPredicate,
                               as, atTimeLink, predicateNode);
}

void AtomSpaceUtil::updateLatestSpatialPredicate(AtomSpace& as,
                                                 Handle atTimeLink,
                                                 Handle predicateNode,
                                                 Handle objectNode)
{
    std::map<Handle, Handle> infoMap;
    std::map<Handle, std::map<Handle, Handle> >::iterator itr = latestSpatialPredicate.find(predicateNode);
    if (itr != latestSpatialPredicate.end()) {
        infoMap = itr->second;
    }
    updateGenericLatestInfoMap(infoMap, as, atTimeLink, objectNode);
    latestSpatialPredicate[predicateNode] = infoMap;
}

void AtomSpaceUtil::updateLatestSchemaPredicate(AtomSpace& as,
                                                Handle atTimeLink,
                                                Handle predicateNode)
{
    updateGenericLatestInfoMap(latestSchemaPredicate,
                               as, atTimeLink, predicateNode);
}

void AtomSpaceUtil::updateGenericLatestSingleInfo(Handle & latestSingleInfoHandle,
                                                  AtomSpace & as,
                                                  Handle atTimeLink)
{
    if (latestSingleInfoHandle != Handle::UNDEFINED) {
        as.removeAtom(latestSingleInfoHandle);
    }
    HandleSeq hs;
    hs.push_back(atTimeLink);
    latestSingleInfoHandle = addLink(as, LATEST_LINK, hs, true);
}

void AtomSpaceUtil::updateLatestModulator(AtomSpace & as, 
                                          Handle atTimeLink, 
                                          Handle modulatorPredicateNode)
{
    updateGenericLatestInfoMap(latestModulators, as, atTimeLink, modulatorPredicateNode); 
}

void AtomSpaceUtil::updateLatestDemand(AtomSpace & as, 
                                       Handle atTimeLink, 
                                       Handle demandPredicateNode)
{
    updateGenericLatestInfoMap(latestDemands, as, atTimeLink, demandPredicateNode); 
}

void AtomSpaceUtil::updateLatestFeeling(AtomSpace & as, 
                                        Handle atTimeLink, 
                                        Handle feelingPredicateNode)
{
    updateGenericLatestInfoMap(latestFeelings, as, atTimeLink, feelingPredicateNode); 
}

void AtomSpaceUtil::updateLatestStimulus(AtomSpace & as, 
                                         Handle atTimeLink, 
                                         Handle stimulusPredicateNode)
{
    updateGenericLatestInfoMap(latestStimulus, as, atTimeLink, stimulusPredicateNode); 
}

void AtomSpaceUtil::updateLatestIsExemplarAvatar(AtomSpace& as,
                                                 Handle atTimeLink)
{
    updateGenericLatestSingleInfo(latestIsExemplarAvatar, as, atTimeLink);
}

Handle AtomSpaceUtil::getFrameElements( AtomSpace& atomSpace, const std::string& frameName, HandleSeq& frameElementsHandles ) 
{
    // Get frame node (DefinedFrameNode)
    Handle frameNode = atomSpace.getHandle( DEFINED_FRAME_NODE, frameName );

    if ( frameNode != Handle::UNDEFINED ) {

        // look up the cache firstly 
        boost::unordered_map<std::string, HandleSeq>::const_iterator it =
            frameElementsCache.find( frameName );
        
        if ( it != frameElementsCache.end() ) {
            frameElementsHandles = it->second;
            return frameNode;
        } 

        // if there is nothing in the cache, then search the atomspace directly
#ifdef USE_GET_HANDLE_SET
        HandleSeq frameElementLink;
        frameElementLink.push_back(frameNode);
        frameElementLink.push_back(Handle::UNDEFINED);
        HandleSeq frameElementLinkHandles;
        atomSpace.getHandleSet(back_inserter(frameElementLinkHandles),
                               frameElementLink, NULL, NULL, 2, FRAME_ELEMENT_LINK, false);
        
        foreach (Handle frameElementLink, frameElementLinkHandles ) {
            frameElementsHandles.push_back( atomSpace.getOutgoing( frameElementLink, 1 ) );
        } 

        // get the parents of this frame
        HandleSeq inheritenceLinks;            
        parentLink.push_back( frameNode );
        parentLink.push_back( Handle::UNDEFINED );
        HandleSeq parentFrames;
        atomSpace.getHandleSet(back_inserter(parentFrames),
                               inheritenceLinks, NULL, NULL, 2, INHERITANCE_LINK, false);            

        foreach (Handle inheritenceLink, inheritenceLinks) {
            Handle parentFrameHandle = atomSpace.getOutgoing( inheritenceLink, 1 );

            // get the frame elements of the parent frame recursively
            if ( atomSpace.getType( parentFrameHandle ) == DEFINED_FRAME_NODE ) {
                getFrameElements( atomSpace, atomSpace.getName( parentFrameHandle ), frameElementsHandles );
            } 
        } 
#else
        HandleSeq parentFrames;
        HandleSeq incomingSet = atomSpace.getIncoming(frameNode);

        foreach(Handle incomingHandle, incomingSet) {
            AtomPtr a(incomingHandle);
            LinkPtr incomingLink(LinkCast(a));

            if (incomingLink->getType() == FRAME_ELEMENT_LINK &&  
                incomingLink->getArity() == 2 && 
                incomingLink->getOutgoingAtom(0) == frameNode) {
                frameElementsHandles.push_back(incomingLink->getOutgoingAtom(1));
            } 
            else if (incomingLink->getType() == INHERITANCE_LINK &&  
                     incomingLink->getArity() == 2 && 
                     incomingLink->getOutgoingAtom(0) == frameNode) {
                parentFrames.push_back(incomingLink->getOutgoingAtom(1));
            }
        }

        foreach(Handle parentFrameHandle, parentFrames) {
            if ( atomSpace.getType( parentFrameHandle ) == DEFINED_FRAME_NODE ) {
                getFrameElements( atomSpace, atomSpace.getName( parentFrameHandle ), frameElementsHandles );
            } 
        } 
#endif

        // update the cache
        frameElementsCache[frameName] = frameElementsHandles;

        return frameNode;

    } // if ( frameNode != Handle::UNDEFINED )
    
    return Handle::UNDEFINED;
}

Handle AtomSpaceUtil::setPredicateFrameFromHandles( AtomSpace& atomSpace, const std::string& frameName, 
                                                    const std::string& frameInstanceName, 
                                                    const std::map<std::string, Handle>& frameElementsValuesHandles, 
                                                    TruthValuePtr truthValue, bool permanent )
{
    Handle frameNode = atomSpace.getHandle( DEFINED_FRAME_NODE, frameName );

    if ( frameNode != Handle::UNDEFINED ) {

        // get all parents of this frame to copy its elements to this one
        std::map<std::string, Handle> frameElements;

        // get the frame elements
        HandleSeq frameElementsHandles;
        getFrameElements( atomSpace, atomSpace.getName( frameNode ), frameElementsHandles );
        unsigned int i;
        for( i = 0; i < frameElementsHandles.size( ); ++i ) {
            std::vector<string> elementNameParts;
            std::string elementName = atomSpace.getName( frameElementsHandles[i] );
            boost::algorithm::split( elementNameParts,
                                     elementName,
                                     boost::algorithm::is_any_of(":") );
            
            OC_ASSERT(elementNameParts.size( ) == 2,
                      "The name of a Frame element must be #FrameName:FrameElementName, but '%s' was given",
                      atomSpace.getName( frameElementsHandles[i] ).c_str( ) );
            
            // only add a new element if it wasn't yet defined (overloading)
            if ( frameElements.find( elementNameParts[1] ) == frameElements.end( ) ) {
                frameElements.insert( std::map<std::string, Handle>::value_type(
                elementNameParts[1], frameElementsHandles[i] ) );
            } // if
        } // for
                

        // check if there are one element value for each Frame element
        if ( frameElements.size( ) > 0 ) {

            Handle frameInstance = addNode(atomSpace,
                                                 PREDICATE_NODE,
                                                 frameInstanceName, true);

            HandleSeq frameInstanceInheritance;
            frameInstanceInheritance.push_back( frameInstance );
            frameInstanceInheritance.push_back( frameNode );
            Handle frameInheritanceLink = addLink( atomSpace, INHERITANCE_LINK, frameInstanceInheritance, true );
            atomSpace.setTV( frameInheritanceLink, TruthValue::FALSE_TV() );
            
            std::map<std::string, Handle>::const_iterator it;
            for( it = frameElements.begin( ); it != frameElements.end( ); ++it ) {
                
                // the element exists, so get its handle and check it
                Handle frameElementHandle = it->second;
                if ( frameElementHandle == Handle::UNDEFINED ) {
                    logger().error( "AtomSpaceUtil::%s - Invalid Undefined frame element node for frame '%s'",
                                    __FUNCTION__, frameName.c_str( ) );
                    return Handle::UNDEFINED;
                } // if

                // then add a new eval link to a new element value if necessary
                std::map<std::string, Handle>::const_iterator itValue =
                    frameElementsValuesHandles.find( it->first );

                // new prepare the name of the element for using at the Frame instance
                std::stringstream frameElementInstanceName;
                frameElementInstanceName << frameInstanceName << "_" << it->first;

                Handle frameElementInstance = 
                    addNode( atomSpace, PREDICATE_NODE, 
                             frameElementInstanceName.str( ), true );

                // Remove any old value
                HandleSeq incomingSet = atomSpace.getIncoming(frameElementInstance);
                foreach(Handle incomingHandle, incomingSet) {
                    AtomPtr a(incomingHandle);
                    LinkPtr incomingLink(LinkCast(a));
                    if (incomingLink->getType() == EVALUATION_LINK &&  
                        incomingLink->getArity() == 2 && 
                        incomingLink->getOutgoingAtom(0) == frameElementInstance) {
                        atomSpace.removeAtom(incomingHandle);
                    } 
                }

                if ( itValue != frameElementsValuesHandles.end( ) && itValue->second != Handle::UNDEFINED ) {                
                    HandleSeq frameElementInheritance;
                    frameElementInheritance.push_back( frameElementInstance );
                    frameElementInheritance.push_back( frameElementHandle );
    
                    Handle frameElementInheritanceLink = addLink( atomSpace, INHERITANCE_LINK, frameElementInheritance, true );
                    if ( permanent ) {
                        atomSpace.setLTI( frameElementInheritanceLink, 1 );
                    } // if
                    atomSpace.setTV( frameElementInheritanceLink, truthValue );

                    HandleSeq predicateFrameElement;
                    predicateFrameElement.push_back( frameInstance );
                    predicateFrameElement.push_back( frameElementInstance );
                                
                    Handle frameElementLink = addLink( atomSpace, FRAME_ELEMENT_LINK, predicateFrameElement, true );
                    if ( permanent ) {
                        atomSpace.setLTI( frameElementLink, 1 );
                    } // if
                    atomSpace.setTV( frameElementLink, truthValue );

                    // set a new value to the frame element
                    HandleSeq predicateFrameValue(2);
                    predicateFrameValue[0] = frameElementInstance;
                    predicateFrameValue[1] = itValue->second;
                    Handle frameElementEvalLink = addLink( atomSpace, EVALUATION_LINK, predicateFrameValue, true );                    
                    if ( permanent ) {
                        atomSpace.setLTI( frameElementEvalLink, 1 );                
                    } // if
                    atomSpace.setTV( frameElementEvalLink, truthValue );
                } else {
                    // Remove any other atoms for representing this element, since it's not
                    // present anymore
                    HandleSeq incomingSet = atomSpace.getIncoming(frameElementInstance);
                    foreach(Handle incomingHandle, incomingSet) {
                        AtomPtr a(incomingHandle);
                        LinkPtr incomingLink(LinkCast(a));
                        if (incomingLink->getArity() == 2 && 
                           ((incomingLink->getType() == INHERITANCE_LINK &&   
                             incomingLink->getOutgoingAtom(0) == frameElementInstance) || 
                            (incomingLink->getType() == FRAME_ELEMENT_LINK &&  
                             incomingLink->getOutgoingAtom(1) == frameElementInstance))) {  
                            atomSpace.removeAtom(incomingHandle);
                        } 
                    }
                    atomSpace.removeAtom(frameElementInstance);
                }

            } // for

            if ( frameElementsValuesHandles.size( ) == 0 ) {
                std::stringstream msg;
                msg << "AtomSpaceUtil::%s - You created a Frame with Handle::UNDEFINED ";
                msg << "in all of its elements, what is probably a useless Frame. Frame name '%s' ";
                msg << " Frame instance name '%s'. # of Frame elements '%d' ";
                msg << " # of Values given '%d' ";
                logger().debug( msg.str( ).c_str( ), __FUNCTION__, frameName.c_str( ), frameInstanceName.c_str( ),
                                frameElements.size( ), frameElementsValuesHandles.size( ) );
            } // if

            atomSpace.setTV( frameInheritanceLink, truthValue );
            if ( permanent ) {
                atomSpace.setLTI( frameInheritanceLink, 1 );
            } // if
            return frameInstance;

        } else {            
            logger().debug( "AtomSpaceUtil::%s - There are no configured frame elements for frame '%s'", 
                            __FUNCTION__, frameName.c_str( ) );
        } // else
        
    } else {
        logger().debug( "AtomSpaceUtil::%s - There is no Registered Frame named '%s'", 
                        __FUNCTION__, frameName.c_str( ) );
    } // else

    return Handle::UNDEFINED;
}

std::map<std::string, Handle> AtomSpaceUtil::getFrameElementInstanceNameValues( AtomSpace& atomSpace, Handle frameInstancePredicateNode ) 
{
    std::map<std::string, Handle> frameElementNameValueMap;

    // first check if this is really a frame instance
    Type type = atomSpace.getType( frameInstancePredicateNode );
    if ( atomSpace.getType( frameInstancePredicateNode ) != PREDICATE_NODE ) {
        logger().error("AtomSpaceUtil::%s - The given handle isn't a PREDICATE_NODE: '%d'.", 
                       __FUNCTION__, type );
        return frameElementNameValueMap;
    } // if

    // get frame instance name (like red@xxx_Color)
    const std::string& frameInstanceName = atomSpace.getName( frameInstancePredicateNode );

    // try to get an InheritanceLink holding both the given frame instance 
    // (a PredicateNode) and an DefinedFrameNode
    std::string frameName;

    HandleSeq inheritanceLink;
    inheritanceLink.push_back( frameInstancePredicateNode );
    inheritanceLink.push_back( Handle::UNDEFINED );

    Type inheritanceLinkTypes[] = { PREDICATE_NODE, DEFINED_FRAME_NODE };
    HandleSeq inheritanceLinks;
    atomSpace.getHandlesByOutgoing( back_inserter( inheritanceLinks ),
                            inheritanceLink,
                            &inheritanceLinkTypes[0], NULL, 2, INHERITANCE_LINK, false );        

    // if it is a frame instance, retrieve the frame name
    if ( inheritanceLinks.size() > 0 ) {
        if ( inheritanceLinks.size() > 1 ) {
            logger().error("AtomSpaceUtil::%s - The given handle represents more than one instance of Frame, what is unacceptable. Only the first occurrence will be considered.", __FUNCTION__ );                
        } 

        frameName = atomSpace.getName( atomSpace.getOutgoing( inheritanceLinks[0], 1 ) );
    } 
    else {
        logger().debug("AtomSpaceUtil::%s - The given handle (%s) isn't a Frame instance. It doesn't inherits from a DEFINED_FRAME_NODE",
                       __FUNCTION__, 
                       frameInstanceName.c_str()
                      );

        return frameElementNameValueMap;
    } 

    // get the frame elements (DefinedFrameElementNodes) of the the given frame 
    // and its parent frames
    HandleSeq frameElementsHandles;
    Handle frame = getFrameElements( atomSpace, frameName, frameElementsHandles );

    foreach ( Handle frameElement, frameElementsHandles ) {
        // Get frame element name (such as #Color:Entity)
        std::string frameElementName = atomSpace.getName(frameElement);  

        // Build frame element instance name (like red@xxx_Color_Entity)
        std::vector<string> frameElementNameSplited;

        boost::algorithm::split( frameElementNameSplited,
                                 frameElementName,
                                 boost::algorithm::is_any_of(":") );

        std::stringstream frameElementInstanceName;
        frameElementInstanceName << frameInstanceName << "_" << frameElementNameSplited[1];

        // get frame element instance (PredicateNode)
        Handle frameElementInstance = atomSpace.getHandle( PREDICATE_NODE, frameElementInstanceName.str() );
        if ( frameElementInstance == Handle::UNDEFINED ) {
            continue;
        } 

        // Get EvaluationLinks holding both frame element instance 
        // (PredicateNode) and its corresponding value (WordInstanceNode in the
        // example below)
        //
        // Example:
        //
        // (EvaluationLink (stv 1 1)
        //     (PredicateNode "red@701fe254-80e7-4329-80b4-8f865b665843_Color_Color")
        //     (WordInstanceNode "red@701fe254-80e7-4329-80b4-8f865b665843")
        //
        HandleSeq elementValue;
        elementValue.push_back( frameElementInstance );
        elementValue.push_back( Handle::UNDEFINED );
        
        HandleSeq evaluationLinks;
        atomSpace.getHandlesByOutgoing( back_inserter( evaluationLinks ),
                                elementValue, NULL, NULL, 2, EVALUATION_LINK, false );

        // store (frame element name, frame element instance value) pair
        if ( evaluationLinks.size() > 0 ) {
            if ( evaluationLinks.size() != 1 ) {
                logger().error( "AtomSpaceUtil::%s - An invalid number of values was defined for the element: %s -> %d",
                                __FUNCTION__,
                                frameElementInstanceName.str().c_str(), 
                                evaluationLinks.size()
                              );
            } 

            frameElementNameValueMap[ frameElementNameSplited[1] ] = atomSpace.getOutgoing( evaluationLinks[0], 1 );
        }// if 

    } // for

    return frameElementNameValueMap;
}


HandleSeq AtomSpaceUtil::retrieveFrameInstancesUsingAnElementValue( AtomSpace& atomSpace, const std::string& frameName, Handle aElementValue ) 
{

    HandleSeq instances;

    HandleSeq frameElementsHandles;
    Handle frame = getFrameElements( atomSpace, frameName, frameElementsHandles );

    if ( frame == Handle::UNDEFINED ) {
        logger().error(
                     "AtomSpaceUtil::%s - Invalid given frame name '%s'.",
                     __FUNCTION__, frameName.c_str());
        return instances;
    } // if

    // get the predicate node associated with the element value
    HandleSeq evalLink;
    evalLink.push_back( Handle::UNDEFINED );
    evalLink.push_back( aElementValue );
    HandleSeq evalLinks;
    Type evalLinkTypes[] = {PREDICATE_NODE, atomSpace.getType( aElementValue ) };
    atomSpace.getHandlesByOutgoing( back_inserter( evalLinks ),
                            evalLink,
                            &evalLinkTypes[0], NULL, 2, EVALUATION_LINK, false );

    // now check if the predicate nodes belongs to a specific frame instance    
    unsigned int i;
    for( i = 0; i < evalLinks.size( ); ++i ) {
        // test against all the frame elements
        unsigned int j;
        for( j = 0; j < frameElementsHandles.size( ); ++j ) {            
            HandleSeq inheritanceElements(2);
            inheritanceElements[0] = atomSpace.getOutgoing( evalLinks[i], 0 );
            inheritanceElements[1] = frameElementsHandles[j];
            Handle inheritance = 
                atomSpace.getHandle(INHERITANCE_LINK, inheritanceElements );
            if ( inheritance != Handle::UNDEFINED ) {
                // ok, it is part of a frame instance, now get the frame
                // predicate

                // get all the predicate nodes that is part of a FrameElementLink
                HandleSeq frameElementLink;
                frameElementLink.push_back( Handle::UNDEFINED );
                frameElementLink.push_back( atomSpace.getOutgoing( evalLinks[i], 0 ) );
                
                HandleSeq frameElementLinks;
                Type frameElementLinkTypes[] = { PREDICATE_NODE, PREDICATE_NODE };
                atomSpace.getHandlesByOutgoing( back_inserter( frameElementLinks ),
                                        frameElementLink,
                                        &frameElementLinkTypes[0], NULL, 2,
                                        FRAME_ELEMENT_LINK, false );
                // finally check if the upper predicate node inherits from
                // the correct frame node
                unsigned int k;
                for( k = 0; k < frameElementLinks.size( ); ++k ) {
                    inheritanceElements[0] = atomSpace.getOutgoing( frameElementLinks[k], 0 );
                    inheritanceElements[1] = frame;

                    inheritance = 
                        atomSpace.getHandle(INHERITANCE_LINK, inheritanceElements );
                    if ( inheritance != Handle::UNDEFINED ) {
                        // Ah, we found a predicate node that identifies a Frame instance
                        instances.push_back( atomSpace.getOutgoing( frameElementLinks[k], 0) );
                    } // if
                                                      
                } // for

            } // if

        } // for                                   
    } // for

    return instances;
    
}

void AtomSpaceUtil::deleteFrameInstance( AtomSpace& atomSpace, Handle frameInstance ) 
{

    // first check if this is really a frame instance
    if ( atomSpace.getType( frameInstance ) != PREDICATE_NODE ) {
        logger().error(
                       "AtomSpaceUtil::%s - The given handle isn't a PREDICATE_NODE: '%d'.", 
                       __FUNCTION__, atomSpace.getType( frameInstance ) );
        return;
    } // if

    std::string frameName;
    { // check the inheritance
#ifdef USE_GET_HANDLE_SET
        HandleSeq inheritanceLink;
        inheritanceLink.push_back( frameInstance );
        inheritanceLink.push_back( Handle::UNDEFINED );

        Type inheritanceLinkTypes[] = { PREDICATE_NODE, DEFINED_FRAME_NODE };
        HandleSeq inheritanceLinks;
        atomSpace.getHandleSet( back_inserter( inheritanceLinks ),
                                inheritanceLink,
                                &inheritanceLinkTypes[0], NULL, 2, INHERITANCE_LINK, false );

        if ( inheritanceLinks.size( ) > 0 ) {
            // ok it is a frame instance
            if ( inheritanceLinks.size( ) > 1 ) {
                logger().error(
                   "AtomSpaceUtil::%s - The given handle represents more than one instance of Frame, what is unacceptable. Only the first occurrence will be considered.", __FUNCTION__ );
            } // if
            frameName = atomSpace.getName( atomSpace.getOutgoing( inheritanceLinks[0], 1 ) );
            atomSpace.removeAtom( inheritanceLinks[0] );
        } else {
            logger().debug(
                "AtomSpaceUtil::%s - The given handle isn't a Frame instance. It doesn't inherits from a DEFINED_FRAME_NODE.", __FUNCTION__ );
            return;
        } // else
#else 
        bool found = false;
        HandleSeq incomingSet = atomSpace.getIncoming(frameInstance);
        foreach(Handle incomingHandle, incomingSet) { 
            AtomPtr a(incomingHandle);
            LinkPtr incomingLink(LinkCast(a));
            if (incomingLink->getType() == INHERITANCE_LINK) {
                if (incomingLink->getArity() == 2 && incomingLink->getOutgoingAtom(0) == frameInstance) {
                    Handle targetHandle = incomingLink->getOutgoingAtom(1);
                    AtomPtr targetAtom(targetHandle);
                    if (targetAtom->getType() == DEFINED_FRAME_NODE) {
                        found = true;
                        atomSpace.removeAtom(incomingHandle);
                        break; // discard aditional inheritance, if any
                    }
                }
            }
        } 
        if (!found) {
            logger().debug(
                "AtomSpaceUtil::%s - The given handle isn't a Frame instance. It doesn't inherit from a DEFINED_FRAME_NODE.", __FUNCTION__ );
            return;
        }
#endif
        
    } // end block

    std::string instanceName = atomSpace.getName( frameInstance );

#ifdef USE_GET_HANDLE_SET
    HandleSeq frameElement;
    frameElement.push_back( frameInstance );
    frameElement.push_back( Handle::UNDEFINED );
    Type frameElementTypes[] = { PREDICATE_NODE, PREDICATE_NODE };
    HandleSeq frameElements;
    atomSpace.getHandleSet( back_inserter( frameElements ),
                            frameElement, &frameElementTypes[0], NULL, 2, FRAME_ELEMENT_LINK, false );
#else
    HandleSeq frameElements;
    HandleSeq incomingSet = atomSpace.getIncoming(frameInstance);
    foreach(Handle incomingHandle, incomingSet) {
        AtomPtr a(incomingHandle);
        LinkPtr incomingLink(LinkCast(a));
        if (incomingLink->getType() == FRAME_ELEMENT_LINK) {
            if (incomingLink->getArity() == 2 && incomingLink->getOutgoingAtom(0) == frameInstance) {
                Handle targetHandle = incomingLink->getOutgoingAtom(1);
                AtomPtr targetAtom(targetHandle);
                if (targetAtom->getType() == PREDICATE_NODE) {
                    frameElements.push_back(incomingHandle);
                }
            }
        }
    }
#endif
    unsigned int j;
    for( j = 0; j < frameElements.size( ); ++j ) {
        Handle elementPredicate = atomSpace.getOutgoing( frameElements[j], 1 );
            
        // check if the elements are part of the same frame instance
#ifdef USE_GET_HANDLE_SET
        HandleSeq inheritance;
        inheritance.push_back( elementPredicate );
        inheritance.push_back( Handle::UNDEFINED );
        
        Type inheritanceTypes[] = { PREDICATE_NODE, DEFINED_FRAME_ELEMENT_NODE };
        HandleSeq inheritances;
        atomSpace.getHandleSet( back_inserter( inheritances ),
                                inheritance, &inheritanceTypes[0], NULL, 2, INHERITANCE_LINK, false );
        if ( inheritances.size( ) != 1 ) {
            logger().error( "AtomSpaceUtil::%s - Invalid frame instance. It has a predicate node linked by a InheritanceLink that isn't a DefinedFrameElementNode", __FUNCTION__ );
            return;
        } // if
#else
        Handle inheritanceLink = Handle::UNDEFINED;
        HandleSeq elemIncomingSet = atomSpace.getIncoming(elementPredicate);
        foreach (Handle elemIncomingHandle, elemIncomingSet) {
            AtomPtr a(elemIncomingHandle);
            LinkPtr elemIncomingLink(LinkCast(a));
            if (elemIncomingLink->getType() == INHERITANCE_LINK) {
                if (elemIncomingLink->getArity() == 2 && elemIncomingLink->getOutgoingAtom(0) == elementPredicate) {
                    Handle targetHandle = elemIncomingLink->getOutgoingAtom(1);
                    AtomPtr targetAtom(targetHandle);
                    if (targetAtom->getType() == DEFINED_FRAME_ELEMENT_NODE) {
                        inheritanceLink = elemIncomingHandle;
                        break;
                    }
                }
            }
        }
        if (inheritanceLink == Handle::UNDEFINED) {
            logger().error( "AtomSpaceUtil::%s - Invalid frame instance. "
                    "It has a predicate node linked by a InheritanceLink "
                    "that isn't a DefinedFrameElementNode", __FUNCTION__ );
            return;
        }
#endif

        // check if the elements are part of the same frame instance
#ifdef USE_GET_HANDLE_SET
        HandleSeq evaluation;
        evaluation.push_back( elementPredicate );
        evaluation.push_back( Handle::UNDEFINED );
        
        HandleSeq values;
        atomSpace.getHandleSet( back_inserter( values ),
                                evaluation, NULL, NULL, 2, EVALUATION_LINK, false );
        if ( values.size( ) != 1 ) {
            logger().error( "AtomSpaceUtil::%s - Invalid number of element value. "
                    "It should be just one but was %d.", __FUNCTION__, values.size() );
            return;
        }
#else
        Handle valueHandle = Handle::UNDEFINED;
        foreach (Handle elemIncomingHandle, elemIncomingSet) {
            AtomPtr a(elemIncomingHandle);
            LinkPtr elemIncomingLink(LinkCast(a));
            if (elemIncomingLink->getType() == EVALUATION_LINK) {
                if (elemIncomingLink->getArity() == 2 && elemIncomingLink->getOutgoingAtom(0) == elementPredicate) {
                    valueHandle = elemIncomingHandle;
                    break;
                }
            }
        }
        if (valueHandle == Handle::UNDEFINED) {
            logger().error( "AtomSpaceUtil::%s - Did not find element value", __FUNCTION__);
            return;
        }
#endif
      
        // first, remove the evaluation link
        HandleSeq elementPair(2);
        elementPair[0] = elementPredicate;
#ifdef USE_GET_HANDLE_SET
        elementPair[1] = atomSpace.getOutgoing( values[0], 1 );
#else
        elementPair[1] = atomSpace.getOutgoing( valueHandle, 1 );
#endif

        Handle link = atomSpace.getHandle( EVALUATION_LINK, elementPair );
        atomSpace.removeAtom( link );

        // second, remove the frame element link
        elementPair[0] = frameInstance;
        elementPair[1] = elementPredicate;
        link = atomSpace.getHandle( FRAME_ELEMENT_LINK, elementPair );
        atomSpace.removeAtom( link );

        // then, remove the inheritance link
#ifdef USE_GET_HANDLE_SET
        atomSpace.removeAtom( inheritances[0] );
#else
        atomSpace.removeAtom( inheritanceLink );
#endif

        // finaly, remove the element predicate node
        atomSpace.removeAtom( elementPredicate );

        
    } // for

}
