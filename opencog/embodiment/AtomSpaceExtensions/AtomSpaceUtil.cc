/*
 * opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Welter Luigi
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


/**
 * Class with util methods for related to AtomSpace manipulation
 *
 * Author: Welter Luigi
 * Copyright(c), 2007
 */

#include <opencog/util/misc.h>
#include <opencog/atomspace/TLB.h>
#include <opencog/atomspace/Node.h>
#include <opencog/atomspace/Temporal.h>
#include <opencog/atomspace/TemporalTable.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/HandleTemporalPairEntry.h>
#include <opencog/atomspace/SpaceServer.h>

#include <opencog/util/Logger.h>

#include "AtomSpaceUtil.h"
#include "PredefinedProcedureNames.h"
#include "CompareAtomTreeTemplate.h"

#include <algorithm>
#include <cctype>

#define IS_HOLDING_PREDICATE_NAME "isHolding"
#define IS_HOLDING_SOMETHING_PREDICATE_NAME "isHoldingSomething"

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
                              bool permanent, bool renew_sti)
{
    Handle result = atomSpace.getHandle(linkType, outgoing);
    if (result == Handle::UNDEFINED) {
        result = atomSpace.addLink(linkType, outgoing);
        if (permanent) {
            atomSpace.setLTI(result, 1);
        }
    } else if (permanent) {
        if (atomSpace.getLTI(result) < 1) {
            atomSpace.setLTI(result, 1);
        }
    } else if (renew_sti) {
        result = atomSpace.addLink(linkType, outgoing);
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

    atomSpace.addTimeInfo(evalLink, timestamp);

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

    atomSpace.addTimeInfo(evalLink, timestamp);

    return evalLink;
}

bool AtomSpaceUtil::isActionPredicatePresent(const AtomSpace& atomSpace,
        const char* actionPredicateName,
        Handle actionExecLink,
        unsigned long sinceTimestamp)
{

    //cout << "Looking for action predicate '" << actionPredicateName << "' after timestamp '" << sinceTimestamp << "' for action: " << TLB::getAtom(actionExecLink)->toString() << endl;
    bool result = false;
    HandleSeq evalListLinkOutgoing;
    evalListLinkOutgoing.push_back(actionExecLink);
    Handle evalListLink = atomSpace.getHandle(LIST_LINK, evalListLinkOutgoing);
    if (CoreUtils::compare(evalListLink, Handle::UNDEFINED)) {
        //cout << "Found the ListLink with the ExecLink inside" << endl;
        Handle predicateNode = atomSpace.getHandle(PREDICATE_NODE,
                               actionPredicateName);
        if (CoreUtils::compare(predicateNode, Handle::UNDEFINED)) {
            //cout << "Found the PredicateNode" << endl;
            HandleSeq evalLinkOutgoing;
            evalLinkOutgoing.push_back(predicateNode);
            evalLinkOutgoing.push_back(evalListLink);
            Handle evalLink = atomSpace.getHandle(EVALUATION_LINK,
                                                  evalLinkOutgoing);
            //cout << "evalLink = " << evalLink << endl;
            if (CoreUtils::compare(evalLink, Handle::UNDEFINED)) {
                //cout << "Found the EvalLink with the PredicateNode and the ListLink in its " << endl;
                list<HandleTemporalPair> ocurrences;
                atomSpace.getTimeInfo(back_inserter(ocurrences),
                                      evalLink,
                                      Temporal(sinceTimestamp),
                                      TemporalTable::NEXT_AFTER_END_OF);
                //if (!ocurrences.empty()) {
                //    cout << "Got the following TimeServer entry: " << ocurrences.front().toString() << " for " << TLB::getAtom(evalLink)->toString() << endl;
                //} else {
                //    cout << "Got no TimeServer entry for " << TLB::getAtom(evalLink)->toString() << endl;
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

    // get temporal pair for the EXACT timestamp. Only spaceMapNodes are
    // considered
    atomSpace.getTimeInfo(back_inserter(temporalPairs),
                          spaceMapNode, temporal,
                          TemporalTable::EXACT);
    if (!temporalPairs.empty()) {

        // found at least one temporalPair. There should be at most one
        spaceMapHandle = atomSpace.getAtTimeLink(temporalPairs[0]);
        if (!spaceServer.containsMap(spaceMapHandle)) {
            spaceMapHandle = Handle::UNDEFINED;
        }
    }

    if (spaceMapHandle == Handle::UNDEFINED) {
        // found none temporalPair with EXACT timestamp. Now looking for
        // timestamps before the one used.
        temporalPairs.clear();
        atomSpace.getTimeInfo(back_inserter(temporalPairs),
                              spaceMapNode,
                              temporal,
                              TemporalTable::PREVIOUS_BEFORE_START_OF);
        while (!temporalPairs.empty()) {
            spaceMapHandle = atomSpace.getAtTimeLink(temporalPairs[0]);
            Temporal* nextTemporal = temporalPairs[0].getTemporal();
            if (spaceServer.containsMap(spaceMapHandle)) {
                break;
            } else {
                spaceMapHandle = Handle::UNDEFINED;
                temporalPairs.clear();
                atomSpace.getTimeInfo(back_inserter(temporalPairs),
                                      spaceMapNode,
                                      *nextTemporal,
                                      TemporalTable::PREVIOUS_BEFORE_START_OF);

            }
        }
    }

    return spaceMapHandle;
}

bool AtomSpaceUtil::getPredicateValueAtSpaceMap(const AtomSpace& atomSpace,
        const std::string predicate,
        const SpaceServer::SpaceMap& sm,
        Handle obj1, Handle obj2)
{
    if (!sm.containsObject(atomSpace.getName(obj1)) ||
            !sm.containsObject(atomSpace.getName(obj2)) ) {
        logger().log(opencog::Logger::ERROR, "AtomSpaceUtil - One or both objects were not present in Space Map");
        return false;
    }

    if (predicate == "near") {

        try {
            //const SpaceServer::ObjectMetadata&
            //    md1 = sm.getMetaData(atomSpace.getName(obj1));
            const Spatial::EntityPtr& entity1 = sm.getEntity( atomSpace.getName(obj1) );
            //SpaceServer::SpaceMapPoint obj1Center( entity1.getPosition( ).x, entity1.getPosition( ).y );
            //const SpaceServer::ObjectMetadata&
            //    md2 = sm.getMetaData(atomSpace.getName(obj2));

            const Spatial::EntityPtr& entity2 = sm.getEntity( atomSpace.getName(obj2) );
            //SpaceServer::SpaceMapPoint obj2Center( entity2.getPosition( ).x, entity2.getPosition( ).y );

            //std::cout << "MD1 X : " << md1.centerX << " Y : " << md1.centerY
            //    << " MD2 X : " << md2.centerX << " Y : " << md2.centerY
            //    << std::endl;
            double dist = ( entity1->getPosition( )
                            - entity2->getPosition( ) ).length( );
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

bool AtomSpaceUtil::getHasSaidValueAtTime(const AtomSpace &atomSpace,
        unsigned long timestamp,
        unsigned long delay,
        Handle from_h,
        Handle to_h,
        const std::string& message,
        bool include_to)
{
    opencog::cassert(TRACE_INFO, delay < timestamp,
                     "timestamp - delay must be positive");
    unsigned long tl = timestamp - delay;
    unsigned long tu = timestamp;
    Temporal temp(tl, tu);
    std::list<HandleTemporalPair> ret;
    atomSpace.getTimeInfo(back_inserter(ret), Handle::UNDEFINED, temp,
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
            opencog::cassert(TRACE_INFO,
                             dynamic_cast<Node*>(TLB::getAtom(to_h)),
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
    const std::string& obj_str = atomSpace.getName(obj);
    bool insm1 = sm1.containsObject(atomSpace.getName(obj));
    bool insm2 = sm2.containsObject(atomSpace.getName(obj));
    //we consider that :
    //1)if the object appears or disappears from the spaceMaps it has moved.
    //2)if it is in neither those spaceMap it hasn't
    if (insm1 && insm2) {
        //check if has moved
        //const SpaceServer::ObjectMetadata& md1 = sm1.getMetaData(obj_str);
        //const SpaceServer::ObjectMetadata& md2 = sm2.getMetaData(obj_str);
        const Spatial::EntityPtr& entity1 = sm1.getEntity( obj_str );
        const Spatial::EntityPtr& entity2 = sm2.getEntity( obj_str );
        //return md1==md2;
        return ( *entity1 == entity2 );
    } else if (!insm1 && !insm2)
        return false; //case 2)
    else return true; //case 1)
}

bool AtomSpaceUtil::isMovingBtwSpaceMap(const AtomSpace& atomSpace,
                                        const SpaceServer::SpaceMap& sm,
                                        Handle obj)
{
    //TODO
    opencog::cassert(TRACE_INFO, false);
    return false;
}

float AtomSpaceUtil::getPredicateValue(const AtomSpace &atomSpace,
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
                                          ( std::string( "AtomSpaceUtil - Predicate not found: " ) + predicateName ).c_str( ) );
    } // if

    // testing if there is a list link already
    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
    if (listLinkHandle == Handle::UNDEFINED) {
        throw opencog::NotFoundException( TRACE_INFO, ( "AtomSpaceUtil - List link not found. predicateName[" + predicateName + "]" ).c_str( ) );
    } // if

    HandleSeq seq;
    seq.push_back(predicateHandle);
    seq.push_back(listLinkHandle);

    std::vector<Handle> allHandles;
    atomSpace.getHandleSet(back_inserter(allHandles),
                           seq, NULL, NULL, 2, EVALUATION_LINK, false);

    if ( allHandles.size() != 1) {
        throw opencog::NotFoundException( TRACE_INFO, ( "AtomSpaceUtil - There is no evaluation link for predicate: " + predicateName ).c_str( ) );
    } // if

    /*
    std::vector<Handle> allHandles;
    atomSpace.getHandleSet(back_inserter(allHandles), seq, NULL, NULL, 2, EVALUATION_LINK, false);

    if(allHandles.size() != 1){

      if(b == Handle::UNDEFINED){
        //TODO: elvys remove logs comment
        //          logger().log(opencog::Logger::DEBUG, "AtomSpaceUtil - [%s ListLink(%s)]: found %d predicates.",
        //            predicateName.c_str(), atomSpace.getName(a).c_str(), allHandles.size());
      } else {
        //          logger().log(opencog::Logger::DEBUG, "AtomSpaceUtil - [%s ListLink(%s, %s)]: found %d predicates.",
        //            predicateName.c_str(), atomSpace.getName(a)).c_str(),
        //            atomSpace.getName(b)).c_str(), allHandles.size());
      } // else

      std::stringstream message;
      message << "The search must return just one item, but ";
      message << allHandles.size( );
      message << "  was found";
      throw opencog::NotFoundException( TRACE_INFO, message.str( ).c_str( ) );

    } // if
    */
    return atomSpace.getTV(allHandles[0]).getMean();
}


bool AtomSpaceUtil::isPredicateTrue(const AtomSpace &atomSpace,
                                    std::string predicateName,
                                    Handle a, Handle b)
{
    try {
        return ( getPredicateValue( atomSpace, predicateName, a, b ) > 0.5 );
    } catch ( opencog::NotFoundException& ex ) {
        return false;
    } // catch
}

bool AtomSpaceUtil::isPetOwner( const AtomSpace& atomSpace,
                                Handle avatar, Handle pet )
{
    HandleSeq seq0;
    seq0.push_back(avatar);
    seq0.push_back(pet);

    // testing if there is a predicate already
    Handle predicateHandle = atomSpace.getHandle(PREDICATE_NODE,
                             OWNERSHIP_PREDICATE_NAME );
    if (predicateHandle == Handle::UNDEFINED) {
        logger().log(opencog::Logger::FINE,
                     "IsFriendly - Found no \"owns\" predicate.");
        return false;
    } // if

    // testing if there is a list link already
    Handle listLinkHandle = atomSpace.getHandle(LIST_LINK, seq0);
    if (listLinkHandle == Handle::UNDEFINED) {
        logger().log(opencog::Logger::FINE,
                     "IsFriendly - Obj %s and %s have no ListLink.",
                     atomSpace.getName(avatar).c_str(),
                     atomSpace.getName(pet).c_str());
        return false;
    } // if

    HandleSeq seq;
    seq.push_back(predicateHandle);
    seq.push_back(listLinkHandle);

    std::vector<Handle> allHandles;
    atomSpace.getHandleSet(back_inserter(allHandles),
                           seq, NULL, NULL, 2, EVALUATION_LINK, false);

    if (allHandles.size() != 1) {
        logger().log(opencog::Logger::WARN,
                     "IsFriendly - Found %d EvalLinks. Should be one.",
                     allHandles.size());
        return false;
    } // if

    return true;

}

bool AtomSpaceUtil::getSizeInfo(const AtomSpace& atomSpace,
                                Handle object,
                                double& length, double& width, double &height)
{

    Handle sizePredicate = atomSpace.getHandle(PREDICATE_NODE,
                           SIZE_PREDICATE_NAME);
    if (sizePredicate == Handle::UNDEFINED) {
        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - No size predicate found.");
        return false;
    }

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
            if (atomSpace.getOutgoing(listLink, 0) == object) {
//TODO: elvys remove logs comment
                length = atof(atomSpace.getName(atomSpace.getOutgoing(listLink, 1)).c_str());
                width = atof(atomSpace.getName(atomSpace.getOutgoing(listLink, 2)).c_str());
                height = atof(atomSpace.getName(atomSpace.getOutgoing(listLink, 3)).c_str());
//                logger().log(opencog::Logger::INFO, "AtomSpaceUtil - Obj %s (width: %.2lf length: %.2lf height: %.2lf)",
//                            atomSpace.getName(object).c_str(), width, length, height);
                return true;
            }
        }
    }
//TODO: elvys remove logs comment
//    logger().log(opencog::Logger::FINE, "AtomSpaceUtil - No size pred for obj %s found.",
//                    atomSpace.getName(object).c_str());
    return false;
}

Handle AtomSpaceUtil::addGenericPropertyPred(AtomSpace& atomSpace,
        std::string predicateName,
        const HandleSeq& ll_out,
        const TruthValue &tv, bool permanent, const Temporal &t)
{
    // truth criterion as defined in
    // https://extranet.vettalabs.com:8443/bin/view/Petaverse/PetaverseCombo
    bool predBool = true;
    if (tv.getMean() >= 0.5) {
        predBool = false;
    }

    Handle ph = atomSpace.getHandle(PREDICATE_NODE, predicateName);
    // if predicate handle not defined and TV equals < 0.5 just return
    if (ph == Handle::UNDEFINED && predBool) {
        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - %s not added (no pred handle and TV less than 0.5).",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    } else {
        ph = AtomSpaceUtil::addNode(atomSpace,
                                    PREDICATE_NODE,
                                    predicateName, true);
    }

    Handle ll = atomSpace.getHandle(LIST_LINK, ll_out);
    // if list link handle not defined and TV equals < 0.5 just return
    if (ll == Handle::UNDEFINED && predBool) {
        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - %s not added (no ListLink and TV less than 0.5)",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    } else {
        ll = atomSpace.addLink(LIST_LINK, ll_out);
    }

    HandleSeq hs2;
    hs2.push_back(ph);
    hs2.push_back(ll);
    Handle el = atomSpace.getHandle(EVALUATION_LINK, hs2);

    // if evaluation link handle not defined and TV equals < 0.5 just return
    if (el == Handle::UNDEFINED && predBool) {
        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - %s not added (no EvalLink and TV less than 0.5).",
                     predicateName.c_str());
        return Handle::UNDEFINED;
    } else {
        el = atomSpace.addLink(EVALUATION_LINK, hs2);
        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - %s added with TV %f.",
                     predicateName.c_str(), tv.getMean());
    }

    atomSpace.setTV(el, tv);

    Handle result;
    // if not undefined temporal then  a time information should be inserted
    // inserted into AtomSpace.
    if (t != UNDEFINED_TEMPORAL) {
        result = atomSpace.addTimeInfo(el, t);
    } else {
        result = el;
    }
    if (permanent) {
        atomSpace.setLTI(result, 1);
    }
    return result;
}

Handle AtomSpaceUtil::getMostRecentEvaluationLink(const AtomSpace& atomSpace,
        const std::string& predicateNodeName )
{

    std::vector<HandleTemporalPair> timestamps;
    getAllEvaluationLinks( atomSpace, timestamps, predicateNodeName );

    if ( timestamps.size() == 0 ) {
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - Found no entries for PredicateNode '%s' in TimeServer.",
                     predicateNodeName.c_str());
        return Handle::UNDEFINED;
    } // if

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
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - Timeserver returned NULL handle for PredicateNode '%s'",
                     predicateNodeName.c_str());
        return Handle::UNDEFINED;
    } // if

    return selectedHandle;
}

float AtomSpaceUtil::getCurrentPetFeelingLevel( const AtomSpace& atomSpace,
        const std::string& petId,
        const std::string& feelingName )
{
    std::string predicateName = petId + "." + feelingName;

    Handle selectedEvalLink = getMostRecentEvaluationLink( atomSpace,
                              predicateName );

    if (selectedEvalLink == Handle::UNDEFINED) {
        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - Found no EvaluationLink for PredicateNode '%s'.",
                     predicateName.c_str());
        return -1;
    } // if

    Handle primaryListLink = atomSpace.getOutgoing(selectedEvalLink, 1);
    if (primaryListLink == Handle::UNDEFINED) {
        logger().log(opencog::Logger::WARN,
                     "AtomSpaceUtil - Found no signals for PredicateNode '%s'. Null primary listLink.",
                     predicateName.c_str());
        return -1;
    } // if

    int i = 0;
    do {
        Handle secondaryListLink = atomSpace.getOutgoing(primaryListLink, i);
        if (secondaryListLink == Handle::UNDEFINED) {
            logger().log(opencog::Logger::WARN,
                         "AtomSpaceUtil - Found no signals for PredicateNode '%s'. Null secondary listLink.",
                         predicateName.c_str());
            return -1;
        }
        Handle paramName = atomSpace.getOutgoing(secondaryListLink, 0);
        if (paramName == Handle::UNDEFINED) {
            logger().log(opencog::Logger::WARN,
                         "AtomSpaceUtil - Found no signals for PredicateNode '%s'. Null paramName.",
                         predicateName.c_str());
            return -1;
        } // if

        if (atomSpace.getName(paramName) == "level") {
            Handle paramValue = atomSpace.getOutgoing(secondaryListLink, 1);
            if (paramValue == Handle::UNDEFINED) {
                logger().log(opencog::Logger::WARN,
                             "AtomSpaceUtil - Found no signals for PredicateNode '%s'. Null paramValue.",
                             predicateName.c_str());
                return -1;
            } // if

            return atof( atomSpace.getName(paramValue).c_str( ) );
        } // if

        i++;

        if (i >= atomSpace.getArity(primaryListLink)) {
            logger().log(opencog::Logger::WARN,
                         "AtomSpaceUtil - Found no signals for PredicateNode '%s'. Invalid listLink.",
                         predicateName.c_str());
            return -1;
        } // if

    } while (true);

    return -1;
}

void AtomSpaceUtil::getAllEvaluationLinks(const AtomSpace& atomSpace,
        std::vector<HandleTemporalPair>& timestamps,
        const std::string& predicateNodeName,
        const Temporal& temporal,
        TemporalTable::TemporalRelationship criterion,
        bool needSort)
{

    logger().log(opencog::Logger::FINE,
                 "AtomSpaceUtil - getAllEvaluationLinks - Searching for PredicateNode '%s'.",
                 predicateNodeName.c_str());

    std::vector<Handle> handles;
    atomSpace.getHandleSet(back_inserter(handles),
                           predicateNodeName.c_str(),
                           PREDICATE_NODE,
                           EVALUATION_LINK, true);

    if (handles.empty()) {
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - getAllEvaluationLinks - Found no EvaluationLink for PredicateNode '%s'",
                     predicateNodeName.c_str());
        return;
    } // if

    for ( unsigned int i = 0; i < handles.size(); ++i ) {
        atomSpace.getTimeInfo(back_inserter(timestamps),
                              handles[i], temporal, criterion);
    } // for

    if (needSort) {
        std::sort(timestamps.begin(),
                  timestamps.end(),
                  HandleTemporalPairEntry::SortComparison());
    }

    logger().log(opencog::Logger::FINE,
                 "AtomSpaceUtil - getAllEvaluationLinks - end.");
}

Handle AtomSpaceUtil::setPredicateValue( AtomSpace& atomSpace,
        std::string predicateName,
        const TruthValue &tv,
        Handle object1,
        Handle object2 )
{
    HandleSeq listLink;
    listLink.push_back( object1 );
    if ( object2 != Handle::UNDEFINED ) {
        listLink.push_back( object2 );
    }
    Handle listLinkHandle = AtomSpaceUtil::addLink(atomSpace,
                            LIST_LINK,
                            listLink);
    Handle predicateHandle = AtomSpaceUtil::addNode(atomSpace,
                             PREDICATE_NODE,
                             predicateName, true);
    HandleSeq evalLink;
    evalLink.push_back( predicateHandle );
    evalLink.push_back( listLinkHandle );
    Handle evalLinkHandle = AtomSpaceUtil::addLink(atomSpace,
                            EVALUATION_LINK,
                            evalLink, true);
    atomSpace.setTV( evalLinkHandle, tv );
    return evalLinkHandle;
}


Handle AtomSpaceUtil::addPropertyPredicate(AtomSpace& atomSpace,
        std::string predicateName,
        Handle object,
        const TruthValue &tv,
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
        const TruthValue& tv,
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
    if ( holderHandle == Handle::UNDEFINED ) {
        return;
    } // if

    Handle objectHandle = Handle::UNDEFINED;
    if ( objectId != "" ) {
        objectHandle = atomSpace.getHandle( SL_OBJECT_NODE, objectId );
        if ( objectHandle != Handle::UNDEFINED ) {
            logger().log( opencog::Logger::DEBUG,
                          "AtomSpaceUtil - Object is a SL_OBJECT_NODE" );
        } else {
            objectHandle = atomSpace.getHandle( SL_ACCESSORY_NODE, objectId );
            if ( objectHandle != Handle::UNDEFINED ) {
                logger().log( opencog::Logger::DEBUG,
                              "AtomSpaceUtil - Object is a SL_ACCESSORY_NODE" );
            } else {
                logger().log( opencog::Logger::ERROR,
                              "AtomSpaceUtil - Object cannot be identified" );
                return;
            } // if
        } // if
    } // if

    if (objectHandle == Handle::UNDEFINED) {
        // drop operation
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - The object's handle is undefined. Holder dropped the object");
        Handle isHoldingOldAtTimeLink = getMostRecentIsHoldingAtTimeLink(atomSpace, holderId);
        if (isHoldingOldAtTimeLink != Handle::UNDEFINED) {
            // Reconfigure the timestamp of the last grabbed object
            Handle isHoldingEvalLink = getTimedHandle(atomSpace,
                                       isHoldingOldAtTimeLink);
            if ( isHoldingEvalLink != Handle::UNDEFINED ) {
                Temporal t = getTemporal(atomSpace, isHoldingOldAtTimeLink);
                // TODO: What if it was not holding anything anymore (this happens if it gets 2 or more consecutive drop operations)
                // Shouldn't the old isHoldingatTimeLink be updated?
                //remove the old atTimeLink
                atomSpace.removeAtom(isHoldingOldAtTimeLink);
                //add the new one
                long unsigned tl = t.getLowerBound();
                Temporal new_temp(tl, std::max(currentTimestamp - 20, tl));
                logger().log(opencog::Logger::DEBUG,
                             "AtomSpaceUtil - setupHoldingObject: new time = '%s'",
                             new_temp.toString().c_str());
                atomSpace.addTimeInfo(isHoldingEvalLink, new_temp); // Now, it can be forgotten.
            } // if
        } // if
        AtomSpaceUtil::setPredicateValue( atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          SimpleTruthValue( 0.0, 1.0 ),
                                          holderHandle );
    } else {
        // grab: it is now holding an object
        // TODO: What if it is already holding another thing? Shouldn't the old isHoldingatTimeLink be updated?
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - Now '%s' is holding '%s' at '%ul'",
                     atomSpace.getName(holderHandle).c_str(),
                     atomSpace.getName(objectHandle).c_str(),
                     currentTimestamp);
        AtomSpaceUtil::setPredicateValue( atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          SimpleTruthValue( 1.0, 1.0 ),
                                          holderHandle );
        Handle isHoldingAtTimeLink =
            AtomSpaceUtil::addPropertyPredicate( atomSpace,
                                                 IS_HOLDING_PREDICATE_NAME,
                                                 holderHandle,
                                                 objectHandle,
                                                 SimpleTruthValue( 1.0, 1.0 ),
                                                 Temporal(currentTimestamp) );
        atomSpace.setLTI(isHoldingAtTimeLink, 1); // Now, it cannot be forgotten (until the agent drop the object)
    } // else
}

Handle AtomSpaceUtil::getLatestHoldingObjectHandle(const AtomSpace& atomSpace,
        const std::string& holderId )
{

    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) {
        return Handle::UNDEFINED;
    } // if

    Handle isHoldingLink = getMostRecentIsHoldingLink( atomSpace, holderId );
    if ( isHoldingLink != Handle::UNDEFINED ) {
        Handle listLink = atomSpace.getOutgoing(isHoldingLink, 1);
        if ( listLink != Handle::UNDEFINED ) {
            Handle objectHandle = atomSpace.getOutgoing(listLink, 1);
            if ( objectHandle != Handle::UNDEFINED ) {
                return objectHandle;
            } // if
            logger().log( opencog::Logger::ERROR,
                          "AtomSpaceUtil - There is no object on list link" );
        } // if
        logger().log( opencog::Logger::ERROR,
                      "AtomSpaceUtil - There is no listlink on isHoldingLink" );
    } // if

    logger().log( opencog::Logger::DEBUG,
                  "AtomSpaceUtil - There is no isHoldingLink for %s",
                  holderId.c_str( ) );
    return Handle::UNDEFINED;
}

bool AtomSpaceUtil::isObjectBeingHolded( const AtomSpace& atomSpace,
        const std::string& objectId )
{
    return ( getObjectHolderHandle( atomSpace, objectId ) != Handle::UNDEFINED );
}

Handle AtomSpaceUtil::getObjectHolderHandle( const AtomSpace& atomSpace,
        const std::string& objectId )
{
    // TODO: try to optimize this method. it is using twice the getHandleSet for
    // isHolding (below and through getLatestHoldingObjectHandle)
    std::vector<Handle> handles;
    atomSpace.getHandleSet( back_inserter(handles),
                            SL_OBJECT_NODE,
                            objectId, true );

    if ( handles.size( ) != 1 ) {
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - No agent is holding object[%s]",
                     objectId.c_str( ) );
        return Handle::UNDEFINED;
    } // if
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

            atomSpace.getTimeInfo( back_inserter( timestamps ), handles[i]);


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

std::string AtomSpaceUtil::getObjectHolderId( const AtomSpace& atomSpace,
        const std::string& objectId )
{
    Handle objectHandle = getObjectHolderHandle( atomSpace, objectId );
    if ( objectHandle != Handle::UNDEFINED ) {
        return atomSpace.getName( objectHandle );
    } // if
    return "";
}


Handle AtomSpaceUtil::getMostRecentIsHoldingAtTimeLink(const AtomSpace& atomSpace,
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
                atomSpace.getTimeInfo( back_inserter( timestamps ),
                                       handles[i]);
            } // if
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

        logger().log( opencog::Logger::DEBUG,
                      "AtomSpaceUtil - The most recent holded object %ul",
                      timestamps[mostRecentIndex].getTemporal( )->getUpperBound( ) );

        return atomSpace.getAtTimeLink(timestamps[mostRecentIndex]);
    } // if
    return Handle::UNDEFINED;
}

Handle AtomSpaceUtil::getMostRecentIsHoldingLink(const AtomSpace& atomSpace,
        const std::string& holderId )
{
    Handle h = getMostRecentIsHoldingAtTimeLink(atomSpace, holderId);
    if (h != Handle::UNDEFINED)
        return atomSpace.getOutgoing(h, 1);
    else return Handle::UNDEFINED;
}

std::string AtomSpaceUtil::getHoldingObjectId(const AtomSpace& atomSpace,
        const std::string& holderId )
{
    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) {
        return "";
    } // if

    if ( !AtomSpaceUtil::isPredicateTrue( atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          holderHandle ) ) {
        return "";
    } // if

    Handle objectHandle = getLatestHoldingObjectHandle( atomSpace, holderId );
    if ( objectHandle == Handle::UNDEFINED ) {
        return "";
    } // if

    return atomSpace.getName(objectHandle);
}

bool AtomSpaceUtil::isHoldingSomething(const AtomSpace& atomSpace,
                                       const std::string& holderId)
{
    Handle holderHandle = getAgentHandle(atomSpace, holderId);
    if (holderHandle == Handle::UNDEFINED) {
        return false;
    }

    return AtomSpaceUtil::isPredicateTrue(atomSpace,
                                          IS_HOLDING_SOMETHING_PREDICATE_NAME,
                                          holderHandle);
}

Handle AtomSpaceUtil::getHoldingObjectHandleAtTime(const AtomSpace& atomSpace,
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
            } // if
            logger().log( opencog::Logger::ERROR,
                          "AtomSpaceUtil - There is no object on list link" );
        } // if
        logger().log( opencog::Logger::ERROR,
                      "AtomSpaceUtil - There is no listlink on isHoldingLink" );
    } // if

    logger().log( opencog::Logger::DEBUG,
                  "AtomSpaceUtil - There is no isHoldingLink for %s",
                  holderId.c_str( ) );
    return Handle::UNDEFINED;
}

Handle AtomSpaceUtil::getIsHoldingLinkAtTime(const AtomSpace& atomSpace,
        const std::string& holderId,
        unsigned long time)
{
    Handle holderHandle = getAgentHandle( atomSpace, holderId );
    if ( holderHandle == Handle::UNDEFINED ) {
        return Handle::UNDEFINED;
    } // if


    std::vector<Handle> handles;
    atomSpace.getHandleSet(back_inserter(handles), IS_HOLDING_PREDICATE_NAME,
                           PREDICATE_NODE, EVALUATION_LINK, true);

    std::vector<HandleTemporalPair> timestamps;
    for (std::vector<Handle>::const_iterator h_i = handles.begin();
            h_i != handles.end(); ++h_i) {
        Handle listLink = atomSpace.getOutgoing(*h_i, 1);
        cassert(TRACE_INFO, listLink != Handle::UNDEFINED,
                "ListLink must be defined");
        cassert(TRACE_INFO, atomSpace.getArity(listLink) == 2,
                "IsHolding predicate must have 2 arguments");
        // get only holder's eval-links occuring within 'time'

        if (atomSpace.getOutgoing(listLink, 0) == holderHandle) {
            logger().log(opencog::Logger::DEBUG,
                         "AtomSpaceUtil - before '%d' timestamps for isHolding pred for '%s'.",
                         timestamps.size(), holderId.c_str());

            atomSpace.getTimeInfo(back_inserter(timestamps), *h_i,
                                  Temporal(time), TemporalTable::INCLUDES);

            logger().log(opencog::Logger::DEBUG,
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
            opencog::cassert(TRACE_INFO, false, s.c_str());
        }
        return timestamps[0].getHandle();
    } else return Handle::UNDEFINED;
}

std::string AtomSpaceUtil::getHoldingObjectIdAtTime(const AtomSpace& as,
        const std::string& holderId,
        unsigned long time)
{
    Handle holderHandle = getAgentHandle( as, holderId );
    if ( holderHandle == Handle::UNDEFINED ) {
        return "";
    } // if

    Handle objectHandle = getHoldingObjectHandleAtTime(as, holderId, time);
    if ( objectHandle == Handle::UNDEFINED ) {
        return "";
    } // if

    return as.getName(objectHandle);
}

std::string AtomSpaceUtil::getObjIdFromName( const AtomSpace& atomSpace,
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
        atomSpace.getHandleSet(back_inserter(wrLinks), outgoing,
                               NULL, NULL, 2, WR_LINK, false);
        if (!wrLinks.empty()) {
            objIdHandle = atomSpace.getOutgoing(wrLinks[0], 1);
            // TODO: check for multiple answers...
        } else {
            logger().log( opencog::Logger::DEBUG,
                          "AtomSpaceUtil::getObjIdFromName: Found avatar name '%s' but no avatar id associated to this name.",
                          objName.c_str());
        }
    } else {
        logger().log( opencog::Logger::DEBUG,
                      "AtomSpaceUtil::getObjIdFromName: didn't find avatar name '%s'",
                      objName.c_str());
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
            atomSpace.getHandleSet(back_inserter(wrLinks), outgoing,
                                   NULL, NULL, 2, WR_LINK, false);
            if (!wrLinks.empty()) {
                objIdHandle = atomSpace.getOutgoing(wrLinks[0], 1);
                // TODO: check for multiple answers...
            } else {
                logger().log( opencog::Logger::DEBUG,
                              "AtomSpaceUtil::getObjIdFromName: Found avatar name '%s' but no avatar id associated to this name.",
                              lcObjName.c_str());
            }
        } else {
            logger().log( opencog::Logger::DEBUG,
                          "AtomSpaceUtil::getObjIdFromName: didn't find avatar name '%s'",
                          lcObjName.c_str());
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
            atomSpace.getHandleSet(back_inserter(wrLinks), outgoing,
                                   NULL, NULL, 2, WR_LINK, false);
            if (!wrLinks.empty()) {
                objIdHandle = atomSpace.getOutgoing(wrLinks[0], 1);
                // TODO: check for multiple answers...
            } else {
                logger().log( opencog::Logger::DEBUG,
                              "AtomSpaceUtil::getObjIdFromName: Found avatar name '%s' but no avatar id associated to this name.",
                              firstCapObjName.c_str());
            }
        } else {
            logger().log( opencog::Logger::DEBUG,
                          "AtomSpaceUtil::getObjIdFromName: didn't find avatar name '%s'",
                          firstCapObjName.c_str());
        }
    }
    if (objIdHandle != Handle::UNDEFINED) {
        result = atomSpace.getName(objIdHandle);
    }
    logger().log( opencog::Logger::DEBUG,
                  "AtomSpaceUtil::getObjIdFromName: returning '%s'",
                  result.c_str());
    return result;
}

Handle AtomSpaceUtil::getMostRecentPetSchemaExecLink(const AtomSpace& atomSpace,
        unsigned long timestamp,
        bool schemaSuccessful)
{

    logger().log(opencog::Logger::DEBUG,
                 "AtomSpaceUtil - getMostRecentPetSchemaExecLink");
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

    if (timestamps.empty()) {
        return Handle::UNDEFINED;
    }

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
        if (listLink == Handle::UNDEFINED) {
            return "";
        } // if

        std::stringstream parameters;
        for (int i = 0; i < atomSpace.getArity(listLink); i++ ) {
            Handle schemaParam = atomSpace.getOutgoing(listLink, i);

            if (schemaParam == Handle::UNDEFINED) {
                logger().log(opencog::Logger::ERROR,
                             "AtomSpaceUtil - Found no param for schema");
                return "";
            } // if

            if (i > 0) {
                parameters << ", ";
            } // if

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

Handle AtomSpaceUtil::getMostRecentAgentActionLink( const AtomSpace& atomSpace,
        const std::string& agentId,
        const Temporal& temporal,
        TemporalTable::TemporalRelationship criterion)
{


    Handle latestActionDoneLink = Handle::UNDEFINED;
    std::string agentType = "unknown";

    // reference:
    // https://extranet.vettalabs.com:8443/bin/view/Petaverse/PerceptionActionInterface

    // get eval links for all agents actions done
    std::vector<HandleTemporalPair> timestamps;
    AtomSpaceUtil::getAllEvaluationLinks( atomSpace, timestamps,
                                          ACTION_DONE_PREDICATE_NAME,
                                          temporal, criterion );

    Temporal previousTemporal(0);

    // filter eval links by agent name and most recent temporal
    unsigned int i;
    for ( i = 0; i < timestamps.size( ); ++i ) {
        Temporal& temporal = *( timestamps[i].getTemporal( ) );

        Handle evalLink = timestamps[i].getHandle( );

        if ( evalLink == Handle::UNDEFINED) {
            logger().log(opencog::Logger::ERROR,
                         "AtomSpaceUtil - AtomSpace returned undefined handle for evaluation link (agent action done predicate)" );
            continue;
        } // if

        Handle agentActionLink = atomSpace.getOutgoing(evalLink, 1);

        if ( agentActionLink == Handle::UNDEFINED) {
            logger().log(opencog::Logger::ERROR,
                         "AtomSpaceUtil - Found no agent action for actionDone predicate." );
            continue;
        } // if

        Handle agentIdNode = atomSpace.getOutgoing(agentActionLink, 0);
        if (agentIdNode == Handle::UNDEFINED ) {
            logger().log(opencog::Logger::ERROR,
                         "AtomSpaceUtil - Found no agent name for actionDone predicate" );
            continue;
        } // if

        int inspectedAgentTypeCode = atomSpace.getType(agentIdNode);
        if ( !atomSpace.isNode( inspectedAgentTypeCode ) ) {
            logger().log(opencog::Logger::FINE,
                         "AtomSpaceUtil - Skipping handles that isn't nodes. Inspected handle type: %d",
                         inspectedAgentTypeCode );
            continue;
        } // if

        const std::string& inspectedAgentId = atomSpace.getName( agentIdNode );

        if ( inspectedAgentId != agentId ) {
            logger().log(opencog::Logger::FINE,
                         "AtomSpaceUtil - Inspected agent id is [%s; type=%d], but required is %s",
                         inspectedAgentId.c_str( ),
                         inspectedAgentTypeCode,
                         agentId.c_str( ) );
            // it is the wrong agent, then skip it
            continue;
        } // if

        Handle agentActionNode = atomSpace.getOutgoing(agentActionLink, 1);
        if (agentActionNode == Handle::UNDEFINED) {
            logger().log(opencog::Logger::ERROR,
                         "AtomSpaceUtil - Found no agent action name for actionDone predicate" );
            continue;
        } // if

        logger().log(opencog::Logger::FINE,
                     "AtomSpaceUtil - Previous temporal[%lu %lu], Inspected temporal[%lu %lu]",
                     previousTemporal.getA(),
                     previousTemporal.getB(),
                     temporal.getA(),
                     temporal.getB() );
        if ( temporal > previousTemporal ) {

            latestActionDoneLink = agentActionLink;
            previousTemporal = temporal;

            if ( inspectedAgentTypeCode == SL_AVATAR_NODE ) {
                agentType = "an avatar";
            } else if ( inspectedAgentTypeCode == SL_PET_NODE ) {
                agentType = "a pet";
            } else if ( inspectedAgentTypeCode == SL_HUMANOID_NODE ) {
                agentType = "an humanoid";
            } else {
                logger().log(opencog::Logger::ERROR,
                             "AtomSpaceUtil - Invalid agentIdNode type: %i",
                             inspectedAgentTypeCode );
                continue;
            } // else

        } // if

    } // for

    logger().log(opencog::Logger::DEBUG,
                 "AtomSpaceUtil::getMostRecentAgentActionLink - Agent %s is %s",
                 agentId.c_str( ),
                 agentType.c_str( ) );

    return latestActionDoneLink;
}

Handle AtomSpaceUtil::getMostRecentAgentActionLink( const AtomSpace& atomSpace,
        const std::string& agentId,
        const std::string& actionName,
        const Temporal& temporal,
        TemporalTable::TemporalRelationship criterion )
{

    Handle agentHandle = getAgentHandle( atomSpace, agentId );
    if ( agentHandle == Handle::UNDEFINED ) {
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - Found no agent identified by %s",
                     agentId.c_str( ) );
        return Handle::UNDEFINED;
    } // if

    Handle predicateNodeHandle = atomSpace.getHandle(PREDICATE_NODE,
                                 ACTION_DONE_PREDICATE_NAME);
    if ( predicateNodeHandle == Handle::UNDEFINED ) {
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - Found no predicate node named %s",
                     ACTION_DONE_PREDICATE_NAME )
        ;
        return Handle::UNDEFINED;
    } // if

    Handle actionNodeHandle = atomSpace.getHandle( NODE, actionName );
    if ( actionNodeHandle == Handle::UNDEFINED ) {
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - Found no NODE for action named %s",
                     actionName.c_str( ) );
        return Handle::UNDEFINED;
    } // if

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
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - the given agent wasn't executed group_command yet" );
        // there is no group command for the given agent
        return Handle::UNDEFINED;
    } // if


    std::vector<HandleTemporalPair> timestamps;

    // filter and remove handles which isn't linked by an evaluation_link that has, as the first outgoing,
    // the predicate node handle ACTION_DONE_PREDICATE_NAME
    //std::vector<Handle> filteredHandles;
    unsigned int i;
    for ( i = 0; i < handles.size( ); ++i ) {
        HandleSeq incomingLinks = atomSpace.getIncoming(handles[i]);
        logger().log(opencog::Logger::DEBUG,
                     "AtomSpaceUtil - %d incoming links were identified",
                     incomingLinks.size( ) );
        unsigned int j;
        for ( j = 0; j < incomingLinks.size( ); ++j ) {
            logger().log(opencog::Logger::DEBUG,
                         "AtomSpaceUtil - %d) type[%s] EVAL_LINK[%s], outgoing0[%s] predicateNode[%s]",
                         j,
                         classserver().getTypeName( atomSpace.getType( incomingLinks[j] ) ).c_str( ),
                         classserver().getTypeName( EVALUATION_LINK ).c_str( ),
                         atomSpace.getOutgoing( incomingLinks[j], 0 ).str( ).c_str( ),
                         predicateNodeHandle.str( ).c_str( ),
                         atomSpace.getOutgoing( incomingLinks[j], 1 ).str( ).c_str( ) );
            if ( atomSpace.getType( incomingLinks[j] ) == EVALUATION_LINK &&
                    atomSpace.getOutgoing( incomingLinks[j], 0 ) == predicateNodeHandle ) {
                atomSpace.getTimeInfo( back_inserter(timestamps),
                                       incomingLinks[j], temporal, criterion );

                //filteredHandles.push_back( incomingLinks[j] );
            } // if
        } // for
    } // for

    logger().log(opencog::Logger::DEBUG,
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

    logger().log(opencog::Logger::DEBUG,
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
                logger().log( opencog::Logger::ERROR,
                              "AtomSpaceUtil - There is no description for this action" );
                return "";
            } // if

            if ( atomSpace.getArity( predicateListLink ) <= 2 ) {
                logger().log( opencog::Logger::ERROR,
                              "AtomSpaceUtil - There is no parameters on the given action" );
                return "";
            } // if

            Handle actionParametersLink = atomSpace.getOutgoing( predicateListLink, 2 );
            if ( actionParametersLink == Handle::UNDEFINED ) {
                logger().log( opencog::Logger::ERROR,
                              "AtomSpaceUtil - There is no parameters on the given action" );
                return "";
            } // if

            int i;
            std::stringstream parameters;
            for ( i = 0; i < atomSpace.getArity(actionParametersLink); ++i ) {
                Handle actionParam = atomSpace.getOutgoing(actionParametersLink, i);
                if ( actionParam == Handle::UNDEFINED) {
                    logger().log(opencog::Logger::ERROR,
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
            logger().log(opencog::Logger::ERROR, "AtomSpaceUtil - Invalid outgoing: %s", ex.getMessage( ) );
        } // catch
    } // if
    return "";
}

Handle AtomSpaceUtil::getRuleImplicationLink(const AtomSpace& atomSpace,
        const std::string& rule)
{

    Handle rulePhraseNode = atomSpace.getHandle(PHRASE_NODE, rule);
    if (rulePhraseNode == Handle::UNDEFINED) {
        logger().log(opencog::Logger::ERROR,
                     "AtomSpaceUtil - Found no PhraseNode for rule '%s'.",
                     rule.c_str());
        return Handle::UNDEFINED;
    }

    std::vector<Handle> ruleReferenceLink;
    atomSpace.getHandleSet(back_inserter(ruleReferenceLink),
                           rulePhraseNode, REFERENCE_LINK, false);
    if (ruleReferenceLink.size() != 1) {
        logger().log(opencog::Logger::ERROR,
                     "AtomSpaceUtil - There should be exactly one ReferenceLink to rule '%s', found '%d'.",
                     rule.c_str(),
                     ruleReferenceLink.size());
        return Handle::UNDEFINED;
    }

    // handle to ImplicationLink
    Handle implicationLink = atomSpace.getOutgoing(ruleReferenceLink[0], 1);
    if (atomSpace.getType(implicationLink) != IMPLICATION_LINK) {
        logger().log(opencog::Logger::ERROR,
                     "AtomSpaceUtil - Outgoing atom index [1] should be an ImplicationLink. Got '%s'.",
                     classserver().getTypeName(atomSpace.getType(implicationLink)).c_str());
        return Handle::UNDEFINED;
    }

    return implicationLink;
}

float AtomSpaceUtil::getRuleImplicationLinkStrength(const AtomSpace& atomSpace,
        const std::string& rule,
        const std::string& agentModeName )
{

    Handle implicationLink = AtomSpaceUtil::getRuleImplicationLink(atomSpace,
                             rule);
    if (implicationLink == Handle::UNDEFINED) {
        logger().log(opencog::Logger::ERROR,
                     "AtomSpaceUtil - Found no ImplicationLink for rule '%s'.",
                     rule.c_str());
        return (-1.0f);
    }
    Handle agentModeNode = atomSpace.getHandle( CONCEPT_NODE, agentModeName );
    if ( agentModeNode == Handle::UNDEFINED ) {
        logger().log(opencog::Logger::ERROR,
                     "AtomSpaceUtil - Found no Handle for the given agent mode '%s'.",
                     agentModeName.c_str());
        return (-1.0f);
    } // if

    // strength is given by link TruthValue
    return (atomSpace.getTV(implicationLink, VersionHandle( CONTEXTUAL,
                            agentModeNode ) ).toFloat());
}

Spatial::Math::Vector3 AtomSpaceUtil::getMostRecentObjectVelocity( const AtomSpace& atomSpace, const std::string& objectId, unsigned long afterTimestamp )
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
            return Spatial::Math::Vector3( atof( atomSpace.getName(atomSpace.getOutgoing(listLink, 1)).c_str() ),  // x
                                           atof( atomSpace.getName(atomSpace.getOutgoing(listLink, 2)).c_str() ),  // y
                                           atof( atomSpace.getName(atomSpace.getOutgoing(listLink, 3)).c_str() )); // z
        } // if
    } // for
    return Spatial::Math::Vector3( 0, 0, 0 );
}

Handle AtomSpaceUtil::getObjectHandle( const AtomSpace& atomSpace,
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
        //opencog::cassert(TRACE_INFO, as.getHandle(CONCEPT_NODE, objectId) != Handle::UNDEFINED);
        return atomSpace.getHandle(CONCEPT_NODE, objectId);
    } else { //Now let's deal with the default case
        HandleSeq tmp;
        atomSpace.getHandleSet(std::back_inserter(tmp),
                               SL_ACCESSORY_NODE, objectId);
        if (tmp.empty()) { //it is not an accessory, let's try a structure
            atomSpace.getHandleSet(std::back_inserter(tmp),
                                   SL_STRUCTURE_NODE, objectId);
        }

        //assume that structure and accessories have distinct id
        opencog::cassert(TRACE_INFO, tmp.size() <= 1);

        return tmp.empty() ? Handle::UNDEFINED : tmp.front();
    }
}

Handle AtomSpaceUtil::getAgentHandle( const AtomSpace& atomSpace,
                                      const std::string& agentId )
{
    Handle agentHandle = atomSpace.getHandle( SL_PET_NODE, agentId );
    if ( agentHandle != Handle::UNDEFINED ) {
        logger().log( opencog::Logger::DEBUG,
                      "AtomSpaceUtil - Agent is a pet" );
    } else {
        agentHandle = atomSpace.getHandle( SL_AVATAR_NODE, agentId );
        if ( agentHandle != Handle::UNDEFINED ) {
            logger().log( opencog::Logger::DEBUG,
                          "AtomSpaceUtil - Agent is an avatar" );
        } else {
            agentHandle = atomSpace.getHandle( SL_HUMANOID_NODE, agentId );
            if ( agentHandle != Handle::UNDEFINED ) {
                logger().log( opencog::Logger::DEBUG,
                              "AtomSpaceUtil - Agent is an humanoid" );
            } // if
        } // if
    } // if
    return agentHandle;
}

Temporal AtomSpaceUtil::getTemporal(AtomSpace& as, Handle atTimeLink)
{
    cassert(TRACE_INFO, atTimeLink != Handle::UNDEFINED,
            "No HandleTemporalPair correspond to Handle::UNDEFINED");
    cassert(TRACE_INFO, as.getType(atTimeLink) == AT_TIME_LINK,
            "The Atom %s must be an atTimeLink",
            TLB::getAtom(atTimeLink)->toString().c_str());
    Handle timeNode = as.getOutgoing(atTimeLink, 0);
    cassert(TRACE_INFO, as.getType(timeNode) == TIME_NODE,
            "The Atom %s must be a TimeNode",
            TLB::getAtom(timeNode)->toString().c_str());

    return Temporal::getFromTimeNodeName(as.getName(timeNode).c_str());
}

Handle AtomSpaceUtil::getTimedHandle(AtomSpace& as, Handle atTimeLink)
{
    cassert(TRACE_INFO, atTimeLink != Handle::UNDEFINED,
            "No HandleTemporalPair correspond to Handle::UNDEFINED");
    cassert(TRACE_INFO, as.getType(atTimeLink) == AT_TIME_LINK,
            "The Atom %s must be an atTimeLink", TLB::getAtom(atTimeLink)->toString().c_str());

    return as.getOutgoing(atTimeLink, 1);
}

// Initialize static variables that stores the LatestLinks for each type of information
std::map<Handle, Handle> AtomSpaceUtil::latestAgentActionDone;
std::map<Handle, Handle> AtomSpaceUtil::latestPhysiologicalFeeling;
std::map<Handle, Handle> AtomSpaceUtil::latestAvatarSayActionDone;
std::map<Handle, Handle> AtomSpaceUtil::latestAvatarActionDone;
std::map<Handle, Handle> AtomSpaceUtil::latestPetActionPredicate;
std::map<Handle, std::map<Handle, Handle> > AtomSpaceUtil::latestSpatialPredicate;
std::map<Handle, Handle> AtomSpaceUtil::latestSchemaPredicate;
Handle AtomSpaceUtil::latestIsExemplarAvatar = Handle::UNDEFINED;

void AtomSpaceUtil::updateGenericLatestInfoMap(std::map<Handle, Handle>& infoMap,
        AtomSpace& as,
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
                               as, atTimeLink, predicateNode);
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

void AtomSpaceUtil::updateLatestPetActionPredicate(AtomSpace& as,
        Handle atTimeLink,
        Handle predicateNode)
{
    updateGenericLatestInfoMap(latestPetActionPredicate,
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

void AtomSpaceUtil::updateGenericLatestSingleInfo(Handle& latestSingleInfoHandle,
        AtomSpace& as,
        Handle atTimeLink)
{
    if (latestSingleInfoHandle != Handle::UNDEFINED) {
        as.removeAtom(latestSingleInfoHandle);
    }
    HandleSeq hs;
    hs.push_back(atTimeLink);
    latestSingleInfoHandle = addLink(as, LATEST_LINK, hs, true);
}

void AtomSpaceUtil::updateLatestIsExemplarAvatar(AtomSpace& as,
        Handle atTimeLink)
{
    updateGenericLatestSingleInfo(latestIsExemplarAvatar, as, atTimeLink);
}

