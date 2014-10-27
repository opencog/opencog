/*
 * opencog/embodiment/WorldWrapper/PAIWorldWrapper.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller, Moshe Looks
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

#include <sstream>
#include <limits>
#include <cstdio>

#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <opencog/util/RandGen.h>
#include <opencog/util/tree.h>

#include <opencog/atomspace/Node.h>
#include <opencog/spacetime/atom_types.h>

#include <opencog/spatial/3DSpaceMap/Pathfinder3D.h>
#include <opencog/spatial/AStarController.h>
#include <opencog/spatial/AStar3DController.h>
#include <opencog/spatial/TangentBug.h>

#include <opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h>
#include <opencog/embodiment/AtomSpaceExtensions/PredefinedProcedureNames.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include "PAIWorldWrapper.h"
#include "WorldWrapperUtil.h"

// The percentage bellow is related to the diagonal of the SpaceMap
#define STEP_SIZE_PERCENTAGE 2.0

#define MAIN_LOGGER_ACTION_PLAN_FAILED logger().debug("PAIWorldWrapper - No action plan has been sent");
#define MAIN_LOGGER_ACTION_PLAN_SUCCEEDED logger().debug("PAIWorldWrapper - The action plan has been successfully sent");

namespace opencog { namespace world {

using namespace AvatarCombo;
using namespace opencog::pai;

/**
 * public methods
 */

//ctor, stor
PAIWorldWrapper::PAIWorldWrapper(PAI& _pai)
        : pai(_pai), _hasPlanFailed(false) { }

PAIWorldWrapper::~PAIWorldWrapper() { }


bool PAIWorldWrapper::isPlanFinished() const
{
    return pai.isPlanFinished(planID);
}

bool PAIWorldWrapper::isPlanFailed() const
{
    return _hasPlanFailed || pai.hasPlanFailed(planID);
}

bool PAIWorldWrapper::sendSequential_and(sib_it from, sib_it to)
throw (ComboException, AssertionException, std::bad_exception)
{
    using namespace ::opencog;

    // DEBUG log
    if (logger().isDebugEnabled()) {
        string actionPlanStr = "{";
        for (sib_it sib = from; sib != to;) {
            stringstream ss;
            ss << combo_tree(sib);
            actionPlanStr += ss.str();
            ++sib;
            if (sib != to)
                actionPlanStr += ", ";
        }
        actionPlanStr += "}";
        logger().debug("PAIWorldWrapper - Attempt to send the following "
                       "sequence of actions: %s",
                       actionPlanStr.c_str());
    }
    // ~DEBUG log

    _hasPlanFailed = false;
    planID = pai.createActionPlan();

    AtomSpace& as = pai.getAtomSpace();
    const SpaceServer::SpaceMap& sm = spaceServer().getLatestMap();
    // treat the case when the action is a compound
    if (WorldWrapperUtil::is_builtin_compound_action(*from)) {
        OC_ASSERT(++sib_it(from) == to); // there is only one compound action
        pre_it it = from;
        builtin_action ba = get_builtin_action(*it);
        avatar_builtin_action_enum bae = get_enum(ba);
        switch (bae) {
      case id::goto_obj: {
            logger().debug("PAIWorldWrapper - Handling goto_obj. "
                           "# of parameters = %d",
                           it.number_of_children() );
            OC_ASSERT(it.number_of_children() == 2);
            OC_ASSERT(is_definite_object(*it.begin()));
            OC_ASSERT(is_contin(*++it.begin()));

            std::string target = get_definite_object( *it.begin() );
            float walkSpeed = get_contin( *++it.begin() );
            logger().debug("PAIWorldWrapper - goto_obj(%s, %f)",
                    target.c_str(), walkSpeed );
          /*  if ( target == "custom_path" ) {
                //printf("PAIWorldWrapper - target : custom_path\n");
                std::string customWaypoints = pai.getAvatarInterface().
                    getCurrentModeHandler().getPropertyValue( "customPath" );

                std::vector<spatial::Point> actionPlan;
                std::vector<std::string> strWayPoints;
                boost::algorithm::split( strWayPoints, customWaypoints, boost::algorithm::is_any_of(";") );

                spatial::Point wayPoint;
                unsigned int i;
                for ( i = 0; i < strWayPoints.size( ); ++i ) {
                    std::istringstream parser( strWayPoints[i] );
                    parser >> wayPoint.first;
                    parser >> wayPoint.second;

                    if ( i > 0 ) {
                        // TODO: use the pathfinding algorithm to go to this wayPoint
                        actionPlan.push_back( wayPoint );
                    } else {
                        std::istringstream nextParser( strWayPoints[i+1] );
                        spatial::Point nextWaypoint;
                        nextParser >> nextWaypoint.first;
                        nextParser >> nextWaypoint.second;

                        getWaypoints( wayPoint, nextWaypoint, actionPlan );
                        ++i;
                    } // else
                } // for

                // register seeking object
                pai.getAvatarInterface().setLatestGotoTarget( std::pair<std::string, spatial::Point>( target, wayPoint ) );

                if ( _hasPlanFailed || !createWalkPlanAction( actionPlan, false, Handle::UNDEFINED, walkSpeed ) ) {
                    if (_hasPlanFailed) {
                        logger().error("PAIWorldWrapper - Failed to create a goto plan to the goal.");
                    } else {
                        logger().info(
                                     "PAIWorldWrapper - No goto plan needed since the goal was already near enough.");
                    } // else
                } // if

                std::vector<std::string> arguments;
                pai.getAvatarInterface( ).getCurrentModeHandler( ).handleCommand( "followCustomPathDone", arguments );
            } else { */
                if ( target == "custom_object" ) {
                    target = pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
                } // if
                Handle targetHandle = toHandle( target );

                if (sm.containsObject(targetHandle))
                {
                    if (!build_goto_plan(targetHandle, Handle::UNDEFINED, walkSpeed ))
                    {
                        if (_hasPlanFailed) {
                            logger().error("PAIWorldWrapper - Failed to create a goto plan to the goal.");
                            throw ComboException(TRACE_INFO, "PAIWorldWrapper - %s.", "Failed to create a goto plan to the goal");
                        } else {
                            logger().info("PAIWorldWrapper - No goto plan needed since the goal was already near enough.");
                            std::vector<std::string> arguments;
                            pai.getAvatarInterface( ).getCurrentModeHandler( ).handleCommand( "gotoDone", arguments );

                        } // else
                        MAIN_LOGGER_ACTION_PLAN_FAILED;
                        return false;
                    }

                }
                else
                {
                    logger().error("PAIWorldWrapper - Goto: target not found.");
                    _hasPlanFailed = true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED;
                    return false;
                }

                std::vector<std::string> arguments;
                pai.getAvatarInterface( ).getCurrentModeHandler( ).handleCommand( "gotoDone", arguments );
          //  } // else

        }
        break;

        case id::gonear_obj: {
            try {
                OC_ASSERT(it.number_of_children() == 2);
                OC_ASSERT(is_definite_object(*it.begin()));
                OC_ASSERT(is_contin(*++it.begin()));

                std::string target = get_definite_object( *it.begin() );
                float walkSpeed = get_contin( *++it.begin() );

                if ( target == "custom_object" ) {
                    target = pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
                } // if

                if ( !build_goto_plan(toHandle(target), Handle::UNDEFINED, walkSpeed ) ) {
                    if ( _hasPlanFailed ) {
                        logger().error("PAIWorldWrapper - Failed to create a goto plan to the goal.");
                    } else {
                        logger().info("PAIWorldWrapper - No goto plan needed "
                                "since the goal was already near enough.");

                    } // else
                    MAIN_LOGGER_ACTION_PLAN_FAILED;
                    return false;
                } // if
            } catch ( NotFoundException& ex ) {
                // Pet will stay at it's current position until it's owner
                // isn't at a valid position
                logger().error( "PAIWorldWrapper - Goto: target not found.");
                _hasPlanFailed = true;
                MAIN_LOGGER_ACTION_PLAN_FAILED;
                return false;
            } // catch

        }
        break;

        case id::gobehind_obj: {

            OC_ASSERT(it.number_of_children() == 2);
            OC_ASSERT(is_definite_object(*it.begin()));
            OC_ASSERT(is_contin(*++it.begin()));

            std::string target = get_definite_object( *it.begin() );
            float walkSpeed = get_contin( *++it.begin() );

            if ( target == "custom_object" ) {
                target = pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
            } // if

            if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(), target)) {
                //can't find the target
                logger().error("PAIWorldWrapper - gobehind_obj: target[%s] not found.",
                             target.c_str() );
                _hasPlanFailed = true;
                MAIN_LOGGER_ACTION_PLAN_FAILED;
                return false;
            } // if

            if ( !build_goto_plan(toHandle(target), toHandle(target), walkSpeed )) {
                if (_hasPlanFailed) {
                    logger().error(
                                 "PAIWorldWrapper - Failed to create a gobehind_obj plan to the goal.");
                } else {
                    logger().info(
                                 "PAIWorldWrapper - No gobehind_obj plan needed since the goal was already near enough.");
                } // if
                MAIN_LOGGER_ACTION_PLAN_FAILED;
                return false;
            } // if

        }
        break;

        case id::go_behind: {
            OC_ASSERT(it.number_of_children() == 3);
            OC_ASSERT(is_definite_object(*it.begin()));
            OC_ASSERT(is_definite_object(*++it.begin()));
            OC_ASSERT(is_contin(*++(++it.begin())));

            //and just goto it, if we can find the args on the map
            if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                              *it.begin()) ||
                    !WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                  *++it.begin())) {
                logger().error(
                             "PAIWorldWrapper - Go behind: args not found.");

                _hasPlanFailed = true;
                MAIN_LOGGER_ACTION_PLAN_FAILED;
                return false;
            }
            float walkSpeed = get_contin( *++(++it.begin()) );

            if (!build_goto_plan(toHandle(get_definite_object(*it.begin())),
                                 toHandle(get_definite_object(*++it.begin()))), walkSpeed ) {
                MAIN_LOGGER_ACTION_PLAN_FAILED;
                return false;
            }
        }
        break;
        case id::follow: {
            OC_ASSERT(it.number_of_children() == 3);
            OC_ASSERT(is_definite_object(*it.begin()));
            OC_ASSERT(is_contin(*++it.begin()));
            OC_ASSERT(is_contin(*++(++it.begin())));
            {
                Handle obj = toHandle(get_definite_object(*it.begin()));
                OC_ASSERT(obj != Handle::UNDEFINED);
                //first goto the obj to follow, if we can find it
                if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                  *it.begin())) {
                    logger().error(
                                 "PAIWorldWrapper - Follow: obj not found.");

                    _hasPlanFailed = true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED;
                    return false;
                }
                float walkSpeed = get_contin( *++it.begin() );
                float duration = get_contin( *++(++it.begin( ) ) );

                if ( duration == 0 ) {
                    duration = pai.getAvatarInterface( ).computeFollowingDuration( );
                } // if

                if (!build_goto_plan(obj,  Handle::UNDEFINED, walkSpeed ) )
                    planID = pai.createActionPlan();
                //then add a follow action at the end
                AvatarAction action(ActionType::FOLLOW());
                action.addParameter(ActionParameter("id",
                                                    ActionParamType::ENTITY(),
                                                    Entity(get_definite_object(*it.begin()),
                                                           resolveType(*it.begin()))));
                action.addParameter(ActionParameter("duration", ActionParamType::FLOAT(), lexical_cast<string>(duration)) );
                pai.addAction(planID, action);
            }
        }
        break;
        default:
            std::stringstream stream (std::stringstream::out);
            stream << "Unrecognized compound schema '"  << combo_tree(it) << "'" << std::endl;
            throw ComboException(TRACE_INFO, "PAIWorldWrapper - %s.", stream.str().c_str());
        }
        //try { //TODO
        if (pai.isActionPlanEmpty(planID)) {
            _hasPlanFailed = true;
            MAIN_LOGGER_ACTION_PLAN_FAILED;
            return false;
        } else {
            if(!config().get_bool("EXTRACTED_ACTION_MODE")) {
                pai.sendActionPlan(planID);
            } else {
                pai.sendExtractedActionFromPlan(planID);
            }
            MAIN_LOGGER_ACTION_PLAN_SUCCEEDED;
            return true;
        }
        //} catch (...) {
        //throw ActionPlanSendingFailure(planID);
        //}

    } else { //non-compound action sequence
        while (from != to) {
            try {
                AvatarAction action = buildAvatarAction(from);
                pai.addAction(planID, action);
                ++from;
            } catch( const StandardException& ex ) {
                std::stringstream ss (stringstream::out);
                ss << combo_tree(from);
                ss << " " << const_cast<StandardException*>( &ex )->getMessage( );
                logger().error( "PAIWorldWrapper::%s - Failed to build AvatarAction '%s'",
                                __FUNCTION__, ss.str().c_str( )  );
                from = to;
            } catch (...) {
                std::stringstream ss (stringstream::out);
                ss << combo_tree(from);
                logger().error("PAIWorldWrapper - Failed to build AvatarAction '%s'.",
                             ss.str().c_str());

                // so no more actions are built for that action plan, as
                // according to and_seq semantics
                from = to;
            }
        }
        if (pai.isActionPlanEmpty(planID)) {
            _hasPlanFailed = true;
            MAIN_LOGGER_ACTION_PLAN_FAILED;
            return false;
        } else {
            if(!config().get_bool("ENABLE_UNITY_CONNECTOR")) {
                pai.sendActionPlan(planID);
            } else {
                pai.sendExtractedActionFromPlan(planID);
            }
            MAIN_LOGGER_ACTION_PLAN_SUCCEEDED;
            return true;
        }
    }
}

combo::vertex PAIWorldWrapper::evalPerception(pre_it it, combo::variable_unifier& vu)
{
    Handle smh = spaceServer().getLatestMapHandle();
    unsigned int current_time = pai.getLatestSimWorldTimestamp();
    OC_ASSERT(smh != Handle::UNDEFINED, "A SpaceMap must exists");
    combo::vertex v = WorldWrapperUtil::evalPerception(smh, current_time,
                      pai.getAtomSpace(),
                      selfName(), ownerName(),
                      it, false, vu);

    //DEBUG log
    if (logger().isDebugEnabled()) {
        stringstream p_ss, r_ss;
        p_ss << combo_tree(it);
        r_ss << v;
        logger().debug("PAIWorldWrapper - Perception %s has been evaluation to %s",
                     p_ss.str().c_str(), r_ss.str().c_str());
    }
    //~DEBUG log

    return v;
}

combo::vertex PAIWorldWrapper::evalIndefiniteObject(indefinite_object io,
        combo::variable_unifier& vu)
{
    Handle smh = spaceServer().getLatestMapHandle();
    unsigned int current_time = pai.getLatestSimWorldTimestamp();
    OC_ASSERT(smh != Handle::UNDEFINED, "A SpaceMap must exists");

    combo::vertex v = WorldWrapperUtil::evalIndefiniteObject(smh, current_time,
                      pai.getAtomSpace(),
                      selfName(), ownerName(),
                      io, false, vu);

    //DEBUG log
    if (logger().isDebugEnabled()) {
        stringstream io_ss, r_ss;
        io_ss << io;
        r_ss << v;
        logger().debug("PAIWorldWrapper - Indefinition object %s has been evaluation to %s",
                     io_ss.str().c_str(), r_ss.str().c_str());
    }
    //~DEBUG log

    return v;
}


/**
 * private methods
 */
/*
void PAIWorldWrapper::clearPlan( std::vector<spatial::Point>& actions,
                                 const spatial::Point& startPoint,
                                 const spatial::Point& endPoint )
{
    const SpaceServer::SpaceMap& sm = pai.getAtomSpace().getSpaceServer().getLatestMap();

    double closestDist = SpaceServer::SpaceMap::eucDist( startPoint,
                         endPoint );
    std::vector<spatial::Point>::iterator eraseFrom = actions.begin( );
    std::vector<spatial::Point>::iterator it;

    for ( it = actions.begin(); it != actions.end(); ++it ) {
        double d = SpaceServer::SpaceMap::eucDist(*it, endPoint);
        if (d < closestDist) {
        eraseFrom = boost::next(it);

            // at some point the following "take the nearest point" heuristic
            // can be made more subtle by adding a weight to the length of the
            // path - e.g. if the path generated by tb is (x1,x2,...,xN) and
            // N is large, and the point xN is only slightly closer to the goal
            // than x2, then maybe we want to just go to x2 and not all the way
            // to xN
            if (d <= sm.radius() * 1.1) break;
            closestDist = d;
        } // if
    } // for
    actions.erase( eraseFrom, actions.end( ) );
}
*/
/*
spatial::Point PAIWorldWrapper::getValidPosition( const spatial::Point& location )
{
    const SpaceServer::SpaceMap& spaceMap = pai.getAtomSpace().getSpaceServer().getLatestMap();

    //make sure that the object location is valid
    spatial::Point correctedLocation = location;
    if ( spaceMap.illegal( location ) ) {
        logger().warn("PAIWorldWrapper - Position (%.2f, %.2f) is invalid "
                "(off the grid, near/inside an obstacle).",
                location.first, location.second);

        correctedLocation = spaceMap.getNearestFreePoint( location );

        logger().warn("PAIWorldWrapper - Changed position to the nearest : "
                "valid point (%.2f, %.2f).",
                correctedLocation.first, correctedLocation.second);
    }

    return correctedLocation;
}

void PAIWorldWrapper::getWaypoints( const spatial::Point& startPoint,
        const spatial::Point& endPoint, std::vector<spatial::Point>& actions )
{
    const std::string pathFindingAlgorithm =
        config().get("NAVIGATION_ALGORITHM");

    const SpaceServer::SpaceMap& sm = pai.getAtomSpace().getSpaceServer().getLatestMap();

    try {
        spatial::Point begin = startPoint;
        spatial::Point end = endPoint;

        spatial::Point correctedAgentLocation = getValidPosition( begin );
        spatial::Point correctedEndLocation = getValidPosition( end );

        if ( correctedAgentLocation != begin ) {
            begin = correctedAgentLocation;
            actions.push_back( begin );
        }

        if ( correctedEndLocation != end ) {
            end = correctedEndLocation;
        }

        if ( pathFindingAlgorithm == "astar") {
            spatial::LSMap2DSearchNode petNode = sm.snap(spatial::Point(begin.first, begin.second));
            spatial::LSMap2DSearchNode goalNode = sm.snap(spatial::Point(end.first, end.second));

            spatial::AStarController AStar;
            SpaceServer::SpaceMap *map = const_cast<SpaceServer::SpaceMap*>(&sm);
            AStar.setMap( map );
            AStar.setStartAndGoalStates(petNode, goalNode);

            //finally, run AStar
            _hasPlanFailed = (AStar.findPath() != spatial::AStarSearch<spatial::LSMap2DSearchNode>::SEARCH_STATE_SUCCEEDED);
            actions = AStar.getShortestCalculatedPath( );

            logger().debug("PAIWorldWrapper - AStar result %s.",
                    !_hasPlanFailed ? "true" : "false");
        } else if ( pathFindingAlgorithm == "hpa" ) {

            SpaceServer::SpaceMap *map = const_cast<SpaceServer::SpaceMap*>(&sm);
            unsigned int maximumClusters = config().get_int("HPA_MAXIMUM_CLUSTERS");
            spatial::HPASearch search( map, 1, maximumClusters );

            _hasPlanFailed = !search.processPath( spatial::math::Vector2( begin.first, begin.second ), spatial::math::Vector2( end.first, end.second ) );
            std::vector<spatial::math::Vector2> pathPoints = search.getProcessedPath( 1 );
            if ( !_hasPlanFailed ) {
                foreach( spatial::math::Vector2 pathPoint, pathPoints ) {
                    actions.push_back( spatial::Point( pathPoint.x, pathPoint.y ) );
                }
            }

            logger().debug("PAIWorldWrapper - HPASearch result %s.",
                    !_hasPlanFailed ? "true" : "false");

        } else {
            spatial::TangentBug::CalculatedPath calculatedPath;
            spatial::TangentBug tb(sm, calculatedPath, rng);

            //place the pet and the goal on the map
            tb.place_pet(begin.first, begin.second);
            tb.place_goal(end.first, end.second);

            //finally, run tangent bug
            _hasPlanFailed = !tb.seek_goal();
            if ( !_hasPlanFailed ) {
                spatial::TangentBug::CalculatedPath::iterator it;
                for ( it = calculatedPath.begin( ); it != calculatedPath.end( ); ++it ) {
                    actions.push_back( boost::get<0>( *it ) );
                }
            }
            logger().debug("PAIWorldWrapper - TangetBug result %s.",
                         !_hasPlanFailed ? "true" : "false");
        } // else
    } catch ( opencog::RuntimeException& e ) {
        _hasPlanFailed = true;
    } catch ( AssertionException& e) {
        _hasPlanFailed = true;
    }
}
*/


//void PAIWorldWrapper::get3DWaypoints( const SpaceServer::SpaceMapPoint& startPoint,
//        const SpaceServer::SpaceMapPoint& endPoint, std::vector<SpaceServer::SpaceMapPoint>& actions,SpaceServer::SpaceMap& sm )
//{
//    SpaceServer::SpaceMapPoint nearestPos;
//    if (spatial::Pathfinder3D::AStar3DPathFinder(&sm,startPoint,endPoint,actions,nearestPos))
//    {
//        printf("Pathfinding successfully! From (%d,%d,%d) to (%d, %d, %d)",
//               startPoint.x,startPoint.y,startPoint.z,endPoint.x, endPoint.y,endPoint.z);
//    }
//    else
//    {
//        printf("Pathfinding failed! From (%d,%d,%d) to (%d, %d, %d), the nearest accessable postion is (%d,%d,%d)",
//               startPoint.x,startPoint.y,startPoint.z,endPoint.x, endPoint.y,endPoint.z,nearestPos.x,nearestPos.y,nearestPos.z);
//    }

//}


/*
bool PAIWorldWrapper::createWalkPlanAction( std::vector<spatial::Point>& actions, bool useExistingId, Handle toNudge, float customSpeed )
{

    if ( actions.empty( ) ) {
        // we're done. No need to create any walk sequency
        logger().debug("PAIWorldWrapper - Zero actions from AStar.");
        return false;
    }

    // --------------------------------------------------------------------
    // transform to a sequence of walk commands
    // --------------------------------------------------------------------

    if (!useExistingId ) {
        planID = pai.createActionPlan( );
    } // if

    foreach(const spatial::Point& it_action, actions ) {
        AvatarAction action;

        if (toNudge != Handle::UNDEFINED) {
            action = AvatarAction(ActionType::NUDGE_TO());
            action.addParameter(ActionParameter("moveableObj",
                                                ActionParamType::ENTITY(),
                                                Entity(pai.getAtomSpace().getName(toNudge),
                                                       resolveType(toNudge))));
            action.addParameter(ActionParameter("target",
                                                ActionParamType::VECTOR(),
                                                Vector(it_action.first,
                                                       it_action.second,
                                                       0.0)));
        } else {
            action = AvatarAction(ActionType::WALK());
            action.addParameter(ActionParameter("target",
                                                ActionParamType::VECTOR(),
                                                Vector(it_action.first,
                                                       it_action.second,
                                                       0.0)));

            float speed = ( customSpeed != 0 ) ?
                    customSpeed : pai.getAvatarInterface().computeWalkingSpeed();
            logger().debug("PAIWorldWrapper::createWalkPlanAction customSpeed[%f] finalSpeed[%f]",
                    customSpeed, speed );
            action.addParameter(ActionParameter("speed", ActionParamType::FLOAT(),
                    lexical_cast<string>( speed) ) );

        } // else
        pai.addAction( planID, action );
    } // foreach

    return true;
}
*/
bool PAIWorldWrapper::createNavigationPlanAction( opencog::pai::PAI& pai,SpaceServer::SpaceMap& sm,const SpaceServer::SpaceMapPoint& startPoint,
                                                  const SpaceServer::SpaceMapPoint& endPoint, opencog::pai::ActionPlanID _planID, bool includingLastStep, float customSpeed )
{
    std::vector<SpaceServer::SpaceMapPoint> actions;

    SpaceServer::SpaceMapPoint nearestPos,bestPos;
    if (spatial::Pathfinder3D::AStar3DPathFinder(&sm,startPoint,endPoint,actions,nearestPos,bestPos,false,false,true))
    {
        printf("Pathfinding successfully! From (%d,%d,%d) to (%d, %d, %d)",
               startPoint.x,startPoint.y,startPoint.z,endPoint.x, endPoint.y,endPoint.z);
    }
    else
    {
        printf("Pathfinding failed! From (%d,%d,%d) to (%d, %d, %d), the nearest accessable postion is (%d,%d,%d)",
               startPoint.x,startPoint.y,startPoint.z,endPoint.x, endPoint.y,endPoint.z,nearestPos.x,nearestPos.y,nearestPos.z);
    }

    // the first pos in actions vector is the begin pos, so there should be at least 2 elements in this vector
    if ( actions.size() < 2 ) {
        // we're done. No need to create any navigation actions
        logger().debug("PAIWorldWrapper - Zero actions from AStar3D.");
        return false;
    }

    // --------------------------------------------------------------------
    // transform to a sequence of navigation commands
    // --------------------------------------------------------------------

    if (_planID == "" ) {
        _planID = pai.createActionPlan( );
    } // if
    vector<SpaceServer::SpaceMapPoint>::iterator it_point,endPointIter;
    it_point = actions.begin();
    it_point ++;

    // get the endPoint
    endPointIter = actions.end();
    endPointIter --;

    while (it_point != actions.end()) {
        AvatarAction action;

        // Now in Unity, we consider jump/climb up one block as normal walking action:

        // The agent need to jump when this pos is higher than last pos
        if (((SpaceServer::SpaceMapPoint)(*(it_point))).z > ((SpaceServer::SpaceMapPoint)(*(it_point-1))).z )
        {
            action = AvatarAction(ActionType::JUMP_TOWARD());
            action.addParameter(ActionParameter("direction",
                                                ActionParamType::VECTOR(),
                                                Vector(((SpaceServer::SpaceMapPoint)(*it_point)).x,
                                                       ((SpaceServer::SpaceMapPoint)(*it_point)).y,
                                                       ((SpaceServer::SpaceMapPoint)(*it_point)).z )));
        }
        else
        {
            action = AvatarAction(ActionType::WALK());
            action.addParameter(ActionParameter("target",
                                                ActionParamType::VECTOR(),
                                                Vector(((SpaceServer::SpaceMapPoint)(*it_point)).x,
                                                       ((SpaceServer::SpaceMapPoint)(*it_point)).y,
                                                       ((SpaceServer::SpaceMapPoint)(*it_point)).z )));

            float speed = ( customSpeed != 0 ) ?
                    customSpeed : pai.getAvatarInterface().computeWalkingSpeed();
            logger().debug("PAIWorldWrapper::createNavigationPlanAction customSpeed[%f] finalSpeed[%f]",
                    customSpeed, speed );
            action.addParameter(ActionParameter("speed", ActionParamType::FLOAT(),
                    lexical_cast<string>( speed) ) );

        }
        pai.addAction( _planID, action );
        it_point++;

        if ((!includingLastStep) && (it_point == endPointIter))
            break;
    } // while

    return true;
}

bool PAIWorldWrapper::build_goto_plan(Handle goalHandle,
                                      Handle goBehind, float walkSpeed )
{
    AtomSpace& atomSpace = pai.getAtomSpace();
    const SpaceServer::SpaceMap& spaceMap = spaceServer().getLatestMap();
    std::string goalName = atomSpace.getName(goalHandle);

    OC_ASSERT(goalHandle != Handle::UNDEFINED);
    OC_ASSERT(spaceMap.containsObject(goalHandle));

    SpaceServer::SpaceMapPoint startPoint;
    SpaceServer::SpaceMapPoint endPoint;
    SpaceServer::SpaceMapPoint targetCenterPosition;
    SpaceServer::SpaceMapPoint direction;

//    double targetAltitude = 0.0;

    startPoint = WorldWrapperUtil::getLocation(spaceMap, atomSpace,
                 WorldWrapperUtil::selfHandle(atomSpace, selfName()));

    if ( goBehind != Handle::UNDEFINED )
    {
        targetCenterPosition = spaceMap.getObjectLocation(goBehind);
        direction = spaceMap.getObjectDirection(goBehind);

        // because it's to go behind, just get the opposite direction;
        direction.x *= -1;
        direction.y *= -1;
    }
    else
    {
        targetCenterPosition = spaceMap.getObjectLocation(goalHandle);
        direction = spaceMap.getObjectDirection(goalHandle);
    }

    if (targetCenterPosition != spatial::BlockVector::ZERO)
    {
        endPoint = spaceMap.getNearFreePointAtDistance(targetCenterPosition, SpaceServer::SpaceMap::AccessDistance, direction );
    }else
    {
        endPoint = spatial::BlockVector::ZERO;
    }

    if (endPoint == spatial::BlockVector::ZERO)
    {
        logger().error("PAIWorldWrapper - Unable to get pet or goal location.");
        _hasPlanFailed = true;
        return false;
    }

    logger().fine("PAIWorldWrapper - Pet position: (%d, %d, %d). "
            "Goal position: (%d, %d, %d) - %s.",
            startPoint.x, startPoint.y,  startPoint.z, endPoint.x,
            endPoint.y, endPoint.z, goalName.c_str());
    // register seeking object
    pai.getAvatarInterface().setLatestGotoTarget(
        std::pair<std::string, SpaceServer::SpaceMapPoint>( goalName, targetCenterPosition ) );

    if (config().get_bool("ENABLE_UNITY_CONNECTOR"))
    {
        std::vector<SpaceServer::SpaceMapPoint> actions;


        float speed = ( walkSpeed != 0 ) ?
                walkSpeed : pai.getAvatarInterface().computeWalkingSpeed();

        return createNavigationPlanAction(  pai, (SpaceServer::SpaceMap&)spaceMap,startPoint,
                                           endPoint, planID, speed );

    }

    return false;
}

AvatarAction PAIWorldWrapper::buildAvatarAction(sib_it from)
{
    AtomSpace& as = pai.getAtomSpace();
    const SpaceServer::SpaceMap& sm = spaceServer().getLatestMap();
    static const std::map<avatar_builtin_action_enum, ActionType> actions2types =
        { {id::drink, ActionType::DRINK()},
          {id::eat, ActionType::EAT()},
          {id::grab, ActionType::GRAB()},
          {id::jump_up, ActionType::JUMP_UP()},
          {id::jump_towards, ActionType::JUMP_TOWARD()},
          {id::jump_forward, ActionType::JUMP_FORWARD()},
          {id::move_head, ActionType::MOVE_HEAD()},
          {id::random_step, ActionType::WALK()},
          {id::rotate, ActionType::ROTATE()},
          {id::step_backward, ActionType::WALK()},
          {id::step_towards, ActionType::WALK()},
          {id::turn_to_face, ActionType::TURN()},
          {id::kick, ActionType::KICK()},
          {id::sit, ActionType::SIT()},
          {id::look_at, ActionType::LOOK_AT()},
          {id::say, ActionType::SAY()},
          {id::build_block, ActionType::BUILD_BLOCK()},
          {id::destroy_block, ActionType::DESTROY_BLOCK()},

          // For Santa Fe Trail problem
          {id::step_forward, ActionType::STEP_FORWARD()},
          {id::rotate_right, ActionType::ROTATE_RIGHT()},
          {id::rotate_left, ActionType::ROTATE_LEFT()},

        };

    OC_ASSERT(WorldWrapperUtil::is_builtin_atomic_action(*from));
    builtin_action ba = get_builtin_action(*from);
    avatar_builtin_action_enum bae = get_enum(ba);

    stringstream ss;
    ss << *from;

    logger().debug("PAIWorldWrapper::%s - Trying to build avatar action '%s' for builtin_action_enum %d'",
                   __FUNCTION__, ss.str().c_str( ), bae);

    /****
         this switch statement deals with special cases. Also,
         commands like scratch needs to have the body-part codes
         translated

         the full list of such schema is:

         drink(drinkable_obj)
         eat(edible_obj)
         grab(pickupable_obj)
         jump_towards(obj)
         jump_forward(height)
         move_head(angle, angle)
         random_step
         rotate(angle)
         step_backward
         step_towards(obj,TOWARDS|AWAY)
         turn_to_face(obj)
    ****/
    auto at_it = actions2types.find(bae);
    {
        stringstream ss;
        ss << bae;
        OC_ASSERT(at_it != actions2types.end(),
                  "PAIWorldWrapper - No action type corresponding to %s was found "
                  "in actions2types", ss.str().c_str());
    }
    AvatarAction action = at_it->second;

    double theta = 0;
    switch (bae) {
        //we're going to do all of the "do x to id y" commands together
    case id::drink:             // drink(drinkable_obj)
    case id::eat:               // eat(edible_obj)
    case id::grab:              // grab(pickupable_obj)
    case id::kick:              // kick(obj)
        action.addParameter(ActionParameter("target",
                                            ActionParamType::ENTITY(),
                                            Entity(get_definite_object(*from.begin()),
                                                   resolveType(*from.begin()))));
        break;

    case id::look_at: {
        if ( from.number_of_children( ) != 1 ) {
            throw InvalidParamException(TRACE_INFO,
                                                 "PAIWorldWrapper - Invalid number of arguments for look_at %d", from.number_of_children( )  );
        } // if

        sib_it arguments = from.begin( );

        // get target name. can be an agent name
        std::string targetName;
        if ( is_indefinite_object( *arguments ) ) {
            combo::variable_unifier vu;
            targetName = get_definite_object( evalIndefiniteObject( get_indefinite_object( *arguments ), vu ) );
        } else {
            targetName = get_definite_object( *arguments );
        } // else

        if ( targetName == "custom_object" ) {
            targetName = pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
        } // else

        action.addParameter( ActionParameter( "target", ActionParamType::ENTITY( ),
                                              Entity( targetName, resolveType(targetName) ) ) );
    }
    break;

    case id::say: {
        if ( from.number_of_children( ) != 2 ) {
            throw InvalidParamException(TRACE_INFO,
                                                 "PAIWorldWrapper - Invalid number of arguments for say %d", from.number_of_children( )  );
        } // if
        sib_it arguments = from.begin( );

        std::string message = "";
        if ( !is_definite_object(*arguments) ) {
            logger().error( "PAIWorldWrapper::%s - The first 'say' argument must be a definite_object which defines the string message" );
        } else {
            message = get_definite_object(*arguments);
        } // else

        if ( message == "custom_message" ) {
            message = pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customMessage" );
        } // if

        // once the say action was executed set the has_something_to_say predicate to false
        // to avoid repetitions
        AtomSpaceUtil::setPredicateValue( as, "has_something_to_say", TruthValue::FALSE_TV( ),
                                          AtomSpaceUtil::getAgentHandle( as, pai.getAvatarInterface( ).getPetId( ) ) );

        action.addParameter( ActionParameter( "message", ActionParamType::STRING( ), message ) );

        ++arguments;

        std::string targetName;
        if ( is_indefinite_object( *arguments ) ) {
            combo::variable_unifier vu;
            targetName = get_definite_object( evalIndefiniteObject( get_indefinite_object( *arguments ), vu ) );
        } else {
            targetName = get_definite_object( *arguments );
        } // else

        if ( targetName == "custom_object" ) {
            targetName = pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
        } // else

        action.addParameter( ActionParameter( "target", ActionParamType::STRING( ), targetName ) );
    }
    break;

        //now rotations
    case id::rotate:
    {
        std::stringstream ss;
        ss << *from.begin();

        action.addParameter(ActionParameter("angle",
                                            ActionParamType::FLOAT(),
                                            ss.str()));
    }
    break;
        //now stepping actions
    case id::random_step:        // random_step
        theta = 2.0 * PI * randGen().randdouble();
        goto build_step;
    case id::step_backward :     // step_backward
        theta = getAngleFacing(WorldWrapperUtil::selfHandle(as,
                               selfName())) + PI;
        if (theta > 2*PI)
            theta -= 2 * PI;
        goto build_step;

    build_step:
        //now compute a step in which direction the pet is going
        {
            SpaceServer::SpaceMapPoint petLoc = WorldWrapperUtil::getLocation(sm, as, WorldWrapperUtil::selfHandle(as, selfName()));

            //double stepSize = (sm.diagonalSize()) * STEP_SIZE_PERCENTAGE / 100; // 2% of the width
            double stepSize = 1.0; // The unit length in a block world.
            double x = (double)petLoc.x + (cos(theta) * stepSize);
            double y = (double)petLoc.y + (sin(theta) * stepSize);

            // if not moving to an illegal position, then no problem, go
            // to computer position
            SpaceServer::SpaceMapPoint pos((int)x, (int)y,petLoc.z);
            if (!sm.checkStandable(pos)) {
                action.addParameter(ActionParameter("target",
                                                    ActionParamType::VECTOR(),
                                                    Vector(x, y, 0.0)));

                // if illegal position, stay in the same place (a walk to
                // the current position
            } else {
                action.addParameter(ActionParameter("target",
                                                    ActionParamType::VECTOR(),
                                                    Vector((double)petLoc.x,
                                                           (double)petLoc.y,
                                                           0.0)));
            }
            action.addParameter(ActionParameter("speed",
                                                ActionParamType::FLOAT(),
                                                lexical_cast<string>(pai.getAvatarInterface().computeWalkingSpeed())));
        }
        break;

        //now a bazillion special cases
    case id::jump_towards:      // jump_towards(obj)
        //we need to compute the obj's position and pass it as and arg to
        //tristan's jumpToward(vector) function
    {
        SpaceServer::SpaceMapPoint p = WorldWrapperUtil::getLocation(sm, as, toHandle(get_definite_object(*from.begin())));
        action.addParameter(ActionParameter("position",
                                            ActionParamType::VECTOR(),
                                            Vector(p.x, p.y, p.z)));
    }
    break;
    case id::jump_forward:      // jump_forward(height)
    {
        std::stringstream ss;
        ss << *from.begin();

        action.addParameter(ActionParameter("height",
                                            ActionParamType::FLOAT(),
                                            ss.str()));
    }
    break;

 /*
    case id::move_head:         // move_head(angle, angle)
        OC_ASSERT(from.number_of_children() == 3);
        action.addParameter(ActionParameter("position",
                                            ActionParamType::VECTOR(),
                                            Vector(0.0, 0.0, 0.0)));
        action.addParameter(ActionParameter("rotation",
                                            ActionParamType::ROTATION(),
                                            Rotation(get_contin(*from.begin()),
                                                     get_contin(*++from.begin()),
                                                     get_contin(*from.last_child()))));
        action.addParameter(ActionParameter("speed",
                                            ActionParamType::FLOAT(),
                                            "1.0"));
        break;
*/
    case id::turn_to_face: {    // turn_to_face(obj)

        OC_ASSERT(from.number_of_children() == 1);
        OC_ASSERT(is_definite_object(*from.begin()));
        /*
          const string& slObjName = get_definite_object(*from.begin());
          double angleFacing = getAngleFacing(WorldWrapperUtil::selfHandle(as, selfName()));
          logger().debug("PAIWorldWrapper: angleFacing = %f", angleFacing);
          logger().debug("PAIWorldWrapper: slObjName = %s", slObjName.c_str());
          OC_ASSERT( WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(), selfName()));
          OC_ASSERT( WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(), slObjName));
          const SpaceServer::SpaceMapPoint& selfObjPoint = WorldWrapperUtil::getLocation(sm, as, toHandle(selfName()));
          const SpaceServer::SpaceMapPoint& slObjPoint =  WorldWrapperUtil::getLocation(sm, as, toHandle(slObjName));
          double deltaX = slObjPoint.first - selfObjPoint.first;
          double deltaY = slObjPoint.second - selfObjPoint.second;
          double angle = atan2f(deltaX,deltaY);
          double rotationAngle = angle - angleFacing;

          logger().debug("PAIWorldWrapper - Final angle = %f. Angle to turn: %f", angle, rotationAngle);
        */

        std::string targetObjectName = get_definite_object(*from.begin());

        float rotationAngle = 0;
        try {
            //const spatial::Object& agentObject = sm.getObject( selfName( ) );
            const spatial::Entity3D* agentEntity = sm.getEntity( toHandle(selfName( )) );
            //rotationAngle = agentObject.metaData.yaw;

            //const spatial::Object& targetObject = sm.getObject( targetObjectName );

            SpaceServer::SpaceMapPoint targetPosition;
            if ( targetObjectName == "custom_position" ) {
                std::stringstream parser(
                    pai.getAvatarInterface( ).getCurrentModeHandler( ).getPropertyValue( "customPosition" )
                );
                parser >> targetPosition.x;
                parser >> targetPosition.y;
                parser >> targetPosition.z;
            } else {
                const spatial::Entity3D* targetEntity = sm.getEntity(toHandle(targetObjectName));
                targetPosition = targetEntity->getPosition( );
            } // else

            //spatial::math::Vector2 agentPosition( agentObject.metaData.centerX, agentObject.metaData.centerY );
            //spatial::math::Vector2 targetPosition( targetObject.metaData.centerX, targetObject.metaData.centerY );

            //spatial::math::Vector2 agentDirection( cos( agentObject.metaData.yaw ), sin( agentObject.metaData.yaw ) );
            spatial::math::Vector3 targetDirection = targetPosition -  agentEntity->getPosition( );

            rotationAngle = atan2f(targetDirection.y, targetDirection.x);

           // logger().debug("PAIWorldWrapper - Agent[pos: %s, ori: %s], Target[pos: %s], Turn Angle: %f", agentEntity->getPosition( ).toString( ).c_str( ),  agentEntity->getOrientation( ).toString( ).c_str( ), targetPosition.toString( ).c_str( ), rotationAngle );

        } catch ( NotFoundException& ex ) {
            logger().debug("PAIWorldWrapper - Cannot find an object inside localspacemap: %s", ex.getMessage( ) );
        } // catch

        action.addParameter(ActionParameter("rotation", ActionParamType::ROTATION(),  Rotation(0, 0, rotationAngle)));

        break;
    }

    case id::build_block: {
        if ( from.number_of_children() > 2 ) {
            throw InvalidParamException( TRACE_INFO,
                                         "PAIWorldWrapper - Invalid number of arguments for build_block %d",
                                         from.number_of_children()
                                       );
        }

        std::stringstream ss;
        ss << *from.begin();

        action.addParameter(ActionParameter("offset",
                                            ActionParamType::FLOAT(),
                                            ss.str()));
        ss.str("");
        ss << *++from.begin();

        action.addParameter(ActionParameter("blockType",
                                            ActionParamType::STRING(),
                                            ss.str()));

    }
    break;

    case id::destroy_block: {
        std::stringstream ss;
        ss << *from.begin();

        action.addParameter(ActionParameter("offset",
                                            ActionParamType::FLOAT(),
                                            ss.str()));
    }
    break;

    default:
        //this will handle simple schema with no arguments - these are:
        /**
           shake_head
           drop
           look_up_turn_head
           jump_up
           sit
           sleep
           hide_face
           widen_eyes
           step_forward
           rotate_left
           rotate_right
        **/
        break;
    }
    return action;
}

string PAIWorldWrapper::toCamelCase(const string& str)
{
    string result = str;
    for (string::iterator it = result.begin();it != result.end() - 1;++it)
        if (*it == '_') {
            it = result.erase(it);
            *it = toupper(*it);
        }
    return result;
}

string PAIWorldWrapper::resolveType(combo::vertex v)
{
    OC_ASSERT(is_definite_object(v));
    OC_ASSERT(toHandle(get_definite_object(v)) != Handle::UNDEFINED);
    return resolveType(toHandle(get_definite_object(v)));
}
string PAIWorldWrapper::resolveType(Handle h)
{
    Type objType = pai.getAtomSpace().getType(h);
    return (objType == AVATAR_NODE ? AVATAR_OBJECT_TYPE :
            objType == PET_NODE ? PET_OBJECT_TYPE :
            objType == HUMANOID_NODE ? HUMANOID_OBJECT_TYPE :
            objType == ACCESSORY_NODE ? ACCESSORY_OBJECT_TYPE :
            objType == STRUCTURE_NODE ? STRUCTURE_OBJECT_TYPE :
            objType == OBJECT_NODE ? ORDINARY_OBJECT_TYPE :
            objType == BLOCK_ENTITY_NODE ? BLOCK_ENTITY_TYPE  :
            UNKNOWN_OBJECT_TYPE);
}

string PAIWorldWrapper::selfName()
{
    return pai.getAvatarInterface().getPetId();
}

string PAIWorldWrapper::ownerName()
{
    return pai.getAvatarInterface().getOwnerId();
}

/**
   do a lookup in:

   AtTimeLink
       TimeNode "$timestamp"
       EvalLink
           PredicateNode "AGISIM_position"
           ListLink
               ObjectNode "$obj_id"
               NumberNode "$pitch"
               NumberNode "$roll"
               NumberNode "$yaw"
**/
double PAIWorldWrapper::getAngleFacing(Handle slobj) throw (ComboException, AssertionException, std::bad_exception)
{
    const AtomSpace& as = pai.getAtomSpace();
    //get the time node of the latest map, via the link AtTimeLink(TimeNode,SpaceMap)
    Handle atTimeLink = spaceServer().getLatestMapHandle();
    OC_ASSERT(atTimeLink != Handle::UNDEFINED);
    const SpaceServer::SpaceMap& sm = spaceServer().getLatestMap();

    if (sm.containsObject(slobj)) {
        //return the yaw
        double result = sm.getEntity(slobj)->getYaw();
        logger().debug("getAngleFacing(%s) => %f", as.getName(slobj).c_str(), result);
        return result;
    }
    std::stringstream stream (std::stringstream::out);
    stream << "Can't find angle that Object '" << as.getName(slobj)
        << "' is facing at" << std::endl;
    throw ComboException(TRACE_INFO, "PAIWorldWrapper - %s.",
                                  stream.str().c_str());
}

Handle PAIWorldWrapper::toHandle(combo::definite_object obj)
{
    return WorldWrapperUtil::toHandle(pai.getAtomSpace(), obj,
                                      selfName(), ownerName());
}

} } // namespace opencog::world
