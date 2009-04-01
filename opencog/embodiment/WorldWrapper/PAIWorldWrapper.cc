/*
 * opencog/embodiment/WorldWrapper/PAIWorldWrapper.cc
 *
 * Copyright (C) 2007-2008 Nil Geisweiller, Moshe Looks
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

#include "PAIWorldWrapper.h"
#include "WorldWrapperUtil.h"
#include <opencog/atomspace/SpaceServer.h>
#include "TangentBug.h"
#include "PVPXmlConstants.h"
#include "AStarController.h"
#include <opencog/atomspace/Node.h>
#include <boost/bind.hpp>
#include "NetworkElement.h"
#include "util/RandGen.h"
#include "PredefinedProcedureNames.h"
#include "HPASearch.h"
#include "PetComboVocabulary.h"

#include <boost/scoped_ptr.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <sstream> 
#include <limits>


// The percentage bellow is related to the diagonal of the SpaceMap
#define STEP_SIZE_PERCENTAGE 2.0

#define MAIN_LOGGER_ACTION_PLAN_FAILED logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - No action plan has been sent");
#define MAIN_LOGGER_ACTION_PLAN_SUCCEEDED logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - The action plan has been successfully sent");

namespace WorldWrapper {

    using namespace PetCombo;
    //  using namespace Spatial;
    using namespace opencog;
    using namespace PerceptionActionInterface;

    /**
     * public methods
     */

    //ctor, stor
    PAIWorldWrapper::PAIWorldWrapper(PAI& pai, opencog::RandGen& _rng)
        : _pai(pai), rng(_rng), _hasPlanFailed(false) { }

    PAIWorldWrapper::~PAIWorldWrapper() { }


    bool PAIWorldWrapper::isPlanFinished() const {
        return _pai.isPlanFinished(_planID);
    }

    bool PAIWorldWrapper::isPlanFailed() const {
        return _hasPlanFailed || _pai.hasPlanFailed(_planID);
    }

    bool PAIWorldWrapper::sendSequential_and(sib_it from, sib_it to)
        throw (opencog::ComboException, opencog::AssertionException, std::bad_exception) {

        //DEBUG log
        if(opencog::Logger::DEBUG <= logger().getLevel()) {
            string actionPlanStr = "{";
            for(sib_it sib=from; sib!=to;) {
                stringstream ss;
                ss << combo_tree(sib);
                actionPlanStr += ss.str();
                ++sib;
                if(sib!=to)
                    actionPlanStr += ", ";
            }
            actionPlanStr += "}";
            logger().log(opencog::Logger::DEBUG,
                            "PAIWorldWrapper - Attempt to send the following sequence of actions: %s",
                            actionPlanStr.c_str());
        }
        //~DEBUG log

        _hasPlanFailed = false;
        _planID = _pai.createActionPlan();

        const SpaceServer::SpaceMap& sm = _pai.getSpaceServer().getLatestMap();
        const AtomSpace& as = _pai.getSpaceServer().getAtomSpace();
        //treat the case when the action is a compound
        if(WorldWrapperUtil::is_builtin_compound_action(*from)) {
            opencog::cassert(TRACE_INFO, ++sib_it(from)==to); //there is only one compound action
            pre_it it = from;
            builtin_action ba=get_builtin_action(*it);
            pet_builtin_action_enum bae = get_enum(ba);
            switch (bae) {
            case id::goto_obj: {
                logger().log(opencog::Logger::DEBUG,
                                "PAIWorldWrapper - Handling goto_obj. # of parameters = %d", it.number_of_children() );
                opencog::cassert(TRACE_INFO, it.number_of_children()==2);
                opencog::cassert(TRACE_INFO, is_definite_object(*it.begin()));
                opencog::cassert(TRACE_INFO, is_contin(*++it.begin()));

	
                std::string target = get_definite_object( *it.begin() );                
                float walkSpeed = get_contin( *++it.begin() );
                logger().log(opencog::Logger::DEBUG,
                                "PAIWorldWrapper - goto_obj(%s, %f)", target.c_str( ), walkSpeed );
                if ( target == "custom_path" ) {
                    std::string customWaypoints =
                        _pai.getPetInterface( ).getCurrentModeHandler( ).getPropertyValue( "customPath" );
	    
                    std::vector<Spatial::Point> actionPlan;

                    std::vector<std::string> strWayPoints;
                    boost::algorithm::split( strWayPoints, customWaypoints, boost::algorithm::is_any_of(";") );
	  
                    Spatial::Point wayPoint;
                    unsigned int i;
                    for( i = 0; i < strWayPoints.size( ); ++i ) {
                        std::istringstream parser( strWayPoints[i] );
                        parser >> wayPoint.first;
                        parser >> wayPoint.second;

                        if ( i > 0 ) {
                            // TODO: use the pathfinding algorithm to go to this wayPoint
                            actionPlan.push_back( wayPoint );
                        } else {
                            std::istringstream nextParser( strWayPoints[i+1] );
                            Spatial::Point nextWaypoint;
                            nextParser >> nextWaypoint.first;
                            nextParser >> nextWaypoint.second;
	      
                            getWaypoints( wayPoint, nextWaypoint, actionPlan );
                            ++i;
                        } // else
                    } // for
	  
                    // register seeking object
                    _pai.getPetInterface( ).setLatestGotoTarget( std::pair<std::string,Spatial::Point>( target, wayPoint ) );
	  
                    if ( _hasPlanFailed || !createWalkPlanAction( actionPlan, false, Handle::UNDEFINED, walkSpeed ) ) {
                        if (_hasPlanFailed) {
                            logger().log(opencog::Logger::ERROR, "PAIWorldWrapper - Failed to create a goto plan to the goal.");
                        } else {
                            logger().log(opencog::Logger::INFO,
                                            "PAIWorldWrapper - No goto plan needed since the goal was already near enough.");
                        } // else	      
                    } // if

                    std::vector<std::string> arguments;
                    _pai.getPetInterface( ).getCurrentModeHandler( ).handleCommand( "followCustomPathDone", arguments );
                } else {
	  
                    if ( target == "custom_object" ) {
                        target = _pai.getPetInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
                    } // if
                    Handle targetHandle = toHandle( target );
                    try {
                        sm.getEntity( as.getName( targetHandle ) );

                        if (!build_goto_plan(targetHandle, false, Handle::UNDEFINED, Handle::UNDEFINED, walkSpeed )) {
                            if (_hasPlanFailed) {
                                logger().log(opencog::Logger::ERROR, "PAIWorldWrapper - Failed to create a goto plan to the goal.");
                            } else {
                                logger().log(opencog::Logger::INFO, "PAIWorldWrapper - No goto plan needed since the goal was already near enough.");
                                std::vector<std::string> arguments;
                                _pai.getPetInterface( ).getCurrentModeHandler( ).handleCommand( "gotoDone", arguments );

                            } // else
                            MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                            return false;
                        } // if

                    } catch( NotFoundException& ex ) {
                        logger().log(opencog::Logger::ERROR, "PAIWorldWrapper - Goto: target not found.");
                        _hasPlanFailed=true;
                        MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                        return false;
                    } // catch

                    std::vector<std::string> arguments;
                    _pai.getPetInterface( ).getCurrentModeHandler( ).handleCommand( "gotoDone", arguments );
                } // else

            } break;

            case id::gonear_obj: {
                try {
                    opencog::cassert(TRACE_INFO, it.number_of_children()==2);
                    opencog::cassert(TRACE_INFO, is_definite_object(*it.begin()));
                    opencog::cassert(TRACE_INFO, is_contin(*++it.begin()));
	  
                    std::string target = get_definite_object( *it.begin() );                    
                    float walkSpeed = get_contin( *++it.begin() );

                    if ( target == "custom_object" ) {
                        target = _pai.getPetInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
                    } // if

                    const Spatial::EntityPtr& entity = sm.getEntity( target );

                    // 3,0% of the maximum world horizontal distance
                    float distanceFromGoal = fabs( sm.xMax( ) - sm.xMin( ) ) * 0.025;

                    // get the object's direction vector (where it's face is pointing)
                    Spatial::Math::Vector3 direction( entity->getDirection( ) );	  

                    Spatial::Point position = sm.getNearFreePointAtDistance( 
                                                                            Spatial::Point( entity->getPosition( ).x, entity->getPosition( ).y ), 
                                                                            distanceFromGoal,
                                                                            Spatial::Point( direction.x, direction.y ) 
                                                                            );

                    logger().log( opencog::Logger::DEBUG, "PAIWorldWrapper - gonear_obj(%s) calculated position(%f, %f) target orientation(%s) target location(%s) distance from target(%f)", target.c_str( ), position.first, position.second, entity->getOrientation( ).toString( ).c_str( ), entity->getPosition( ).toString( ).c_str( ), distanceFromGoal );

                    if ( !buildGotoPlan( position, walkSpeed ) ) {
                        if ( _hasPlanFailed ) {
                            logger().log(opencog::Logger::ERROR,
                                            "PAIWorldWrapper - Failed to create a goto plan to the goal.");
                        } else {
                            logger().log(opencog::Logger::INFO,
                                            "PAIWorldWrapper - No goto plan needed since the goal was already near enough.");

                        } // else
                        MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                        return false;
                    } // if
                } catch( NotFoundException& ex ) {
                    // Pet will stay at it's current position until it's owner isn't at a valid position
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrappe - Goto: target not found.");
                    _hasPlanFailed = true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                } // catch

            }	break;

            case id::gobehind_obj: {

                opencog::cassert(TRACE_INFO, it.number_of_children()==2);
                opencog::cassert(TRACE_INFO, is_definite_object(*it.begin()));
                opencog::cassert(TRACE_INFO, is_contin(*++it.begin()));

                std::string target = get_definite_object( *it.begin() );                
                float walkSpeed = get_contin( *++it.begin() );
 
                if ( target == "custom_object" ) {
                    target = _pai.getPetInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
                } // if

                if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),  target ) ) { //can't find the target
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrapper - gobehind_obj: target[%s] not found.",
                                    target.c_str( ) );
                    _hasPlanFailed = true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                } // if
	
                Handle targetHandle = toHandle( target );
                //const std::string& targetId = as.getName( targetHandle );

                //const SpaceServer::ObjectMetadata& md = sm.getMetaData( targetId );
                const Spatial::EntityPtr& entity = sm.getEntity( target );

                //SpaceServer::SpaceMapPoint goalLocation =
                //  WorldWrapperUtil::getLocation(sm, as, targetHandle );

                // get the object's direction vector (where it's face is pointing)
                //Spatial::Math::Vector2 correctDirection( std::cos(md.yaw), std::sin(md.yaw) );
                Spatial::Math::Vector3 direction( entity->getDirection( ) );

                Spatial::Point goalPoint = sm.findFree( Spatial::Point( entity->getPosition( ).x, entity->getPosition( ).y ), Spatial::Point( direction.x, direction.y ) );
	
                //Spatial::Point startPoint( md.centerX, md.centerY );

                // register seeking object
                _pai.getPetInterface( ).setLatestGotoTarget( 
                                                            std::pair<std::string,Spatial::Point>( target, Spatial::Point( entity->getPosition( ).x, entity->getPosition( ).y ) ) );

                if ( !buildGotoPlan( goalPoint, walkSpeed ) ){
                    if (_hasPlanFailed) {
                        logger().log(opencog::Logger::ERROR,
                                        "PAIWorldWrapper - Failed to create a gobehind_obj plan to the goal.");
                    } else {
                        logger().log(opencog::Logger::INFO,
                                        "PAIWorldWrapper - No gobehind_obj plan needed since the goal was already near enough.");
                    } // if
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                } // if

            }	break;

            case id::heel:
                opencog::cassert(TRACE_INFO, it.number_of_children()==0);
                //first goto the owner, if we can find it
                if (!WorldWrapperUtil::inSpaceMap(sm, as,
                                                  selfName(), ownerName(),
                                                  combo::vertex("owner"))) {
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrapper - Heel: owner not found.");

                    _hasPlanFailed=true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                }
                if(!build_goto_plan(WorldWrapperUtil::ownerHandle(_pai.getSpaceServer().getAtomSpace(),
                                                                  ownerName())))
                    _planID=_pai.createActionPlan();
                //then add a heel action at the end
                _pai.addAction(_planID,PetAction(ActionType::HEEL()));
                break;
            case id::nudge_to:
                opencog::cassert(TRACE_INFO, it.number_of_children()==2);
                opencog::cassert(TRACE_INFO, is_definite_object(*it.begin()));
                opencog::cassert(TRACE_INFO, is_definite_object(*++it.begin()));
                //make sure we can find the obj to nudge and its destination
                if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                  *it.begin()) ||
                    !WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                  *++it.begin())) {
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrapper - Nudge: obj or destination not found.");

                    _hasPlanFailed=true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                }
                //first goto the obj that we want to nudge
                {
                    bool useExistingPlan=build_goto_plan(toHandle(get_definite_object(*it.begin())));
                    //now build a nudging plan adding to it
                    if (!build_goto_plan(toHandle(get_definite_object(*++it.begin())),
                                         useExistingPlan,
                                         toHandle(get_definite_object(*it.begin())))
                        && !useExistingPlan) {
                        MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                        return false;
                    }
                }
                break;
            case id::go_behind: {
                opencog::cassert(TRACE_INFO, it.number_of_children()==3);
                opencog::cassert(TRACE_INFO, is_definite_object(*it.begin()));
                opencog::cassert(TRACE_INFO, is_definite_object(*++it.begin()));
                opencog::cassert(TRACE_INFO, is_contin(*++(++it.begin())));

                //and just goto it, if we can find the args on the map
                if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                  *it.begin()) ||
                    !WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                  *++it.begin())) {
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrapper - Go behind: args not found.");

                    _hasPlanFailed=true;
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                }
                float walkSpeed = get_contin( *++(++it.begin()) );

                if (!build_goto_plan(toHandle(get_definite_object(*it.begin())),
                     false, Handle::UNDEFINED, toHandle(get_definite_object(*++it.begin()))), walkSpeed ) {
                    MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                    return false;
                }
            } break;
            case id::follow: {
                opencog::cassert(TRACE_INFO, it.number_of_children()==3);
                opencog::cassert(TRACE_INFO, is_definite_object(*it.begin()));
                opencog::cassert(TRACE_INFO, is_contin(*++it.begin()));
                opencog::cassert(TRACE_INFO, is_contin(*++(++it.begin())));
                {
                    Handle obj=toHandle(get_definite_object(*it.begin()));
                    opencog::cassert(TRACE_INFO, obj != Handle::UNDEFINED);
                    //first goto the obj to follow, if we can find it
                    if (!WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(),
                                                      *it.begin())) {
                        logger().log(opencog::Logger::ERROR,
                                        "PAIWorldWrapper - Follow: obj not found.");

                        _hasPlanFailed=true;
                        MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                        return false;
                    }
                    float walkSpeed = get_contin( *++it.begin() );
                    float duration = get_contin( *++(++it.begin( ) ) );

                    if ( duration == 0 ) {
                        duration = _pai.getPetInterface( ).computeFollowingDuration( );
                    } // if

                    if (!build_goto_plan(obj, false, Handle::UNDEFINED, Handle::UNDEFINED, walkSpeed ) )
                        _planID=_pai.createActionPlan();
                    //then add a follow action at the end
                    PetAction action(ActionType::FOLLOW());
                    action.addParameter(ActionParameter("id",
                                                        ActionParamType::ENTITY(),
                                                        Entity(get_definite_object(*it.begin()),
                                                               resolveType(*it.begin()))));
                    action.addParameter(ActionParameter("duration",ActionParamType::FLOAT(), lexical_cast<string>(duration)) );
                    _pai.addAction(_planID,action);
                }
            } break;
            default:
                std::stringstream stream (std::stringstream::out);
                stream << "Unrecognized compound schema '"  << combo_tree(it) << "'" << std::endl;
                throw opencog::ComboException(TRACE_INFO, "PAIWorldWrapper - %s.", stream.str().c_str());
            }
            //try { //TODO
            if(_pai.isActionPlanEmpty(_planID)) {
                _hasPlanFailed = true;
                MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                return false;
            }
            else {
                _pai.sendActionPlan(_planID);
                MAIN_LOGGER_ACTION_PLAN_SUCCEEDED; //macro see top of the file
                return true;
            }
            //} catch (...) {
            //throw ActionPlanSendingFailure(_planID);
            //}

        }
        else { //non-compound action sequence
            while(from!=to) {
                try {
                    PetAction action = buildPetAction(from);
                    _pai.addAction(_planID, action);
                    ++from;
                } catch(...){
                    //log error
                    std::stringstream ss (stringstream::out);
                    ss << combo_tree(from);
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrapper - Failed to build PetAction '%s'.",
                                    ss.str().c_str());
                    //~log error
                    
                    from = to; //so no more actions are built for that action plan, as according to and_seq semantics
                }
            }
            //TODO : add expection
            //try {
            if(_pai.isActionPlanEmpty(_planID)) {
                _hasPlanFailed = true;
                MAIN_LOGGER_ACTION_PLAN_FAILED; //macro see top of the file
                return false;
            }
            else {
                _pai.sendActionPlan(_planID);
                MAIN_LOGGER_ACTION_PLAN_SUCCEEDED; //macro see top of the file
                return true;
            }
            //} catch (...) {
            //throw ActionPlanSendingFailure(_planID);
            //}
        }
    }

    combo::vertex PAIWorldWrapper::evalPerception(pre_it it, combo::variable_unifier& vu) {
        Handle smh = _pai.getSpaceServer().getLatestMapHandle();
        unsigned int current_time = _pai.getLatestSimWorldTimestamp();
        opencog::cassert(TRACE_INFO, smh!=Handle::UNDEFINED,
                          "A SpaceMap must exists");
        combo::vertex v = WorldWrapperUtil::evalPerception(rng, smh, current_time,
                                                           _pai.getSpaceServer(),
                                                           selfName(), ownerName(),
                                                           it, false, vu);

        //DEBUG log
        if(opencog::Logger::DEBUG <= logger().getLevel()) {
            stringstream p_ss, r_ss;
            p_ss << combo_tree(it);
            r_ss << v;
            logger().log(opencog::Logger::DEBUG,
                            "PAIWorldWrapper - Perception %s has been evaluation to %s",
                            p_ss.str().c_str(), r_ss.str().c_str());
        }
        //~DEBUG log

        return v;
    }

    combo::vertex PAIWorldWrapper::evalIndefiniteObject(indefinite_object io,
                                                        combo::variable_unifier& vu) {
        Handle smh = _pai.getSpaceServer().getLatestMapHandle();
        unsigned int current_time = _pai.getLatestSimWorldTimestamp();
        opencog::cassert(TRACE_INFO, smh!=Handle::UNDEFINED,
                          "A SpaceMap must exists");

        combo::vertex v = WorldWrapperUtil::evalIndefiniteObject(rng, smh, current_time,
                                                                 _pai.getSpaceServer(),
                                                                 selfName(), ownerName(),
                                                                 io, false, vu);
    
        //DEBUG log
        if(opencog::Logger::DEBUG <= logger().getLevel()) {
            stringstream io_ss, r_ss;
            io_ss << io;
            r_ss << v;
            logger().log(opencog::Logger::DEBUG,
                            "PAIWorldWrapper - Indefinition object %s has been evaluation to %s",
                            io_ss.str().c_str(), r_ss.str().c_str());
        }
        //~DEBUG log

        return v;
    }
  

    /**
     * private methods
     */
    void PAIWorldWrapper::clearPlan( std::vector<Spatial::Point>& actions,
                                     const Spatial::Point& startPoint,
                                     const Spatial::Point& endPoint ) {
        const SpaceServer::SpaceMap& sm = _pai.getSpaceServer().getLatestMap();

        double closestDist = SpaceServer::SpaceMap::eucDist( startPoint,
                                                             endPoint );
        std::vector<Spatial::Point>::iterator eraseFrom = actions.begin( );
        std::vector<Spatial::Point>::iterator it;

        for ( it = actions.begin(); it != actions.end(); ++it ) {
            double d = SpaceServer::SpaceMap::eucDist(*it, endPoint);
            if(d < closestDist){
                eraseFrom = next(it);

                // at some point the following "take the nearest point"
                // heuristic can be
                // made more subtle by adding a weight to the length of the path -
                // e.g. if the path generated by tb is (x1,x2,...,xN) and N is large,
                // and the point xN is only slightly closer to the goal than x2, then
                // maybe we want to just go to x2 and not all the way to xN
                if(d <= sm.radius() * 1.1){
                    break;
                } // if
                closestDist = d;
            } // if
        } // for
        actions.erase( eraseFrom, actions.end( ) );
    }

    Spatial::Point PAIWorldWrapper::getValidPosition( const Spatial::Point& location ) {
        const SpaceServer::SpaceMap& spaceMap = _pai.getSpaceServer().getLatestMap();

        //make sure that the object location is valid
        Spatial::Point correctedLocation = location;
        if ( spaceMap.illegal( location ) ) {
            logger().log(opencog::Logger::WARN,
                            "PAIWorldWrapper - Position (%.2f, %.2f) is invalid (off the grid, near/inside an obstacle).",
                            location.first, location.second);

            correctedLocation = spaceMap.getNearestFreePoint( location );

            logger().log(opencog::Logger::WARN,
                            "PAIWorldWrapper - Changed position to the nearest valid point (%.2f, %.2f).",
                            correctedLocation.first, correctedLocation.second);
        } // if

        return correctedLocation;
    }

    void PAIWorldWrapper::getWaypoints( const Spatial::Point& startPoint, const Spatial::Point& endPoint, std::vector<Spatial::Point>& actions ) {
        const std::string pathFindingAlgorithm =
            MessagingSystem::NetworkElement::parameters.get("NAVIGATION_ALGORITHM");

        const SpaceServer::SpaceMap& sm = _pai.getSpaceServer().getLatestMap();

        try {
            Spatial::Point begin = startPoint;
            Spatial::Point end = endPoint;

            Spatial::Point correctedAgentLocation = getValidPosition( begin );
            Spatial::Point correctedEndLocation = getValidPosition( end );

            if ( correctedAgentLocation != begin ) {
                begin = correctedAgentLocation;
                actions.push_back( begin );
            } // if

            if ( correctedEndLocation != end ) {
                end = correctedEndLocation;
            } // if     

            if( pathFindingAlgorithm == "astar") {
                Spatial::LSMap2DSearchNode petNode = sm.snap(Spatial::Point(begin.first, begin.second));
                Spatial::LSMap2DSearchNode goalNode = sm.snap(Spatial::Point(end.first, end.second));

                Spatial::AStarController AStar;
                SpaceServer::SpaceMap *map = const_cast<SpaceServer::SpaceMap*>(&sm);
                AStar.setMap( map );
                AStar.setStartAndGoalStates(petNode, goalNode);

                //finally, run AStar
                _hasPlanFailed = (AStar.findPath() != AStarSearch<Spatial::LSMap2DSearchNode>::SEARCH_STATE_SUCCEEDED);
                actions = AStar.getShortestCalculatedPath( );

                logger().log(opencog::Logger::DEBUG,
                                "PAIWorldWrapper - AStar result %s.", !_hasPlanFailed ? "true" : "false");
            } else if ( pathFindingAlgorithm == "hpa" ) {

                SpaceServer::SpaceMap *map = const_cast<SpaceServer::SpaceMap*>(&sm);
                unsigned int maximumClusters = atoi( MessagingSystem::NetworkElement::parameters.get( "HPA_MAXIMUM_CLUSTERS").c_str( ) );
                Spatial::HPASearch search( map, 1, maximumClusters );

                _hasPlanFailed = !search.processPath( Spatial::Math::Vector2( begin.first, begin.second ), Spatial::Math::Vector2( end.first, end.second ) );
                std::vector<Spatial::Math::Vector2> pathPoints = search.getProcessedPath( 1 );
                if ( !_hasPlanFailed ) {
                    foreach( Spatial::Math::Vector2 pathPoint, pathPoints ) {
                        actions.push_back( Spatial::Point( pathPoint.x, pathPoint.y ) );
                    } // foreach
                } // if

                logger().log(opencog::Logger::DEBUG,
                                "PAIWorldWrapper - HPASearch result %s.", !_hasPlanFailed ? "true" : "false");

            } else {
                Spatial::TangentBugBits::TangentBug::CalculatedPath calculatedPath;
                Spatial::TangentBugBits::TangentBug tb(sm, calculatedPath, rng);

                //place the pet and the goal on the map
                tb.place_pet(begin.first, begin.second);
                tb.place_goal(end.first, end.second);

                //finally, run tangent bug
                _hasPlanFailed = !tb.seek_goal();
                if ( !_hasPlanFailed ) {
                    Spatial::TangentBugBits::TangentBug::CalculatedPath::iterator it;
                    for( it = calculatedPath.begin( ); it != calculatedPath.end( ); ++it ) {
                        actions.push_back( boost::get<0>( *it ) );
                    } // for
                } // if
                logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - TangetBug result %s.",
                                !_hasPlanFailed ? "true" : "false");
            } // else          
        } catch ( opencog::RuntimeException& e ) {
            _hasPlanFailed = true;
        } catch ( opencog::AssertionException& e) {
            _hasPlanFailed = true;
        } // catch
    }
  
    bool PAIWorldWrapper::buildGotoPlan( const Spatial::Point& position, float customSpeed ) {

        const AtomSpace& as = _pai.getSpaceServer().getAtomSpace();
        const SpaceServer::SpaceMap& sm = _pai.getSpaceServer().getLatestMap();
        std::vector<Spatial::Point> actions;

        Spatial::Point startPoint = WorldWrapperUtil::getLocation(sm, as, 
                                                                  WorldWrapperUtil::selfHandle( as, selfName( ) ) );
        Spatial::Point endPoint = position;
    
        getWaypoints( startPoint, endPoint, actions );

        if ( !_hasPlanFailed ) {
            clearPlan( actions, startPoint, endPoint );
            return createWalkPlanAction( actions, false, Handle::UNDEFINED, customSpeed );
        } // if
  
        // plan has failed
        return false;
    }

    bool PAIWorldWrapper::createWalkPlanAction( std::vector<Spatial::Point>& actions, bool useExistingId, Handle toNudge, float customSpeed ) {

        if ( actions.empty( ) ){
            // we're done. No need to create any walk sequency
            logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - Zero actions from AStar.");
            return false;
        }

        // --------------------------------------------------------------------
        // transform to a sequence of walk commands
        // --------------------------------------------------------------------

        if (!useExistingId ) {
            _planID = _pai.createActionPlan( );
        } // if

        foreach(const Spatial::Point& it_action, actions ) {
            PetAction action;

            if (toNudge != Handle::UNDEFINED) {
                action=PetAction(ActionType::NUDGE_TO());
                action.addParameter(ActionParameter("moveableObj",
                                                    ActionParamType::ENTITY(),
                                                    Entity(_pai.getSpaceServer().getAtomSpace().getName(toNudge),
                                                           resolveType(toNudge))));
                action.addParameter(ActionParameter("target",
                                                    ActionParamType::VECTOR(),
                                                    Vector(it_action.first,
                                                           it_action.second,
                                                           0.0)));
            } else {
                action=PetAction(ActionType::WALK());
                action.addParameter(ActionParameter("target",
                                                    ActionParamType::VECTOR(),
                                                    Vector(it_action.first,
                                                           it_action.second,
                                                           0.0)));

                float speed = ( customSpeed != 0 ) ? customSpeed : _pai.getPetInterface().computeWalkingSpeed();
                logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper::createWalkPlanAction customSpeed[%f] finalSpeed[%f]", customSpeed, speed );
                action.addParameter(ActionParameter("speed", ActionParamType::FLOAT(), lexical_cast<string>( speed) ) );

            } // else
            _pai.addAction( _planID, action );
        } // foreach

        return true;
    }

    bool PAIWorldWrapper::build_goto_plan(Handle goalHandle,
                                          bool useExistingID,
                                          Handle toNudge,
                                          Handle goBehind, float walkSpeed ) {

        const AtomSpace& atomSpace = _pai.getSpaceServer().getAtomSpace();
        const SpaceServer::SpaceMap& spaceMap = _pai.getSpaceServer().getLatestMap();
        std::string goalName = atomSpace.getName(goalHandle);

        opencog::cassert(TRACE_INFO, goalHandle != Handle::UNDEFINED);
        opencog::cassert(TRACE_INFO, spaceMap.containsObject(goalName));

        Spatial::Point startPoint;
        Spatial::Point endPoint;
        Spatial::Point targetCenterPosition;

        try {
            startPoint = WorldWrapperUtil::getLocation(spaceMap, atomSpace,
                                                       WorldWrapperUtil::selfHandle(atomSpace, selfName()));
            targetCenterPosition = WorldWrapperUtil::getLocation(spaceMap, atomSpace, goalHandle );
            if ( goBehind != Handle::UNDEFINED ) {
                endPoint = spaceMap.behindPoint(WorldWrapperUtil::getLocation(spaceMap, atomSpace, goBehind), goalName);
            } else {
                endPoint = spaceMap.nearbyPoint(startPoint, goalName);
                if(spaceMap.gridIllegal(spaceMap.snap(endPoint))){
                    logger().log(opencog::Logger::ERROR,
                                    "PAIWorldWrapper - nearby point selected and invalid point.");
                }
            } // else
        } catch( opencog::AssertionException& e ){
            logger().log(opencog::Logger::ERROR,
                            "PAIWorldWrapper - Unable to get pet or goal location.");
            _hasPlanFailed = true;
            return false;
        } // catch

        logger().log(opencog::Logger::FINE,
                        "PAIWorldWrapper - Pet position: (%.2f, %.2f). Goal position: (%.2f, %.2f) - %s.",
                        startPoint.first, startPoint.second,  endPoint.first, endPoint.second, goalName.c_str());
        // register seeking object
        _pai.getPetInterface( ).setLatestGotoTarget(
                                                    std::pair<std::string,Spatial::Point>( goalName, targetCenterPosition ) );

        return buildGotoPlan( endPoint, walkSpeed );
    }

    PetAction PAIWorldWrapper::buildPetAction(sib_it from) {
        unsigned int current_time = _pai.getLatestSimWorldTimestamp();
        const AtomSpace& as = _pai.getSpaceServer().getAtomSpace();
        const SpaceServer::SpaceMap& sm = _pai.getSpaceServer().getLatestMap();
        static const std::map<pet_builtin_action_enum,ActionType> actions2types=
            boost::assign::map_list_of
            (id::bark, ActionType::BARK())
            (id::bare_teeth_at, ActionType::BARE_TEETH())
            (id::bark_at, ActionType::BARK())
            (id::chew, ActionType::CHEW())
            (id::dream, ActionType::DREAM())
            (id::drink, ActionType::DRINK())
            (id::eat, ActionType::EAT())
            (id::grab, ActionType::GRAB())
            (id::growl_at, ActionType::GROWL())
            (id::jump_up, ActionType::JUMP_UP())
            (id::jump_towards, ActionType::JUMP_TOWARD())
            //(id::move_left_ear, ActionType::LEFT_EAR_?()) each ear movement has its own command
            //(id::move_right_ear, ActionType::RIGHT_EAR_?())
            (id::lick_at, ActionType::LICK())
            (id::move_head, ActionType::MOVE_HEAD())
            (id::random_step, ActionType::WALK())
            (id::rotate_left, ActionType::TURN())
            (id::rotate_right, ActionType::TURN())
            // (id::scratch_self, ActionType::SCRATCH_SELF_*()) // each body part has itw own  scratch command
            (id::scratch_other, ActionType::SCRATCH_OTHER())
            (id::scratch_ground_back_legs, ActionType::SCRATCH_GROUND_BACK_LEGS())
            (id::sniff_at, ActionType::SNIFF_AT())
            (id::sniff_avatar_part, ActionType::SNIFF_AVATAR_PART())
            (id::sniff_pet_part, ActionType::SNIFF_PET_PART())
            (id::step_backward, ActionType::WALK())
            (id::step_forward, ActionType::WALK())
            (id::step_towards, ActionType::WALK())
            (id::tail_flex, ActionType::TAIL_FLEX())
            (id::turn_to_face, ActionType::TURN())
            (id::wag, ActionType::WAG())
            (id::bite, ActionType::BITE())
            (id::pet, ActionType::PET())
            (id::kick, ActionType::KICK())
            (id::group_command, ActionType::GROUP_COMMAND())
            (id::receive_latest_group_commands, ActionType::RECEIVE_LATEST_GROUP_COMMANDS())
            (id::sit, ActionType::SIT())
            (id::look_at, ActionType::LOOK_AT())      
            (id::whine_at, ActionType::WHINE());

        opencog::cassert(TRACE_INFO, WorldWrapperUtil::is_builtin_atomic_action(*from));
        builtin_action ba = get_builtin_action(*from);
        pet_builtin_action_enum bae = get_enum(ba);

        /****
             this switch statement deals with
             special cases - e.g. step_forward needs to get translated into a walk
             command based on the pet's current position

             also, commands like scratch needs to have the body-part codes translated

             the full list of such schema is:

             bare_teeth_at(obj)
             bark_at(obj)
             chew(obj)
             dream(obj)
             drink(drinkable_obj)
             eat(edible_obj)
             grab(pickupable_obj)
             growl_at(obj)
             jump_towards(obj)
             move_left_ear(TWITCH|PERK|BACK)
             lick_at(obj)
             move_head(angle, angle)
             random_step
             move_right_ear(TWITCH|PERK|BACK)
             rotate_left
             rotate_right
             scratch_other(obj)
             scratch_self(NOSE|RIGHT_EAR|LEFT_EAR|NECK|RIGHT_SHOULDER|LEFT_SHOULDER)
             sniff_at(obj)
             sniff_avatar_part(avatar,RIGHT_FOOT|LEFT_FOOT|RIGHT_HAND|LEFT_HAND|CROTCH|BUTT)
             sniff_pet_part(pet,NOSE|NECK|BUTT)
             step_backward
             step_forward
             step_towards(obj,TOWARDS|AWAY)
             tail_flex(position)
             turn_to_face(obj)
             bite(obj)
             whine_at(obj)
        ****/
        PetAction action;
        if (actions2types.find(bae)!=actions2types.end()) {
            action=actions2types.find(bae)->second;
        } else {
            logger().log(opencog::Logger::FINE,
                            "PAIWorldWrapper - No action type was found to build pet action at actions2types" );
        } // else

        double theta=0;
        switch(bae) {
            //we're going to do all of the "do x to id y" commands together
        case id::bare_teeth_at:     // bare_teeth_at(obj)
        case id::bark_at:           // bark_at(obj)
        case id::chew:              // chew(obj)
        case id::dream:             // dream(obj)
        case id::drink:             // drink(drinkable_obj)
        case id::eat:               // eat(edible_obj)
        case id::grab:              // grab(pickupable_obj)
        case id::growl_at:          // growl_at(obj)
        case id::lick_at:           // lick_at(obj)
        case id::scratch_other:     // scratch_other(obj)
        case id::sniff_at:          // sniff_at(obj)
        case id::whine_at:          // whine_at(obj)
        case id::bite:              // bite(obj)
        case id::pet:               // pet(obj)
        case id::kick:              // kick(obj)
            action.addParameter(ActionParameter("target",
                                                ActionParamType::ENTITY(),
                                                Entity(get_definite_object(*from.begin()),
                                                       resolveType(*from.begin()))));
            break;

        case id::look_at: {    
            if ( from.number_of_children( ) != 1 ) {
                throw opencog::InvalidParamException(TRACE_INFO,
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
                targetName = _pai.getPetInterface( ).getCurrentModeHandler( ).getPropertyValue( "customObject" );
            } // else
        
            action.addParameter( ActionParameter( "target", ActionParamType::ENTITY( ), 
                                                  Entity( targetName, resolveType(targetName) ) ) );
        } break;
    
        case id::group_command: {    // group_command(string, string [, arg_list] )
            logger().log(opencog::Logger::DEBUG,
                            "PAIWorldWrapper - Building PetAction group_command. number_of_children: %d",
                            from.number_of_children( ) );
      
            if ( from.number_of_children( ) < 2 ) {
                throw opencog::InvalidParamException(TRACE_INFO,
                                                      "PAIWorldWrapper - Invalid number of arguments for group_command" );
            } // if

            sib_it arguments = from.begin( );
            combo::variable_unifier vu;
            // get target name. can be an agent name or all_agents
            std::string targetName;
      
            if ( is_indefinite_object( *arguments ) ) {
                targetName = get_definite_object( evalIndefiniteObject( get_indefinite_object( *arguments ), vu ) );
            } else {
                targetName = get_definite_object( *arguments );
            } // else

            ++arguments;
      
            std::string commandName = get_definite_object( *arguments );

            std::vector<std::string> actionDoneParameters;
            actionDoneParameters.push_back( commandName );
      
            //std::vector<std::string> parameters;
            std::stringstream parameters;
            for(++arguments; arguments != from.end(); arguments++) {
                // retrieve a string containing a given parameter
                std::string parameter;

                if ( is_indefinite_object( *arguments ) ) {
                    parameter = get_definite_object( evalIndefiniteObject( get_indefinite_object( *arguments ), vu ) );
                } else {
                    parameter = get_definite_object( *arguments );
                } // else

                if ( parameter == "randbool" ) {
                    parameter = ((rng.randint() %2)==0) ? "0" : "1";
                } else if ( parameter == "rand" ) {
                    std::stringstream rand;
                    rand << rng.randdouble( );
                    parameter = rand.str( );
                } // if


                //parameters.push_back( parameter );
                parameters << parameter;
                sib_it nextArgument = arguments;
                ++nextArgument;
                actionDoneParameters.push_back( parameter );
                if ( nextArgument != from.end( ) ) {
                    parameters << ";";
                } // if
            } // for


            action.addParameter( ActionParameter( "target", ActionParamType::STRING( ), targetName ) );
            action.addParameter( ActionParameter( "command", ActionParamType::STRING( ), commandName ) );
            action.addParameter( ActionParameter( "parameters", ActionParamType::STRING( ), parameters.str() ) );
      
            _pai.getPetInterface( ).getCurrentModeHandler( ).handleCommand( "groupCommandDone", actionDoneParameters );

        } break;

        case id::receive_latest_group_commands: {
            opencog::cassert(TRACE_INFO, from.number_of_children()==0);

            logger().log(opencog::Logger::DEBUG,
                            "PAIWorldWrapper - receiving latest_group_commands from all agents" );

            //_pai.getPetInterface( ).getCurrentModeHandler( ).handleCommand( "receivedGroupCommand", commandArguments );

            // retrieve all agents
            std::vector<Handle> agentsHandles;
            _pai.getSpaceServer().getAtomSpace( ).getHandleSet( back_inserter(agentsHandles), SL_AVATAR_NODE, false );
            _pai.getSpaceServer().getAtomSpace( ).getHandleSet( back_inserter(agentsHandles), SL_HUMANOID_NODE, false );
            _pai.getSpaceServer().getAtomSpace( ).getHandleSet( back_inserter(agentsHandles), SL_PET_NODE, false );
      
      
            Handle selfHandle = WorldWrapperUtil::selfHandle(  _pai.getSpaceServer().getAtomSpace( ), selfName( ) );
            unsigned int i;
            for( i = 0; i < agentsHandles.size( ); ++i ) {
                if ( agentsHandles[i] == Handle::UNDEFINED || agentsHandles[i] == selfHandle ) {
                    // ignore non agents entities
                    // ignore self
                    continue;
                } // if
                std::string agentId = _pai.getSpaceServer().getAtomSpace( ).getName( agentsHandles[i] );
                logger().log(opencog::Logger::DEBUG, 
                                "PAIWorldWrapper - verifying agent[%s]", agentId.c_str( ) );

                unsigned long delay_past = 10 * PerceptionActionInterface::PAIUtils::getTimeFactor();
                unsigned long t_past = ( delay_past < current_time ? current_time - delay_past : 0 );
	
                Handle agentActionLink = AtomSpaceUtil::getMostRecentAgentActionLink( _pai.getSpaceServer().getAtomSpace( ), agentId, "group_command", Temporal( t_past, current_time ), TemporalTable::OVERLAPS );
	
                if ( agentActionLink == Handle::UNDEFINED ) {
                    logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - agent[%s] did not sent group command during the last 30 seconds", agentId.c_str( ) );
                    continue;
                } // if


                std::string parametersString = 
                    AtomSpaceUtil::convertAgentActionParametersToString( _pai.getSpaceServer().getAtomSpace( ), agentActionLink );
	
                std::vector<std::string> commandParameters;
                boost::split( commandParameters, parametersString, boost::is_any_of( ";" ) );
                // trim all elements
                unsigned int k;
                for( k = 0; k < commandParameters.size( ); ++k ) { boost::trim( commandParameters[k] ); }

                logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - number of parameters[%d], string[%s]", commandParameters.size( ), parametersString.c_str( ) );

                if ( commandParameters.size( ) > 0 && ( commandParameters[0] == "all_agents" || 
                                                        commandParameters[0] == selfName( ) ) ) {
                    std::vector<std::string> commandArguments;
                    commandArguments.push_back( agentId );
	  
                    unsigned int j;
                    for( j = 1; j < commandParameters.size( ); ++j ) {
                        commandArguments.push_back( commandParameters[j] );
                    } // for
	  
                    _pai.getPetInterface( ).getCurrentModeHandler( ).handleCommand( "receivedGroupCommand", commandArguments );
	  
                } // if
            } // for      	
      
        } break;

        //we'll do all of the cases where there's bodyPart arg converted to the right integer constants here

        //these have the target and specific parts
        case id::sniff_avatar_part: // sniff_avatar_part(avatar,RIGHT_FOOT|LEFT_FOOT|RIGHT_HAND|LEFT_HAND|CROTCH|BUTT)
            {
                opencog::cassert(TRACE_INFO, from.number_of_children()==2);
                action.addParameter(ActionParameter("avatar",
                                                    ActionParamType::ENTITY(),
                                                    Entity(get_definite_object(*from.begin()),
                                                           resolveType(*from.begin()))));
                combo::vertex v = *from.last_child();
                opencog::cassert(TRACE_INFO, is_action_symbol(v),
                                  "It is assumed v is an action_symbol");
                combo::pet_action_symbol_enum ase = get_enum(get_action_symbol(v));
                string part;
                // From Pet Action spec: RIGHT_FOOT = 0, LEFT_FOOT = 1, BUTT = 2, RIGHT_HAND = 2, LEFT_HAND = 3, CROTCH = 4
                switch(ase) {
                case id::RIGHT_FOOT:
                    part = "0";
                    break;
                case id::LEFT_FOOT:
                    part = "1";
                    break;
                case id::BUTT:
                    part = "2";
                    break;
                case id::RIGHT_HAND:
                    part = "3";
                    break;
                case id::LEFT_HAND:
                    part = "4";
                    break;
                case id::CROTCH:
                    part = "5";
                    break;
                default:
                    logger().log(opencog::Logger::ERROR, "PAIWorldWrapper - Invalid avatar part as parameter for sniff_avatar_part: %s.", instance(ase)->get_name().c_str());
                    opencog::cassert(TRACE_INFO, false);
                }
                action.addParameter(ActionParameter("part", ActionParamType::INT(), part));
            }
            break;
        case id::sniff_pet_part:    // sniff_pet_part(pet,NOSE|NECK|BUTT)
            {
                opencog::cassert(TRACE_INFO, from.number_of_children()==2);
                action.addParameter(ActionParameter("pet",
                                                    ActionParamType::ENTITY(),
                                                    Entity(get_definite_object(*from.begin()),
                                                           resolveType(*from.begin()))));
                combo::vertex v = *from.last_child();
                opencog::cassert(TRACE_INFO, is_action_symbol(v),
                                  "It is assumed v is an action_symbol");
                pet_action_symbol_enum ase = get_enum(get_action_symbol(v));
                string part;
                switch(ase) {
                case id::NOSE:
                    part = "0";
                    break;
                case id::NECK:
                    part = "1";
                    break;
                case id::BUTT:
                    part = "2";
                    break;
                default:
                    logger().log(opencog::Logger::ERROR, "PAIWorldWrapper - Invalid pet part as parameter for sniff_pet_part: %s.", instance(ase)->get_name().c_str());
                    opencog::cassert(TRACE_INFO, false);
                }
                action.addParameter(ActionParameter("part", ActionParamType::INT(), part));
            }
            break;
            //these have specific actions for each part
        case id::scratch_self:      // scratch_self(NOSE|RIGHT_EAR|LEFT_EAR|NECK|RIGHT_SHOULDER|LEFT_SHOULDER)
            {
                combo::vertex v = *from.begin();
                opencog::cassert(TRACE_INFO, is_action_symbol(v),
                                  "It is assumed v is an action_symbol");
                combo::pet_action_symbol_enum ase = get_enum(get_action_symbol(v));
                switch(ase) {
                case id::NOSE:
                    action=ActionType::SCRATCH_SELF_NOSE();
                    break;
                case id::RIGHT_EAR:
                    action=ActionType::SCRATCH_SELF_RIGHT_EAR();
                    break;
                case id::LEFT_EAR:
                    action=ActionType::SCRATCH_SELF_LEFT_EAR();
                    break;
                case id::NECK:
                    action=ActionType::SCRATCH_SELF_NECK();
                    break;
                case id::RIGHT_SHOULDER:
                    action=ActionType::SCRATCH_SELF_RIGHT_SHOULDER();
                    break;
                case id::LEFT_SHOULDER:
                    action=ActionType::SCRATCH_SELF_LEFT_SHOULDER();
                    break;
                default:
                    logger().log(opencog::Logger::ERROR, "PAIWorldWrapper - Invalid pet part as parameter for scratch_self: %s.", instance(ase)->get_name().c_str());
                    opencog::cassert(TRACE_INFO, false);
                }
            }
            break;

        case id::move_left_ear:  // move_left_ear(TWITCH|PERK|BACK)
        case id::move_right_ear: // move_right_ear(TWITCH|PERK|BACK)
            if (*from.begin()==instance(id::TWITCH)) {
                action=(bae == instance(id::move_left_ear))?ActionType::LEFT_EAR_TWITCH():ActionType::RIGHT_EAR_TWITCH();
            } else if (*from.begin()==instance(id::PERK)) {
                action=(bae == instance(id::move_left_ear))?ActionType::LEFT_EAR_PERK():ActionType::RIGHT_EAR_PERK();
            } else {
                action=(bae == instance(id::move_left_ear))?ActionType::LEFT_EAR_BACK():ActionType::RIGHT_EAR_BACK();
            }
            break;

            //now rotations
        case id::rotate_left:      // rotate_left
            action.addParameter(ActionParameter("rotation",
                                                ActionParamType::ROTATION(),
                                                Rotation(0.0,0.0,_pai.getPetInterface().computeRotationAngle())));
            break;

        case id::rotate_right:     // rotate_right
            action.addParameter(ActionParameter("rotation",
                                                ActionParamType::ROTATION(),
                                                Rotation(0.0,0.0,-_pai.getPetInterface().computeRotationAngle())));
            break;

            //now stepping actions
        case id::random_step:        // random_step
            theta=2.0*PI*rng.randdouble();
            goto build_step;
        case id::step_backward :     // step_backward
            theta=getAngleFacing(WorldWrapperUtil::selfHandle(_pai.getSpaceServer().getAtomSpace(),
                                                              selfName()))+PI;
            if (theta>2*PI)
                theta-=2*PI;
            goto build_step;
        case id::step_forward:      // step_forward
            theta=getAngleFacing(WorldWrapperUtil::selfHandle(_pai.getSpaceServer().getAtomSpace(),
                                                              selfName()));

        build_step:
            //now compute a step in which direction the pet is going
            {
                Spatial::Point petLoc = WorldWrapperUtil::getLocation(sm, as, WorldWrapperUtil::selfHandle(_pai.getSpaceServer().getAtomSpace(), selfName()));

                double stepSize = (sm.diagonalSize())*STEP_SIZE_PERCENTAGE/100; // 2% of the width
                double x = (double)petLoc.first + (cos(theta) * stepSize);
                double y = (double)petLoc.second + (sin(theta) * stepSize);

                // if not moving to an illegal position, then no problem, go
                // to computer position
                if(!sm.illegal(Spatial::Point(x, y))){
                    action.addParameter(ActionParameter("target",
                                                        ActionParamType::VECTOR(),
                                                        Vector(x, y, 0.0)));

                    // if illegal position, stay in the same place (a walk to
                    // the current position
                } else {
                    action.addParameter(ActionParameter("target",
                                                        ActionParamType::VECTOR(),
                                                        Vector((double)petLoc.first,
                                                               (double)petLoc.second,
                                                               0.0)));
                }
                action.addParameter(ActionParameter("speed",
                                                    ActionParamType::FLOAT(),
                                                    lexical_cast<string>(_pai.getPetInterface().computeWalkingSpeed())));
            }
            break;
        case id::step_towards:      // step_towards(obj,TOWARDS|AWAY)
            opencog::cassert(TRACE_INFO, from.number_of_children()==2);
            opencog::cassert(TRACE_INFO,
                              *from.last_child()==instance(id::TOWARDS) ||
                              *from.last_child()==instance(id::AWAY));
            {
                double stepSize = (sm.diagonalSize())*STEP_SIZE_PERCENTAGE/100; // 2% of the width

                // object position
                SpaceServer::SpaceMapPoint p = WorldWrapperUtil::getLocation(sm, as, toHandle(get_definite_object(*from.begin())));

                double len = pow((p.first * p.first) + (p.second * p.second), 0.5);
                p.first *= min(stepSize / len, 1.0);
                p.second *= min(stepSize / len, 1.0);

                if (*from.last_child() == instance(id::AWAY)) {
                    p.first = -p.first;
                    p.second = -p.second;
                }
                action.addParameter(ActionParameter("target",
                                                    ActionParamType::VECTOR(),
                                                    Vector(p.first,p.second,0.0)));
                action.addParameter(ActionParameter("speed",
                                                    ActionParamType::FLOAT(),
                                                    lexical_cast<string>(_pai.getPetInterface().computeWalkingSpeed())));
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
                                                    Vector(p.first,p.second,0.0)));
            }
            break;
        case id::move_head:         // move_head(angle, angle)
            opencog::cassert(TRACE_INFO, from.number_of_children()==3);
            action.addParameter(ActionParameter("position",
                                                ActionParamType::VECTOR(),
                                                Vector(0.0,0.0,0.0)));
            action.addParameter(ActionParameter("rotation",
                                                ActionParamType::ROTATION(),
                                                Rotation(get_contin(*from.begin()),
                                                         get_contin(*++from.begin()),
                                                         get_contin(*from.last_child()))));
            action.addParameter(ActionParameter("speed",
                                                ActionParamType::FLOAT(),
                                                "1.0"));
            break;

        case id::tail_flex:         // tail_flex(position)
            action.addParameter(ActionParameter("position",
                                                ActionParamType::VECTOR(),
                                                Vector(0.0,0.0,0.0)));
            //action.addParameter(ActionParameter("rotation",
            //ActionParamType::ROTATION(),
            //Rotation(0.0,0.0,get_contin(*from.begin()))));
            break;

        case id::turn_to_face:      // turn_to_face(obj)
            {
	
                opencog::cassert(TRACE_INFO, from.number_of_children()==1); 
                opencog::cassert(TRACE_INFO,is_definite_object(*from.begin())); 
                /*
                  const string& slObjName = get_definite_object(*from.begin());
                  double angleFacing = getAngleFacing(WorldWrapperUtil::selfHandle(as, selfName()));
                  logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper: angleFacing = %f", angleFacing); 
                  logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper: slObjName = %s", slObjName.c_str()); 
                  opencog::cassert(TRACE_INFO,  WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(), selfName())); 
                  opencog::cassert(TRACE_INFO,  WorldWrapperUtil::inSpaceMap(sm, as, selfName(), ownerName(), slObjName)); 
                  const SpaceServer::SpaceMapPoint& selfObjPoint = WorldWrapperUtil::getLocation(sm, as, toHandle(selfName()));
                  const SpaceServer::SpaceMapPoint& slObjPoint =  WorldWrapperUtil::getLocation(sm, as, toHandle(slObjName));
                  double deltaX = slObjPoint.first - selfObjPoint.first; 
                  double deltaY = slObjPoint.second - selfObjPoint.second; 
                  double angle = atan2f(deltaX,deltaY); 
                  double rotationAngle = angle - angleFacing;
	
                  logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - Final angle = %f. Angle to turn: %f", angle, rotationAngle);
                */
	
                std::string targetObjectName = get_definite_object(*from.begin());

                float rotationAngle = 0;
                try {
                    //const Spatial::Object& agentObject = sm.getObject( selfName( ) );
                    const Spatial::EntityPtr& agentEntity = sm.getEntity( selfName( ) );
                    //rotationAngle = agentObject.metaData.yaw;

                    //const Spatial::Object& targetObject = sm.getObject( targetObjectName );

                    Spatial::Math::Vector3 targetPosition;
                    if ( targetObjectName == "custom_position" ) {
                        std::stringstream parser( 
                                                 _pai.getPetInterface( ).getCurrentModeHandler( ).getPropertyValue( "customPosition" )
                                                 );
                        parser >> targetPosition.x;
                        parser >> targetPosition.y;
                        parser >> targetPosition.z; 
                    } else {
                        const Spatial::EntityPtr& targetEntity = sm.getEntity( targetObjectName );
                        targetPosition = targetEntity->getPosition( );
                    } // else

                    //Spatial::Math::Vector2 agentPosition( agentObject.metaData.centerX, agentObject.metaData.centerY );
                    //Spatial::Math::Vector2 targetPosition( targetObject.metaData.centerX, targetObject.metaData.centerY );

                    //Spatial::Math::Vector2 agentDirection( cos( agentObject.metaData.yaw ), sin( agentObject.metaData.yaw ) );
                    Spatial::Math::Vector3 targetDirection = targetPosition -  agentEntity->getPosition( );
	  
                    rotationAngle = atan2f(targetDirection.y, targetDirection.x);

                    logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - Agent[pos: %s, ori: %s], Target[pos: %s], Turn Angle: %f", agentEntity->getPosition( ).toString( ).c_str( ),  agentEntity->getOrientation( ).toString( ).c_str( ), targetPosition.toString( ).c_str( ), rotationAngle );

                } catch( NotFoundException& ex ) {
                    logger().log(opencog::Logger::DEBUG, "PAIWorldWrapper - Cannot find an object inside localspacemap: %s", ex.getMessage( ) );
                } // catch
	
                action.addParameter(ActionParameter("rotation", ActionParamType::ROTATION(),  Rotation(0,0,rotationAngle)));

                break;
            }

        default:
            //this will handle simple schema with no arguments - these are:
            /**
               anticipate_play
               bare_teeth
               bark
               beg
               belch
               shake_head
               clean
               run_in_circle
               drop
               back_flip
               growl
               look_up_turn_head
               jump_up
               lick
               lieDown
               tap_dance
               pee
               play_dead
               poo
               trick_for_food
               lean_rock_dance
               sit
               sleep
               fearful_posture
               sniff
               speak
               hide_face
               stretch
               vomit
               whine
               widen_eyes
            **/
            stringstream ss;
            ss << *from;
            action=PetAction(ActionType::getFromName(toCamelCase(ss.str())));
        }
        return action;
    }

    string PAIWorldWrapper::toCamelCase(string str) {
        for (string::iterator it=str.begin();it!=str.end()-1;++it)
            if (*it=='_') {
                it=str.erase(it);
                *it=toupper(*it);
            }
        return str;
    }

    string PAIWorldWrapper::resolveType(combo::vertex v) {
        opencog::cassert(TRACE_INFO, is_definite_object(v));
        opencog::cassert(TRACE_INFO, toHandle(get_definite_object(v)) != Handle::UNDEFINED);
        return resolveType(toHandle(get_definite_object(v)));
    }
    string PAIWorldWrapper::resolveType(Handle h) {
        Type objType=TLB::getAtom(h)->getType();
        return (objType==SL_AVATAR_NODE ? AVATAR_OBJECT_TYPE :
                objType==SL_PET_NODE ? PET_OBJECT_TYPE :
                objType==SL_HUMANOID_NODE ? HUMANOID_OBJECT_TYPE :
                objType==SL_ACCESSORY_NODE ? ACCESSORY_OBJECT_TYPE :
                objType==SL_STRUCTURE_NODE ? STRUCTURE_OBJECT_TYPE :
                objType==SL_OBJECT_NODE ? ORDINARY_OBJECT_TYPE :
                UNKNOWN_OBJECT_TYPE);
    }

    string PAIWorldWrapper::selfName() {
        return _pai.getPetInterface().getPetId();
    }

    string PAIWorldWrapper::ownerName() {
        return _pai.getPetInterface().getOwnerId();
    }

    /**
       do a lookup in:

       AtTimeLink
       TimeNode "$timestamp"
       EvalLink
       PredicateNode "AGISIM_position"
       ListLink
       SLObjectNode "$obj_id"
       NumberNode "$pitch"
       NumberNode "$roll"
       NumberNode "$yaw"
    **/
    double PAIWorldWrapper::getAngleFacing(Handle slobj) throw (opencog::ComboException, opencog::AssertionException, std::bad_exception) {
        const AtomSpace& as=_pai.getSpaceServer().getAtomSpace();
        //get the time node of the latest map, via the link AtTimeLink(TimeNode,SpaceMap)
        Handle atTimeLink=_pai.getSpaceServer().getLatestMapHandle();
        opencog::cassert(TRACE_INFO, atTimeLink != Handle::UNDEFINED);
#if 1
        const SpaceServer::SpaceMap& sm = _pai.getSpaceServer().getLatestMap();
        const string& slObjName = as.getName(slobj);
        if (sm.containsObject(slObjName)) {
            //return the yaw
            double result = sm.getEntity(slObjName)->getOrientation( ).getRoll( );//sm.getMetaData(slObjName).yaw;
            logger().log(opencog::Logger::DEBUG, "getAngleFacing(%s) => %f", as.getName(slobj).c_str(), result);
            return result;
        }
#else
        opencog::cassert(TRACE_INFO, TLB::getAtom(atTimeLink)->getType()==AT_TIME_LINK);
        opencog::cassert(TRACE_INFO, TLB::getAtom(atTimeLink)->getArity()==2);

        Handle timeNode=TLB::getAtom(atTimeLink)->getOutgoingSet()[0];
        opencog::cassert(TRACE_INFO, TLB::getAtom(timeNode)->getType()==TIME_NODE);

        //now use it to lookup the AtTimeLink to our obj
        HandleSeq outgoing = boost::assign::list_of(timeNode)(Handle::UNDEFINED);
        Type types[]={TIME_NODE,EVALUATION_LINK};
        std::vector<Handle> tmp;
        as.getHandleSet(back_inserter(tmp),
                        outgoing,types,Handle::UNDEFINED,2,AT_TIME_LINK,true);

        foreach(Handle h,tmp) {
            //check for well-formedness
            if (TLB::getAtom(TLB::getAtom(TLB::getAtom(h)))->getArity()==2 &&
                TLB::getAtom(TLB::getAtom(TLB::getAtom(h))->getOutgoingSet()[1])->getArity()==2) {
                Handle evalLink = TLB::getAtom(TLB::getAtom(h))->getOutgoingSet()[1];
                if (as.getType(TLB::getAtom(evalLink)->getOutgoingSet()[0])==PREDICATE_NODE &&
                    as.getName(TLB::getAtom(evalLink)->getOutgoingSet()[0])==AGISIM_ROTATION_PREDICATE_NAME &&
                    TLB::getAtom(TLB::getAtom(evalLink)->getOutgoingSet()[1])->getArity()==4) {
                    Handle ll=TLB::getAtom(TLB::getAtom(TLB::getAtom(h)->getOutgoingSet()[1]))->getOutgoingSet()[1];
                    if (TLB::getAtom(ll)->getOutgoingSet()[0]==slobj) {
                        //return the yaw
                        opencog::cassert(TRACE_INFO, TLB::getAtom(ll)->getOutgoingSet().size()==4);
                        double result = lexical_cast<double>(as.getName(as.getOutgoing(ll,3)));
                        logger().log(opencog::Logger::DEBUG, "getAngleFacing(%s) => %f", as.getName(slobj).c_str(), result);
                        return result;
                    }
                }
            }
        }
#endif
        //_pai.getAtomSpace().print();
        std::stringstream stream (std::stringstream::out);
        stream << "Can't find angle that SLObject '" << as.getName(slobj)
               << "' is facing at" << std::endl;
        throw opencog::ComboException(TRACE_INFO, "PAIWorldWrapper - %s.",
                                       stream.str().c_str());
    }

    Handle PAIWorldWrapper::toHandle(combo::definite_object obj) {
        return WorldWrapperUtil::toHandle(_pai.getSpaceServer().getAtomSpace(), obj,
                                          selfName(), ownerName());
    }

}
