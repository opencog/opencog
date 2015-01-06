/*
 * opencog/embodiment/Control/PredicateUpdaters/PetPsychePredicatesUpdater.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Samir Araujo
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


#include <opencog/atomspace/SimpleTruthValue.h>

#include "PetPsychePredicatesUpdater.h"
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAIUtils.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <opencog/spatial/MovableEntity.h>


using namespace opencog::oac;
using namespace opencog;
using namespace opencog::pai;

PetPsychePredicatesUpdater::PetPsychePredicatesUpdater(AtomSpace &_atomSpace ) : BasicPredicateUpdater(_atomSpace)
{

    this->latestSimWorldTimestamp = 0L;
}

PetPsychePredicatesUpdater::~PetPsychePredicatesUpdater()
{
}
#if 0
spatial::math::Triangle PetPsychePredicatesUpdater::createFieldOfViewTriangle(Handle agent)
{
    const SpaceServer::SpaceMap& spaceMap = atomSpace.getSpaceServer().getLatestMap( );

    const spatial::Entity3D* entity = spaceMap.getEntity( agent );

//  const SpaceServer::ObjectMetadata& agentMetaData =
//    spaceMap.getMetaData( atomSpace.getName(agent) );
    SpaceServer::SpaceMapPoint agentCenter( entity->getPosition( ).x, entity->getPosition( ).y );

    // get the object's direction vector (where it's face is pointing)
    spatial::math::Vector3 agentFovDirection = entity->getDirection( );//( std::cos(agentMetaData.yaw), std::sin(agentMetaData.yaw) );

    // agent fov will reach 30% of the map width
    float agentFovReach = fabs( spaceMap.xMax( ) - spaceMap.xMin( ) ) * 0.3;

    spatial::math::Vector3 maximumPoint = agentFovDirection * agentFovReach;

    spatial::math::Line limitLine( spatial::math::Quaternion( spatial::math::Vector3::Z_UNIT, M_PI / 2).rotate( agentFovDirection ) + maximumPoint, spatial::math::Quaternion( spatial::math::Vector3::Z_UNIT, -M_PI / 2).rotate( agentFovDirection ) + maximumPoint );

    // agent fov will be 160 degrees
    spatial::math::Vector3 leftPoint( spatial::math::Quaternion( spatial::math::Vector3::Z_UNIT, 80*M_PI / 180 ).rotate( agentFovDirection ) * agentFovReach );

    spatial::math::Vector3 rightPoint( spatial::math::Quaternion( spatial::math::Vector3::Z_UNIT, -80*M_PI / 180 ).rotate( agentFovDirection ) * agentFovReach );

    spatial::math::Vector3 intersectionPoint;

    spatial::math::LineSegment leftLine( spatial::math::Vector3( 0, 0 ), leftPoint );
    if ( !limitLine.intersects( leftLine, &intersectionPoint ) ) {
        logger().error("PetPsychePredicatesUpdater - Lines should intersect limitLine(%s) leftLine(%s)", limitLine.toString( ).c_str( ), leftLine.toString( ).c_str( ) );
    } // if
    leftPoint = intersectionPoint;

    spatial::math::Line rightLine( spatial::math::Vector3( 0, 0 ), rightPoint );
    if ( !limitLine.intersects( rightLine, &intersectionPoint ) ) {
        logger().error("PetPsychePredicatesUpdater - Lines should intersect limitLine(%s) rightLine(%s) fovDirection(%s) reach(%f) maxPoint(%s)", limitLine.toString( ).c_str( ), rightLine.toString( ).c_str( ), agentFovDirection.toString( ).c_str( ), agentFovReach, maximumPoint.toString( ).c_str( ) );
    } // if
    rightPoint = intersectionPoint;

    
    if ( !spatial::math::lineIntersection( limitLine, spatial::math::LineSegment( spatial::math::Vector3( 0,0 ), leftPoint ), intersectionPoint ) ) {
      logger().error("PetPsychePredicatesUpdater - Line intersection cannot be null lineA(%f,%f - %f, %f) lineB(%f,%f - %f,%f).", limitLine.pointA.x, limitLine.pointA.y, limitLine.pointB.x, limitLine.pointB.y, 0, 0, leftPoint.x, leftPoint.y );
    } // if


    leftPoint = intersectionPoint;
    if ( !lineIntersection( limitLine, spatial::math::LineSegment( spatial::math::Vector3( 0,0 ), rightPoint ), intersectionPoint ) ) {
      logger().error("PetPsychePredicatesUpdater - Line intersection cannot be null lineA(%f,%f - %f, %f) lineB(%f,%f - %f,%f).", limitLine.pointA.x, limitLine.pointA.y, limitLine.pointB.x, limitLine.pointB.y, 0, 0, rightPoint.x, rightPoint.y );
    } // if
    rightPoint = intersectionPoint;




    // translate the points from 0,0 to the current agent position
    spatial::math::Vector3 agentPosition( agentCenter.first, agentCenter.second );
    leftPoint += agentPosition;
    rightPoint += agentPosition;

    return spatial::math::Triangle( agentPosition, 0, 0 );

}
#endif

void PetPsychePredicatesUpdater::update(Handle object, Handle pet, unsigned long timestamp)
{
//    logger().info("PetPsychePredicatesUpdater - updating context predicates..." );

//    if ( pet == Handle::UNDEFINED ) {
//        logger().error("PetPsychePredicatesUpdater - Got an undefined handle for the pet." );
//        return;
//    } // if

//    // there is no map, no update is possible
//    Handle spaceMapHandle = atomSpace.getSpaceServer().getLatestMapHandle();
//    if (spaceMapHandle == Handle::UNDEFINED) {
//        logger().info("PetPsychePredicatesUpdater - there is no space map defined at this moment..." );
//        return;
//    } // if

//    const SpaceServer::SpaceMap& spaceMap = atomSpace.getSpaceServer().getLatestMap();

//    const std::string& agentName = atomSpace.getName(pet);
//    if ( !spaceMap.containsObject(pet) ) {
//        logger().error("PetPsychePredicatesUpdater - Pet was not inserted in the map yet." );
//        return;
//    }

//    std::vector<std::string> entities;
//    spaceMap.findEntitiesWithClassFilter(back_inserter(entities), "block");
//    float meanValue;
//    bool atHome = false;
//    bool atNight = false;

//    bool nearFriend = false;
//    bool nearEnemy = false;
//    bool nearOwner = false;
//    bool nearFood = false;
//    bool nearWater = false;
//    bool nearPooPlace = false;
//    bool nearPeePlace = false;
//    // this predicate is needed by learning mode. It means near, but not much
//    bool nextOwner = false;

//    const spatial::Entity3D& agentEntity = spaceMap.getEntity( pet );

//    SpaceServer::SpaceMapPoint petCenter( agentEntity->getPosition( ).x, agentEntity->getPosition( ).y );

//    bool needsArtificialFOV = ( atomSpace.getType(pet) == PET_NODE );

//    
//    spatial::math::Triangle petFieldOfView;
//    if ( needsArtificialFOV ) {
//        petFieldOfView = createFieldOfViewTriangle(pet);
//    } // if
//    
//    for ( std::string entity : entities ) {
//        logger().debug("PetPsychePredicatesUpdater - inspecting entity %s", entity.c_str( ) );
//        Handle entityHandle = getHandle(entity);
//        if ( entityHandle == Handle::UNDEFINED ) {
//            logger().debug("PetPsychePredicatesUpdater - there is no entity %s defined at atomspace", entity.c_str( ) );
//            continue;
//        } // if

//        std::string elementId = atomSpace.getName( entityHandle );
//        //const SpaceServer::ObjectMetadata& elementMetaData =
//        //  spaceMap.getMetaData( elementId );
//        const spatial::EntityPtr& objectEntity = spaceMap.getEntity( elementId );
//        SpaceServer::SpaceMapPoint elementCenter( objectEntity->getPosition( ).x, objectEntity->getPosition( ).y );

//        { // log pet and element positions
//            std::stringstream message;
//            message << "PetPsychePredicatesUpdater - ";
//            message << "pet pos(" << petCenter.first << " " << petCenter.second << ") ";
//            message << "pet yaw(rad: " << agentEntity->getOrientation( ).getRoll( );
//            message << " deg: " << ( agentEntity->getOrientation( ).getRoll( ) * 180 / M_PI) << ")";
//            message << "element pos(" << elementCenter.first << " " << elementCenter.second << ") ";
//            message << "dist(" << SpaceServer::SpaceMap::eucDist( petCenter, elementCenter ) << ")";
//            logger().debug(message.str( ).c_str( ) );
//        } // end block


//        bool isNearEntity = AtomSpaceUtil::isPredicateTrue(atomSpace, "near", entityHandle, pet );
//        bool isNextEntity = AtomSpaceUtil::isPredicateTrue(atomSpace, "next", entityHandle, pet );
//        bool isMoving = AtomSpaceUtil::isPredicateTrue(atomSpace, "is_moving", entityHandle );

//        //const spatial::Object& elementObject = spaceMap.getObject( elementId );


//        const spatial::EntityPtr& elementEntity = spaceMap.getEntity( elementId );
//        const spatial::math::BoundingBox& bb = elementEntity->getBoundingBox( );
//        if ( needsArtificialFOV ) {

//            std::vector<spatial::math::LineSegment> bottomSegments;
//            bottomSegments.push_back( spatial::math::LineSegment( bb.getCorner( spatial::math::BoundingBox::FAR_LEFT_BOTTOM ), bb.getCorner( spatial::math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
//            bottomSegments.push_back( spatial::math::LineSegment( bb.getCorner( spatial::math::BoundingBox::FAR_RIGHT_BOTTOM ), bb.getCorner( spatial::math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
//            bottomSegments.push_back( spatial::math::LineSegment( bb.getCorner( spatial::math::BoundingBox::NEAR_RIGHT_BOTTOM ), bb.getCorner( spatial::math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
//            bottomSegments.push_back( spatial::math::LineSegment( bb.getCorner( spatial::math::BoundingBox::NEAR_LEFT_BOTTOM ), bb.getCorner( spatial::math::BoundingBox::FAR_LEFT_BOTTOM ) ) );


//            // TODO: change LocalSpaceMap to support object type on addObject
//            // and use the Entity FieldOfView function to test the visibility
//            meanValue = 0.0f;
//           
//            unsigned int i;
//            for ( i = 0; i < bottomSegments.size( ); ++i ) {
//                // at least on corner inside FOV
//                if ( petFieldOfView.isInside( bottomSegments[i].pointA ) ) { // pointB is the pointA of the next segment
//                    meanValue = 1.0f;
//                } // if
//            } // if
//            
            
//            { // setting Obviousness Frame
//                std::string level = meanValue > 0.7 ? "High" : meanValue > 0.3 ? "Medium" : "Low";
//                std::map<std::string, Handle> elements;
//                elements["Attribute"] = atomSpace.addNode( CONCEPT_NODE, "Visible" );
//                elements["Degree"] = atomSpace.addNode( CONCEPT_NODE, level );
//                elements["Phenomenon"] = atomSpace.addNode( SEME_NODE, entity );
//                elements["Perceiver"] = atomSpace.addNode( SEME_NODE, agentName );

//                HandleSeq agentLocation;
//                agentLocation.push_back( atomSpace.addNode( NUMBER_NODE,
//                    boost::lexical_cast<std::string>( agentEntity->getPosition( ).x ) ) );
//                agentLocation.push_back( atomSpace.addNode( NUMBER_NODE,
//                    boost::lexical_cast<std::string>( agentEntity->getPosition( ).y ) ) );
//                agentLocation.push_back( atomSpace.addNode( NUMBER_NODE,
//                    boost::lexical_cast<std::string>(agentEntity->getPosition( ).z ) ) );
    
//                elements["Location_of_protagonist"] = atomSpace.addLink( LIST_LINK, agentLocation );
//                AtomSpaceUtil::setPredicateFrameFromHandles(
//                   atomSpace, "#Obviousness", agentName + "_" + entity + "_inside_pet_fov",
//                      elements, SimpleTruthValue( meanValue, 1.0) );

//            } // end block
//            AtomSpaceUtil::setPredicateValue( atomSpace, "inside_pet_fov", SimpleTruthValue( meanValue, 1.0f ), pet, entityHandle );
//            logger().debug("PetPsychePredicatesUpdater - %s is inside pet fov? %s", entity.c_str( ), ( meanValue ? "y" : "n" ) );
//        } // if

//        bool isMovingToward = false;

//        if ( isMoving ) {
//            unsigned long timeBetweenTicks = timestamp - this->latestSimWorldTimestamp;
//            unsigned long timeDuringAgentLastAction = timeBetweenTicks * 2;

//            spatial::math::Vector3 velocity3D =
//                AtomSpaceUtil::getMostRecentObjectVelocity(atomSpace,
//                    entity,  timestamp - timeDuringAgentLastAction );

//            spatial::math::Vector3 velocity( velocity3D.x, velocity3D.y );
//            velocity.normalise( );

//            // retrieve pet and target metadata
//            const spatial::LocalSpaceMap2D& map = atomSpace.getSpaceServer().getLatestMap();
//            const spatial::EntityPtr& targetEntity = map.getEntity( entity );
//            //const spatial::Object& targetObject = map.getObject( entity );
//            const spatial::EntityPtr& agentEntity = map.getEntity( atomSpace.getName(pet) );
//            //const spatial::Object& petObject = map.getObject(atomSpace.getName(pet));

//            float distance = agentEntity->distanceTo( *targetEntity );

//            // verify if the target will possibly collide with pet if it continues it's current path
//            spatial::MovableEntityPtr futureTargetEntity( new spatial::MovableEntity( targetEntity->getId( ), targetEntity->getName( ),
//                    ( targetEntity->getPosition( ) + (velocity * distance * 2 ) ),
//                    targetEntity->getDimension( ), targetEntity->getOrientation( ),
//                    targetEntity->getExpansionRadius( ) ) );


//            if ( agentEntity->distanceTo( *futureTargetEntity ) < 0.00000001 ) {
//                isMovingToward = true;
//            } // if

//            //spatial::Vector3 futurePosition targetPath( targetCenter, (velocity * distance * 2 ) + petCenter );

//            
//            for ( const spatial::math::LineSegment& segment : petObject.borderSegments ) {
//            if ( spatial::getDistanceBetweenSegments( segment, targetPath ) < 0.00000001 ) {
//              isMovingToward = true;
//             } // if
//            } // foreach
//           
//        } // if

//        SimpleTruthValue movingTowardTV( 0.0f, 1.0f );
//        if (isMovingToward) {
//            movingTowardTV.setMean( 1.0 );
//        } // if

//        AtomSpaceUtil::setPredicateValue( atomSpace, "is_moving_toward", movingTowardTV, entityHandle, pet);
//        { // defining is_moving_toward Frame
//            std::map<std::string, Handle> elements;
//            elements["Theme"] = atomSpace.addNode( SEME_NODE, entity );
            
//            const spatial::LocalSpaceMap2D& map = atomSpace.getSpaceServer().getLatestMap();
//            const spatial::EntityPtr& targetEntity = map.getEntity( entity );
//            HandleSeq movementDirection;
//            movementDirection.push_back( atomSpace.addNode( NUMBER_NODE,
//                boost::lexical_cast<std::string>(targetEntity->getDirection( ).x ) ) );
//            movementDirection.push_back( atomSpace.addNode( NUMBER_NODE,
//                boost::lexical_cast<std::string>(targetEntity->getDirection( ).y ) ) );
//            movementDirection.push_back( atomSpace.addNode( NUMBER_NODE,
//                boost::lexical_cast<std::string>(targetEntity->getDirection( ).z ) ) );
//            elements["Direction"] = atomSpace.addLink( LIST_LINK, movementDirection );
//            elements["Goal"] = atomSpace.addNode( SEME_NODE, agentName );
//            elements["Path"] = atomSpace.addNode( CONCEPT_NODE, "Straightforward" );
            
//            AtomSpaceUtil::setPredicateFrameFromHandles(
//               atomSpace, "#Motion_directional", entity + "_" + agentName + "_is_moving_toward",
//                  elements, movingTowardTV );
//        } // end block


//        if ( atomSpace.getType(entityHandle) == AVATAR_NODE ) {
//            logger().debug("PetPsychePredicatesUpdater - entity %s is an avatar", entity.c_str( ) );

//            bool isOwner = AtomSpaceUtil::isPetOwner( atomSpace, entityHandle, pet );
//            bool isFriend = AtomSpaceUtil::isPredicateTrue( atomSpace, "friend", entityHandle, pet );
//            bool isEnemy = AtomSpaceUtil::isPredicateTrue( atomSpace, "enemy", entityHandle, pet );

//            if ( isNearEntity ) {
//                nearOwner |= isOwner;
//                nearFriend |= isFriend;
//                nearEnemy |= isEnemy;
//            } // if
//            if ( isNextEntity ) {
//                nextOwner |= isOwner;
//            } // if

//            logger().debug("PetPsychePredicatesUpdater - Entity %s properties: owner[%s] friend[%s] enemy[%s]", entity.c_str( ), ( isOwner ? "t" : "f" ), ( isFriend ? "t" : "f" ), ( isEnemy ? "t" : "f" ) );

//        } else if ( atomSpace.getType(entityHandle) == OBJECT_NODE ) {

//            logger().debug("PetPsychePredicatesUpdater - entity %s is an object", entity.c_str( ) );

//            bool isEdible = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_edible", entityHandle );
//            bool isDrinkable = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_drinkable", entityHandle );
//            bool isPooPlace = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_poo_place", entityHandle );
//            bool isPeePlace = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_pee_place", entityHandle );
//            if ( isNearEntity ) {
//                nearFood |= isEdible;
//                nearWater |= isDrinkable;
//                nearPooPlace |= isPooPlace;
//                nearPeePlace |= isPeePlace;
//            } // if

//            logger().debug("PetPsychePredicatesUpdater - Entity %s properties: edible[%s] drinkable[%s] pooPlace[%s] peePlace[%s]", entity.c_str( ), ( isEdible ? "t" : "f" ), ( isDrinkable ? "t" : "f" ), ( isPooPlace ? "t" : "f" ), ( isPeePlace ? "t" : "f" ) );

//            Handle conceptNodeHandle = atomSpace.getHandle( CONCEPT_NODE, "pet_home" );
//            if ( conceptNodeHandle == Handle::UNDEFINED ) {
//                continue;
//            } // if

//            HandleSeq seq;
//            seq.push_back(entityHandle);
//            seq.push_back(conceptNodeHandle);

//            Handle petHome = atomSpace.getHandle( INHERITANCE_LINK, seq );
//            if ( petHome == Handle::UNDEFINED ) {
//                continue;
//            } // if

//            atHome |= ( atomSpace.getMean(petHome) == 1.0f );

//        } // else if

//    } // foreach


//    tm timeInfo = PAIUtils::getTimeInfo( timestamp );

//    // night 19:00 -> 6:00
//    atNight = ( timeInfo.tm_hour > 19 || timeInfo.tm_hour < 6 );

//    logger().debug("PetPsychePredicatesUpdater - current time: day[%d] month[%d] year[%d] hour[%d] min[%d] sec[%d]", timeInfo.tm_mday, timeInfo.tm_mon + 1, timeInfo.tm_year + 1900, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec );

//    // setup predicates
//    meanValue = atHome ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "at_home", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = !atHome ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "outside", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearOwner ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_owner", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearFriend ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_friend", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearEnemy ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_enemy", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearFood ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_food", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearWater ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_water", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearPooPlace ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_poo_place", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nearPeePlace ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "near_pee_place", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = nextOwner ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "next_owner", SimpleTruthValue( meanValue, 1.0f ), pet );

//    meanValue = atNight ? 1.0f : 0.0f;
//    AtomSpaceUtil::setPredicateValue( atomSpace, "night", SimpleTruthValue( meanValue, 1.0f ), pet );

//    logger().debug("PetPsychePredicatesUpdater - pet is at home: %s", (atHome ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is outside home: %s", (!atHome ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is near owner: %s", (nearOwner ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is near friend: %s", (nearFriend ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is near enemy: %s", (nearEnemy ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is near food: %s", (nearFood ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is near water: %s", (nearWater ? "t" : "f" ) );

//    logger().debug("PetPsychePredicatesUpdater - pet is near poo place: %s", (nearPooPlace ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is near pee place: %s", (nearPeePlace ? "t" : "f" ) );
//    logger().debug("PetPsychePredicatesUpdater - pet is next to owner (Used on learning mode): %s", (nextOwner ? "t" : "f" ) );

//    logger().debug("PetPsychePredicatesUpdater - is at night: %s", (atNight ? "t" : "f" ) );

//    logger().info("PetPsychePredicatesUpdater - context predicates updated." );

//    this->latestSimWorldTimestamp = timestamp;

}

