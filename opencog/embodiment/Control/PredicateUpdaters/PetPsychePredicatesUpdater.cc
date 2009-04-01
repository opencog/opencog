/*
 * opencog/embodiment/Control/PredicateUpdaters/PetPsychePredicatesUpdater.cc
 *
 * Copyright (C) 2007-2008 Samir Araujo
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

#include <opencog/atomspace/SimpleTruthValue.h>

#include "PetPsychePredicatesUpdater.h"
#include "AtomSpaceUtil.h"
#include "PAIUtils.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <opencog/spatial/MovableEntity.h>


using namespace OperationalPetController;
using namespace opencog;

PetPsychePredicatesUpdater::PetPsychePredicatesUpdater(SpaceServer &_spaceServer ) : BasicPredicateUpdater(_spaceServer.getAtomSpace()), spaceServer(_spaceServer)
	{

	this->latestSimWorldTimestamp = 0L;
}

PetPsychePredicatesUpdater::~PetPsychePredicatesUpdater(){
}

Spatial::Math::Triangle PetPsychePredicatesUpdater::createFieldOfViewTriangle(Handle agent) {
  const SpaceServer::SpaceMap& spaceMap = spaceServer.getLatestMap( );

  const Spatial::EntityPtr& entity = spaceMap.getEntity( atomSpace.getName(agent) );


//  const SpaceServer::ObjectMetadata& agentMetaData = 
//    spaceMap.getMetaData( atomSpace.getName(agent) );
  SpaceServer::SpaceMapPoint agentCenter( entity->getPosition( ).x, entity->getPosition( ).y );
  
  // get the object's direction vector (where it's face is pointing)
  Spatial::Math::Vector3 agentFovDirection = entity->getDirection( );//( std::cos(agentMetaData.yaw), std::sin(agentMetaData.yaw) );
  
  // agent fov will reach 30% of the map width
  float agentFovReach = fabs( spaceMap.xMax( ) - spaceMap.xMin( ) ) * 0.3;
  
  Spatial::Math::Vector3 maximumPoint = agentFovDirection * agentFovReach;

  Spatial::Math::Line limitLine( Spatial::Math::Quaternion( Spatial::Math::Vector3::Z_UNIT, M_PI/2).rotate( agentFovDirection ) + maximumPoint, Spatial::Math::Quaternion( Spatial::Math::Vector3::Z_UNIT, -M_PI/2).rotate( agentFovDirection ) + maximumPoint );

  // agent fov will be 160 degrees
  Spatial::Math::Vector3 leftPoint( Spatial::Math::Quaternion( Spatial::Math::Vector3::Z_UNIT, 80*M_PI/180 ).rotate( agentFovDirection ) * agentFovReach );

  Spatial::Math::Vector3 rightPoint( Spatial::Math::Quaternion( Spatial::Math::Vector3::Z_UNIT, -80*M_PI/180 ).rotate( agentFovDirection ) * agentFovReach );

  Spatial::Math::Vector3 intersectionPoint; 

  Spatial::Math::LineSegment leftLine( Spatial::Math::Vector3( 0,0 ), leftPoint );
  if ( !limitLine.intersects( leftLine, &intersectionPoint ) ) {
    logger().log(opencog::Logger::ERROR, "PetPsychePredicatesUpdater - Lines should intersect limitLine(%s) leftLine(%s)", limitLine.toString( ).c_str( ), leftLine.toString( ).c_str( ) );
  } // if
  leftPoint = intersectionPoint;
  
  Spatial::Math::Line rightLine( Spatial::Math::Vector3( 0,0 ), rightPoint );
  if ( !limitLine.intersects( rightLine, &intersectionPoint ) ) {
    logger().log(opencog::Logger::ERROR, "PetPsychePredicatesUpdater - Lines should intersect limitLine(%s) rightLine(%s) fovDirection(%s) reach(%f) maxPoint(%s)", limitLine.toString( ).c_str( ), rightLine.toString( ).c_str( ), agentFovDirection.toString( ).c_str( ), agentFovReach, maximumPoint.toString( ).c_str( ) );
  } // if
  rightPoint = intersectionPoint;

  /*
  if ( !Spatial::Math::lineIntersection( limitLine, Spatial::Math::LineSegment( Spatial::Math::Vector3( 0,0 ), leftPoint ), intersectionPoint ) ) {
    logger().log(opencog::Logger::ERROR, "PetPsychePredicatesUpdater - Line intersection cannot be null lineA(%f,%f - %f, %f) lineB(%f,%f - %f,%f).", limitLine.pointA.x, limitLine.pointA.y, limitLine.pointB.x, limitLine.pointB.y, 0, 0, leftPoint.x, leftPoint.y );
  } // if
  

  leftPoint = intersectionPoint;
  if ( !lineIntersection( limitLine, Spatial::Math::LineSegment( Spatial::Math::Vector3( 0,0 ), rightPoint ), intersectionPoint ) ) {
    logger().log(opencog::Logger::ERROR, "PetPsychePredicatesUpdater - Line intersection cannot be null lineA(%f,%f - %f, %f) lineB(%f,%f - %f,%f).", limitLine.pointA.x, limitLine.pointA.y, limitLine.pointB.x, limitLine.pointB.y, 0, 0, rightPoint.x, rightPoint.y );
  } // if
  rightPoint = intersectionPoint;
  */



  // translate the points from 0,0 to the current agent position
  Spatial::Math::Vector3 agentPosition( agentCenter.first, agentCenter.second );
  leftPoint += agentPosition;
  rightPoint += agentPosition;

  return Spatial::Math::Triangle( agentPosition, leftPoint, rightPoint );  

}

void PetPsychePredicatesUpdater::update(Handle object, Handle pet, unsigned long timestamp ) {
  logger().log(opencog::Logger::INFO, "PetPsychePredicatesUpdater - updating context predicates..." );
  
  if ( pet == Handle::UNDEFINED ) {
    logger().log(opencog::Logger::ERROR, "PetPsychePredicatesUpdater - Got an undefined handle for the pet." );    
    return;
  } // if
 
  // there is no map, no update is possible
  Handle spaceMapHandle = spaceServer.getLatestMapHandle();
  if (spaceMapHandle == Handle::UNDEFINED) {
    logger().log(opencog::Logger::INFO, "PetPsychePredicatesUpdater - there is no space map defined at this moment..." );
    return;
  } // if

  const SpaceServer::SpaceMap& spaceMap = spaceServer.getLatestMap();
    
  const std::string& petName = atomSpace.getName(pet);
  if ( !spaceMap.containsObject(petName) ) {
    logger().log(opencog::Logger::ERROR, "PetPsychePredicatesUpdater - Pet was not inserted in the map yet." );    
    return;
  }

  vector<std::string> entities; 
  spaceMap.findAllEntities( back_inserter(entities) );
  float meanValue;    
  bool atHome = false;
  bool atNight = false;

  bool nearFriend = false;
  bool nearEnemy = false;
  bool nearOwner = false;
  bool nearFood = false;
  bool nearWater = false;
  bool nearPooPlace = false;
  bool nearPeePlace = false;
  // this predicate is needed by learning mode. It means near, but not much
  bool nextOwner = false;
  
  const Spatial::EntityPtr& agentEntity = spaceMap.getEntity( petName );

  SpaceServer::SpaceMapPoint petCenter( agentEntity->getPosition( ).x, agentEntity->getPosition( ).y );

  bool needsArtificialFOV = ( atomSpace.getType(pet) == SL_PET_NODE );

  Spatial::Math::Triangle petFieldOfView;
  if ( needsArtificialFOV ) {
    petFieldOfView = createFieldOfViewTriangle(pet);
  } // if
      
  foreach( std::string entity, entities ) {
    logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - inspecting entity %s", entity.c_str( ) );
    Handle entityHandle = getHandle(entity);
    if( entityHandle == Handle::UNDEFINED ) {
      logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - there is no entity %s defined at atomspace", entity.c_str( ) );
      continue;
    } // if
    
    std::string elementId = atomSpace.getName( entityHandle );
    //const SpaceServer::ObjectMetadata& elementMetaData = 
    //  spaceMap.getMetaData( elementId );
    const Spatial::EntityPtr& objectEntity = spaceMap.getEntity( elementId );
    SpaceServer::SpaceMapPoint elementCenter( objectEntity->getPosition( ).x, objectEntity->getPosition( ).y );

    { // log pet and element positions
      std::stringstream message;
      message << "PetPsychePredicatesUpdater - ";
      message << "pet pos(" << petCenter.first << " " << petCenter.second << ") ";
      message << "pet yaw(rad: " << agentEntity->getOrientation( ).getRoll( );
      message << " deg: " << ( agentEntity->getOrientation( ).getRoll( ) * 180/M_PI) << ")"; 
      message << "element pos(" << elementCenter.first << " " << elementCenter.second << ") ";
      message << "dist(" << SpaceServer::SpaceMap::eucDist( petCenter, elementCenter ) << ")";
      logger().log(opencog::Logger::DEBUG, message.str( ).c_str( ) );
    } // end block
    

    bool isNearEntity = AtomSpaceUtil::isPredicateTrue(atomSpace, "near", entityHandle, pet );
    bool isNextEntity = AtomSpaceUtil::isPredicateTrue(atomSpace, "next", entityHandle, pet );
    bool isMoving = AtomSpaceUtil::isPredicateTrue(atomSpace, "is_moving", entityHandle );

    //const Spatial::Object& elementObject = spaceMap.getObject( elementId );
    

    const Spatial::EntityPtr& elementEntity = spaceMap.getEntity( elementId );
    const Spatial::Math::BoundingBox& bb = elementEntity->getBoundingBox( );
    if ( needsArtificialFOV ) {

      std::vector<Spatial::Math::LineSegment> bottomSegments;
      bottomSegments.push_back( Spatial::Math::LineSegment( bb.getCorner( Spatial::Math::BoundingBox::FAR_LEFT_BOTTOM ), bb.getCorner( Spatial::Math::BoundingBox::FAR_RIGHT_BOTTOM ) ) );
      bottomSegments.push_back( Spatial::Math::LineSegment( bb.getCorner( Spatial::Math::BoundingBox::FAR_RIGHT_BOTTOM ), bb.getCorner( Spatial::Math::BoundingBox::NEAR_RIGHT_BOTTOM ) ) );
      bottomSegments.push_back( Spatial::Math::LineSegment( bb.getCorner( Spatial::Math::BoundingBox::NEAR_RIGHT_BOTTOM ), bb.getCorner( Spatial::Math::BoundingBox::NEAR_LEFT_BOTTOM ) ) );
      bottomSegments.push_back( Spatial::Math::LineSegment( bb.getCorner( Spatial::Math::BoundingBox::NEAR_LEFT_BOTTOM ), bb.getCorner( Spatial::Math::BoundingBox::FAR_LEFT_BOTTOM ) ) );
      

      // TODO: change LocalSpaceMap to support object type on addObject
      // and use the Entity FieldOfView function to test the visibility
      meanValue = 0.0f;
      unsigned int i;
      for( i = 0; i < bottomSegments.size( ); ++i ) {
	// at least on corner inside FOV
	if ( petFieldOfView.isInside( bottomSegments[i].pointA ) ){ // pointB is the pointA of the next segment
	  meanValue = 1.0f;
	} // if
      } // if
      
      AtomSpaceUtil::setPredicateValue( atomSpace, "inside_pet_fov", SimpleTruthValue( meanValue, 1.0f ), pet, entityHandle );
      logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - %s is inside pet fov? %s", entity.c_str( ), ( meanValue ? "y" : "n" ) );
    } // if

    bool isMovingToward = false;
    
    if ( isMoving ) {
      unsigned long timeBetweenTicks = timestamp - this->latestSimWorldTimestamp;
      unsigned long timeDuringAgentLastAction = timeBetweenTicks * 2;
      
      Spatial::Math::Vector3 velocity3D = AtomSpaceUtil::getMostRecentObjectVelocity(atomSpace, entity,  timestamp - timeDuringAgentLastAction );
      Spatial::Math::Vector3 velocity( velocity3D.x, velocity3D.y );
      velocity.normalise( );

      // retrieve pet and target metadata
      const Spatial::LocalSpaceMap2D& map = spaceServer.getLatestMap();
      const Spatial::EntityPtr& targetEntity = map.getEntity( entity );
      //const Spatial::Object& targetObject = map.getObject( entity );
      const Spatial::EntityPtr& agentEntity = map.getEntity( atomSpace.getName(pet) );
      //const Spatial::Object& petObject = map.getObject(atomSpace.getName(pet));
      
      float distance = agentEntity->distanceTo( targetEntity );

      // verify if the target will possibly collide with pet if it continues it's current path
      Spatial::MovableEntityPtr futureTargetEntity( new Spatial::MovableEntity( targetEntity->getId( ), targetEntity->getName( ), 
						 ( targetEntity->getPosition( ) + (velocity * distance * 2 ) ),
						 targetEntity->getDimension( ), targetEntity->getOrientation( ), 
						 targetEntity->getExpansionRadius( ) ) );


      if ( agentEntity->distanceTo( futureTargetEntity ) < 0.00000001 ) {
	isMovingToward = true;
      } // if

      //Spatial::Vector3 futurePosition targetPath( targetCenter, (velocity * distance * 2 ) + petCenter );

      /*
      foreach( const Spatial::Math::LineSegment& segment, petObject.borderSegments ) {
	if ( Spatial::getDistanceBetweenSegments( segment, targetPath ) < 0.00000001 ) {
    		  isMovingToward = true;
    	  } // if
      } // foreach
      */
    } // if
    
    if(isMovingToward){
    	AtomSpaceUtil::setPredicateValue( atomSpace, "is_moving_toward", SimpleTruthValue( 1.0f, 1.0f ), entityHandle, pet);    	
    } else {
    	AtomSpaceUtil::setPredicateValue( atomSpace, "is_moving_toward", SimpleTruthValue( 0.0f, 1.0f ), entityHandle, pet);
    }    
    
    if ( atomSpace.getType(entityHandle) == SL_AVATAR_NODE ) {      
      logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - entity %s is an avatar", entity.c_str( ) );
      
      bool isOwner = AtomSpaceUtil::isPetOwner( atomSpace, entityHandle, pet );
      bool isFriend = AtomSpaceUtil::isPredicateTrue( atomSpace, "friend", entityHandle, pet );
      bool isEnemy = AtomSpaceUtil::isPredicateTrue( atomSpace, "enemy", entityHandle, pet );

      if ( isNearEntity ) {
	nearOwner |= isOwner;
	nearFriend |= isFriend;
	nearEnemy |= isEnemy;
      } // if
      if ( isNextEntity ) {
	nextOwner |= isOwner;
      } // if      

      logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - Entity %s properties: owner[%s] friend[%s] enemy[%s]", entity.c_str( ), ( isOwner ? "t" : "f" ), ( isFriend ? "t" : "f" ), ( isEnemy ? "t" : "f" ) );

    } else if ( atomSpace.getType(entityHandle) == SL_OBJECT_NODE ) {

      logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - entity %s is an object", entity.c_str( ) );

      bool isEdible = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_edible", entityHandle );
      bool isDrinkable = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_drinkable", entityHandle );
      bool isPooPlace = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_poo_place", entityHandle );
      bool isPeePlace = AtomSpaceUtil::isPredicateTrue( atomSpace, "is_pee_place", entityHandle );
      if ( isNearEntity ) {
	nearFood |= isEdible;
	nearWater |= isDrinkable;
	nearPooPlace |= isPooPlace;
	nearPeePlace |= isPeePlace;
      } // if

      logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - Entity %s properties: edible[%s] drinkable[%s] pooPlace[%s] peePlace[%s]", entity.c_str( ), ( isEdible ? "t" : "f" ), ( isDrinkable ? "t" : "f" ), ( isPooPlace ? "t" : "f" ), ( isPeePlace ? "t" : "f" ) );

      Handle conceptNodeHandle = atomSpace.getHandle( CONCEPT_NODE, "pet_home" );
      if( conceptNodeHandle == Handle::UNDEFINED ) {
	continue;
      } // if
      
      HandleSeq seq;
      seq.push_back(entityHandle);
      seq.push_back(conceptNodeHandle);
      
      Handle petHome = atomSpace.getHandle( INHERITANCE_LINK, seq );
      if( petHome == Handle::UNDEFINED ) {
	continue;
      } // if
      
      atHome |= ( atomSpace.getTV(petHome).getMean() == 1.0f );

    } // else if

  } // foreach

  
  tm timeInfo = PerceptionActionInterface::PAIUtils::getTimeInfo( timestamp );

  // night 19:00 -> 6:00
  atNight = ( timeInfo.tm_hour > 19 || timeInfo.tm_hour < 6 );

  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - current time: day[%d] month[%d] year[%d] hour[%d] min[%d] sec[%d]", timeInfo.tm_mday, timeInfo.tm_mon+1, timeInfo.tm_year+1900, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec );  

  // setup predicates  
  
  meanValue = atHome ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "home", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = !atHome ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "outside", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearOwner ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_owner", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearFriend ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_friend", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearEnemy ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_enemy", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearFood ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_food", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearWater ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_water", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearPooPlace ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_poo_place", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nearPeePlace ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "near_pee_place", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = nextOwner ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "next_owner", SimpleTruthValue( meanValue, 1.0f ), pet );

  meanValue = atNight ? 1.0f : 0.0f;
  AtomSpaceUtil::setPredicateValue( atomSpace, "night", SimpleTruthValue( meanValue, 1.0f ), pet );

  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is at home: %s", (atHome ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is outside home: %s", (!atHome ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near owner: %s", (nearOwner ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near friend: %s", (nearFriend ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near enemy: %s", (nearEnemy ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near food: %s", (nearFood ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near water: %s", (nearWater ? "t" : "f" ) );

  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near poo place: %s", (nearPooPlace ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is near pee place: %s", (nearPeePlace ? "t" : "f" ) );
  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - pet is next to owner (Used on learning mode): %s", (nextOwner ? "t" : "f" ) );

  logger().log(opencog::Logger::DEBUG, "PetPsychePredicatesUpdater - is at night: %s", (atNight ? "t" : "f" ) );

  logger().log(opencog::Logger::INFO, "PetPsychePredicatesUpdater - context predicates updated." );

  this->latestSimWorldTimestamp = timestamp;

}

