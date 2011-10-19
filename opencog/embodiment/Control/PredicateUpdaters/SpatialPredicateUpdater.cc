/*
 * opencog/embodiment/Control/PredicateUpdaters/SpatialPredicateUpdater.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari Heljakka, Welter Luigi, Samir Araujo
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

#include "SpatialPredicateUpdater.h"
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/spatial/Entity.h>

using namespace opencog::oac;
using namespace opencog;
using namespace spatial;

SpatialPredicateUpdater::SpatialPredicateUpdater(AtomSpace &_atomSpace) :
        BasicPredicateUpdater(_atomSpace) {}

SpatialPredicateUpdater::~SpatialPredicateUpdater()
{
}

void SpatialPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp )
{
struct timeval timer_start, timer_end;
time_t elapsed_time = 0;
gettimeofday(&timer_start, NULL);

    // there is no map, no update is possible
    Handle spaceMapHandle = atomSpace.getSpaceServer().getLatestMapHandle();
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().warn( "SpatialPredicateUpdater::%s - No space map handle found!", __FUNCTION__);
        return;
    }
    const SpaceServer::SpaceMap& spaceMap = atomSpace.getSpaceServer().getLatestMap();

    logger().debug( "SpatialPredicateUpdater::%s - Processing timestamp '%lu'",
            __FUNCTION__, timestamp );
    if ( lastTimestamp != timestamp ) {
        lastTimestamp = timestamp;
        processedEntities.clear( );
    } // if

    std::vector<std::string> entities;
    spaceMap.findEntitiesWithClassFilter(back_inserter(entities), "block");

    const std::string& entityAId = atomSpace.getName( object );
    if ( processedEntities.find( entityAId ) != processedEntities.end( ) ) {
        return;
    } // if
    processedEntities.insert( entityAId );

    try {
        const spatial::EntityPtr& entityA = spaceMap.getEntity( entityAId );

        bool mapContainsEntity = spaceMap.containsObject( entityAId );

        // A more efficient version which only processes objects that are nearby in the grid.
        // Note: It won't remove previous links correctly (i.e. when objects move away from each other).
        // Could easily just remove all of them before the loop below (or give them 0 TV) and then replace them in the loop.
//        Point center = spaceMap.centerOf(entityAId);
//        GridPoint gridPoint = spaceMap.snap(center);
//        //! @todo WARNING: Since Near and Next are reversed, it'll be necessary to change this if you correct it.
//        //! Also the grid distance used is only an approximation
//        spatial::Distance distance = spaceMap.getNextDistance();
//        spatial::Distance gridDistance = distance/(spaceMap.xDim()+spaceMap.yDim());
//        spaceMap.findEntities(gridPoint, gridDistance, back_inserter(entities));
        logger().debug( "SpatialPredicateUpdater::%s - Nearby entities: %d",
                        __FUNCTION__, entities.size());

        for( unsigned int i = 0; i < entities.size( ); ++i ) {
            const std::string& entityBId = entities[i];
            if ( processedEntities.find( entityBId ) != processedEntities.end( ) ) {
                continue;
            } // if
            Handle entityBHandle = getHandle( entityBId );

            if ( !mapContainsEntity ) {
                logger().debug( "SpatialPredicateUpdater::%s - Removing predicates "
                        "from '%s' and '%s'", __FUNCTION__,
                        entityAId.c_str( ), entityBId.c_str( ) );
                setPredicate( object, entityBHandle, "near", 0.0f, timestamp);
                setPredicate( object, entityBHandle, "next", 0.0f, timestamp);
            } else {
                const spatial::EntityPtr& entityB = spaceMap.getEntity( entityBId );
                double distance = entityA->distanceTo( *entityB );
                logger().debug( "SpatialPredicateUpdater::%s - Adding predicates "
                        "for '%s' and '%s'. distance '%f'", __FUNCTION__,
                        entityAId.c_str( ), entityBId.c_str( ), distance );

                spatial::math::Vector3 minCorner( spaceMap.xMin( ), spaceMap.yMin( ) );
                spatial::math::Vector3 maxCorner( spaceMap.xMax( ), spaceMap.yMax( ) );

                double mapDiagonal = ( maxCorner - minCorner ).length( );

                double nearDistance = spaceMap.getNearDistance( );
                double nextDistance = spaceMap.getNextDistance( );

                logger().debug( "SpatialPredicateUpdater::%s - nearDistance '%f'",
                                __FUNCTION__, nearDistance );

                setPredicate( object, entityBHandle, "near",
                        ( distance < nearDistance ) ? 1.0 : 0.0f,
                        timestamp);
                setPredicate( object, entityBHandle, "next",
                        ( distance < nextDistance ) ? 1.0 - (distance/nextDistance) : 0.0f,
                        timestamp);

                float mean = 1.0 - (distance/mapDiagonal);
                setPredicate( object, entityBHandle, "proximity", mean,
                         timestamp);

            } // else
        } // for
    } catch( const opencog::NotFoundException& ex ) {
// Disable the log below, which prints almost useless info in log file. When the
// object is removed, it will fail to find the object. That is normal and happens  
// quite often for consumable objects, such as FoodCube
//        logger().error( "SpatialPredicateUpdater::%s - Entity not found '%s'",
//                        __FUNCTION__, ex.getMessage( ) );
    } // catch

gettimeofday(&timer_end, NULL);
elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
    (timer_end.tv_usec - timer_start.tv_usec);
timer_start = timer_end; 
logger().warn("SpatialPredicateUpdater::%s - process distance relations: consumed %f seconds", 
               __FUNCTION__, 
               1.0 * elapsed_time/1000000
             );

    computeAllSpatialRelations(pet, object, timestamp);

gettimeofday(&timer_end, NULL);
elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
    (timer_end.tv_usec - timer_start.tv_usec);
timer_start = timer_end; 
logger().warn("SpatialPredicateUpdater::%s - process other relations: consumed %f seconds", 
               __FUNCTION__, 
               1.0 * elapsed_time/1000000
             );
}

void addRelationsToAtomSpace(std::list<Entity::SPATIAL_RELATION> relations, string entityA_id, string entityB_id, string entityC_id, AtomSpace& atomSpace, unsigned int timestamp);

//! @todo Doesn't process 3-object relations (i.e. BETWEEN). Should use more filtering to restrict its search (e.g. based on the
//! spatial grid, and/or apriori knowledge about which things can happen (e.g. "A is between B and C" can only happen if
//! "B is left of A" and "C is right of A".) Responds only when an object has moved. (Should it also be every time the agent moves? -
//! these relations are evaluated from a certain perspective...)
void SpatialPredicateUpdater::computeAllSpatialRelations(Handle objectA, Handle observer, unsigned long timestamp)
{
    HandleSeq resultingFrames;

    Handle objectB;
    Handle objectC;

    // there is no map, no update is possible
    if (!atomSpace.getSpaceServer().isLatestMapValid()) {
        logger().warn( "%s - No space map handle found!", __FUNCTION__);
        return;
    }
    const SpaceServer::SpaceMap& spaceMap =
        atomSpace.getSpaceServer( ).getLatestMap( );

    double besideDistance = spaceMap.getNextDistance( );

    //std::vector<std::string> entitiesA;
    std::vector<std::string> entitiesB;
    std::vector<std::string> entitiesC;

    //spaceMap.getAllObjects( std::back_inserter( entitiesA ) );
    spaceMap.getAllObjects( std::back_inserter( entitiesB ) );
    spaceMap.getAllObjects( std::back_inserter( entitiesC ) );

    std::string entityID_A = atomSpace.getName(objectA);

    logger().debug( "%s - objectA: %s. %d candidates for objectB. %d candidates for objectC",
                    __FUNCTION__,  entityID_A.c_str(), entitiesB.size( ), entitiesC.size( ) );

    int numRelations = 0;

    try {
        const spatial::EntityPtr& observerEntity = spaceMap.getEntity( atomSpace.getName( observer ) );

        unsigned int j, k;
        const spatial::EntityPtr& entityA = spaceMap.getEntity( entityID_A );
        for( j = 0; j < entitiesB.size( ); ++j ) {
            if ( entityID_A == entitiesB[j] ) {
                continue;
            } // if
            const spatial::EntityPtr& entityB = spaceMap.getEntity( entitiesB[j] );
            std::list<Entity::SPATIAL_RELATION> relations;

            // All size-2 and all size-3 relations
            relations = entityA->computeSpatialRelations( *observerEntity, besideDistance, *entityB );
            addRelationsToAtomSpace(relations, entityID_A, entitiesB[j], "", atomSpace, timestamp);
            numRelations += relations.size();

            // todo: avoid redundant computation here.
//            for( k = 0; k < entitiesC.size( ); ++k ) {
//                if ( entityID_A == entitiesC[k] || entitiesB[j] == entitiesC[k] ) {
//                    continue;
//                } // if
//                const spatial::EntityPtr& entityC = spaceMap.getEntity( entitiesC[k] );
//                relations = entityA->computeSpatialRelations( *observerEntity, besideDistance, *entityB, *entityC );
//                addRelationsToAtomSpace(relations, entityID_A, entitiesB[j], entitiesC[k], atomSpace, timestamp);
//                numRelations += relations.size();
//            } // for
        } // for
    } catch( const opencog::NotFoundException& ex ) {
// Disable the log below, which prints almost useless info in log file. When the
// object is removed, it will fail to find the object. That is normal and happens  
// quite often for consumable objects, such as FoodCube
//        logger().error( "%s - %s", __FUNCTION__, ex.getMessage( ) );
        return;
    } // if

    logger().debug( "%s - Finished evaluating: %d spatial relations are true. %d combinations of this and 2 other objects",
            __FUNCTION__, numRelations, entitiesB.size()*entitiesC.size());

    return;
}

void addRelationsToAtomSpace(std::list<Entity::SPATIAL_RELATION> relations, std::string entityA_id, std::string entityB_id, std::string entityC_id, AtomSpace& atomSpace, unsigned int timestamp)
{
//    const SpaceServer::SpaceMap& spaceMap =
//        atomSpace.getSpaceServer().getLatestMap();
    HandleSeq justEntityA, justEntityB, justEntityC;
    atomSpace.getHandleSet(back_inserter(justEntityA), OBJECT_NODE, entityA_id, true);
    atomSpace.getHandleSet(back_inserter(justEntityB), OBJECT_NODE, entityB_id, true);

    Handle entityA, entityB, entityC;

    entityA = justEntityA[0];
    entityB = justEntityB[0];

    if (entityC_id.size()) {
        atomSpace.getHandleSet(back_inserter(justEntityC), OBJECT_NODE, entityC_id, true);
        entityC = justEntityC[0];
    } else {
        entityC == Handle::UNDEFINED;
    }

    SimpleTruthValue tv( 1, 1 );
    foreach (Entity::SPATIAL_RELATION rel, relations) {
        string predicateName = Entity::spatialRelationToString(rel);
        if (entityC == Handle::UNDEFINED) {
            Handle eval = AtomSpaceUtil::setPredicateValue( atomSpace, predicateName, tv, entityA, entityB );
            Handle atTime = atomSpace.getTimeServer().addTimeInfo(eval, timestamp);
            atomSpace.setTV(atTime, tv);
        } else {
            Handle eval = AtomSpaceUtil::setPredicateValue( atomSpace, predicateName, tv, entityA, entityB, entityC );
            Handle atTime = atomSpace.getTimeServer().addTimeInfo(eval, timestamp);
            atomSpace.setTV(atTime, tv);
        }
    }
}

void SpatialPredicateUpdater::setPredicate( const Handle& entityA, const Handle& entityB, const std::string& predicateName, float mean, unsigned long timestamp)
{
    // Don't record 0-strength relations
    if (mean == 0) return;
    SimpleTruthValue tv( mean, 1 );
    Handle direction1 = AtomSpaceUtil::setPredicateValue( atomSpace, predicateName, tv, entityA, entityB );
    Handle direction2 = AtomSpaceUtil::setPredicateValue( atomSpace, predicateName, tv, entityB, entityA );
    Handle atTime1 = atomSpace.getTimeServer().addTimeInfo(direction1, timestamp);
    Handle atTime2 = atomSpace.getTimeServer().addTimeInfo(direction2, timestamp);
    atomSpace.setTV(atTime1, tv);
    atomSpace.setTV(atTime2, tv);
}
