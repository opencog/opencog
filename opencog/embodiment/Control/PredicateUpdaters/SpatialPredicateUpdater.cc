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

using namespace opencog::oac;
using namespace opencog;
using namespace spatial;

SpatialPredicateUpdater::SpatialPredicateUpdater(AtomSpace & _atomSpace) :
        BasicPredicateUpdater(_atomSpace) {}

SpatialPredicateUpdater::~SpatialPredicateUpdater()
{
}

void SpatialPredicateUpdater::update(Handle object, Handle pet, unsigned long timestamp)
{
struct timeval timer_start, timer_end;
time_t elapsed_time = 0;
gettimeofday(&timer_start, NULL);

    // If there is no map, none update is possible
    Handle spaceMapHandle = atomSpace.getSpaceServer().getLatestMapHandle();
    if (spaceMapHandle == Handle::UNDEFINED) {
        logger().warn( "SpatialPredicateUpdater::%s - No space map handle found!", __FUNCTION__);
        return;
    }
    const SpaceServer::SpaceMap & spaceMap = atomSpace.getSpaceServer().getLatestMap();

    logger().debug( "SpatialPredicateUpdater::%s - Processing timestamp '%lu'",
            __FUNCTION__, timestamp );
    if ( lastTimestamp != timestamp ) {
        lastTimestamp = timestamp;
        processedEntities.clear( );
    } 

    // Get all the entities that are NOT blocks
    std::vector<std::string> entities;
    spaceMap.findEntitiesWithClassFilter(back_inserter(entities), "block");

    this->computeDistanceSpatialRelations(spaceMap, entities, object, timestamp); 

gettimeofday(&timer_end, NULL);
elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
    (timer_end.tv_usec - timer_start.tv_usec);
timer_start = timer_end; 
logger().warn("SpatialPredicateUpdater::%s - process distance relations: consumed %f seconds", 
               __FUNCTION__, 
               1.0 * elapsed_time/1000000
             );

    this->computeDirectionalSpatialRelations(spaceMap, entities, pet, object, timestamp);

gettimeofday(&timer_end, NULL);
elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
    (timer_end.tv_usec - timer_start.tv_usec);
timer_start = timer_end; 
logger().warn("SpatialPredicateUpdater::%s - process directional relations: consumed %f seconds", 
               __FUNCTION__, 
               1.0 * elapsed_time/1000000
             );
}

void SpatialPredicateUpdater::
computeDistanceSpatialRelations(const SpaceServer::SpaceMap & spaceMap,
                                std::vector <std::string> & entities,
                                Handle object, 
                                unsigned long timestamp
                               )
{
    // Check if the object has been processed before
    const std::string & entityAId = atomSpace.getName(object);
    if ( processedEntities.find(entityAId) != processedEntities.end() ) {
        return;
    } 
    processedEntities.insert(entityAId);

    // Get 'near' and 'next' distance threshold
    // Any two objects closer than the threshold would be considered 'near' or 
    // 'next' to each other 
    spatial::math::Vector3 minCorner( spaceMap.xMin(), spaceMap.yMin() );
    spatial::math::Vector3 maxCorner( spaceMap.xMax(), spaceMap.yMax() );

    double mapDiagonal = (maxCorner - minCorner).length();

    double nearDistance = spaceMap.getNearDistance();
    double nextDistance = spaceMap.getNextDistance();

    logger().debug("SpatialPredicateUpdater::%s - "
                   "nearDistance '%f', nextDistance '%f'",
                    __FUNCTION__, 
                    nearDistance, nextDistance 
                  );

    try {
        const spatial::EntityPtr & entityA = spaceMap.getEntity(entityAId);
        bool bMapContainsEntity = spaceMap.containsObject(entityAId);

        foreach (const std::string & entityBId, entities) {
            if ( processedEntities.find(entityBId) != processedEntities.end() ) {
                continue;
            } 

            Handle entityBHandle = getHandle(entityBId);

            if ( !bMapContainsEntity ) {
                logger().debug("SpatialPredicateUpdater::%s - Removing predicates "
                               "from '%s' and '%s'",
                               __FUNCTION__,
                               entityAId.c_str(), entityBId.c_str() 
                              );
                addSymmetricalRelation(object, entityBHandle, "near", 0.0f, timestamp);
                addSymmetricalRelation(object, entityBHandle, "next", 0.0f, timestamp);
            } 
            else {
                const spatial::EntityPtr & entityB = spaceMap.getEntity(entityBId);
                double distance = entityA->distanceTo(*entityB);
                logger().debug("SpatialPredicateUpdater::%s - Adding predicates "
                               "for '%s' and '%s'. distance '%f'", 
                               __FUNCTION__,
                               entityAId.c_str(), entityBId.c_str(), distance 
                              );

                // TODO: use fuzzy logic for setting the truth value
                //       check if the formulas for 'near' and 'next' are correct
                addSymmetricalRelation( object, entityBHandle, "near",
                                        (distance < nearDistance) ? 1.0 : 0.0f,
                                        timestamp
                                      );

                addSymmetricalRelation( object, entityBHandle, "next",
                                        (distance < nextDistance) ? 
                                            1.0 - distance/nextDistance : 0.0f,
                                        timestamp
                                      );

                addSymmetricalRelation( object, entityBHandle, "proximity",
                                        1.0 - distance/mapDiagonal,
                                        timestamp
                                      );

            }// if ( !bMapContainsEntity ) 
        }// for
    } 
    catch( const opencog::NotFoundException& ex ) {
        // Usually it is ok to just skip the exception, because when the object 
        // is removed, it will fail to find the object. That is normal and happens  
        // quite often for consumable objects, such as FoodCube
    }// try
}

void addRelationsToAtomSpace(std::list<Entity::SPATIAL_RELATION> relations, 
                             string entityA_id, string entityB_id, string entityC_id,
                             AtomSpace& atomSpace,
                             unsigned int timestamp
                            );

//@todo We could make this function faster via approaches below (for the moment
//      it is fast enough in our simple block world).
//
//      1. Use more filtering to restrict its search (e.g. based on the
//         spatial grid, and/or apriori knowledge about which things can happen 
//         (e.g. "A is between B and C" can only happen if "B is left of A" and 
//         "C is right of A")
//      2. Responds only when an object has moved. (Should it also be every time
//         the agent moves? - these relations are evaluated from a certain 
//         perspective...)
//
void SpatialPredicateUpdater::
computeDirectionalSpatialRelations(const SpaceServer::SpaceMap & spaceMap,
                                   std::vector <std::string> & entities,
                                   Handle objectA,
                                   Handle observer,
                                   unsigned long timestamp
                                  )
{
    HandleSeq resultingFrames;

    Handle objectB;
    Handle objectC;

    double besideDistance = spaceMap.getNextDistance();
    std::string entityID_A = atomSpace.getName(objectA);

    int numRelations = 0;

    try {
        const spatial::EntityPtr & observerEntity =
            spaceMap.getEntity( atomSpace.getName(observer) );

        const spatial::EntityPtr & entityA = spaceMap.getEntity(entityID_A);

        foreach (const std::string & entityID_B, entities) {
            if (entityID_A == entityID_B) {
                continue;
            } 

            const spatial::EntityPtr & entityB = spaceMap.getEntity(entityID_B);
            std::list<Entity::SPATIAL_RELATION> relations;

            // Compute size-2 directional spatial relations
            relations = entityA->computeSpatialRelations( *observerEntity, besideDistance, *entityB );
            addRelationsToAtomSpace(relations, entityID_A, entityID_B, "", atomSpace, timestamp);
            numRelations += relations.size();

            // Compute size-3 directional spatial relations
            // TODO: avoid redundant computation here.
            foreach (const std::string & entityID_C, entities) {
                if ( entityID_A == entityID_C || entityID_B == entityID_C ) {
                    continue;
                } 

                const spatial::EntityPtr & entityC = spaceMap.getEntity(entityID_C);
                relations = entityA->computeSpatialRelations(*observerEntity, 
                                                             besideDistance,
                                                             *entityB,
                                                             *entityC 
                                                            );
                this->addRelationsToAtomSpace(relations, 
                                              entityID_A, entityID_B, entityID_C, 
                                              atomSpace, timestamp
                                             );

                numRelations += relations.size();

            }// foreach (const std::string & entityID_C, entities)

        }// foreach (const std::string & entityID_B, entities)
    } 
    catch( const opencog::NotFoundException & ex ) {
        // Usually it is ok to just skip the exception, because when the object 
        // is removed, it will fail to find the object. That is normal and happens  
        // quite often for consumable objects, such as FoodCube
        return;
    }// try

    logger().debug("%s - Finished evaluating: %d spatial relations are true.", 
                   __FUNCTION__, numRelations
                  );

    return;
}

void SpatialPredicateUpdater::
addRelationsToAtomSpace(std::list<Entity::SPATIAL_RELATION> & relations, 
                        std::string entityA_id, 
                        std::string entityB_id, 
                        std::string entityC_id, 
                        AtomSpace & atomSpace, unsigned long timestamp)
{
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

    SimpleTruthValue tv(1, 1);

    foreach (Entity::SPATIAL_RELATION rel, relations) {
        string predicateName = Entity::spatialRelationToString(rel);
        if (entityC == Handle::UNDEFINED) {
            Handle eval = AtomSpaceUtil::setPredicateValue(atomSpace,
                                                           predicateName, 
                                                           tv,
                                                           entityA, entityB 
                                                          );
            Handle atTime = atomSpace.getTimeServer().addTimeInfo(eval, timestamp);
            atomSpace.setTV(atTime, tv);
        } 
        else {
            Handle eval = AtomSpaceUtil::setPredicateValue(atomSpace,
                                                           predicateName, 
                                                           tv, 
                                                           entityA, entityB, entityC
                                                          );
            Handle atTime = atomSpace.getTimeServer().addTimeInfo(eval, timestamp);
            atomSpace.setTV(atTime, tv);
        }
    }
}

void SpatialPredicateUpdater::
addSymmetricalRelation(const Handle & entityA, const Handle & entityB, 
                       const std::string& predicateName, 
                       float mean, unsigned long timestamp
                      )
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

