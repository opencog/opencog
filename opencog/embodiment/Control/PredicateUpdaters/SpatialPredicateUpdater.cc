/*
 * opencog/embodiment/Control/PredicateUpdaters/SpatialPredicateUpdater.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Ari Heljakka, Welter Luigi, Samir Araujo
 *
 * Updated: by Jinhua Chua, on 2011-12-05
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

void SpatialPredicateUpdater::update(std::vector<Handle> & objects,
                                     Handle pet, 
                                     unsigned long timestamp
                                    )
{
    /*
//struct timeval timer_start, timer_end;
//time_t elapsed_time = 0;
//gettimeofday(&timer_start, NULL);

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
        this->spatialRelationCache.clear(); 
    } 

    // Get all the entities that are NOT blocks
    std::vector<std::string> entities;
    spaceMap.findEntitiesWithClassFilter(back_inserter(entities), "block");

    this->compute2SizeSpatialRelations(spaceMap, objects, entities, pet, timestamp); 

    // TODO: Extremely slow. Use better algorithm later. 
//    this->computeObserverInvolvedSpatialRelations(spaceMap, entities, pet, timestamp); 

//gettimeofday(&timer_end, NULL);
//elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
//    (timer_end.tv_usec - timer_start.tv_usec);
//timer_start = timer_end; 
//logger().warn("SpatialPredicateUpdater::%s - process 2-size spatial relations: consumed %f seconds", 
//               __FUNCTION__, 
//               1.0 * elapsed_time/1000000
//             );

    this->compute3SizeSpatialRelations(spaceMap, objects, entities, pet, timestamp); 

//gettimeofday(&timer_end, NULL);
//elapsed_time += ((timer_end.tv_sec - timer_start.tv_sec) * 1000000) +
//    (timer_end.tv_usec - timer_start.tv_usec);
//timer_start = timer_end; 
//logger().warn("SpatialPredicateUpdater::%s - process 3-size spatial relations: consumed %f seconds", 
//               __FUNCTION__, 
//               1.0 * elapsed_time/1000000
//             );

*/
}

SpatialPredicateUpdater::SPATIAL_RELATION_VECTOR SpatialPredicateUpdater::
swapRelations(SPATIAL_RELATION_VECTOR & relations)
{
    SPATIAL_RELATION_VECTOR newRelations; 

    foreach (Entity::SPATIAL_RELATION rel, relations) {
        switch (rel) {
            case Entity::LEFT_OF:     newRelations.push_back(Entity::RIGHT_OF);    break; 
            case Entity::RIGHT_OF:    newRelations.push_back(Entity::LEFT_OF);     break; 
            case Entity::ABOVE:       newRelations.push_back(Entity::BELOW);       break; 
            case Entity::BELOW:       newRelations.push_back(Entity::ABOVE);       break; 
            case Entity::BEHIND:      newRelations.push_back(Entity::IN_FRONT_OF); break; 
            case Entity::IN_FRONT_OF: newRelations.push_back(Entity::BEHIND);      break; 
            case Entity::BESIDE:      newRelations.push_back(Entity::BESIDE);      break; 
            case Entity::NEAR:        newRelations.push_back(Entity::NEAR);        break; 
            case Entity::FAR_:        newRelations.push_back(Entity::FAR_);        break;
            case Entity::TOUCHING:    newRelations.push_back(Entity::TOUCHING);    break; 
            case Entity::INSIDE:      newRelations.push_back(Entity::OUTSIDE);     break; 
            case Entity::OUTSIDE:     newRelations.push_back(Entity::INSIDE);      break; 
            case Entity::ADJACENT:    newRelations.push_back(Entity::ADJACENT);    break; 
            default: break;                                       
        } // switch (rel)
    } // foreach (Entity::SPATIAL_RELATION rel, relations) 

    return newRelations; 
}

void SpatialPredicateUpdater::
computeObserverInvolvedSpatialRelations(const SpaceServer::SpaceMap & spaceMap, 
                                        std::vector <std::string> & entities, 
                                        Handle observer, 
                                        unsigned long timestamp
                                       )
{/*
    const std::string & observerName = atomSpace.getName(observer); 
    const spatial::EntityPtr & observerEntity = spaceMap.getEntity(observer);
    int numRelations = 0;

    foreach (const std::string & entityID_B, entities) {
        const spatial::EntityPtr & entityB = spaceMap.getEntity(entityID_B);
        Handle objectB = getHandle(entityID_B); 

        if (observer == objectB) {
            continue;
        } 

        SPATIAL_RELATION_VECTOR relationsOB, relationsBO;

        const math::Vector3 & observerPosition = observerEntity->getPosition();
        const math::Vector3 & objectPosition = entityB->getPosition();  
        math::Vector3 objectDirection = objectPosition - observerPosition;
        math::Vector3 observerDirection = (observerEntity->getDirection() * objectDirection.length()+1.0);

        // Angle from observerDirection (pet facing direction) to objectDirection (pet to to B)
        double angle; 
        angle = std::atan2( objectDirection.y, objectDirection.x ) - 
                std::atan2( observerDirection.y, observerDirection.x );

        if ( angle > M_PI ) {
            angle -= M_PI*2.0;
        } 
        else if ( angle < -M_PI ) {
            angle += M_PI*2.0;
        }

        angle *= 180.0/M_PI;

        // Compute size-2 directional spatial relations (observer is the reference object)
        if (angle >= -45 && angle <= 45) 
            relationsBO.push_back(Entity::IN_FRONT_OF); 
        else if (angle >= 45 && angle <= 135)
            relationsBO.push_back(Entity::LEFT_OF);
        else if (angle >= -135 && angle <= -45)
            relationsBO.push_back(Entity::RIGHT_OF);
        else
            relationsBO.push_back(Entity::BEHIND); 

        if ( objectPosition.z - entityB->getHeight()*0.5 > observerPosition.z + observerEntity->getHeight() ) 
            relationsBO.push_back(Entity::ABOVE); 

        relationsOB = this->swapRelations(relationsBO); 

        this->addSpatialRelations(relationsOB, atomSpace, timestamp, observer, objectB); 
        this->addSpatialRelations(relationsBO, atomSpace, timestamp, objectB, observer); 

        this->spatialRelationCache.addRelation(observerName, entityID_B, relationsOB); 
        this->spatialRelationCache.addRelation(entityID_B, observerName, relationsBO); 

        numRelations ++; 
    } // foreach (const std::string & entityID_B, entities)

    logger().debug("%s - Finished evaluating: %d observer involved spatial relations", 
                   __FUNCTION__, numRelations
                  );
                  */
}

void SpatialPredicateUpdater::
compute2SizeSpatialRelations(const SpaceServer::SpaceMap & spaceMap, 
                             std::vector<Handle> & objects, 
                             std::vector <std::string> & entities, 
                             Handle observer, 
                             unsigned long timestamp
                            )
{
    /*
    double besideDistance = spaceMap.getNextDistance();

    try {
        const spatial::EntityPtr & observerEntity = 
            spaceMap.getEntity( atomSpace.getName(observer) );

        foreach (Handle objectA, objects) {
            int numRelations = 0;
            std::string entityID_A = atomSpace.getName(objectA); 

            const spatial::EntityPtr & entityA = spaceMap.getEntity(entityID_A);

            foreach (const std::string & entityID_B, entities) {
                if ( entityID_A == entityID_B ||
                     this->spatialRelationCache.isCached(entityID_A, entityID_B) ) {
                    continue;
                } 

                const spatial::EntityPtr & entityB = spaceMap.getEntity(entityID_B);
                Handle objectB = getHandle(entityID_B); 
                SPATIAL_RELATION_VECTOR relationsAB, relationsBA;

                // Compute size-2 directional spatial relations (B is the reference object)
                relationsAB = entityA->computeSpatialRelations( *observerEntity, besideDistance, *entityB );
                relationsBA = this->swapRelations(relationsAB); 

                this->addSpatialRelations(relationsAB, atomSpace, timestamp, objectA, objectB); 
                this->addSpatialRelations(relationsBA, atomSpace, timestamp, objectB, objectA); 

                this->spatialRelationCache.addRelation(entityID_A, entityID_B, relationsAB); 
                this->spatialRelationCache.addRelation(entityID_B, entityID_A, relationsBA); 

                numRelations += relationsAB.size();

            } // foreach (const std::string & entityID_B, entities)

            logger().debug("%s - Finished evaluating: %d 2-size spatial relations related to '%s'", 
                           __FUNCTION__, numRelations, entityID_A.c_str()
                          );

        } // foreach (Handle objectA, objects)
    } 
    catch( const opencog::NotFoundException & ex ) {
        // Usually it is ok to just skip the exception, because when the object 
        // is removed, it will fail to find the object. That is normal and happens  
        // quite often for consumable objects, such as FoodCube
        return;
    } // try
    */
}

void SpatialPredicateUpdater::
compute3SizeSpatialRelations(const SpaceServer::SpaceMap & spaceMap, 
                             std::vector<Handle> & objects, 
                             std::vector <std::string> & entities, 
                             Handle observer, 
                             unsigned long timestamp
                            )
{/*
    spaceMap.getNextDistance();

    try
    {
        spaceMap.getEntity( atomSpace.getName(observer) );

        std::vector <std::string>::iterator iter_entityB, iter_entityC; 

        foreach (Handle objectA, objects)
        {
            std::string entityID_A = atomSpace.getName(objectA); 
            int numRelations = 0;

            spaceMap.getEntity(entityID_A);

            for (iter_entityB = entities.begin(); iter_entityB != entities.end(); ++ iter_entityB) {
                const std::string & entityID_B = *iter_entityB; 

                if ( entityID_A == entityID_B )
                    continue; 

                 for (iter_entityC = iter_entityB; iter_entityC != entities.end(); ++ iter_entityC) {
                    const std::string & entityID_C = * iter_entityC; 

                    if ( entityID_C == entityID_A || entityID_C == entityID_B )
                        continue;

                    SPATIAL_RELATION_VECTOR relationsAB, relationsAC; 

                    if ( !spatialRelationCache.getRelation(entityID_A, entityID_B, relationsAB) ||
                         !spatialRelationCache.getRelation(entityID_A, entityID_C, relationsAC) )
                        continue; 

                    Handle objectB = getHandle(entityID_B); 
                    Handle objectC = getHandle(entityID_C);

                    SPATIAL_RELATION_VECTOR relationsABC;

                    if ( this->isBetween(relationsAB, relationsAC) ) {
                        relationsABC.push_back(Entity::BETWEEN); 
                        this->addSpatialRelations(relationsABC, atomSpace, timestamp,
                                                  objectA, objectB, objectC
                                                 ); 
                    }

                    numRelations += relationsABC.size();

                } // foreach (const std::string & entityID_C, entities)
            } // foreach (const std::string & entityID_B, entities)

            logger().debug("%s - Finished evaluating: %d 3-size spatial relations related to '%s'", 
                           __FUNCTION__, numRelations, entityID_A.c_str()
                          );

        } // foreach (Handle objectA, objects)
    } 
    catch( const opencog::NotFoundException & ex ) {
        // Usually it is ok to just skip the exception, because when the object 
        // is removed, it will fail to find the object. That is normal and happens  
        // quite often for consumable objects, such as FoodCube
        return;
    } // try
    */
}

bool SpatialPredicateUpdater::isBetween(const SPATIAL_RELATION_VECTOR & relationsAB, 
                                        const SPATIAL_RELATION_VECTOR & relationsAC
                                       )
{
    /*
    bool bLeftAB   = false, 
         bRightAB  = false,
         bAboveAB  = false, 
         bBelowAB  = false, 
         bFrontAB  = false, 
         bBehindAB = false;

    bool bLeftAC   = false, 
         bRightAC  = false,
         bAboveAC  = false, 
         bBelowAC  = false, 
         bFrontAC  = false, 
         bBehindAC = false; 

    foreach (Entity::SPATIAL_RELATION relAB, relationsAB) {
        switch (relAB) {
            case Entity::LEFT_OF:     bLeftAB  = true; break; 
            case Entity::RIGHT_OF:    bRightAB = true; break; 
            case Entity::ABOVE:       bAboveAB = true; break; 
            case Entity::BELOW:       bBelowAB = true; break; 
            case Entity::BEHIND:      bBelowAB = true; break; 
            case Entity::IN_FRONT_OF: bFrontAB = true; break; 
            default: break; 
        } 
    } // foreach

    foreach (Entity::SPATIAL_RELATION relAC, relationsAC) {
        switch (relAC) {
            case Entity::LEFT_OF:     bLeftAC  = true; break; 
            case Entity::RIGHT_OF:    bRightAC = true; break; 
            case Entity::ABOVE:       bAboveAC = true; break; 
            case Entity::BELOW:       bBelowAC = true; break; 
            case Entity::BEHIND:      bBelowAC = true; break; 
            case Entity::IN_FRONT_OF: bFrontAC = true; break; 
            default: break; 
        } 
    } // foreach

    return (bLeftAB   && bRightAC)  ||
           (bRightAB  && bLeftAC)   ||
           (bAboveAB  && bBelowAC)  ||
           (bBelowAB  && bAboveAC)  ||
           (bFrontAB  && bBehindAC) ||
           (bBehindAB && bFrontAC); 
           */
}

void SpatialPredicateUpdater::
addSpatialRelations(const SPATIAL_RELATION_VECTOR & relations, 
                    AtomSpace & atomSpace, unsigned long timestamp, 
                    Handle objectA, Handle objectB, Handle objectC 
                   )
{

    // Clear all the relations firstly
    if ( objectC == Handle::UNDEFINED ) {
        for (int rel = 0; rel < (int)Entity::TOTAL_RELATIONS; ++ rel) {
            string predicateName = Entity::spatialRelationToString( (Entity::SPATIAL_RELATION) rel );
            Handle eval = AtomSpaceUtil::setPredicateValue(atomSpace,
                                                           predicateName, 
                                                           TruthValue::FALSE_TV(), 
                                                           objectA, objectB, objectC
                                                          );
        }
    }
    
    // Set relations
    foreach (Entity::SPATIAL_RELATION rel, relations) {
        string predicateName = Entity::spatialRelationToString(rel);
        Handle eval = AtomSpaceUtil::setPredicateValue(atomSpace,
                                                       predicateName, 
                                                       TruthValue::TRUE_TV(), 
                                                       objectA, objectB, objectC
                                                      );
        Handle atTime = atomSpace.getTimeServer().addTimeInfo(eval, timestamp);
        atomSpace.setTV(atTime, TruthValue::TRUE_TV());
    }
}

bool SpatialPredicateUpdater::SpatialRelationCache::
isCached(std::string entityA_id, std::string entityB_id)
{
    std::string key = entityA_id + entityB_id; 
    return this->_entityRelationMap.find(key) != this->_entityRelationMap.end(); 
}

bool SpatialPredicateUpdater::SpatialRelationCache::
getRelation(std::string entityA_id, std::string entityB_id, SPATIAL_RELATION_VECTOR & relation)
{
    std::string key = entityA_id + entityB_id; 
    boost::unordered_map <std::string, SPATIAL_RELATION_VECTOR>::iterator
        iter_relation = this->_entityRelationMap.find(key); 

    if ( iter_relation != this->_entityRelationMap.end() ) {
        relation = iter_relation->second;  
        return true; 
    }
    else
        return false; 
}

void SpatialPredicateUpdater::SpatialRelationCache::
addRelation(std::string entityA_id, std::string entityB_id, const SPATIAL_RELATION_VECTOR & relation)
{
    std::string key = entityA_id + entityB_id;
    this->_entityRelationMap[key] = relation; 
}

void SpatialPredicateUpdater::SpatialRelationCache::clear()
{
    this->_entityRelationMap.clear(); 
}

