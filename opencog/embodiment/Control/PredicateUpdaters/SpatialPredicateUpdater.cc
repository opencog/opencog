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

#include <cstdlib>
#include <iterator>
#include <map>
#include <set>
#include <vector>

#include <opencog/atomspace/SimpleTruthValue.h>

#include "SpatialPredicateUpdater.h"
#include <opencog/embodiment/AtomSpaceExtensions/AtomSpaceUtil.h>
#include <opencog/embodiment/Control/OperationalAvatarController/EventDetectionAgent.h>
#include <opencog/spacetime/SpaceTime.h>
#include <opencog/spacetime/SpaceServer.h>

using namespace opencog::oac;
using namespace opencog;
using namespace spatial;

SpatialPredicateUpdater::SpatialPredicateUpdater(AtomSpace & _atomSpace) :
        BasicPredicateUpdater(_atomSpace)
{
    is_only_update_about_avatars = config().get_bool("UPDATE_SPATIAL_PREDICATES_ONLY_ABOUT_AVATAR");
    update_types = config().get("UPDATE_SPATIAL_PREDICATES_FOR");

    enableCollectActions = config().get_bool("ENABLE_ACTION_COLLECT");

    RELATIONS_NEED_TO_UPDATE.push_back(ABOVE);
    RELATIONS_NEED_TO_UPDATE.push_back(BELOW);
    RELATIONS_NEED_TO_UPDATE.push_back(TOUCHING);
    RELATIONS_NEED_TO_UPDATE.push_back(NEAR);
    RELATIONS_NEED_TO_UPDATE.push_back(FAR_);
}

SpatialPredicateUpdater::~SpatialPredicateUpdater()
{
}

void SpatialPredicateUpdater::update(std::vector<Handle> & objects,
                                     Handle pet, 
                                     unsigned long timestamp
                                    )
{
    static bool hasDoneFirstTimeUPdate = false;
    if (objects.size() == 0)
        return;

    int beginTime = time(NULL);

    if (! hasDoneFirstTimeUPdate)
        printf("Begin the first time spatial predicate update! Begin time: %d. Wait...\n",beginTime);

    // first get 3 Entity lists from the space map: Avatar list, Non-blockEntity list and blockEntity list
    set<const spatial::Entity3D*> avatarList;
    vector<const spatial::Entity3D*> nonBlockEntityList;
    vector<const spatial::Entity3D*> blockEntityList;
    vector<const spatial::Entity3D*> allEntities;


    const SpaceServer::SpaceMap & spaceMap = spaceServer().getLatestMap();

    std::vector<Handle>::iterator handleIt;
    for(handleIt = objects.begin(); handleIt != objects.end(); ++ handleIt)
    {
        const Entity3D* e = spaceMap.getEntity( (Handle)(*handleIt));
        if (! e)
            continue;
        if (e->isBlockEntity())
            blockEntityList.push_back(e);
        else if ( spaceMap.isAvatarEntity(e))
            avatarList.insert(e);
        else
            nonBlockEntityList.push_back(e);

        allEntities.push_back(e);
    }



    if (is_only_update_about_avatars)
        this->computeRelationshipsBetweenObjectsAndAvatars(spaceMap, avatarList, nonBlockEntityList, blockEntityList, pet, timestamp);
    else
        this->computeRelationshipsBetweenAllObjects(spaceMap, allEntities, pet, timestamp);

    int endTime = time(NULL);

    if (! hasDoneFirstTimeUPdate)
    {
        printf("Spatial predicate update complete! Total time: %f seconds.\n",(endTime - beginTime)/1000.0f);
        hasDoneFirstTimeUPdate = true;
    }
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

//    foreach (spatial::SPATIAL_RELATION rel, relations) {
//        switch (rel) {
//            case spatial::LEFT_OF:     newRelations.push_back(spatial::RIGHT_OF);    break;
//            case spatial::RIGHT_OF:    newRelations.push_back(spatial::LEFT_OF);     break;
//            case spatial::ABOVE:       newRelations.push_back(spatial::BELOW);       break;
//            case spatial::BELOW:       newRelations.push_back(spatial::ABOVE);       break;
//            case spatial::BEHIND:      newRelations.push_back(spatial::IN_FRONT_OF); break;
//            case spatial::IN_FRONT_OF: newRelations.push_back(spatial::BEHIND);      break;
//            case spatial::BESIDE:      newRelations.push_back(spatial::BESIDE);      break;
//            case spatial::NEAR:        newRelations.push_back(spatial::NEAR);        break;
//            case spatial::FAR_:        newRelations.push_back(spatial::FAR_);        break;
//            case spatial::TOUCHING:    newRelations.push_back(spatial::TOUCHING);    break;
//            case spatial::INSIDE:      newRelations.push_back(spatial::OUTSIDE);     break;
//            case spatial::OUTSIDE:     newRelations.push_back(spatial::INSIDE);      break;
//            case spatial::ADJACENT:    newRelations.push_back(spatial::ADJACENT);    break;
//            default: break;
//        } // switch (rel)
//    } // foreach (spatial::SPATIAL_RELATION rel, relations)

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
            relationsBO.push_back(spatial::IN_FRONT_OF);
        else if (angle >= 45 && angle <= 135)
            relationsBO.push_back(spatial::LEFT_OF);
        else if (angle >= -135 && angle <= -45)
            relationsBO.push_back(spatial::RIGHT_OF);
        else
            relationsBO.push_back(spatial::BEHIND);

        if ( objectPosition.z - entityB->getHeight()*0.5 > observerPosition.z + observerEntity->getHeight() ) 
            relationsBO.push_back(spatial::ABOVE);

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

// compute all the relationships between all objects
void SpatialPredicateUpdater::computeRelationshipsBetweenAllObjects(const SpaceServer::SpaceMap & spaceMap,
                                                                    vector<const spatial::Entity3D*> &allEntities,
                                                                    Handle observer,
                                                                    unsigned long timestamp)
{

// todo:
}

// compute only the relationships between objects and avatar , which means every piece here descrips a relationship between an object and an avatar
void SpatialPredicateUpdater::computeRelationshipsBetweenObjectsAndAvatars(const SpaceServer::SpaceMap & spaceMap,
                                            std::set<const spatial::Entity3D*>& avatars,
                                            std::vector<const spatial::Entity3D*>& nonblockEntities,
                                            std::vector<const spatial::Entity3D*>& blockEntities,
                                            Handle observer,
                                            unsigned long timestamp)
{
    // if there is any avatar updated in this frame, then we have to update all the relationships between this avatar and all the other object on the map
    if (!avatars.empty())
    {
        const map<Handle, Entity3D*> allNoneBlockEntities= spaceMap.getAllNoneBlockEntities();

        const map<int, spatial::BlockEntity*> allBlockEntities= spaceMap.getBlockEntityList();

        std::set<const spatial::Entity3D*>::const_iterator avatarIt;
        for (avatarIt = avatars.begin(); avatarIt != avatars.end(); avatarIt ++)
        {
            const Entity3D* avatar = (const Entity3D*)(*avatarIt);
            // first update all the relationships between this avatar and all the nonblock entities
            map<Handle, Entity3D*>::const_iterator nonBlockIt;
            for (nonBlockIt = allNoneBlockEntities.begin(); nonBlockIt != allNoneBlockEntities.end(); nonBlockIt ++ )
            {
                const Entity3D* nonBE = (const Entity3D*)(nonBlockIt->second);
                if (nonBE == avatar)
                    continue;

                set<spatial::SPATIAL_RELATION> relations = spaceMap.computeSpatialRelations(avatar,nonBE);
                addSpatialRelations(relations,atomSpace,timestamp,avatar->mEntityNode, nonBE->mEntityNode);

                set<spatial::SPATIAL_RELATION> relations2 = spaceMap.computeSpatialRelations(nonBE,avatar);
                addSpatialRelations(relations2,atomSpace,timestamp, nonBE->mEntityNode,avatar->mEntityNode);
            }

            // update all the relationships between this avatar and all the block entities if needed
            if (this->update_types == "block_entity_and_non_block_entity") // see the config file
            {
                map<int, spatial::BlockEntity*>::const_iterator blockIt;
                for (blockIt = allBlockEntities.begin(); blockIt != allBlockEntities.end(); blockIt ++ )
                {
                    const Entity3D* bE = (const Entity3D*)(blockIt->second);

                    set<spatial::SPATIAL_RELATION> relations = spaceMap.computeSpatialRelations(avatar,bE);
                    addSpatialRelations(relations,atomSpace,timestamp,avatar->mEntityNode, bE->mEntityNode);

                    set<spatial::SPATIAL_RELATION> relations2 = spaceMap.computeSpatialRelations(bE,avatar);
                    addSpatialRelations(relations2,atomSpace,timestamp, bE->mEntityNode,avatar->mEntityNode);
                }
            }

            // to do: to update all the relationships between this avatar and all the blocks if needed

        }
    }

    // update the relations between the nonblockEntities changed in this frame with all the avatars on the map
    map<Handle, Entity3D*>::const_iterator ait;
    for (ait = spaceMap.getAllAvatarList().begin(); ait != spaceMap.getAllAvatarList().end(); ait ++ )
    {
        Entity3D* avatar = (Entity3D*)(ait->second);

        // if this avatar is in the changed avatar list, then it's alreay been updated above, just skip it
        if (avatars.find(avatar) != avatars.end() )
            continue;

        // first update all the relationships between this avatar and the changed nonblock entities
        std::vector<const spatial::Entity3D*>::const_iterator nonBlockIt;
        for (nonBlockIt = nonblockEntities.begin(); nonBlockIt != nonblockEntities.end(); nonBlockIt ++ )
        {
            const Entity3D* nonBE = (const Entity3D*)(*nonBlockIt);
            if (nonBE == avatar)
                continue;

            set<spatial::SPATIAL_RELATION> relations = spaceMap.computeSpatialRelations(avatar,nonBE);
            addSpatialRelations(relations,atomSpace,timestamp,avatar->mEntityNode, nonBE->mEntityNode);

            set<spatial::SPATIAL_RELATION> relations2 = spaceMap.computeSpatialRelations(nonBE, avatar);
            addSpatialRelations(relations2,atomSpace,timestamp, nonBE->mEntityNode ,avatar->mEntityNode);
        }

        // update all the relationships between this avatar and the changed block entities if needed
        if (this->update_types == "block_entity_and_non_block_entity") // see the config file
        {
            std::vector<const spatial::Entity3D*>::const_iterator blockIt;
            for (blockIt = blockEntities.begin(); blockIt != blockEntities.end(); blockIt ++ )
            {
                const Entity3D* bE = (const Entity3D*)(*blockIt);

                set<spatial::SPATIAL_RELATION> relations = spaceMap.computeSpatialRelations(avatar,bE);
                addSpatialRelations(relations,atomSpace,timestamp,avatar->mEntityNode, bE->mEntityNode);

                set<spatial::SPATIAL_RELATION> relations2 = spaceMap.computeSpatialRelations(bE, avatar);
                addSpatialRelations(relations2,atomSpace,timestamp,bE->mEntityNode,avatar->mEntityNode);
            }
        }

    }

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
                        relationsABC.push_back(spatial::BETWEEN);
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

    foreach (spatial::SPATIAL_RELATION relAB, relationsAB) {
        switch (relAB) {
            case spatial::LEFT_OF:     bLeftAB  = true; break;
            case spatial::RIGHT_OF:    bRightAB = true; break;
            case spatial::ABOVE:       bAboveAB = true; break;
            case spatial::BELOW:       bBelowAB = true; break;
            case spatial::BEHIND:      bBelowAB = true; break;
            case spatial::IN_FRONT_OF: bFrontAB = true; break;
            default: break; 
        } 
    } // foreach

    foreach (spatial::SPATIAL_RELATION relAC, relationsAC) {
        switch (relAC) {
            case spatial::LEFT_OF:     bLeftAC  = true; break;
            case spatial::RIGHT_OF:    bRightAC = true; break;
            case spatial::ABOVE:       bAboveAC = true; break;
            case spatial::BELOW:       bBelowAC = true; break;
            case spatial::BEHIND:      bBelowAC = true; break;
            case spatial::IN_FRONT_OF: bFrontAC = true; break;
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
return false;
}

void SpatialPredicateUpdater::
addSpatialRelations(const set<spatial::SPATIAL_RELATION> & relations,
                    AtomSpace & atomSpace, unsigned long timestamp, 
                    Handle objectA, Handle objectB, Handle objectC 
                   )
{
    set<spatial::SPATIAL_RELATION>::const_iterator sit;
    // Set relations
    vector<spatial::SPATIAL_RELATION> ::const_iterator rit;
    for (rit = RELATIONS_NEED_TO_UPDATE.begin(); rit != RELATIONS_NEED_TO_UPDATE.end(); rit ++)
    {
        string predicateName = SpaceServer::SpaceMap::spatialRelationToString((spatial::SPATIAL_RELATION)(*rit));
        sit = relations.find((spatial::SPATIAL_RELATION)(*rit));
        Handle eval;
        if (sit != relations.end()) // these objects have this relation
        {
            eval = AtomSpaceUtil::setPredicateValue(atomSpace,
                                                       predicateName,
                                                       TruthValue::TRUE_TV(),
                                                       objectA, objectB, objectC
                                                      );            
        }
        else
        { // these objects do not have this relation
            eval = AtomSpaceUtil::setPredicateValue(atomSpace,
                                                       predicateName,
                                                       TruthValue::FALSE_TV(),
                                                       objectA, objectB, objectC
                                                      );

        }

        timeServer().addTimeInfo(eval, timestamp);

//        if (enableCollectActions)
//            oac::EventDetectionAgent::addAnEvent(eval,timestamp,oac::EVENT_TYPE_SPATIAL_PREDICATE);

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
    std::unordered_map <std::string, SPATIAL_RELATION_VECTOR>::iterator
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

