/*
 * opencog/spacetime/SpaceServer.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
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
#include <string>
#include <vector>
#include <cstdlib>

#include <opencog/util/Logger.h>
#include <opencog/util/StringTokenizer.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/AtomSpaceImpl.h>
#include <opencog/atomspace/SimpleTruthValue.h>

#include <opencog/spatial/3DSpaceMap/Block3D.h>
#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>

#include <opencog/spacetime/atom_types.h>
#include "SpaceServer.h"
#include "TimeServer.h"

//#define DPRINTF printf
#define DPRINTF(...)
// To simplify debug log output
#define ATOM_AS_STRING(h) (atomspace->atomAsString(h).c_str())

using namespace opencog;
using namespace opencog::spatial;

SpaceServer::SpaceServer(AtomSpace &_atomspace) :
    atomspace(&_atomspace)
{
    // Default values (should only be used for test purposes)
    agentRadius = 0.25;
    agentHeight = 0;
    xMin = 0;
    xMax = 256;
    yMin = 0;
    yMax = 256;
    xDim = 1024;
    yDim = 1024;
    floorHeight = 0;
    curSpaceMapHandle = Handle::UNDEFINED;
    curMap = NULL;

    // connect signals
    removedAtomConnection = _atomspace.atomSpaceAsync->addAtomSignal(boost::bind(&SpaceServer::atomAdded, this, _1, _2));
    addedAtomConnection = _atomspace.atomSpaceAsync->removeAtomSignal(boost::bind(&SpaceServer::atomRemoved, this, _1, _2));
}

SpaceServer::~SpaceServer()
{
    // disconnect signals
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();

    clear();
}

void SpaceServer::setTimeServer(TimeServer* ts)
{
    timeser = ts;
}

void SpaceServer::atomAdded(AtomSpaceImpl* a, Handle h)
{
}

void SpaceServer::atomRemoved(AtomSpaceImpl* a, AtomPtr atom)
{
    Type type = atom->getType();
    if (classserver().isA(type, OBJECT_NODE)) {
        removeSpaceInfo(atom->getHandle());
    }
}

void SpaceServer::setAgentRadius(unsigned int _radius)
{
    if (agentRadius != _radius) {
        agentRadius = _radius;
        logger().info("SpaceServer - AgentRadius: %d", agentRadius);
    }
}

void SpaceServer::setAgentHeight(unsigned int _height)
{
    if (agentHeight != _height) {
        agentHeight = _height;
        curMap->setAgentHeight(_height);
        logger().info("SpaceServer - AgentRadius: %d", agentHeight);
    }


}
/*
// this function does not be used in 3d map
SpaceServer::SpaceMap* SpaceServer::addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle)
{
    SpaceMap* map;
    HandleToSpaceMap::iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
        // a new map
        logger().info("SpaceServer - New map: xMin: %.3lf, xMax: %.3lf, yMin: %.3lf, yMax: %.3lf, zMin: %.3lf, zMax: %.3lf",
                     xMin, xMax, yMin, yMax, zMin, zMax);

        if (!sortedMapHandles.empty()) {
            Handle latestMapHandle = sortedMapHandles.back();
            SpaceMap* latestMap = spaceMaps[latestMapHandle];
            if (latestMap->xMin() == xMin &&
                    latestMap->xMax() == xMax &&
                    latestMap->yMin() == yMin &&
                    latestMap->yMax() == yMax) {
                // latest map dimensions match new map dimensions
                bool mapReused = false;
                if (keepPreviousMap) {
                    if (sortedMapHandles.size() > 1 &&
                            persistentMapHandles.find(latestMapHandle) == persistentMapHandles.end()) {
                        Handle lastButOneMapHandle = *(sortedMapHandles.end() - 2);
                        SpaceMap* lastButOneMap = spaceMaps[lastButOneMapHandle];
                        // Check if the 2 latest maps are equals
                        if (*latestMap == *lastButOneMap) {
                            logger().debug("SpaceServer - The 2 previous maps are equals. "
                                    "Previous map (%s) transfered to new map (%s).",
                                    ATOM_AS_STRING(latestMapHandle),
                                    ATOM_AS_STRING(spaceMapHandle));
                            sortedMapHandles.erase(sortedMapHandles.end() - 1);
                            spaceMaps.erase(latestMapHandle);
                            mapRemoved(latestMapHandle);
                            map = latestMap; // reuse the spaceMap object
                            mapReused = true;
                            persistentMapHandles.insert(lastButOneMapHandle);
                            mapPersisted(lastButOneMapHandle);
                            logger().debug("SpaceServer - Map (%s) marked as persistent.",
                                    ATOM_AS_STRING(lastButOneMapHandle));
                        } else {
                            persistentMapHandles.insert(latestMapHandle);
                            mapPersisted(latestMapHandle);
                            logger().debug("SpaceServer - Map (%s) marked as persistent.",
                                    ATOM_AS_STRING(latestMapHandle));
                        }
                    } else {
                        persistentMapHandles.insert(latestMapHandle);
                        mapPersisted(latestMapHandle);
                        logger().debug("SpaceServer - Map (%s) marked as persistent.",
                                ATOM_AS_STRING(latestMapHandle));
                    }
                } else if (persistentMapHandles.find(latestMapHandle) == persistentMapHandles.end()) {
                    logger().debug("SpaceServer - Previous map (%s) transfered to new map (%s).",
                            ATOM_AS_STRING(latestMapHandle),
                            ATOM_AS_STRING(spaceMapHandle));
                    sortedMapHandles.erase(sortedMapHandles.end() - 1);
                    spaceMaps.erase(latestMapHandle);
                    mapRemoved(latestMapHandle);
                    map = latestMap; // reuse the spaceMap object
                    mapReused = true;
                }
                if (!mapReused) {
                    // Create the new one by cloning the latest map
                    logger().debug("SpaceServer - New map (%s) cloned from previous map (%s).",
                            ATOM_AS_STRING(spaceMapHandle),
                            ATOM_AS_STRING(latestMapHandle));
                    map = latestMap->clone();
                }
            } else {
                // latest map dimensions do not match new map dimensions.
                // Create an empty map
                logger().debug("SpaceServer - New map (%s) created by copying.",
                        ATOM_AS_STRING(spaceMapHandle));
                map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, agentRadius, agentHeight, floorHeight);
                // Copy each object in latest map into the new map
                map->copyObjects(*latestMap);
                if (!keepPreviousMap) {
                    logger().debug("SpaceServer - Previous map (%s) removed.",
                            ATOM_AS_STRING(latestMapHandle));

                    sortedMapHandles.erase(sortedMapHandles.end() - 1);
                    spaceMaps.erase(latestMapHandle);
                    mapRemoved(latestMapHandle);
                    delete latestMap;
                }
            }
        } else {
            // Create first map
            map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, agentRadius, agentHeight, floorHeight);
            logger().debug("SpaceServer - First map (%s) created",
                    ATOM_AS_STRING(spaceMapHandle));
        }
        spaceMaps[spaceMapHandle] = map;
        sortedMapHandles.push_back(spaceMapHandle);
        mapPersisted(spaceMapHandle); // Ensure the latest map will not be removed by forgetting mechanism
        logger().debug("SpaceServer - spaceMaps size: %u, sortedMapHandles size: %u",
                spaceMaps.size(), sortedMapHandles.size());

    } else {
        // get the existing map
        map = itr->second;
    }
    return map;
}

bool SpaceServer::add(bool keepPreviousMap, Handle spaceMapHandle, const std::string& objectId,
                      double centerX, double centerY, double centerZ,
                      double length, double width, double height,
                      double yaw, const std::string& entityClass, bool isObstacle)
{


    SpaceMap* map = addOrGetSpaceMap(keepPreviousMap, spaceMapHandle);
    DPRINTF("SpaceServer::add After addOrGet\n");

    logger().fine(
                 "SpaceServer::add map->xMin() = %lf, map->xMax() = %lf, map->yMin() = %lf, "
                 "map->yMax() = %lf, map->xGridWidth() = %lf, map->yGridWidth() = %lf",
                 map->xMin(), map->xMax(), map->yMin(), map->yMax(),
                 map->xGridWidth(), map->yGridWidth());

    SpaceServer::ObjectMetadata metadata(centerX, centerY, centerZ, length, width, height, yaw, entityClass);
    bool mapContainsObject = map->containsObject(objectId);
    bool needUpdate = false;
    DPRINTF("SpaceServer::add After contains\n");
    if (mapContainsObject) {
        //const SpaceServer::ObjectMetadata& oldMetadata = map->getMetaData(objectId);
        const spatial::EntityPtr& oldEntity = map->getEntity(objectId);
        SpaceServer::ObjectMetadata oldMetadata( oldEntity->getPosition( ).x,
                                                 oldEntity->getPosition( ).y,
                                                 oldEntity->getPosition( ).z,
                                                 oldEntity->getLength( ),
                                                 oldEntity->getWidth( ),
                                                 oldEntity->getHeight( ),
                                                 oldEntity->getOrientation( ).getRoll( ) );

        DPRINTF("SpaceServer::add After getMetaData\n");

        if (metadata != oldMetadata) {
            needUpdate = true;
            logger().fine(
                         "SpaceServer::add Old metadata (x=%lf, y=%lf, length=%lf, width=%lf, "
                         "height=%lf, yaw=%lf) is different: object must be updated",
                         oldMetadata.centerX, oldMetadata.centerY, oldMetadata.centerZ,
                         oldMetadata.length, oldMetadata.width, oldMetadata.height,
                         oldMetadata.yaw);
        } else {
            bool wasObstacle = map->isObstacle(objectId);
            if (isObstacle != wasObstacle) {
                needUpdate = true;
                logger().fine("SpaceServer::add Object is %san obstacle now. "
                        "So, it must be updated.", isObstacle ? " " : "not ");
            }
        }
    } else {
        logger().fine("SpaceServer::add Object does not exist in the map yet. "
                "So, it will be added.");
    }

    if (!mapContainsObject || needUpdate) {

        logger().debug("SpaceServer - add(mapH=%lu, objId=%s, x=%lf, y=%lf, "
                "length=%lf, width=%lf, height=%lf, yaw=%lf, isObstacle=%d)",
                 spaceMapHandle.value(), objectId.c_str(), centerX, centerY, centerZ,
                 length, width, height, yaw, isObstacle);

        if (mapContainsObject) {
            logger().fine(
                         "SpaceServer::add - updating object into the space map");

            map->updateObject(objectId, metadata, isObstacle );

        } else {
            logger().fine("SpaceServer::add - adding object into the space map");
            if (entityClass == "block") {
                map->addBlock(objectId, metadata);
            } else {
                map->addObject(objectId, metadata, isObstacle );
            }
        }
        return true;
    }
    return false;
}

void SpaceServer::add(Handle spaceMapHandle, SpaceMap * map)
{

    logger().info("SpaceServer - New map (%s) added",
                 ATOM_AS_STRING(spaceMapHandle));
    sortedMapHandles.push_back(spaceMapHandle);
    spaceMaps[spaceMapHandle] = map;
    logger().debug("SpaceServer - spaceMaps size: %d",
                 spaceMaps.size());

}

void SpaceServer::remove(bool keepPreviousMap, Handle spaceMapHandle, const std::string& objectId)
{
    logger().info("SpaceServer::remove()");
    SpaceMap* map = addOrGetSpaceMap(keepPreviousMap, spaceMapHandle);
    if (map->containsObject(objectId)) map->removeObject(objectId);
}
*/

const SpaceServer::SpaceMap& SpaceServer::getMap(Handle spaceMapHandle) const
    throw (opencog::RuntimeException, std::bad_exception)
{
    logger().fine("SpaceServer::getMap() for mapHandle = %s",
            spaceMapHandle != Handle::UNDEFINED ?
            ATOM_AS_STRING(spaceMapHandle)
            : "Handle::UNDEFINED");

    HandleToSpaceMap::const_iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
        throw opencog::RuntimeException(TRACE_INFO,
                "SpaceServer - Found no SpaceMap associate with handle: '%s'.",
                ATOM_AS_STRING(spaceMapHandle));
    }

    return *(itr->second);
}

const bool SpaceServer::containsMap(Handle spaceMapHandle) const
{
    return (spaceMaps.find(spaceMapHandle) != spaceMaps.end());
}

const bool SpaceServer::isLatestMapValid() const
{
    return ((curSpaceMapHandle != Handle::UNDEFINED)&&(curMap != 0));
}

SpaceServer::SpaceMap& SpaceServer::getLatestMap() const
    throw (opencog::AssertionException, std::bad_exception)
{
    return *curMap;
}

Handle SpaceServer::getLatestMapHandle() const
{
    return curSpaceMapHandle;
}

/*
Handle SpaceServer::getOlderMapHandle() const
{
    if (sortedMapHandles.empty()) {
        return Handle::UNDEFINED;
    }
    return sortedMapHandles.front();
}
*/
/*
Handle SpaceServer::getPreviousMapHandle(Handle spaceMapHandle) const
{
    Handle result = Handle::UNDEFINED;
    if (spaceMapHandle != Handle::UNDEFINED) {
        std::vector<Handle>::const_iterator itr = std::lower_bound( 
                sortedMapHandles.begin(), sortedMapHandles.end(), spaceMapHandle);
        if (itr != sortedMapHandles.begin() && *itr == spaceMapHandle) {
            result = *(--itr);
        }
    }
    return result;
}*/
/*
Handle SpaceServer::getNextMapHandle(Handle spaceMapHandle) const
{
    Handle result = Handle::UNDEFINED;
    if (spaceMapHandle != Handle::UNDEFINED) {
        std::vector<Handle>::const_iterator itr = std::lower_bound(
                sortedMapHandles.begin(), sortedMapHandles.end(), spaceMapHandle);
        if (*itr == spaceMapHandle) {
            ++itr;
            if (itr != sortedMapHandles.end()) result = *itr;
        }
    }
    return result;
}
*/
void SpaceServer::removeMap(Handle spaceMapHandle)
{
    HandleToSpaceMap::iterator itr = spaceMaps.find(spaceMapHandle);
    if (itr != spaceMaps.end()) {
    /*    std::vector<Handle>::iterator itr_map =
            std::find(sortedMapHandles.begin(), sortedMapHandles.end(), spaceMapHandle);
        if (*itr_map == spaceMapHandle) {
            sortedMapHandles.erase(itr_map);
        } else {
            if (itr_map != sortedMapHandles.end()) {
                logger().error("SpaceServer::removeSpaceMap - Removed is not mapHandle.\n");
                sortedMapHandles.erase(itr_map);
            }
            logger().error("SpaceServer::removeSpaceMap - Trying to remove non-existent map."
                    " spaceMapSize = %d sortedMapHandlesSize = %d\n", 
                    spaceMaps.size(), sortedMapHandles.size());
        }
*/
        delete itr->second;
        spaceMaps.erase(itr);

    }
}

/*
void SpaceServer::markMapAsPersistent(Handle spaceMapHandle)
{
    HandleToSpaceMap::const_iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
        throw opencog::RuntimeException(TRACE_INFO,
                                        "SpaceServer - Found no SpaceMap associate with handle: '%s'.",
                                        ATOM_AS_STRING(spaceMapHandle));
    }
    persistentMapHandles.insert(spaceMapHandle);
    mapPersisted(spaceMapHandle);
}
*/
/*
bool SpaceServer::isMapPersistent(Handle spaceMapHandle) const
{
    return (persistentMapHandles.find(spaceMapHandle) != persistentMapHandles.end());
}*/
/*
void SpaceServer::removeObject(const std::string& objectId)
{
    for (HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); ++itr) {
        itr->second->removeObject(objectId);
    }
}
*/
unsigned int SpaceServer::getSpaceMapsSize() const
{
    return spaceMaps.size();
}

void SpaceServer::clear()
{
    for (HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); ++itr) {
        delete itr->second;
    }
    spaceMaps.clear();
}

SpaceServer::SpaceMap* SpaceServer::cloneTheLatestSpaceMap() const
{
    return curMap->clone();
}

/*
Handle SpaceServer::getSpaceMapNode(std::string _mapName)
{
    Handle result = spaceMapNodeHandle;
    if (result == Handle::UNDEFINED) {
        result = atomspace->addNode(SPACE_MAP_NODE, SpaceServer::SPACE_MAP_NODE_NAME)->get_result();
        atomspace->setLTI(result, 1);
    } else {
        if (atomspace->getLTI(result)->get_result() < 1) {
            atomspace->setLTI(result, 1);
        }
    }
    return result;
}*/
/*
bool SpaceServer::addSpaceInfo(Handle objectNode, unsigned long timestamp,
                              int objX, int objY, int objZ,
                              int objLength, int objWidth, int objHeight,
                              double objYaw, bool isObstacle, const std::string& entityClass) {

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = timeServer->addTimeInfo(spaceMapNode, timestamp);
    bool result =  add( keepPreviousMap, spaceMapAtTimeLink, atomspace->getName(objectNode)->get_result(),
                        objX, objY, objZ, objLength, objWidth, objHeight, objYaw, entityClass, isObstacle);

    return result;
}
*/

bool SpaceServer::addSpaceInfo(Handle objectNode, bool isSelfObject, unsigned long timestamp,
                              int objX, int objY, int objZ,
                              int objLength, int objWidth, int objHeight,
                              double objYaw, bool isObstacle,  std::string entityClass, std::string objectName, std::string material)
{
    if (curSpaceMapHandle == Handle::UNDEFINED)
    {
        logger().error("SpaceServer::addSpaceInfo - No space map now!");
        return false;
    }

    timeser->addTimeInfo(curSpaceMapHandle, timestamp);

    // we should distinguish to add a block or other object
    // because when adding a block, maybe cause some change in the terrain and structures

    opencog::spatial::BlockVector pos(objX, objY, objZ);

    if (entityClass == "block")
    {
        // it's a block
        curMap->addSolidUnitBlock(pos,objectNode, material);

    }
    else
    {
        curMap->addNoneBlockEntity(objectNode,pos,objWidth,objLength,objHeight,objYaw,objectName, entityClass,isSelfObject, true);
    }
    return true;
}

Handle SpaceServer::addOrGetSpaceMap(unsigned long timestamp, std::string _mapName, int _xMin, int _yMin, int _zMin,
                                     int _xDim, int _yDim, int _zDim, int _floorHeight)
{
    Handle spaceMapNode = atomspace->getHandle(SPACE_MAP_NODE,_mapName);

    // There is not a space map node for this map in the atomspace, so create a new one
    if (spaceMapNode == Handle::UNDEFINED)
    {
        spaceMapNode = atomspace->addNode(SPACE_MAP_NODE,_mapName);
        atomspace->setLTI(spaceMapNode, 1);
        timeser->addTimeInfo(spaceMapNode, timestamp);

        SpaceMap* newSpaceMap = new SpaceMap( _mapName, _xMin,  _yMin, _zMin,  _xDim,  _yDim,  _zDim,  _floorHeight);

        // add into the map set
        spaceMaps.insert(map<Handle,SpaceMap*>::value_type(spaceMapNode,newSpaceMap));
        curMap = newSpaceMap;
    }
    else
    {
        curMap = spaceMaps[spaceMapNode];
    }

    curSpaceMapHandle = spaceMapNode;
    return spaceMapNode;
}

void SpaceServer::removeSpaceInfo(Handle objectNode, unsigned long timestamp)
{
    if (curSpaceMapHandle == Handle::UNDEFINED)
    {
        logger().error("SpaceServer::addSpaceInfo - No space map now!");
        return;
    }

    if (timestamp != 0)
        timeser->addTimeInfo(curSpaceMapHandle, timestamp);

    if (atomspace->getType(objectNode) == STRUCTURE_NODE)
    {
        curMap->removeSolidUnitBlock(objectNode);
    }
    else
    {
        curMap->removeNoneBlockEntity(objectNode);
    }

    logger().debug("%s(%s)\n", __FUNCTION__, atomspace->getName(objectNode).c_str());

}
/*
void SpaceServer::cleanupSpaceServer(){

    // sanity checks
    if (getSpaceMapsSize() < 1) {
        logger().debug("SpaceServer - No need to clean SpaceServer. It has no space map yet.");
        return;
    }

    // sanity tests passed, cleaning SpaceServer
    Handle spaceMapNode = getSpaceMapNode();

    // get all HandleTemporalPairs associated with the SpaceMap concept node.
    std::vector<HandleTemporalPair> pairs;
    timeServer->getTimeInfo(back_inserter(pairs), spaceMapNode);

    int j = 0;

    // remember to leave at least one map in SpaceServer, the newer one.
    for(unsigned int i = 0; i < pairs.size() - 1; i++){

        // get SpaceMap handles
        Handle mapHandle = timeServer->getAtTimeLink(pairs[i]);

        // mapHandle not among the ones that should be preserved
        if (!containsMap(mapHandle) || !isMapPersistent(mapHandle)){
            j++;
            logger().debug("SpaceServer - Removing map (%s)",
                    ATOM_AS_STRING(mapHandle));
            // remove map from SpaceServer, and timeInfo from TimeServer and AtomSpace
            atomspace->removeAtom(mapHandle, true);
        }
    }
    logger().debug("SpaceServer - Number of deleted maps: %d.", j);
}
*/
void SpaceServer::mapRemoved(Handle mapId)
{
    // Remove this atom from AtomSpace since its map does not exist anymore 
    atomspace->removeAtom(mapId);
}

void SpaceServer::mapPersisted(Handle mapId)
{
    // set LTI to a value that prevents the corresponding atom to be removed
    // from AtomSpace
    atomspace->setLTI(mapId, 1);
}

std::string SpaceServer::getMapIdString(Handle mapHandle) const
{
    // Currently the mapHandle is of AtTimeLink(TimeNode:"<timestamp>" , ConceptNode:"SpaceMap")
    // So, just get the name of the TimeNode as its string representation
    // return atomspace->getName(atomspace->getOutgoing(mapHandle, 0))->get_result();

    return atomspace->getName(mapHandle);
}    

// TODO
std::string SpaceServer::mapToString(Handle mapHandle) const
{

    std::stringstream stringMap;
    stringMap.precision(25);

    stringMap << getMapIdString(mapHandle);
    // see localspacemap toString

    return stringMap.str( );
}

// TODO
Handle SpaceServer::mapFromString(const std::string& stringMap)
{
/*
    unsigned long timestamp;
    std::stringstream mapParser;
    {
        std::stringstream parser( stringMap );
        parser.precision(25);    
        parser >> timestamp;
        mapParser << parser.rdbuf( );
    }        

    SpaceServer::TimestampMap timestampMap(timestamp, SpaceMap::fromString( mapParser.str( ) ) );

    return timestampMap;
    */
    return Handle::UNDEFINED;

}


SpaceServer& SpaceServer::operator=(const SpaceServer& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "SpaceServer - Cannot copy an object of this class");
}

SpaceServer::SpaceServer(const SpaceServer& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "SpaceServer - Cannot copy an object of this class");
}

void SpaceServer::markCurMapPerceptedForFirstTime()
{
    if (curMap)
        curMap->hasPerceptedMoreThanOneTimes = true;
}

void SpaceServer::findAllBlockEntitiesOnTheMap()
{
    if (curMap)
        curMap->findAllBlockEntitiesOnTheMap();
}

void SpaceServer::addBlockEntityNodes(HandleSeq &toUpdateHandles)
{
    vector<opencog::spatial::BlockEntity*>::iterator it = curMap->newAppearBlockEntityList.begin();
    opencog::spatial::BlockEntity* entity;
    for (; it != curMap->newAppearBlockEntityList.end(); ++it)
    {
        entity = (opencog::spatial::BlockEntity*)(*it);
        entity->mEntityNode = atomspace->addNode(BLOCK_ENTITY_NODE, opencog::toString(entity->getEntityID()));
        atomspace->setSTI(entity->mEntityNode, 10000);

        if (! AtomSpace::isHandleInSeq(entity->mEntityNode, toUpdateHandles))
            toUpdateHandles.push_back(entity->mEntityNode);
    }

    curMap->newAppearBlockEntityList.clear();
}

// add blocklist to an entity
void SpaceServer::addBlocksLisitPredicateToEntity(opencog::spatial::BlockEntity* _entity, const unsigned long timeStamp)
{

    //  Add every block a eval link that it's part of this block entity:
    //    (AtTimeLink (stv 1 1)
    //       (TimeNode "15818205490")
    //       (EvaluationLink (stv 1 0.0012484394)
    //          (PredicateNode "part-of")
    //          (ListLink
    //              (StructureNode "id_CHUNK_3_2_0_BLOCK_1_12_99")
    //              (BlockEntityNode "blockentity7" (av 1000 0 0))
    //
    //          )
    //       )
    //    )

    vector<opencog::spatial::Block3D*> blocks = (vector<opencog::spatial::Block3D*>&)(_entity->getBlockList());
    vector<opencog::spatial::Block3D*>::iterator it = blocks.begin();
    for (; it != blocks.end(); ++it)
    {
        opencog::spatial::Block3D* b = (opencog::spatial::Block3D*)(*it);
        HandleSeq unitBlockNodes = curMap->getAllUnitBlockHandlesOfABlock(*b);
        foreach(Handle blockNode, unitBlockNodes)
        {
            TruthValuePtr tv(SimpleTruthValue::createTV(1.0, 1.0));
            Handle evalLink =  addPropertyPredicate("part-of", blockNode, _entity->mEntityNode, tv);
            timeser->addTimeInfo(evalLink,timeStamp);
        }
    }

    /*
    //    (AtTimeLink (stv 1 1)
    //       (TimeNode "15818205490")
    //       (EvaluationLink (stv 1 0.0012484394)
    //          (PredicateNode "block-list")
    //          (ListLink
    //             (BlockEntityNode "7" (av 1000 0 0))
    //             (ListLink
    //                (StructureNode "id_CHUNK_3_2_0_BLOCK_1_12_99")
    //                (StructureNode "id_CHUNK_3_2_0_BLOCK_2_12_99")
    //                (StructureNode "id_CHUNK_3_2_0_BLOCK_2_11_99")
    //             )
    //          )
    //       )
    //    )
    HandleSeq blocklist;
    vector<opencog::spatial::Block3D*> blocks = (vector<opencog::spatial::Block3D*>&)(_entity->getBlockList());
    vector<opencog::spatial::Block3D*>::iterator it = blocks.begin();
    for (; it != blocks.end(); ++it)
    {
        opencog::spatial::Block3D* b = (opencog::spatial::Block3D*)(*it);
        HandleSeq unitBlockNodes = curMap->getAllUnitBlockHandlesOfABlock(*b);
        blocklist.insert(blocklist.end(), unitBlockNodes.begin(),unitBlockNodes.end());
    }

    Handle blocklistLink = atomspace->addLink(LIST_LINK, blocklist);

    SimpleTruthValue tv(1.0, 1.0);
    Handle evalLink = addPropertyPredicate(BLOCK_LIST, _entity->mEntityNode, blocklistLink,tv);

    timeser->addTimeInfo(evalLink,timeStamp);
    */

}

void SpaceServer::updateBlockEntityProperties(opencog::spatial::BlockEntity* _entity, unsigned long timestamp)
{
    addBlocksLisitPredicateToEntity(_entity, timestamp);

    // add the primary properties

    TruthValuePtr tv(SimpleTruthValue::createTV(1.0, 1.0));

    Handle evalLink;
    Handle numberNode;

    // add width : the x extent
    int x = _entity->getWidth();
    numberNode = atomspace->addNode(NUMBER_NODE,  opencog::toString(x).c_str() );
    evalLink = addPropertyPredicate( "width", _entity->mEntityNode, numberNode,tv);
    timeser->addTimeInfo(evalLink,timestamp);

    // add length: the y extent
    int y = _entity->getHeight();
    numberNode = atomspace->addNode(NUMBER_NODE,  opencog::toString(y).c_str() );
    evalLink = addPropertyPredicate("length", _entity->mEntityNode, numberNode,tv);
    timeser->addTimeInfo(evalLink,timestamp);

    // add height: how tall it's, the z extent
    int z = _entity->getHeight();
    numberNode = atomspace->addNode(NUMBER_NODE,  opencog::toString(z).c_str() );
    evalLink = addPropertyPredicate("height", _entity->mEntityNode, numberNode,tv);
    timeser->addTimeInfo(evalLink,timestamp);


    // todo: add secondary properties

}

void SpaceServer::updateBlockEntitiesProperties(unsigned long timestamp,HandleSeq &toUpdateHandles)
{
    vector<opencog::spatial::BlockEntity*>::iterator it = curMap->updateBlockEntityList.begin();
    opencog::spatial::BlockEntity* entity;
    for (; it != curMap->updateBlockEntityList.end(); ++it)
    {
        entity = (opencog::spatial::BlockEntity*)(*it);
        updateBlockEntityProperties(entity, timestamp);
        toUpdateHandles.push_back(entity->mEntityNode);
    }

    curMap->updateBlockEntityList.clear();

}

Handle SpaceServer::addPropertyPredicate(
        std::string predicateName,
        Handle a,
        Handle b,
        TruthValuePtr tv)
{
    Handle ph = atomspace->addNode(PREDICATE_NODE, predicateName);

    HandleSeq ll_out;
    ll_out.push_back(a);
    ll_out.push_back(b);

    Handle ll =  atomspace->addLink(LIST_LINK, ll_out);

    HandleSeq hs2;
    hs2.push_back(ph);
    hs2.push_back(ll);
    Handle result = atomspace->addLink(EVALUATION_LINK, hs2);

    atomspace->setTV(result, tv);

    //atomspace->setLTI(result, 1);
    return result;
}

