/*
 * opencog/embodiment/SpaceServer.cc
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
#include "SpaceServer.h"

#include <opencog/util/Logger.h>
#include <opencog/util/StringTokenizer.h>
#include <opencog/util/StringManipulator.h>
#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/AtomSpaceAsync.h>
#include <opencog/atomspace/TimeServer.h>
#include <opencog/atomspace/atom_types.h>

#include <string>
#include <vector>
#include <cstdlib>

//#define DPRINTF printf
#define DPRINTF(...)
// To simplify debug log output
#define ATOM_AS_STRING(h) (atomspace->atomAsString(h)->get_result().c_str())

using namespace opencog;

const char* SpaceServer::SPACE_MAP_NODE_NAME = "SpaceMap";

SpaceServer::SpaceServer(AtomSpaceAsync &_atomspace): atomspace(&_atomspace)
{
    // Default values (should only be used for test purposes)
    agentRadius = 0.25;
    xMin = 0;
    xMax = 256;
    yMin = 0;
    yMax = 256;
    xDim = 1024;
    yDim = 1024;
    timeServer = NULL;
}

SpaceServer::~SpaceServer()
{
    for (HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); itr++) {
        delete itr->second;
    }
}

void SpaceServer::setTimeServer(TimeServer *ts)
{
    timeServer = ts;
}

void SpaceServer::setAgentRadius(double _radius)
{
    if (agentRadius != _radius) {
        agentRadius = _radius;
        logger().info("SpaceServer - AgentRadius: %.3lf", agentRadius);
    }
}

void SpaceServer::setMapBoundaries(double _xMin, double _xMax, double _yMin, double _yMax,
                                   unsigned int _xDim, unsigned int _yDim)
{
    xMin = _xMin;
    xMax = _xMax;
    yMin = _yMin;
    yMax = _yMax;
    xDim = _xDim;
    yDim = _yDim;
    logger().info("SpaceServer - MapBondaries: xMin: %.3lf, xMax: %.3lf, "
            "yMin: %.3lf, yMax: %.3lf, xDim %d, yDim %d.",
            xMin, xMax, yMin, yMax, xDim, yDim);
}

SpaceServer::SpaceMap* SpaceServer::addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle)
{
    SpaceMap* map;
    HandleToSpaceMap::iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
        // a new map
        logger().info("SpaceServer - New map: xMin: %.3lf, xMax: %.3lf, yMin: %.3lf, yMax: %.3lf",
                     xMin, xMax, yMin, yMax);

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
                map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, agentRadius);
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
            map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, agentRadius);
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
                      double yaw, bool isObstacle)
{


    SpaceMap* map = addOrGetSpaceMap(keepPreviousMap, spaceMapHandle);
    DPRINTF("SpaceServer::add After addOrGet\n");

    logger().fine(
                 "SpaceServer::add map->xMin() = %lf, map->xMax() = %lf, map->yMin() = %lf, "
                 "map->yMax() = %lf, map->xGridWidth() = %lf, map->yGridWidth() = %lf",
                 map->xMin(), map->xMax(), map->yMin(), map->yMax(),
                 map->xGridWidth(), map->yGridWidth());

    SpaceServer::ObjectMetadata metadata(centerX, centerY, centerZ, length, width, height, yaw);
    bool mapContainsObject = map->containsObject(objectId);
    bool needUpdate = false;
    DPRINTF("SpaceServer::add After contains\n");
    if (mapContainsObject) {
        //const SpaceServer::ObjectMetadata& oldMetadata = map->getMetaData(objectId);
        const spatial::EntityPtr& oldEntity = map->getEntity( objectId );
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
            map->addObject(objectId, metadata, isObstacle );
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
    return (!sortedMapHandles.empty());
}

const SpaceServer::SpaceMap& SpaceServer::getLatestMap() const
    throw (opencog::AssertionException, std::bad_exception)
{
    OC_ASSERT(isLatestMapValid(), "SpaceServer - No lastestMap avaiable to return.");
    HandleToSpaceMap::const_iterator itr = spaceMaps.find(getLatestMapHandle());
    return *(itr->second);
}

Handle SpaceServer::getLatestMapHandle() const
{
    if (sortedMapHandles.empty()) {
        return Handle::UNDEFINED;
    }
    return sortedMapHandles.back();
}

Handle SpaceServer::getOlderMapHandle() const
{
    if (sortedMapHandles.empty()) {
        return Handle::UNDEFINED;
    }
    return sortedMapHandles.front();
}

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
}

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

void SpaceServer::removeMap(Handle spaceMapHandle)
{
    HandleToSpaceMap::iterator itr = spaceMaps.find(spaceMapHandle);
    if (itr != spaceMaps.end()) {
        std::vector<Handle>::iterator itr_map =
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

        delete itr->second;
        spaceMaps.erase(itr);

    }
}

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

bool SpaceServer::isMapPersistent(Handle spaceMapHandle) const
{
    return (persistentMapHandles.find(spaceMapHandle) != persistentMapHandles.end());
}

void SpaceServer::removeObject(const std::string& objectId)
{
    for (HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); itr++) {
        itr->second->removeObject(objectId);
    }
}

unsigned int SpaceServer::getSpaceMapsSize() const
{
    return spaceMaps.size();
}

void SpaceServer::clear()
{
    for (HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); itr++) {
        delete itr->second;
    }
    spaceMaps.clear();
}

Handle SpaceServer::getSpaceMapNode() 
{
    Handle result = spaceMapNodeHandle;
    if (result == Handle::UNDEFINED) {
        result = atomspace->addNode(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME)->get_result();
        atomspace->setLTI(result, 1);
    } else {
        if (atomspace->getLTI(result)->get_result() < 1) {
            atomspace->setLTI(result, 1);
        }
    }
    return result;
}

bool SpaceServer::addSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp,
                              double objX, double objY, double objZ,
                              double objLength, double objWidth, double objHeight,
                              double objYaw, bool isObstacle) {

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = timeServer->addTimeInfo(spaceMapNode, timestamp);
    bool result =  add( keepPreviousMap, spaceMapAtTimeLink, atomspace->getName(objectNode)->get_result(),
                        objX, objY, objZ, objLength, objWidth, objHeight, objYaw, isObstacle);

    return result;
}

Handle SpaceServer::addSpaceMap(unsigned long timestamp, SpaceServer::SpaceMap * spaceMap){

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = timeServer->addTimeInfo(spaceMapNode, timestamp);
    add(spaceMapAtTimeLink, spaceMap);

    return spaceMapAtTimeLink;
}

Handle SpaceServer::removeSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp) {

    logger().debug("%s(%s)\n", __FUNCTION__, atomspace->getName(objectNode)->get_result().c_str());

    Handle spaceMapNode = getSpaceMapNode();
    Handle spaceMapAtTimeLink = timeServer->addTimeInfo(spaceMapNode, timestamp);
    remove(keepPreviousMap, spaceMapAtTimeLink, atomspace->getName(objectNode)->get_result());

    return spaceMapAtTimeLink;
}

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
            atomspace->removeAtom(mapHandle, true)->get_result();
        }
    }
    logger().debug("SpaceServer - Number of deleted maps: %d.", j);
}

void SpaceServer::mapRemoved(Handle mapId)
{
    // Remove this atom from AtomSpace since its map does not exist anymore 
    atomspace->removeAtom(mapId)->get_result();
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
    return atomspace->getName(atomspace->getOutgoing(mapHandle, 0)->get_result())->get_result();
}    

std::string SpaceServer::mapToString(Handle mapHandle) const
{

    std::stringstream stringMap;
    stringMap.precision(25);
    const SpaceMap& map = getMap(mapHandle);

    stringMap << getMapIdString(mapHandle);
    stringMap << " ";

    stringMap << SpaceMap::toString( map );

    return stringMap.str( );
}

SpaceServer::TimestampMap SpaceServer::mapFromString(const std::string& stringMap)
{

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
}

SpaceServer& SpaceServer::operator=(const SpaceServer& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "SpaceServer - Cannot copy an object of this class");
}

/*class Fakeatomspace: public SpaceServerContainer {
    void mapRemoved(Handle mapId) {}
    void mapPersisted(Handle mapId) {}
    std::string getMapIdString(Handle mapId) {return "";}

};*/

SpaceServer::SpaceServer(const SpaceServer& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "SpaceServer - Cannot copy an object of this class");
}

