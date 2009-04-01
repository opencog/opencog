/*
 * opencog/embodiment/SpaceServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Luigi
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
#include "TLB.h"

#include <opencog/util/Logger.h>
#include <opencog/util/StringTokenizer.h>
#include <opencog/util/StringManipulator.h>

#include <opencog/atomspace/atom_types.h>

#include <string>
#include <vector>
#include <cstdlib>
#include <tr1/functional>

using namespace opencog;

const char* SpaceServer::SPACE_MAP_NODE_NAME = "SpaceMap";
#define DELIMITER " "

SpaceServer::SpaceServer(AtomSpace &as): atomSpace(as) {
    // Default values (should only be used for test purposes)
    petRadius = 0.25;
    xMin = 0;
    xMax = 256;
    yMin = 0;
    yMax = 256;
    xDim = 1024;
    yDim = 1024;

    latestSpaceMap = Handle::UNDEFINED;

    //connect signals
    AtomTable& at = (AtomTable&) as.getAtomTable();
    addedAtomConnection = at.addAtomSignal().connect(std::tr1::bind(&SpaceServer::atomAdded, this, std::tr1::placeholders::_1));
    removedAtomConnection = at.removeAtomSignal().connect(std::tr1::bind(&SpaceServer::atomRemoved, this, std::tr1::placeholders::_1));
    mergedAtomConnection = at.mergeAtomSignal().connect(std::tr1::bind(&SpaceServer::atomMerged, this, std::tr1::placeholders::_1));
}

SpaceServer::~SpaceServer() {
    addedAtomConnection.disconnect();
    removedAtomConnection.disconnect();
    mergedAtomConnection.disconnect();
    for(HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); itr++) {
        delete itr->second;
    }
}

void SpaceServer::setPetRadius(double _radius) {
    if (petRadius != _radius) {
        petRadius = _radius;
        logger().log(opencog::Logger::INFO, "SpaceServer - PetRadius: %.3lf", petRadius);
    }
}

void SpaceServer::setMapBoundaries(double _xMin, double _xMax, double _yMin, double _yMax,
                                   unsigned int _xDim, unsigned int _yDim) {
    xMin = _xMin;
    xMax = _xMax;
    yMin = _yMin;
    yMax = _yMax;
    xDim = _xDim;
    yDim = _yDim;
    logger().log(opencog::Logger::INFO, "SpaceServer - MapBondaries: xMin: %.3lf, xMax: %.3lf, yMin: %.3lf, yMax: %.3lf, xDim %d, yDim %d.", xMin, xMax, yMin, yMax, xDim, yDim);
}

SpaceServer::SpaceMap* SpaceServer::addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle) {
    SpaceMap* map;
    HandleToSpaceMap::iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
        // a new map
        logger().log(opencog::Logger::INFO,
            "SpaceServer - New map: xMin: %.3lf, xMax: %.3lf, yMin: %.3lf, yMax: %.3lf",
            xMin, xMax, yMin, yMax);
    // TODO: check if this is really needed
    //    updateLatestSpaceMap(spaceMapHandle);

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
                    if (sortedMapHandles.size() > 1 && persistentMapHandles.find(latestMapHandle) == persistentMapHandles.end()) {
                        Handle lastButOneMapHandle = *(sortedMapHandles.end()-2);
                        SpaceMap* lastButOneMap = spaceMaps[lastButOneMapHandle];
                        // Check if the 2 latest maps are equals
                        if(*latestMap == *lastButOneMap) {
                            logger().log(opencog::Logger::DEBUG, "SpaceServer - The 2 previous maps are equals. Previous map (%s) transfered to new map (%s).", TLB::getAtom(latestMapHandle)->toString().c_str(), TLB::getAtom(spaceMapHandle)->toString().c_str());
                            sortedMapHandles.erase(sortedMapHandles.end()-1);
                            spaceMaps.erase(latestMapHandle);
                            atomSpace.removeAtom(latestMapHandle);
                            map = latestMap; // reuse the spaceMap object
                            mapReused = true;
                            persistentMapHandles.insert(lastButOneMapHandle);
                            atomSpace.setLTI(lastButOneMapHandle, 1);
                            logger().log(opencog::Logger::DEBUG, "SpaceServer - Map (%s) marked as persistent.", TLB::getAtom(lastButOneMapHandle)->toString().c_str());
                        } else {
                            persistentMapHandles.insert(latestMapHandle);
                            atomSpace.setLTI(latestMapHandle, 1);
                            logger().log(opencog::Logger::DEBUG, "SpaceServer - Map (%s) marked as persistent.", TLB::getAtom(latestMapHandle)->toString().c_str());
                        }
                    } else {
                        persistentMapHandles.insert(latestMapHandle);
                        atomSpace.setLTI(latestMapHandle, 1);
                        logger().log(opencog::Logger::DEBUG, "SpaceServer - Map (%s) marked as persistent.", TLB::getAtom(latestMapHandle)->toString().c_str());
                    }
                } else if (persistentMapHandles.find(latestMapHandle) == persistentMapHandles.end()) {
                    logger().log(opencog::Logger::DEBUG, "SpaceServer - Previous map (%s) transfered to new map (%s).", TLB::getAtom(latestMapHandle)->toString().c_str(), TLB::getAtom(spaceMapHandle)->toString().c_str());
                    sortedMapHandles.erase(sortedMapHandles.end()-1);
                    spaceMaps.erase(latestMapHandle);
                    map = latestMap; // reuse the spaceMap object
		    		mapReused = true;
                }
                if (!mapReused) {
                    // Create the new one by cloning the latest map
                    logger().log(opencog::Logger::DEBUG, "SpaceServer - New map (%s) cloned from previous map (%s).", TLB::getAtom(spaceMapHandle)->toString().c_str(), TLB::getAtom(latestMapHandle)->toString().c_str());
                    map = latestMap->clone();
                }
            } else {
                // latest map dimensions do not match new map dimensions.
                // Create an empty map
                logger().log(opencog::Logger::DEBUG, "SpaceServer - New map (%s) created by copying.", TLB::getAtom(spaceMapHandle)->toString().c_str());
                map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, petRadius);
                // Copy each object in latest map into the new map
                map->copyObjects(*latestMap);
                if (!keepPreviousMap) {
                    logger().log(opencog::Logger::DEBUG, "SpaceServer - Previous map (%s) removed.", TLB::getAtom(latestMapHandle)->toString().c_str());

                    sortedMapHandles.erase(sortedMapHandles.end()-1);
                    spaceMaps.erase(latestMapHandle);
                    delete latestMap;
                }
            }
        } else {
            // Create first map
            map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, petRadius);
            logger().log(opencog::Logger::DEBUG, "SpaceServer - First map (%s) created", TLB::getAtom(spaceMapHandle)->toString().c_str());
        }
        spaceMaps[spaceMapHandle] = map;
        sortedMapHandles.push_back(spaceMapHandle);
        logger().log(opencog::Logger::DEBUG, "SpaceServer - spaceMaps size: %u, sortedMapHandles size: %u", spaceMaps.size(), sortedMapHandles.size());

    } else {
        // get the existing map
        map = itr->second;
    }
    return map;
}

bool SpaceServer::add(bool keepPreviousMap, Handle spaceMapHandle, const std::string& objectId,
                      double centerX, double centerY,
                      double length, double width, double height,
                      double yaw, bool isObstacle) {


    SpaceMap* map = addOrGetSpaceMap(keepPreviousMap, spaceMapHandle);
//    logger().log(opencog::Logger::FINE, "SpaceServer::add After addOrGet");

    logger().log(opencog::Logger::FINE,
        "SpaceServer::add map->xMin() = %lf, map->xMax() = %lf, map->yMin() = %lf, map->yMax() = %lf, map->xGridWidth() = %lf, map->yGridWidth() = %lf",
            map->xMin(), map->xMax(), map->yMin(), map->yMax(),
            map->xGridWidth(), map->yGridWidth());

    SpaceServer::ObjectMetadata metadata(centerX, centerY, length, width, height, yaw);
    bool mapContainsObject = map->containsObject(objectId);
    bool needUpdate = false;
//    logger().log(opencog::Logger::FINE, "SpaceServer::add After contains");
    if (mapContainsObject) {
      //const SpaceServer::ObjectMetadata& oldMetadata = map->getMetaData(objectId);
      const Spatial::EntityPtr& oldEntity = map->getEntity( objectId );
      SpaceServer::ObjectMetadata oldMetadata( oldEntity->getPosition( ).x, 
					       oldEntity->getPosition( ).y,
					       oldEntity->getLength( ),
					       oldEntity->getWidth( ),
					       oldEntity->getHeight( ),
					       oldEntity->getOrientation( ).getRoll( ) );
      
//    logger().log(opencog::Logger::FINE, "SpaceServer::add After getMetaData");

        if (metadata != oldMetadata) {
            needUpdate = true;
            logger().log(opencog::Logger::FINE,
                    "SpaceServer::add Old metadata (x=%lf, y=%lf, length=%lf, width=%lf, height=%lf, yaw=%lf) is different: object must be updated",
                    oldMetadata.centerX, oldMetadata.centerY,
                    oldMetadata.length, oldMetadata.width, oldMetadata.height,
                    oldMetadata.yaw);
        } else {
            bool wasObstacle = map->isObstacle(objectId);
            if (isObstacle != wasObstacle) {
                needUpdate = true;
                logger().log(opencog::Logger::FINE, "SpaceServer::add Object is %san obstacle now. So, it must be updated.",
                    isObstacle?" ":"not ");
            }
        }
    } else {
        logger().log(opencog::Logger::FINE, "SpaceServer::add Object does not exist in the map yet. So, it will be added.");
    }

    if (!mapContainsObject || needUpdate) {

        logger().log(opencog::Logger::DEBUG,
        "SpaceServer - add(mapH=%lu, objId=%s, x=%lf, y=%lf, length=%lf, width=%lf, height=%lf, yaw=%lf, isObstacle=%d)",
                    spaceMapHandle.value(), objectId.c_str(), centerX, centerY,
                    length, width, height, yaw, isObstacle);

        if (mapContainsObject) {
            logger().log(opencog::Logger::FINE,
                    "SpaceServer::add - updating object into the space map");

	    map->updateObject(objectId, metadata, isObstacle );

        } else {
            logger().log(opencog::Logger::FINE, "SpaceServer::add - adding object into the space map");
              map->addObject(objectId, metadata, isObstacle );
        }
        return true;
    }
    return false;
}

void SpaceServer::add(Handle spaceMapHandle, SpaceMap * map){

    logger().log(opencog::Logger::INFO, "SpaceServer - New map (%s) added",
                     TLB::getAtom(spaceMapHandle)->toString().c_str());
    sortedMapHandles.push_back(spaceMapHandle);
    spaceMaps[spaceMapHandle] = map;
    logger().log(opencog::Logger::DEBUG, "SpaceServer - spaceMaps size: %d",
                     spaceMaps.size());

}

void SpaceServer::remove(bool keepPreviousMap, Handle spaceMapHandle, const std::string& objectId) {
    logger().log(opencog::Logger::INFO, "SpaceServer::remove()");
    SpaceMap* map = addOrGetSpaceMap(keepPreviousMap, spaceMapHandle);
    if (map->containsObject(objectId)) map->removeObject(objectId);
}

const SpaceServer::SpaceMap& SpaceServer::getMap(Handle spaceMapHandle) const throw (opencog::RuntimeException, std::bad_exception) {
    logger().log(opencog::Logger::FINE, "SpaceServer::getMap() for mapHandle = %s", spaceMapHandle != Handle::UNDEFINED ? TLB::getAtom(spaceMapHandle)->toString().c_str(): "Handle::UNDEFINED");

    HandleToSpaceMap::const_iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
       throw opencog::RuntimeException(TRACE_INFO,
                "SpaceServer - Found no SpaceMap associate with handle: '%s'.",
                TLB::getAtom(spaceMapHandle)->toString().c_str());
    }

    return *(itr->second);
}

const bool SpaceServer::containsMap(Handle spaceMapHandle) const {
    return (spaceMaps.find(spaceMapHandle) != spaceMaps.end());
}

const bool SpaceServer::isLatestMapValid() const {
    return (!sortedMapHandles.empty());
}

const SpaceServer::SpaceMap& SpaceServer::getLatestMap() const throw (opencog::AssertionException, std::bad_exception) {
    cassert(TRACE_INFO, isLatestMapValid(), "SpaceServer - No lastestMap avaiable to return.");
    HandleToSpaceMap::const_iterator itr = spaceMaps.find(getLatestMapHandle());
    return *(itr->second);
}

Handle SpaceServer::getLatestMapHandle() const {
    if (sortedMapHandles.empty()) {
        return Handle::UNDEFINED;
    }
    return sortedMapHandles.back();
}

Handle SpaceServer::getOlderMapHandle() const {
    if (sortedMapHandles.empty()) {
        return Handle::UNDEFINED;
    }
    return sortedMapHandles.front();
}

Handle SpaceServer::getPreviousMapHandle(Handle spaceMapHandle) const {
    Handle result = Handle::UNDEFINED;
    if (spaceMapHandle != Handle::UNDEFINED) {
        std::vector<Handle>::const_iterator itr = std::lower_bound(sortedMapHandles.begin(), sortedMapHandles.end(), spaceMapHandle);
        if (itr != sortedMapHandles.begin() && *itr == spaceMapHandle) {
            result = *(--itr);
        }
    }
    return result;
}

Handle SpaceServer::getNextMapHandle(Handle spaceMapHandle) const {
    Handle result = Handle::UNDEFINED;
    if (spaceMapHandle != Handle::UNDEFINED) {
        std::vector<Handle>::const_iterator itr = std::lower_bound(sortedMapHandles.begin(), sortedMapHandles.end(), spaceMapHandle);    
        if(*itr == spaceMapHandle) {
	  ++itr;
	  if(itr != sortedMapHandles.end())
            result = *itr;
        }
    }
    return result;
}

void SpaceServer::removeMap(Handle spaceMapHandle) {
    HandleToSpaceMap::iterator itr = spaceMaps.find(spaceMapHandle);
    if (itr != spaceMaps.end()) {

        std::vector<Handle>::iterator itr_map = std::find(sortedMapHandles.begin()
        										, sortedMapHandles.end(), spaceMapHandle);
		if (*itr_map == spaceMapHandle) {
	        sortedMapHandles.erase(itr_map);
		} else {
			if (itr_map != sortedMapHandles.end()) {
			    logger().log(opencog::Logger::ERROR, "SpaceServer::removeSpaceMap - Removed is not mapHandle.\n");
		        sortedMapHandles.erase(itr_map);
			}
		    logger().log(opencog::Logger::ERROR
		    				, "SpaceServer::removeSpaceMap - Trying to remove inexisting map. spaceMapSize = %d sortedMapHandlesSize = %d\n"
		    				, spaceMaps.size(), sortedMapHandles.size());
		}

		delete itr->second;
        spaceMaps.erase(itr);

    }
}

void SpaceServer::markMapAsPersistent(Handle spaceMapHandle) {
    HandleToSpaceMap::const_iterator itr = spaceMaps.find(spaceMapHandle);

    if (itr == spaceMaps.end()) {
       throw opencog::RuntimeException(TRACE_INFO,
                "SpaceServer - Found no SpaceMap associate with handle: '%s'.",
                TLB::getAtom(spaceMapHandle)->toString().c_str());
    }
    persistentMapHandles.insert(spaceMapHandle);
    atomSpace.setLTI(spaceMapHandle, 1);
}

bool SpaceServer::isMapPersistent(Handle spaceMapHandle) const {
    return (persistentMapHandles.find(spaceMapHandle) != persistentMapHandles.end());
}

void SpaceServer::removeObject(const std::string& objectId) {
    for(HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); itr++) {
        itr->second->removeObject(objectId);
    }
}

const char* SpaceServer::getId() const {
    static const char* id = "SpaceServer";
    return id;
}

unsigned int SpaceServer::getSpaceMapsSize() const {
    return spaceMaps.size();
}

void SpaceServer::saveRepository(FILE * fp) const {
    logger().log(opencog::Logger::DEBUG, "Saving %s (%ld)\n", getId(), ftell(fp));
    unsigned int mapSize = spaceMaps.size();
    fwrite(&mapSize, sizeof(unsigned int), 1, fp);
    for(std::vector<Handle>::const_iterator itr = sortedMapHandles.begin(); itr != sortedMapHandles.end(); itr++) {
        Handle mapHandle = *itr;
        fwrite(&mapHandle, sizeof(Handle), 1, fp);
        SpaceMap* map = spaceMaps.find(mapHandle)->second;
        float xMin,xMax,yMin,yMax,radius;
        unsigned int xDim, yDim;
        xMin = map->xMin();
        xMax = map->xMax();
        yMin = map->yMin();
        yMax = map->yMax();
        radius = map->radius();
        xDim = map->xDim();
        yDim = map->yDim();
        fwrite(&xMin, sizeof(float), 1, fp);
        fwrite(&xMax, sizeof(float), 1, fp);
        fwrite(&yMin, sizeof(float), 1, fp);
        fwrite(&yMax, sizeof(float), 1, fp);
        fwrite(&radius, sizeof(float), 1, fp);
        fwrite(&xDim, sizeof(unsigned int), 1, fp);
        fwrite(&yDim, sizeof(unsigned int), 1, fp);
        map->save(fp);
    }
    unsigned int persistentHandlesSize = persistentMapHandles.size();
    fwrite(&persistentHandlesSize, sizeof(unsigned int), 1, fp);
    for(std::set<Handle>::const_iterator itr = persistentMapHandles.begin(); itr != persistentMapHandles.end(); itr++) {
        Handle mapHandle = *itr;
        fwrite(&mapHandle, sizeof(Handle), 1, fp);
    }
}

void SpaceServer::loadRepository(FILE *fp, opencog::HandleMap<opencog::Atom*> *conv) {
    logger().log(opencog::Logger::DEBUG, "Loading %s (%ld)\n", getId(), ftell(fp));

    unsigned int mapSize;
    fread(&mapSize, sizeof(unsigned int), 1, fp);
    for(unsigned int i = 0; i < mapSize; i++) {
        Handle mapHandle;
        fread(&mapHandle, sizeof(Handle), 1, fp);
        float xMin,xMax,yMin,yMax,radius;
        unsigned int xDim, yDim;
        fread(&xMin, sizeof(float), 1, fp);
        fread(&xMax, sizeof(float), 1, fp);
        fread(&yMin, sizeof(float), 1, fp);
        fread(&yMax, sizeof(float), 1, fp);
        fread(&radius, sizeof(float), 1, fp);
        fread(&xDim, sizeof(unsigned int), 1, fp);
        fread(&yDim, sizeof(unsigned int), 1, fp);
        SpaceMap* map = new SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, radius);
        map->load(fp);

        cassert(TRACE_INFO, conv->contains(mapHandle),
                "SpaceServer - HandleMap conv does not contain mapHandle.");
        Handle newMapHandle = TLB::getHandle(conv->get(mapHandle));
        spaceMaps[newMapHandle] = map;
        sortedMapHandles.push_back(newMapHandle);
    }
    unsigned int persistentHandlesSize;
    fread(&persistentHandlesSize, sizeof(unsigned int), 1, fp);
    for(unsigned int i = 0; i < persistentHandlesSize; i++) {
        Handle mapHandle;
        fread(&mapHandle, sizeof(Handle), 1, fp);
        cassert(TRACE_INFO, conv->contains(mapHandle),
                "SpaceServer - HandleMap conv does not contain mapHandle.");
        Handle newMapHandle = TLB::getHandle(conv->get(mapHandle));
        persistentMapHandles.insert(newMapHandle);
    }
}

void SpaceServer::clear() {
    for(HandleToSpaceMap::iterator itr = spaceMaps.begin(); itr != spaceMaps.end(); itr++) {
        delete itr->second;
    }
    spaceMaps.clear();
}

bool SpaceServer::addSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp,
                              double objX, double objY,
                              double objLength, double objWidth, double objHeight,
                              double objYaw, bool isObstacle) {

    Handle spaceMapNode = addSpaceMapNode();
    Handle spaceMapAtTimeLink = atomSpace.addTimeInfo(spaceMapNode, timestamp);
    bool result =  add( keepPreviousMap, spaceMapAtTimeLink, atomSpace.getName(objectNode),
                        objX, objY, objLength, objWidth, objHeight, objYaw, isObstacle);

    //if (!keepPreviousMap) cleanupSpaceServer();
    return result;
}

Handle SpaceServer::addSpaceMap(unsigned long timestamp, SpaceServer::SpaceMap * spaceMap){

    Handle spaceMapNode = addSpaceMapNode();
    Handle spaceMapAtTimeLink = atomSpace.addTimeInfo(spaceMapNode, timestamp);
    add(spaceMapAtTimeLink, spaceMap);

    return spaceMapAtTimeLink;
}

Handle SpaceServer::removeSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp) {

    logger().log(opencog::Logger::DEBUG, "%s(%s)\n", __FUNCTION__, atomSpace.getName(objectNode).c_str());

    Handle spaceMapNode = addSpaceMapNode();
    Handle spaceMapAtTimeLink = atomSpace.addTimeInfo(spaceMapNode, timestamp);
    remove(keepPreviousMap, spaceMapAtTimeLink, atomSpace.getName(objectNode));

    //if (!keepPreviousMap) cleanupSpaceServer();
    return spaceMapAtTimeLink;
}


std::string SpaceServer::mapToString(Handle mapHandle) const{

  std::stringstream stringMap;
  stringMap.precision(16);
  SpaceMap map = getMap(mapHandle);

    // Currently the mapHandle is of AtTimeLink(TimeNode:"<timestamp>" , ConceptNode:"SpaceMap")
    // So, in order to get the timestamp is the name of the TimeNode
  stringMap << atomSpace.getName(atomSpace.getOutgoing(mapHandle, 0));
  stringMap << DELIMITER;
  stringMap << map.xMin();
  stringMap << DELIMITER;
  stringMap << map.xMax();
  stringMap << DELIMITER;
  stringMap << map.yMin();
  stringMap << DELIMITER;
  stringMap << map.yMax();
  stringMap << DELIMITER;
  stringMap << map.radius();
  stringMap << DELIMITER;
  stringMap << map.xDim();
  stringMap << DELIMITER;
  stringMap << map.yDim();
  stringMap << DELIMITER;

  stringMap << mapObjectsToString(map);
  //std::cout << "Codified string: " << stringMap.str( ) << std::endl;
  return stringMap.str( );
}

std::string SpaceServer::mapObjectsToString(const SpaceServer::SpaceMap& map) const {
  std::stringstream mapObjects;
  mapObjects.precision(16);
  std::vector<std::string> mapObjectsIds;
  
  map.findAllEntities(back_inserter(mapObjectsIds));
  mapObjects << mapObjectsIds.size();
  mapObjects << DELIMITER;
  
  for(unsigned int i = 0; i < mapObjectsIds.size(); i++){        
    const Spatial::EntityPtr& entity = map.getEntity( mapObjectsIds[i] );

    // object name
    mapObjects << entity->getName( );
    mapObjects << DELIMITER;

    mapObjects << entity->getPosition( ).x;
    mapObjects << DELIMITER;

    mapObjects << entity->getPosition( ).y;
    mapObjects << DELIMITER;

    mapObjects << entity->getLength( );
    mapObjects << DELIMITER;
    
    mapObjects << entity->getWidth( );
    mapObjects << DELIMITER;

    mapObjects << entity->getHeight( );
    mapObjects << DELIMITER;

    mapObjects << entity->getOrientation( ).getRoll( );      
    mapObjects << DELIMITER;

    mapObjects << (entity->getBooleanProperty( Spatial::Entity::OBSTACLE ) ? "y" :"n");
    mapObjects << DELIMITER;

  } // for
  return mapObjects.str( );
}

SpaceServer::TimestampMap SpaceServer::mapFromString(const std::string& stringMap){

  std::stringstream parser( stringMap );
  parser.precision(16);
  
  //opencog::StringTokenizer st(stringMap, std::string(DELIMITER));
  unsigned long timestamp;
  parser >> timestamp;
  //= atoll(st.nextToken().c_str());

    double xMin,xMax,yMin,yMax,radius;
    unsigned int xDim, yDim;

    /*
    xMin   = atof(st.nextToken().c_str());
    xMax   = atof(st.nextToken().c_str());
    yMin   = atof(st.nextToken().c_str());
    yMax   = atof(st.nextToken().c_str());
    radius = atof(st.nextToken().c_str());
    xDim   = atoi(st.nextToken().c_str());
    yDim   = atoi(st.nextToken().c_str());
    */
    parser >> xMin >> xMax >> yMin >> yMax >> radius >> xDim >> yDim;
    // create the new map
    SpaceMap *spaceMap = new SpaceServer::SpaceMap(xMin, xMax, xDim, yMin, yMax, yDim, radius);

    int numObjects = 0;// = atoi(st.nextToken().c_str());
    parser >> numObjects;
    for(int i = 0; i < numObjects; i++){
      std::string objId;
      parser >> objId;
      //string objId = st.nextToken().c_str();

        // Object Metadata
        // NOTE: Temp variables must be used. Otherwise (passing them directly to ObjMetaData's constructor), the
        // compiler will evaluate them in the inverse order (from the right to the left)
      double centerX, centerY, length, width, height, yaw;
      std::string obstacleFlag;
      parser >> centerX >> centerY >> length >> width >> height >> yaw >> obstacleFlag;
      

      /*
      double centerX = atof(st.nextToken().c_str());
        double centerY = atof(st.nextToken().c_str());
        double length = atof(st.nextToken().c_str());
        double width = atof(st.nextToken().c_str());
        double height = atof(st.nextToken().c_str());
        double yaw = atof(st.nextToken().c_str());
	bool isObstacle = ( st.nextToken( ) == "y" );
	*/
        SpaceServer::ObjectMetadata metadata(centerX, centerY, length, width, height, yaw);

        spaceMap->addObject(objId, metadata, ( obstacleFlag == "y" ) );

	//std::cout << "Created entity: " << spaceMap->getEntity( objId )->toString( ) << " MetaData: " << centerX << " " << centerY << " " << length << " " << width << " " << height << " " << yaw << " " << obstacleFlag << std::endl;
    }

    SpaceServer::TimestampMap timestampMap(timestamp, spaceMap);

    return timestampMap;
}

/* TODO: Check if this is really needed
void SpaceServer::updateLatestSpaceMap(Handle atTimeLink)
{
    if (latestSpaceMap != Handle::UNDEFINED) 
    {
        atomSpace.removeAtom(latestSpaceMap);
    }
    HandleSeq hs;
    hs.push_back(atTimeLink);
    latestSpaceMap = atomSpace.getHandle(LATEST_LINK, hs);
    if (latestSpaceMap == Handle::UNDEFINED) 
    {
        latestSpaceMap = atomSpace.addLink(LATEST_LINK, hs);
        atomSpace.setLTI(latestSpaceMap, 1);
    } 
    else 
    {
        if (atomSpace.getLTI(latestSpaceMap) < 1) 
        {
            atomSpace.setLTI(latestSpaceMap, 1);
        }
    }
}
*/

Handle SpaceServer::addSpaceMapNode() 
{
    Handle result = atomSpace.getHandle(CONCEPT_NODE, SPACE_MAP_NODE_NAME);
    if (result == Handle::UNDEFINED) 
    {
        result = atomSpace.addNode(CONCEPT_NODE, SPACE_MAP_NODE_NAME);
        atomSpace.setLTI(result, 1);
    } 
    else 
    {
        if (atomSpace.getLTI(result) < 1) 
        {
            atomSpace.setLTI(result, 1);
        }
    }
    return result;
}

void SpaceServer::cleanupSpaceServer(){

    // sanity checks
    if (getSpaceMapsSize() < 1) {
        logger().log(opencog::Logger::DEBUG,
                       "AtomSpace - No need to clean SpaceServer. It has no space map yet.");
        return;
    }

    // sanity tests passed, cleaning SpaceServer
    Handle spaceMapNode = addSpaceMapNode();

    // get all HandleTemporalPairs associated with the SpaceMap concept node.
    std::vector<HandleTemporalPair> pairs;
    atomSpace.getTimeInfo(back_inserter(pairs), spaceMapNode);

    int j = 0;
    // remember to leave at least one map in SpaceServer, the newer one.
    for(unsigned int i = 0; i < pairs.size() - 1; i++){

        // get SpaceMap handles
        Handle mapHandle = atomSpace.getAtTimeLink(pairs[i]);

        // mapHandle not among the ones that should be preserved
        if (!containsMap(mapHandle) || !isMapPersistent(mapHandle)){
            j++;
            logger().log(opencog::Logger::DEBUG, "AtomSpace - Removing map (%s)", TLB::getAtom(mapHandle)->toString().c_str());
            // remove map from SpaceServer, and timeInfo from TimeServer and AtomSpace
            atomSpace.removeAtom(mapHandle, true);
        }
    }
    logger().log(opencog::Logger::DEBUG, "AtomSpace - Number of deleted maps: %d.", j);
}

void SpaceServer::atomAdded(Handle h) {
    //logger().log(opencog::Logger::DEBUG, "SpceServer::atomAdded(%lu)", h.value());
}

void SpaceServer::atomRemoved(Handle h) {
    //logger().log(opencog::Logger::DEBUG, "SpceServer::atomRemoved(%lu)", h.value());
    Type type = atomSpace.getType(h);
    if (type == AT_TIME_LINK) {
        Handle timedAtom = atomSpace.getOutgoing(h, 1);
        // outgoingSet[1] is a SpaceMap concept node, remove related map
        // from SpaceServer
        if( getAtomSpace( ).getHandle(CONCEPT_NODE, SPACE_MAP_NODE_NAME) == timedAtom ){
           this->removeMap(h);
        } // if
    } else if ( atomSpace.inheritsType(type, SL_OBJECT_NODE) ) {
        this->removeObject(atomSpace.getName(h));
    } // else if
}

void SpaceServer::atomMerged(Handle h) {
    //logger().log(opencog::Logger::DEBUG, "SpceServer::atomMerged(%lu)", h.value());
    // Restore the default STI value if it has decayed 
    // TODO: Remove this code when the merge of atoms consider the STI values this way as well.
    if (atomSpace.getSTI(h) < AttentionValue::DEFAULTATOMSTI) {
        atomSpace.setSTI(h, AttentionValue::DEFAULTATOMSTI);
    }
}
