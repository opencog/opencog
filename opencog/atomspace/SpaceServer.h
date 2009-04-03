/*
 * opencog/atomspace/SpaceServer.h
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
#ifndef SPACESERVER_H
#define SPACESERVER_H
/**
 * SpaceServer.h
 * This class is used to associate spatial information (coordinates) to atom handles (representing
 * entities) at a given timestamp.
 * It implements SavableRepository so that it can be saved and loaded by SavingLoading class.
 *
 * NOTE: This API and the spatial information for each entity in SpaceServer are provisory
 * and they will be improved as new features are needed...
 *
 * @author Welter Luigi
 */
#include "SavableRepository.h"
#include "types.h"
#include "SpaceServerContainer.h"

#include <opencog/util/exceptions.h>
#include <opencog/spatial/LocalSpaceMap2D.h>

#include <exception>
#include <string>
#include <map>


namespace opencog
{

class SpaceServer : public SavableRepository
{

public:

    static const char* SPACE_MAP_NODE_NAME;
    Handle latestSpaceMap;

    typedef Spatial::Point SpaceMapPoint;
    typedef Spatial::LocalSpaceMap2D SpaceMap;
    typedef Spatial::ObjectMetaData ObjectMetadata;
    typedef std::map<Handle, SpaceMap*> HandleToSpaceMap;
    typedef std::pair<unsigned long, SpaceMap*> TimestampMap;

    explicit SpaceServer(SpaceServerContainer&);
    virtual ~SpaceServer();


    /**
     * Gets a const reference to a specific SpaceMap for make queries
     * @throws RuntimeException if the given Handle is not a valid SpaceMap handle
     * or if there is no SpaceMap for that node
     * in SpaceServer
     */
    const SpaceMap& getMap(Handle spaceMapHandle) const throw (opencog::RuntimeException, std::bad_exception);

    /**
     * Checks if this SpaceServer contains a map with the given handle
     **/
    const bool containsMap(Handle spaceMapHandle) const;

    /**
     * Gets a const reference to the latest (more recent) map in this SpaceServer
     */
    const SpaceMap& getLatestMap() const throw (opencog::AssertionException, std::bad_exception);

    /**
     * Return true iff reference for latest map is not NULL
     */
    const bool isLatestMapValid() const;

    /**
     * Gets the Handle of the latest (more recent) map in this SpaceServer
     */
    Handle getLatestMapHandle() const;

    /**
     * Gets the Handle of the first (older) map in this SpaceServer
     */
    Handle getOlderMapHandle() const;

    /**
     * Gets the Handle of the previous map to the map associated to the given spaceMap handle.
     * @return the Handle of the desired spaceMap or Handle::UNDEFINED, if such map does not exist or
     * if the given spaceMap handle is not of a map inside this SpaceServer.
     */
    Handle getPreviousMapHandle(Handle spaceMapHandle) const;

    /**
     * Gets the Handle of the next map to the map associated to the given spaceMap handle.
     * @return the Handle of the desired spaceMap or Handle::UNDEFINED, if such map does not exist or
     * if the given spaceMap handle is not of a map inside this SpaceServer.
     */
    Handle getNextMapHandle(Handle spaceMapHandle) const;


    /**
     * Gets the number of SpaceMaps stores within the SpaceServer
     */
    unsigned int getSpaceMapsSize() const;

    /**
     * Remove the spaceMap for the given handle.
     */
    void removeMap(Handle spaceMapHandle);

    /**
     * Mark the spaceMap as persistent so that it cannot be removed by internal cleanups.
     */
    void markMapAsPersistent(Handle spaceMapHandle);

    /**
     * Check if the spaceMap with the given handle is persistent or not.
     */
    bool isMapPersistent(Handle spaceMapHandle) const;

    /**
     * Adds or updates into the space map with the given hangle, the object with the given id and spatial properties.
     * @return true if any property of the object has changed (or it's a new object). False, otherwise.
     */
    bool add(bool keepPreviousMap, Handle spaceMapHandle, const std::string& objectId,
             double centerX, double centerY, double length, double width,
             double height, double yaw, bool isObstacle = true);

    // Add a whole space map into the SpaceServer.
    // NOTE: This is just used when a whole space map is received from a remote
    // SpaceServer (in LearningServer, for instance).
    void add(Handle spaceMapHandle, SpaceMap * spaceMap);

    // Removes the objectNode from the map
    void remove(bool keepPreviousMap, Handle spaceMapHandle, const std::string& objectId);

    /**
     * Remove the object related to the given id from all spaceMaps in the SpaceServer.
     */
    void removeObject(const std::string& objectId);

    void setAgentRadius(double radius);
    void setMapBoundaries(double xMin, double xMax, double yMin, double yMax,
                          unsigned int xDim, unsigned int yDim);

    // Methods from SavableRepository interface:
    const char* getId() const;
    void saveRepository(FILE *) const;
    void loadRepository(FILE *fp, opencog::HandleMap<opencog::Atom*> *conv);
    void clear();

    /**
     * Converts the map identified by the given Handle into a string
     * representation of it.
     */
    std::string mapToString(Handle mapHandle) const;

    /**
     * Convert a string representation of a map to a Timestamped map.
     */
    static TimestampMap mapFromString(const std::string &stringMap);

    /**
     * Update the reference to the latest map in the SpaceServer.
     */
    // TODO: check if this is really needed
    //void updateLatestSpaceMap(Handle atTimeLink);


private:

    /**
     * Container where SpaceServer is inserted to (usualy an AtomSpace)
     */
    SpaceServerContainer& container;

    /**
     * space maps contained by this SpaceServer. Each space map is
     * associated to an Atom handle, which is associated to a specific timestamp.
     */
    HandleToSpaceMap spaceMaps;
    std::vector<Handle> sortedMapHandles;
    std::set<Handle> persistentMapHandles;

    /**
     * Current xMin
     */
    double xMin;

    /**
     * Current yMin
     */
    double yMin;

    /**
     * Current xMax
     */
    double xMax;

    /**
     * Current yMax
     */
    double yMax;

    /**
     * Current X direction grid map dimension
     */
    unsigned int xDim;

    /**
     * Current Y direction grid map dimension
     */
    unsigned int yDim;

    /**
     * Current agent radius
     */
    double agentRadius;

    SpaceMap* addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle);

    /**
     *
     */
    std::string mapObjectsToString(const SpaceServer::SpaceMap& map) const;

};
} // namespace opencog

#endif // SPACESERVER_H
