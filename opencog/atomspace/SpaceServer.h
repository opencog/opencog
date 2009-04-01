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
#include "AtomSpace.h"
#include "types.h"
#include "HandleMap.h"

#include <opencog/util/exceptions.h>
#include <opencog/embodiment/Spatial/LocalSpaceMap2D.h>

#include <exception>
#include <string>
#include <map>


namespace opencog {

class SpaceServer : public SavableRepository {

public:

    static const char* SPACE_MAP_NODE_NAME;
    Handle latestSpaceMap;

    //typedef Spatial::LocalSpaceMap2D<Handle, double, hashHandle, ObjMetaData> SpaceMap;
    typedef Spatial::Point SpaceMapPoint;
    typedef Spatial::LocalSpaceMap2D SpaceMap;
    typedef Spatial::ObjectMetaData ObjectMetadata;
    typedef std::map<Handle, SpaceMap*> HandleToSpaceMap;
    typedef std::pair<unsigned long, SpaceMap*> TimestampMap;

    explicit SpaceServer(AtomSpace &as);
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
     * Gets AtomSpace linked with spaceServer
     */
    AtomSpace & getAtomSpace() const {return atomSpace;}

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
     * Remove the object related to the given id from all spaceMaps in the SpaceServer.
     */
    void removeObject(const std::string& objectId);

    void setPetRadius(double radius);
    void setMapBoundaries(double xMin, double xMax, double yMin, double yMax,
                          unsigned int xDim, unsigned int yDim);

    // Methods from SavableRepository interface:
    const char* getId() const;
    void saveRepository(FILE *) const;
    void loadRepository(FILE *fp, opencog::HandleMap<opencog::Atom*> *conv);
    void clear();

    /**
     *  Methods moved from Petaverse AtomSpace.
     */

    /**
     * Adds space information about an object represented by a Node.
     * @param objectNode the Handle of the node that represents the object to be associated to the space info
     * @param timestamp The timestamp to be associated to this operation.
     * @param the remaining arguments are related to object's spatial information
     * @return true if any property of the object has changed (or it's a new object). False, otherwise.
     */
    bool addSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp,
                              double objX, double objY,
                              double objLength, double objWidth, double objHeight,
                              double objYaw, bool isObstacle = true);
    /**
     * Add a whole space map into the SpaceServer.
     * NOTE: This is just used when a whole space map is received
     * from a remote SpaceServer (in LearningServer, for instance).
     */
    Handle addSpaceMap(unsigned long timestamp, SpaceServer::SpaceMap * spaceMap);

    /**
     * Removes space information about an object from the latest map (object is no longer at map's range)
     * @param objectNode the Handle of the node that represents the object to be removed from space map
     * @param timestamp The timestamp to be associated to this operation.
     * @return handle of the atom that represents the SpaceMap (at the given timestamp) where the object was removed
     */
    Handle removeSpaceInfo(bool keepPreviousMap, Handle objectNode, unsigned long timestamp);


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

    /**
     * Creates the space map node, if not created yet.
     * returns its handle of the node.
     */
    Handle addSpaceMapNode();

     /**
     * Gets all SpaceMap handles that would be needed inside the given interval.
     * For getting the SpaceMap of each handle returned,
     * use the spaceServer.getMap(Handle spaceMapHandle) method.
     * @param out  the output iterator where the resulting handles will be added.
     * @param startMoment the start of the time interval for searching the maps
     * @param endMoment the end of the time interval for searching the maps
     *
     * Example of usage:
     *     HandleSeq result;
     *     spaceServer.getMapHandles(back_inserter(result),start,end);
     *     foreach(Handle h, result) {
     *         const SpaceMap& map = spaceServer().getMap(h);
     *         ...
     *     }
     */

    template<typename OutputIterator>
    OutputIterator getMapHandles(   OutputIterator outIt,
                                    unsigned long startMoment, unsigned long endMoment) const {
        Temporal t(startMoment, endMoment);
        vector<HandleTemporalPair> pairs;
        Handle spaceMapNode = atomSpace.getHandle(CONCEPT_NODE, SpaceServer::SPACE_MAP_NODE_NAME);
	    if (spaceMapNode != Handle::UNDEFINED) {
            // Gets the first map before the given interval, if any
            atomSpace.getTimeInfo(back_inserter(pairs), spaceMapNode, t, TemporalTable::PREVIOUS_BEFORE_START_OF);
            // Gets all maps inside the given interval, if any
            atomSpace.getTimeInfo(back_inserter(pairs), spaceMapNode, t, TemporalTable::STARTS_WITHIN);
            foreach(HandleTemporalPair pair, pairs) {
                *(outIt++) = atomSpace.getAtTimeLink(pair);
            }
        }
        return outIt;
    }

    /**
     * Remove old maps from SpaceServer in order to save memory. SpaceMaps
     * associated with exemplar sections, i.e., marked as persistent and the
     * latest (newest) space map are preserved.
     *
     * IMPORTANT: This function cannot be called while any trick exemplar is in progress.
     */
    void cleanupSpaceServer();

private:

    AtomSpace & atomSpace;
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
     * Current pet radius
     */
    double petRadius;

    /**
     * signal connections used to keep track of atom addition/removal/merging in the AtomTable
     */
    boost::signals::connection addedAtomConnection;
    boost::signals::connection removedAtomConnection; 
    boost::signals::connection mergedAtomConnection; 

    SpaceMap* addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle);

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
     *
     */
    std::string mapObjectsToString(const SpaceServer::SpaceMap& map) const;

    /**
     * Method to receive atom removal signals from AtomTable
     */
    void atomRemoved(Handle h);

    /**
     * Method to receive atom addition signals from AtomTable
     */
    void atomAdded(Handle h);

    /**
     * Method to receive atom merge signals from AtomTable
     */
    void atomMerged(Handle h);

};
} // namespace opencog

#endif // SPACESERVER_H
