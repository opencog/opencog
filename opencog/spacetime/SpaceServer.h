/*
 * opencog/spacetime/SpaceServer.h
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
#ifndef _OPENCOG_SPACESERVER_H
#define _OPENCOG_SPACESERVER_H

/**
 * SpaceServer.h
 * This class is used to associate spatial information (coordinates)
 * to atom handles (representing entities) at a given timestamp.
 * It implements SavableRepository so that it can be saved and loaded
 * by SavingLoading class.
 *
 * NOTE: This API and the spatial information for each entity in
 * SpaceServer are provisory and they will be improved as new features
 * are needed...
 *
 * @author Welter Luigi
 */
#include <exception>
#include <string>
#include <map>

#include <boost/signals2.hpp>

#include <opencog/util/exceptions.h>

#include <opencog/atomspace/Handle.h>
#include <opencog/atomspace/TruthValue.h>

#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>
#include <opencog/spatial/3DSpaceMap/Octree3DMapManager.h>
#include <opencog/spatial/3DSpaceMap/BlockEntity.h>

#include "SpaceServerContainer.h"
#include "Temporal.h"

namespace opencog
{
/** \addtogroup grp_spacetime
 *  @{
 */

class AtomSpace;
class TimeServer;
class SpaceServerSavable;
typedef std::string TimeDomain;
extern TimeDomain DEFAULT_TIMEDOMAIN;
/**
 * New SpaceServer for 3D map, using Octree3DMapManager, not using LocalSpaceMap2D anymore.
 * The biggest change in orgnization is every Map handle match one map for a scene (such as a room)
 * The former SpaceServer would generate a new map copy when there is a change in the map (such as something moved)
 * Now, we don't save all the copies. Now one scene only has one map handle (not create a lot via the time)
 * But the robot will enter more than one scene in the virtual world, so our HandleToSpaceMap (std::map<Handle, SpaceMap*>),
 * will save all the scenes.
 * @author Shujing ke  rainkekekeke@gmail.com
 */

class SpaceServer
{
    friend class SpaceServerSavable;

public:

    typedef spatial::BlockVector SpaceMapPoint;
    typedef spatial::Octree3DMapManager SpaceMap;
    typedef std::map<Handle, SpaceMap*> HandleToSpaceMap;

    explicit SpaceServer(AtomSpace&);
    virtual ~SpaceServer();

    /**
     * Gets a const reference to a specific SpaceMap for make queries
     * @throws RuntimeException if the given Handle is not a valid
     *        SpaceMap handle or if there is no SpaceMap for that node
     *         in SpaceServer
     */
    const SpaceMap& getMap(Handle spaceMapHandle) const
        throw (opencog::RuntimeException, std::bad_exception);
    /**
     * Checks if this SpaceServer contains a map with the given handle
     **/
    const bool containsMap(Handle spaceMapHandle) const;

    /**
     * Gets a const reference to the latest (more recent) map in this
     * SpaceServer
     */
    SpaceMap& getLatestMap() const
        throw (opencog::AssertionException, std::bad_exception);

    /**
     * Gets the Handle of the latest map (== the map for current scene)  in this SpaceServer
     */
    Handle getLatestMapHandle() const;

    /**
     * Sometimes, like when doing planning, we need to clone a spaceMap for reasoning about the things just happen in imagination, but not really happen,
     * to avoid really changing the real spaceMap.
     */
    SpaceMap* cloneTheLatestSpaceMap() const;
    SpaceMap* cloneSpaceMap(Handle spaceMapHandle) const;

    /**
     * create a new spaceMap for a new scene 
     * (it will create a new Octree3DMapMananger)
     * if there is already a spaceMap of this _mapName, 
     * just get it and set it to be the current map, 
     * not to create a new spaceMap
     */

    Handle addOrGetSpaceMap(octime_t timestamp, std::string _mapName,double _resolution, int _floorHeight, float _agentHeight, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);

    /**
     * comment@20150520 by YiShan
     * In the new embodiment code there may be multiple scenes, 
     * So the user may want to add space info in different space map.
     * To fix this, we insert the second parameter "spaceMapHandle"
     * to allow user to add space info in different scene.
     * Some member functions having the same problem also have the argument.
     */
    bool addSpaceInfo(Handle objectNode, Handle spaceMapHandle, 
                      bool isSelfObject, bool isAvatarEntity, 
                      octime_t timestamp,
                      double objX, double objY, double objZ,
                      const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);


    void removeSpaceInfo(Handle objectNode, Handle spaceMapHandle, octime_t timestamp = 0, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);

    /**
     * SpaceServerContainer virtual methods:
     */
    void mapRemoved(Handle mapId);
    void mapPersisted(Handle mapId);
    std::string getMapIdString(Handle mapId) const;

    
    /**
     * Sets the agent radius needed to define which grid cells are free for
     * navigation purposes
     */
    void setAgentRadius(unsigned int radius);

    /**
     * Sets the agent height.
     */
    void setAgentHeight(unsigned int height, Handle spaceMapHandle);

    void setTimeServer(TimeServer*);

    /**
     * Remove the spaceMap for the given handle.
     */
    void removeMap(Handle spaceMapHandle);

    /**
     * Gets the number of SpaceMaps stores within the SpaceServer
     */
    unsigned int getSpaceMapsSize() const;

    /**
     * Return true iff reference for latest map is not NULL
     */
    const bool isLatestMapValid() const;

    void clear();

    /**
     * Converts the map identified by the given Handle into a string
     * representation of it.
     */
    std::string mapToString(Handle mapHandle) const;

    /**
     * TODO
     */
    Handle mapFromString(const std::string& stringMap);

    /**
     * Overrides and declares copy constructor and equals operator 
     * as deleted function for avoiding large object copying by mistake.
     */
    SpaceServer& operator=(const SpaceServer&)=delete;
    SpaceServer(const SpaceServer&)=delete;

    void markCurMapPerceptedForFirstTime();

	//
	// Comment on 20150716 by Yi-Shan,
	// The following is old functions about BlockEntity add/remove/query
	// Because the BlockEntity feature has not been designed well, 
	// so we comment out all the code related to BlockEntity
	// Once we need to use it/decide to do it, maybe we'll need the legacy code.
	

    /**
     * after the first time percept a map, 
     * we should find all the blockEntities on this map
     */
    //void findAllBlockEntitiesOnTheMap(Handle spaceMapHandle);

    /**
     *  add all the newly constructed BlockEntity nodes to the atomspace
     */
    //void addBlockEntityNodes(HandleSeq &toUpdateHandles, Handle spaceMapHandle);

    /**
     *  add blocklist to an entity
     */
    //void addBlocksListPredicateToEntity(opencog::spatial::BlockEntity* _entity, const octime_t timeStamp, Handle spaceMapHandle, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);


    /**
     * add properties predicate link to an entity node when there is a change
     * this including addBlocksListPredicateToEntity
     */
    //void updateBlockEntityProperties(opencog::spatial::BlockEntity* entity, octime_t timestamp,Handle spaceMapHandle, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);

    //void updateBlockEntitiesProperties(octime_t timestamp, HandleSeq &toUpdateHandles, Handle spaceMapHandle, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);




private:

    AtomSpace* atomspace;
    TimeServer* timeser;

    /**
     * signal connections used to keep track of atom removal in the SpaceMap
     */
    boost::signals2::connection removedAtomConnection;
    boost::signals2::connection addedAtomConnection;

    void atomRemoved(AtomPtr);
    void atomAdded(Handle);

    Handle addPropertyPredicate(
        std::string predicateName,
        Handle,
        Handle,
        TruthValuePtr);

    /**
     * space maps contained by this SpaceServer. Each space map is
     * associated to an Atom handle, which is associated to a
     * specific zone map.
     */
    HandleToSpaceMap spaceMaps;

    /**
     * comment@20150520 by YiShan
     * Because we'd like to have multiple map in the new embodiment code,
     * we use the "spaceMaps" data members to represent all the current scenes we know. 
     * The "curMap" and "curSpaceMapHandle" members are used in the old embodiment code to represent a single current scene.
     * To be compatible to the old code we still preserve these two members.
     */
    SpaceMap* curMap;
    Handle curSpaceMapHandle;

    /**
     * Current agent radius
     */
    unsigned int agentRadius;

    /**
     * Current agent height
     */
    unsigned int agentHeight;


    /** 
     * comment@20150520 by YiShan
     * The following are some old data members which are not used in the new code.
     * Since they're used in the commented "addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle) function, we just comment them out.

    // Current xMin
     
    int xMin;

    // Current yMin
     
    int yMin;

    // Current zMin
     
    int zMin;

    // Current xMax
     
    int xMax;

    // Current yMax
     
    int yMax;

    // Current zMax
    int zMax;

    // Current X direction grid map dimension
    unsigned int xDim;

    // Current Y direction grid map dimension
    unsigned int yDim;

    // Current Z direction grid map dimension
    unsigned int zDim;

    // Current floor height (z)
    int floorHeight;

    */

};







//class SpaceServer
//{
//    friend class SpaceServerSavable;

//    Handle spaceMapNodeHandle;
//public:

//    static const char* SPACE_MAP_NODE_NAME;

//    typedef spatial::Point SpaceMapPoint;
//    typedef spatial::LocalSpaceMap2D SpaceMap;
//    typedef spatial::ObjectMetaData ObjectMetadata;
//    typedef std::map<Handle, SpaceMap*> HandleToSpaceMap;
//    typedef std::pair<octime_t, SpaceMap*> TimestampMap;

//    typedef spatial::Octree3DMapManager SpaceMap3D;

//    explicit SpaceServer(AtomSpaceAsync&);
//    virtual ~SpaceServer();

//    //! Set the timeServer
//    void setTimeServer(TimeServer *ts);

//    /**
//     * Gets a const reference to a specific SpaceMap for make queries
//     * @throws RuntimeException if the given Handle is not a valid
//     *        SpaceMap handle or if there is no SpaceMap for that node
//     *         in SpaceServer
//     */
//    const SpaceMap& getMap(Handle spaceMapHandle) const
//        throw (opencog::RuntimeException, std::bad_exception);

//    /**
//     * Checks if this SpaceServer contains a map with the given handle
//     **/
//    const bool containsMap(Handle spaceMapHandle) const;

//    /**
//     * Gets a const reference to the latest (more recent) map in this
//     * SpaceServer
//     */
//    const SpaceMap& getLatestMap() const
//        throw (opencog::AssertionException, std::bad_exception);

//    /**
//     * Return true iff reference for latest map is not NULL
//     */
//    const bool isLatestMapValid() const;

//    /**
//     * Gets the Handle of the latest (more recent) map in this SpaceServer
//     */
//    Handle getLatestMapHandle() const;

//    /**
//     * Gets the Handle of the first (older) map in this SpaceServer
//     */
//    Handle getOlderMapHandle() const;

//    /**
//     * Gets the Handle of the previous map to the map associated to the
//     * given spaceMap handle.
//     * @return the Handle of the desired spaceMap or Handle::UNDEFINED,
//     *      if such map does not exist or if the given spaceMap handle
//     *      is not of a map inside this SpaceServer.
//     */
//    Handle getPreviousMapHandle(Handle spaceMapHandle) const;

//    /**
//     * Gets the Handle of the next map to the map associated to the
//     * given spaceMap handle.
//     * @return the Handle of the desired spaceMap or Handle::UNDEFINED,
//     *         if such map does not exist or if the given spaceMap handle
//     *         is not of a map inside this SpaceServer.
//     */
//    Handle getNextMapHandle(Handle spaceMapHandle) const;

//    /**
//     * Gets the number of SpaceMaps stores within the SpaceServer
//     */
//    unsigned int getSpaceMapsSize() const;

//    /**
//     * Remove the spaceMap for the given handle.
//     */
//    void removeMap(Handle spaceMapHandle);

//    /**
//     * Mark the spaceMap as persistent so that it cannot be removed
//     * by internal cleanups.
//     */
//    void markMapAsPersistent(Handle spaceMapHandle);

//    /**
//     * Check if the spaceMap with the given handle is persistent or not.
//     */
//    bool isMapPersistent(Handle spaceMapHandle) const;

//    /**
//     * Adds or updates into the space map with the given handle, the
//     * object with the given id and spatial properties.
//     * @return true if any property of the object has changed (or it's
//     *          a new object). False, otherwise.
//     */
//    bool add(bool keepPreviousMap, Handle spaceMapHandle,
//             const std::string& objectId,
//             double centerX, double centerY, double centerZ, double length, double width,
//             double height, double yaw, const std::string& entityClass, bool isObstacle = true);

//    // Add a whole space map into the SpaceServer.
//    // NOTE: This is just used when a whole space map is received from a remote
//    // SpaceServer (in LearningServer, for instance).
//    void add(Handle spaceMapHandle, SpaceMap * spaceMap);

//    // Removes the object from the map
//    void remove(bool keepPreviousMap,
//                Handle spaceMapHandle, const std::string& objectId);

//    /**
//     * Remove the object related to the given id from all spaceMaps
//     * in the SpaceServer.
//     */
//    void removeObject(const std::string& objectId);

//    // Sets the agent radius needed to define which grid cells are free for
//    // navigation purposes
//    void setAgentRadius(double radius);

//    // Sets the agent height.
//    void setAgentHeight(double height);

//    // Sets the boundaries of the map and the number of grid points it will have
//    // in each dimension.:w
//    void setMapBoundaries(double _xMin, double _xMax, double _yMin, double _yMax,
//                          unsigned int _xDim, unsigned int _yDim, double _floorHeight,
//                          double _zMin = 0.0f, double _zMax = 0.0f, unsigned int _zDim = 0);

//    void clear();

//    /**
//     * Converts the map identified by the given Handle into a string
//     * representation of it.
//     */
//    std::string mapToString(Handle mapHandle) const;

//    /**
//     * Convert a string representation of a map to a Timestamped map.
//     */
//    static TimestampMap mapFromString(const std::string &stringMap);

//    /**
//     * Adds space information about an object represented by a Node.
//     * @param objectNode the Handle of the node that represents the object to be associated to the space info
//     * @param timestamp The timestamp to be associated to this operation.
//     * @param the remaining arguments are related to object's spatial information
//     * @return true if any property of the object has changed (or it's a new object). False, otherwise.
//     */
//    bool addSpaceInfo(bool keepPreviousMap, Handle objectNode, octime_t timestamp,
//                              double objX, double objY, double objZ,
//                              double objLength, double objWidth, double objHeight,
//                              double objYaw, bool isObstacle = true, const std::string& entityClass = "common");
//    /**
//     * Add a whole space map into the SpaceServer.
//     * NOTE: This is just used when a whole space map is received
//     * from a remote SpaceServer (in LearningServer, for instance).
//     */
//    Handle addSpaceMap(octime_t timestamp, SpaceServer::SpaceMap * spaceMap);

//    Handle getSpaceMapNode();

//    /**
//     * Removes space information about an object from the latest map (object is
//     * no longer at map's range)
//     * @param objectNode the Handle of the node that represents the object to
//     * be removed from space map
//     * @param timestamp The timestamp to be associated to this operation.
//     * @return handle of the atom that represents the SpaceMap (at the given
//     * timestamp) where the object was removed
//     */
//    Handle removeSpaceInfo(bool keepPreviousMap, Handle objectNode, octime_t timestamp);

//    /**
//     * Remove old maps from SpaceServer in order to save memory. SpaceMaps
//     * associated with exemplar sections, i.e., marked as persistent and the
//     * latest (newest) space map are preserved.
//     *
//     * IMPORTANT: This function cannot be called while any trick exemplar is in progress.
//     */
//    void cleanupSpaceServer();

//    // SpaceServerContainer virtual methods:
//    void mapRemoved(Handle mapId);
//    void mapPersisted(Handle mapId);
//    std::string getMapIdString(Handle mapId) const;

//private:

//    /**
//     * Container where SpaceServer is inserted to (usualy an AtomSpace)
//     */
//    AtomSpaceAsync* atomspace;
//    TimeServer* timeServer;

//    /**
//     * space maps contained by this SpaceServer. Each space map is
//     * associated to an Atom handle, which is associated to a
//     * specific timestamp.
//     */
//    HandleToSpaceMap spaceMaps;
//    std::vector<Handle> sortedMapHandles;
//    std::set<Handle> persistentMapHandles;

//    /**
//     * Current xMin
//     */
//    double xMin;

//    /**
//     * Current yMin
//     */
//    double yMin;

//    /**
//     * Current zMin
//     */
//    double zMin;

//    /**
//     * Current xMax
//     */
//    double xMax;

//    /**
//     * Current yMax
//     */
//    double yMax;

//    /**
//     * Current zMax
//     */
//    double zMax;

//    /**
//     * Current X direction grid map dimension
//     */
//    unsigned int xDim;

//    /**
//     * Current Y direction grid map dimension
//     */
//    unsigned int yDim;

//    /**
//     * Current Z direction grid map dimension
//     */
//    unsigned int zDim;

//    /**
//     * Current agent radius
//     */
//    double agentRadius;

//    /**
//     * Current floor height
//     */
//    double floorHeight;

//    /**
//     * Current agent height
//     */
//    double agentHeight;

//    SpaceMap* addOrGetSpaceMap(bool keepPreviousMap, Handle spaceMapHandle);

//    /**
//     * Overrides and declares copy constructor and equals operator as private
//     * for avoiding large object copying by mistake.
//     */
//    SpaceServer& operator=(const SpaceServer&);
//    SpaceServer(const SpaceServer&);

//};


/** @}*/
} // namespace opencog

#endif // _OPENCOG_SPACESERVER_H
