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
 *
 * Comment on 20150831 by @YiShan
 * Currently we seperate the entity operation in old SpaceMap to EntityRecorder class
 * This makes SpaceServer a little messy. But it's good for readibility of SpaceMap.
 */


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

#include <opencog/util/exceptions.h>

#include <opencog/atoms/base/Handle.h>
#include <opencog/truthvalue/TruthValue.h>

#include <opencog/spatial/3DSpaceMap/Block3DMapUtil.h>
#include <opencog/spatial/3DSpaceMap/OpencogOcTree.h>
#include <opencog/spatial/3DSpaceMap/EntityRecorder.h>

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
     * But the robot will enter more than one scene in the virtual world, so our HandleToScenes (std::map<Handle, Scene >),
     * will save all the scenes.
     * @author Shujing ke  rainkekekeke@gmail.com
     */

    class SpaceServer
    {
        friend class SpaceServerSavable;
        /**
         * Currently it's just a inner wrapper for SpaceMap and EntityRecorder
         * since it's ugly to use pair to access them.
         * The user of SpaceServer doesn't need to know this.
         */
        struct Scene
        {
        Scene(const string& mapName, double resolution):
            spaceMap(mapName, resolution), entityRecorder(){}
            spatial::OpencogOcTree spaceMap;
            spatial::EntityRecorder entityRecorder;
        };

        typedef std::map<Handle, Scene> HandleToScenes;

    public:

        typedef spatial::EntityRecorder EntityRecorder;
        typedef spatial::BlockVector SpaceMapPoint;
        typedef spatial::OpencogOcTree SpaceMap;

        explicit SpaceServer(AtomSpace&);
        virtual ~SpaceServer();

        /**
         * Gets a const reference to a specific SpaceMap for make queries
         * @throws RuntimeException if the given Handle is not a valid
         *        SpaceMap handle or if there is no SpaceMap for that node
         *         in SpaceServer
         */
        const SpaceMap& getMap(Handle spaceMapHandle) const;

        /**
         * Gets a const reference to a specific EntityRecorder for make queries
         * @throws RuntimeException if the given Handle is not a valid
         *        SpaceMap handle or if there is no EntityRecorder for that node
         *         in SpaceServer
         */
        const EntityRecorder& getEntityRecorder(Handle spaceMapHandle) const;

        /**
         * Checks if this SpaceServer contains a map with the given handle
         **/
        const bool containsMap(Handle spaceMapHandle) const;

        /**
         * Gets a const reference to the latest (more recent) map in this
         * SpaceServer
         */
        const SpaceMap& getLatestMap() const;

        const SpaceServer::EntityRecorder& getLatestEntityRecorder() const;

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

        Handle addOrGetSpaceMap(octime_t timestamp, std::string _mapName, double _resolution, const TimeDomain& timeDomain = DEFAULT_TIMEDOMAIN);

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

    private:

        AtomSpace* atomspace;
        TimeServer* timeser;

        /**
         * signal connections used to keep track of atom removal in the SpaceMap
         */
        int removedAtomConnection;
        int addedAtomConnection;

        void atomRemoved(const AtomPtr&);
        void atomAdded(const Handle&);

        Handle addPropertyPredicate(std::string predicateName,
                                    Handle,
                                    Handle,
                                    TruthValuePtr);

        /**
         * space maps contained by this SpaceServer. Each space map is
         * associated to an Atom handle, which is associated to a
         * specific zone map.
         */
        HandleToScenes scenes;

        /**
         * comment@20150520 by YiShan
         * Because we'd like to have multiple map in the new embodiment code,
         * we use the "spaceMaps" data members to represent all the current scenes we know.
         * The "curMap" and "curSpaceMapHandle" members are used in the old embodiment code to represent a single current scene.
         * To be compatible to the old code we still preserve these two members.
         */
        SpaceMap* curMap;
        EntityRecorder* curEntityRecorder;
        Handle curSpaceMapHandle;

        /**
         * Current agent radius
         */
        unsigned int agentRadius;

        /**
         * Current agent height
         */
        unsigned int agentHeight;

    };

} // namespace opencog

#endif // _OPENCOG_SPACESERVER_H
