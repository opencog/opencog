/*
 * opencog/embodiment/Learning/Filter/EntityRelevanceFilter.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Carlos Lopes
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

#ifndef ENTITYRELEVANCEFILTER_H_
#define ENTITYRELEVANCEFILTER_H_

#include <set>
#include <vector>
#include <string>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/spacetime/SpaceServer.h>

#include <opencog/embodiment/AtomSpaceExtensions/atom_types.h>
#include <opencog/embodiment/Learning/behavior/WorldProvider.h>

typedef std::vector<opencog::combo::definite_object> definite_object_vec;
typedef definite_object_vec::iterator definite_object_vec_it;
typedef definite_object_vec::const_iterator definite_object_vec_const_it;
typedef std::set<definite_object_vec> definite_object_vec_set;
typedef definite_object_vec_set::iterator definite_object_vec_set_it;
typedef definite_object_vec_set::const_iterator definite_object_vec_set_const_it;
typedef std::map<opencog::combo::definite_object, definite_object_vec_set> agent_to_actions;
typedef agent_to_actions::iterator agent_to_actions_it;
typedef agent_to_actions::const_iterator agent_to_actions_const_it;

using namespace opencog;

class EntityRelevanceFilter
{

private:
    std::vector<Handle> spaceMapHandles;
    std::vector<std::string> spaceMapIds;

    const std::string _selfID; //the ID corresponding to self
    const std::string _ownerID; //the ID corresponding to owner

public:
    EntityRelevanceFilter();
    //selfID and owner are given here to be changed into
    //"self" and "owner" respectively
    EntityRelevanceFilter(const SpaceServer::SpaceMap &spaceMap,
                          const std::string& selfID,
                          const std::string& ownerID);
    ~EntityRelevanceFilter();

    /**
     * Gets the entities (i.e. definite_object) in a LocaSpaceMap2D related to the given type (and its decendents).
     * NOTE: the type MUST be a NODE or some descendent
     *
     * @param type The atom type
     * @return A vector containing all the entities names
     */
    const opencog::combo::definite_object_set getEntities(Type type = OBJECT_NODE) const;

    /**
       * Gets the entities (i.e. definite_object) of a given type (and its decendents) in all LocaSpaceMap2D related to a given trick.
       * NOTE: the type MUST be a NODE or some descendent
       *
       * @param wp WorldProvider to search over
       * @param trick name of the trick
       * @param selfID id of the pet, to change it into "self"
       * @param ownerID id of the owner, to change it into "owner"
       * @param type The atom type
       * @return A set containing all the entities names
       */
    const opencog::combo::definite_object_set getEntities(const WorldProvider& wp,
            const std::string& trick,
            const std::string& selfID,
            const std::string& ownerID,
            Type type = OBJECT_NODE) const;


    /**
       * Gets the set of messages that span over the exemplar trick
       * possibly addressed to toID
       *
       * @param wp             WorldProvider to search over
       * @param trick          the name of the trick
       * @param toID           ID of destination
       * @param exclude_prefix whether it exclude the prefix
       *                       "to:toID: "
       * @param strict_within  whether the start and stop time
       *                       are included in the temporal
       *                       this is used to possibly avoid
       *                       messages from the owner to
       *                       start and stop learning
       * @return A set containing all messages meeting the constraint
       */
    const opencog::combo::message_set getMessages(const WorldProvider& wp,
                                         const std::string& trick,
                                         const std::string& toID = std::string(),
                                         bool exclude_prefix = false,
                                         bool strict_within = true) const;

    /**
       * Gets the set of messages that span over the interval t
       * possibly addressed to toID
       *
       * @param wp             WorldProvider to search over
       * @param t              time interval where to look for
       * @param toID           ID of destination
       * @param exclude_prefix whether it exclude the prefix
       *                       "to:toID: "
       * @param strict_within  whether the start and stop time
       *                       are included in the temporal
       *                       this is used to possibly avoid
       *                       messages from the owner to
       *                       start and stop learning
       * @return A set containing all messages meeting the constraint
       */
    const opencog::combo::message_set getMessages(const WorldProvider& wp,
                                         Temporal t,
                                         const std::string& toID = std::string(),
                                         bool exclude_prefix = false,
                                         bool strict_within = true) const;

    /**
    * Gets the set of messages that span over the interval t
    * possibly addressed to toID
    *
    * @param as     AtomSpace to search over
    * @param t              time interval where to look for
    * @param toID           ID of destination
    * @param exclude_prefix whether it exclude the prefix
    *                       "to:toID: "
    * @param strict_within  whether the start and stop time
    *                       are included in the temporal
    *                       this is used to possibly avoid
    *                       messages from the owner to
    *                       start and stop learning
    * @return A set containing all messages meeting the constraint
    */
    const opencog::combo::message_set getMessages(AtomSpace& as,
                                         Temporal t,
                                         const std::string& toID = std::string(),
                                         bool exclude_prefix = false,
                                         bool strict_within = true) const;

    /**
    * Gets the set of all agent actions
    * (under the form of action definite_objects)
    * of which the actionDone ending time span over a trick
    *
    * @param wp             WorldProvider to search over
    * @param trick          the name of the trick
    * @param exclude_self   that flag is used because we may
    *                       not want to consider proprioceptions
    *                       here
    * @return a map associating agent id with their actions definite_object
    */
    const agent_to_actions getAgentActions(const WorldProvider& wp,
                                           const std::string& trick,
                                           const std::string& selfID,
                                           const std::string& ownerID,
                                           const std::set<std::string>& exclude_set = std::set<std::string>()) const;

    /**
    * Gets the set of all agent actions
    * (under the form of action definite_objects)
    * of which the actionDone ending time span over t
    *
    * @param as             WorldProvider to search over
    * @param t              time interval where to look in
    * @param exclude_self   that flag is used because we may
    *                       not want to consider proprioceptions
    *                       here
    * @return a map associating agent id with their actions definite_object
    */
    const agent_to_actions getAgentActions(const WorldProvider& wp,
                                           const Temporal& t,
                                           const std::string& selfID,
                                           const std::string& ownerID,
                                           const std::set<std::string>& exclude_set = std::set<std::string>()) const;

    /**
    * Gets the set of all agent actions
    * (under the form of action definite_objects)
    * of which the actionDone ending time span over t
    *
    * @param as             AtomSpace to search over
    * @param t              time interval where to look in
    * @param exclude_self   that flag is used because we may
    *                       not want to consider proprioceptions
    *                       here
    * @return a map associating agent id with their actions definite_object
    */
    const agent_to_actions getAgentActions(AtomSpace& as,
                                           const Temporal& t,
                                           const std::string& selfID,
                                           const std::string& ownerID,
                                           const std::set<std::string>& exclude_set = std::set<std::string>()) const;

}; // class
//}  // namespace

#endif
