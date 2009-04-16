/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldState.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
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

#ifndef VIRTUAL_WORLD_STATE_H
#define VIRTUAL_WORLD_STATE_H

#include "VirtualEntity.h"
#include "VirtualWorldActions.h"
#include "VirtualIndefiniteObjects.h"
#include <map>
#include <set>

namespace VirtualWorldData
{

struct WorldEntitiesAndAgents {
private:
    std::vector<std::string> names;

    // all entities present in the world (virtually)
    std::map<std::string, VirtualEntity> worldEntities;

    // all agents present in the world (virtually)
    std::map<std::string, VirtualAgent> worldAgents;

public:

    typedef std::map<std::string, VirtualEntity> stringToEntityMap;
    typedef std::map<std::string, VirtualEntity>::iterator stringToEntityMapIt;
    typedef std::map<std::string, VirtualEntity>::const_iterator stringToEntityMapConstIt;

    typedef std::map<std::string, VirtualAgent> stringToAgentMap;
    typedef std::map<std::string, VirtualAgent>::iterator stringToAgentMapIt;
    typedef std::map<std::string, VirtualAgent>::const_iterator stringToAgentMapConstIt;


    //
    void add(const VirtualEntity & entity);

    //
    const VirtualEntity * find(const std::string & name);

    //
    const std::vector<std::string> & getNames() const;
};

/**
 *
 */
class VirtualWorldState
{

public:

    typedef std::set<std::string> stringSet;
    typedef std::set<std::string>::iterator stringSetIt;

    typedef std::multimap<std::string, std::string> stringToStringMultimap;
    typedef std::multimap<std::string, std::string>::iterator stringToStringMultimapIt;

    typedef std::map<std::string, std::string> stringToStringMap;
    typedef std::map<std::string, std::string>::iterator stringToStringMapIt;

    typedef std::map<std::string, AgentAction> stringToAgentActionMap;
    typedef std::map<std::string, AgentAction>::iterator stringToAgentActionMapIt;

    VirtualWorldState();
    ~VirtualWorldState();

    /**
     *
     */
    const std::string & getPetId() const;

    /**
     *
     */
    const std::string & getPetOwnerId() const;

    /**
     *
     */
    const std::string & getPetMode() const;


    /**
     * Returns a const reference to the indefinite objects structure
     */
    IndefiniteObjects & getIndefiniteObjects();

    /**
     * Return a vector containing all the world entites names
     */
    const std::vector<std::string> & getWorldEntities() const;

    /**
     * Return the pointer to the entity whose key is its name. If no entity
     * is found, NULL is returned.
     */
    const VirtualEntity * findEntity(const std::string & obj);

    /**
     * Return true if the two entities are near and false otherwise.
     */
    bool nearObjects(const std::string & obj1, const std::string & obj2) const;

    /**
     * Return true if the two entities are next and false otherwise.
     */
    bool nextObjects(const std::string & obj1, const std::string & obj2) const;

    /**
     * Return true if obj1 owns obj2 and false otherwise.
     */
    bool ownerObjects(const std::string & obj1, const std::string & obj2) const;

    /**
     * Return true if obj1 owns obj2 and false otherwise.
     */
    bool movingTowardObjects(const std::string & obj1, const std::string & obj2) const;

    /**
     * Return true if obj2 is inside obj1's FoV.
     */
    bool isInsideFoVObjects(const std::string & obj1, const std::string & obj2) const;

    /**
     * Return true if obj1 has the given relation with obj2 and false
     * otherwise.
     */
    bool relationObjects(const std::string & obj1, const std::string & obj2,
                         const std::string & relation);

    /**
     * Return true if obj1 has the given relation with obj2 and false
     * otherwise.
     */
    bool hasSaidObjects(const std::string & obj, const std::string & message);


    /**
     * Return the number of action repetitions
     */
    int getCurrentActionRepetition() const;

    /**
     * Return true if the informed state is equal to the one of the agent
     * and false otherwise
     */
    bool isAgentState(int agent_state) const;

    /**
     * Return true if the specified params corresponds to the last action
     * executed by a given agent.
     */
    bool isLastAgentAction(const std::string & agent, const std::string & action,
                           const std::vector<std::string> & params);

    /**
     *
     */
    bool isLastPetSchema(const std::string & schema, const std::string & result,
                         const std::vector<std::string> & params);


    /**
     *
     */
    void setPetId(const std::string & petId);

    /**
     *
     */
    void setPetOwnerId(const std::string & petOwnerId);

    /**
     *
     */
    void setPetMode(const std::string & petMode);

    /**
     *
     */
    void setCurrentActionRepetition(int current_action_repetition);

    /**
     *
     */
    void setAgentState(int agent_state);

    /**
     *
     */
    void addEntity(const VirtualEntity & entity);

    /**
     *
     */
    void addNearObjects(const std::string & obj1, const std::string & obj2);

    /**
     *
     */
    void addNextObjects(const std::string & obj1, const std::string & obj2);

    /**
     *
     */
    void addOwnerObjects(const std::string & obj1, const std::string & obj2);

    /**
     *
     */
    void addMovingTowardObjects(const std::string & obj1, const std::string & obj2);

    /*
     * obj1 is the one in whose FoV we look for obj2
     */
    void addInsideFoVObjects(const std::string & obj1, const std::string & obj2);

    /**
     *
     */
    void addRelationObjects(const std::string & obj1,
                            const std::string & obj2,
                            const std::string relation);

    /*
     *
     */
    void addHasSaidObjects(const std::string & obj, const std::string & message);

    /**
     *
     */
    void addLastAgentAction(const std::string & agent, const std::string action,
                            const std::vector<std::string> & params);

    /**
     *
     */
    void addLastPetSchema(const std::string & schema, const std::string & result,
                          const std::vector<std::string> & params);

private:

    //
    WorldEntitiesAndAgents entitiesAndAgents;

    // represents entities that are near to each other. Set key is formed by
    // the concatenation of booth entities names. The order metters but the
    // relationship is simmetric.
    std::set<std::string> worldNearEntities;

    // represents entities that are next to each other. Set key is formed by
    // the concatenation of booth entities names. The order metters but the
    // relationship is simmetric.
    std::set<std::string> worldNextEntities;

    // represents entities where the first one owns the second. Set key is
    // formed by the concatenation of booth entities names. The order
    // metters since the relationship is not simmetric.
    std::set<std::string> worldOwnerEntities;

    // represents entities where the first one is moving toward the second.
    // Set key is formed by the concatenation of booth entities names. The
    // order metters but the relationship is simmetric.
    std::set<std::string> worldMovingTowardEntities;

    // represents entities where the second one is inside the first one FoV.
    // Set key is formed by the concatenation of booth entities names. The
    // order metters since the relationship is not simmetric.
    std::set<std::string> worldInsideFoVEntities;

    // represents the relations a couple of entities have. Multimap key is
    // the concatenation of booth entities names, the second item on the
    // map represents the relations enabled for that entities. The name
    // order metters but the relations are simmetric.
    std::multimap<std::string, std::string> worldRelationsEntities;

    // represents the has_said information for world entities. Map key is
    // the name of the entity, the second item on the map represents the
    // message it has said.
    std::map<std::string, std::string> worldHasSaidEntities;

    //
    IndefiniteObjects indefiniteObjects;

    //
    int current_action_repetition;

    //
    int agent_state;

    // represents the last action executed by an agent. Map key is the
    // agents name, the second item on the map represent the executed action
    // with its parameters.
    std::map<std::string, AgentAction> lastAgentAction;

    // represents the last schema executed by the pet (with result and
    // arguments)
    PetSchema lastPetSchema;

    // the pet id
    std::string petId;

    // the pet owner's id
    std::string petOwnerId;

    // the pet's mode: LEARNING_MODE, PLAYING_MODE, SCAVENGER_HUNT_MODE
    std::string petMode;

}; // class
}  // namespace

#endif
