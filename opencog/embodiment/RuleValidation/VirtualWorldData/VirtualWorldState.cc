/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldState.cc
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

#include "VirtualWorldState.h"

using namespace VirtualWorldData;

void WorldEntitiesAndAgents::add(const VirtualEntity & entity)
{
    if (entity.getType() == VirtualWorldData::PET) {
        worldAgents.insert(std::pair<std::string, VirtualAgent>(
                               entity.getName(), (const VirtualAgent &)entity));
    } else {
        worldEntities.insert(std::pair<std::string, VirtualEntity>(
                                 entity.getName(), entity));
    }

    names.push_back(entity.getName());
}

const VirtualEntity * WorldEntitiesAndAgents::find(const std::string & key)
{
    stringToEntityMapIt e_it = worldEntities.find(key);
    if (e_it != worldEntities.end()) {
        return (&(e_it->second));
    } else {
        stringToAgentMapIt a_it = worldAgents.find(key);
        if (a_it != worldAgents.end()) {
            return (&(a_it->second));
        }
    }
    return NULL;
}

const std::vector<std::string> & WorldEntitiesAndAgents::getNames() const
{
    return this->names;
}

VirtualWorldState::VirtualWorldState()
{
    this->petId = "";
    this->petOwnerId = "";
    this->agent_state = 0;
    this->petMode = "PLAYING_MODE";
    this->current_action_repetition = 0;

}

VirtualWorldState::~VirtualWorldState()
{
}

const std::string & VirtualWorldState::getPetId() const
{
    return this->petId;
}

const std::string & VirtualWorldState::getPetOwnerId() const
{
    return this->petOwnerId;
}

const std::string & VirtualWorldState::getPetMode() const
{
    return this->petMode;
}

IndefiniteObjects &  VirtualWorldState::getIndefiniteObjects()
{
    return this->indefiniteObjects;
}

const std::vector<std::string> & VirtualWorldState::getWorldEntities() const
{
    return this->entitiesAndAgents.getNames();
}

const VirtualEntity *  VirtualWorldState::findEntity(const std::string & obj)
{

    std::string key;
    if (obj == "self") {
        key = this->petId;
    } else if (obj == "owner") {
        key = this->petOwnerId;
    } else {
        key = obj;
    }

    return entitiesAndAgents.find(key);
}

bool  VirtualWorldState::nearObjects(const std::string & obj1, const std::string & obj2) const
{
    std::string key = obj1 + obj2;
    stringSetIt it = worldNearEntities.find(key);

    if (it == worldNearEntities.end()) {
        return false;
    }
    return true;
}

bool  VirtualWorldState::nextObjects(const std::string & obj1, const std::string & obj2) const
{
    std::string key = obj1 + obj2;
    stringSetIt it = worldNextEntities.find(key);

    if (it == worldNextEntities.end()) {
        return false;
    }
    return true;
}

bool  VirtualWorldState::ownerObjects(const std::string & obj1, const std::string & obj2) const
{
    std::string key = obj1 + obj2;
    stringSetIt it = worldOwnerEntities.find(key);

    if (it == worldOwnerEntities.end()) {
        return false;
    }
    return true;
}

bool VirtualWorldState::movingTowardObjects(const std::string & obj1, const std::string & obj2) const
{
    std::string key = obj1 + obj2;
    stringSetIt it = worldMovingTowardEntities.find(key);

    if (it == worldMovingTowardEntities.end()) {
        return false;
    }
    return true;
}

bool VirtualWorldState::isInsideFoVObjects(const std::string & obj1, const std::string & obj2) const
{
    std::string key = obj1 + obj2;
    stringSetIt it = worldInsideFoVEntities.find(key);

    if (it == worldInsideFoVEntities.end()) {
        return false;
    }
    return true;
}

bool VirtualWorldState::relationObjects(const std::string & obj1, const std::string & obj2,
                                        const std::string & relation)
{
    std::string key = obj1 + obj2;
    stringToStringMultimapIt it = worldRelationsEntities.find(key);

    // objs pair have no relation at all
    if (it ==  worldRelationsEntities.end()) {
        return false;
    }

    // objs have relations, search for the one needed (if any)
    std::pair<stringToStringMultimapIt, stringToStringMultimapIt> range = worldRelationsEntities.equal_range(key);
    for (it = range.first; it != range.second; ++it) {
        if ((*it).second == relation) {
            return true;
        }
    }
    return false;
}

bool VirtualWorldState::hasSaidObjects(const std::string & obj, const std::string & message)
{
    stringToStringMapIt it = worldHasSaidEntities.find(obj);

    if (it == worldHasSaidEntities.end()) {
        return false;
    }

    if (it->second == message) {
        return true;
    }
    return false;
}

int VirtualWorldState::getCurrentActionRepetition() const
{
    return this->current_action_repetition;
}

bool VirtualWorldState::isAgentState(int agent_state) const
{
    return this->agent_state == agent_state;
}

bool VirtualWorldState::isLastAgentAction(const std::string & agent, const std::string & action,
        const std::vector<std::string> & params)
{

    stringToAgentActionMapIt it = lastAgentAction.find(agent);

    if (it == lastAgentAction.end()) {
        return false;
    }

    AgentAction aa = it->second;

    // agent and action
    if (agent != aa.agent || action != aa.action) {
        return false;
    }

    // could not have more params than the stored action
    if (params.size() > aa.params.size()) return false;

    // action parameters, if one fail then error
    for (unsigned int i = 0; i < aa.params.size(); i++) {
        if (params[i] != aa.params[i]) return false;
    }
    return true;
}

bool VirtualWorldState::isLastPetSchema(const std::string & schema, const std::string & result,
                                        const std::vector<std::string> & params)
{

    // schema and result
    if (schema != lastPetSchema.schema || result != lastPetSchema.result) {
        return false;
    }

    // could not have more params than the stored action
    if (params.size() > lastPetSchema.params.size()) return false;

    // action parameters, if one fail then error
    for (unsigned int i = 0; i < lastPetSchema.params.size(); i++) {
        if (params[i] != lastPetSchema.params[i]) return false;
    }
    return true;
}

/* ----------------------------------------------------------------------------
 * Private functions
 * ----------------------------------------------------------------------------
 */
void VirtualWorldState::setPetId(const std::string & petId)
{
    this->petId = petId;
}

void VirtualWorldState::setPetOwnerId(const std::string & petOwnerId)
{
    this->petOwnerId = petOwnerId;
}

void VirtualWorldState::setPetMode(const std::string & petMode)
{
    this->petMode = petMode;
}

void VirtualWorldState::setCurrentActionRepetition(int current_action_repetition)
{
    this->current_action_repetition = current_action_repetition;
}

void VirtualWorldState::setAgentState(int agent_state)
{
    this->agent_state = agent_state;
}

void VirtualWorldState::addEntity(const VirtualEntity & entity)
{
    this->entitiesAndAgents.add(entity);
}

void VirtualWorldState::addNearObjects(const std::string & obj1, const std::string & obj2)
{
    std::string key = (obj1 + obj2);
    std::pair<stringSetIt, bool> result;

    worldNearEntities.erase(key);
    result = worldNearEntities.insert(key);

    if (!result.second) {
        //log some information about an objet already been in the set
    }
}

void VirtualWorldState::addNextObjects(const std::string & obj1, const std::string & obj2)
{
    std::string key = (obj1 + obj2);
    std::pair<stringSetIt, bool> result;

    worldNextEntities.erase(key);
    result = worldNextEntities.insert(key);

    if (!result.second) {
        //log some information about an objet already been in the set
    }
}

void VirtualWorldState::addOwnerObjects(const std::string & obj1, const std::string & obj2)
{
    std::string key = (obj1 + obj2);
    std::pair<stringSetIt, bool> result;

    worldOwnerEntities.erase(key);
    result = worldOwnerEntities.insert(key);

    if (!result.second) {
        //log some information about an objet already been in the set
    }
}

void VirtualWorldState::addMovingTowardObjects(const std::string & obj1, const std::string & obj2)
{
    std::string key = (obj1 + obj2);
    std::pair<stringSetIt, bool> result;

    worldMovingTowardEntities.erase(key);
    result = worldMovingTowardEntities.insert(key);

    if (!result.second) {
        //log some information about an objet already been in the set
    }
}

void VirtualWorldState::addInsideFoVObjects(const std::string & obj1, const std::string & obj2)
{
    std::string key = (obj1 + obj2);
    std::pair<stringSetIt, bool> result;

    worldInsideFoVEntities.erase(key);
    result = worldInsideFoVEntities.insert(key);

    if (!result.second) {
        //log some information about an objet already been in the set
    }
}


void VirtualWorldState::addRelationObjects(const std::string & obj1,
        const std::string & obj2,
        const std::string relation)
{
    std::string key = (obj1 + obj2);
    worldRelationsEntities.insert(std::pair<std::string, std::string>(key, relation));
}

void VirtualWorldState::addHasSaidObjects(const std::string & obj, const std::string & message)
{
    // to make sure that has said is always updated
    worldHasSaidEntities.erase(obj);
    worldHasSaidEntities.insert(std::pair<std::string, std::string>(obj, message));
}

void VirtualWorldState::addLastAgentAction(const std::string & agent, const std::string action,
        const std::vector<std::string> & params)
{

    lastAgentAction.erase(agent);

    AgentAction agentAction(agent, action, params);
    lastAgentAction.insert(std::pair<std::string, AgentAction>(agent, agentAction));
}

void VirtualWorldState::addLastPetSchema(const std::string & schema, const std::string & result,
        const std::vector<std::string> & params)
{
    this->lastPetSchema = PetSchema(schema, result, params);
}

