/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualEntity.cc
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

#include "VirtualEntity.h"

using namespace VirtualWorldData;

VirtualEntity::VirtualEntity(const std::string & _n, const VirtualEntityType _t) : name(_n), type(_t)
{
    init();
}

VirtualEntity::VirtualEntity()
{
    this->name = "no_name";
    this->type = OBJECT;

    init();
}

VirtualEntity::~VirtualEntity() {}

void VirtualEntity::init()
{

    this->noisy = false;
    this->small = false;
    this->edible = false;
    this->movable = false;
    this->drinkable = false;
    this->pee_place = false;
    this->poo_place = false;
    this->pickupable = false;

    this->moving = false;
}

void VirtualEntity::setName(const std::string & name)
{
    this->name = name;
}

void VirtualEntity::setType(VirtualEntityType type)
{
    this->type = type;
}

void VirtualEntity::setNoisy(bool noisy)
{
    this->noisy = noisy;
}

void  VirtualEntity::setSmall(bool small)
{
    this->small = small;
}

void  VirtualEntity::setEdible(bool edible)
{
    this->edible = edible;
}

void  VirtualEntity::setMovable(bool movable)
{
    this->movable = movable;
}

void  VirtualEntity::setDrinkable(bool drinkable)
{
    this->drinkable = drinkable;
}

void  VirtualEntity::setPeePlace(bool pee_place)
{
    this->pee_place = pee_place;
}

void  VirtualEntity::setPooPlace(bool poo_place)
{
    this->poo_place = poo_place;
}

void  VirtualEntity::setPickupable(bool pickupable)
{
    this->pickupable = pickupable;
}

void  VirtualEntity::setMoving(bool moving)
{
    this->moving = moving;
}

// getters
const std::string & VirtualEntity::getName() const
{
    return this->name;
}

VirtualEntityType VirtualEntity::getType() const
{
    return this->type;
}

bool VirtualEntity::isNoisy() const
{
    return this->noisy;
}

bool VirtualEntity::isSmall() const
{
    return this->small;
}

bool VirtualEntity::isEdible() const
{
    return this->edible;
}

bool VirtualEntity::isMovable() const
{
    return this->movable;
}

bool VirtualEntity::isPeePlace() const
{
    return this->pee_place;
}

bool VirtualEntity::isPooPlace() const
{
    return this->poo_place;
}

bool VirtualEntity::isDrinkable() const
{
    return this->drinkable;
}

bool VirtualEntity::isPickupable() const
{
    return this->pickupable;
}

bool VirtualEntity::isMoving() const
{
    return this->moving;
}

// type related getters - in an isX form
bool VirtualEntity::isPet() const
{
    return (this->type == PET);
}

bool VirtualEntity::isAvatar() const
{
    return (this->type == AVATAR);
}

bool VirtualEntity::isObject() const
{
    return (this->type == OBJECT);
}

/* ------------------------------------------
 * VirtualAgent function implementations
 * ------------------------------------------
 */
VirtualAgent::VirtualAgent(const std::string & name) : VirtualEntity(name, PET)
{
    init();
}

VirtualAgent::VirtualAgent() : VirtualEntity("", PET)
{
    init();
}


VirtualAgent::~VirtualAgent() {}

void VirtualAgent::init()
{
    this->hunger = 0.0f;
    this->thirst = 0.0f;
    this->energy = 0.0f;
    this->fitness = 0.0f;
    this->pee_urgency = 0.0f;
    this->poo_urgency = 0.0f;

    // emotional feelings
    this->excitement = 0.0f;
    this->happiness = 0.0f;
    this->gratitude = 0.0f;
    this->pride = 0.0f;
    this->anger = 0.0f;
    this->love = 0.0f;
    this->hate = 0.0f;
    this->fear = 0.0f;

    // traits
    this->aggressiveness = 0.0f;
    this->curiosity = 0.0f;
    this->playfulness = 0.0f;
    this->friendliness = 0.0f;
    this->fearfulness = 0.0f;
    this->appreciativeness = 0.0f;
    this->excitability = 0.0f;

    // other
    this->novelty = false;
    this->learning = false;
    this->learnedTricks = false;
    this->requestedSchema = false;
    this->askedToTry = false;
}

// setters - feelings
void VirtualAgent::setHunger(float hunger)
{
    this->hunger = hunger;
}

void VirtualAgent::setThirst(float thirst)
{
    this->thirst = thirst;
}

void VirtualAgent::setEnergy(float energy)
{
    this->energy = energy;
}

void VirtualAgent::setFitness(float fitness)
{
    this->fitness = fitness;
}

void VirtualAgent::setPeeUrgency(float peei_urgency)
{
    this->pee_urgency = pee_urgency;
}

void VirtualAgent::setPooUrgency(float poo_urgency)
{
    this->poo_urgency = poo_urgency;
}

// setters - traits
void VirtualAgent::setAggressiveness(float aggressiveness)
{
    this->aggressiveness = aggressiveness;
}

void VirtualAgent::setCuriosity(float curiosity)
{
    this->curiosity = curiosity;
}

void VirtualAgent::setPlayfulness(float playfulness)
{
    this->playfulness = playfulness;
}

void VirtualAgent::setFriendliness(float friendliness)
{
    this->friendliness = friendliness;
}

void VirtualAgent::setFearfulness(float fearfulness)
{
    this->fearfulness = fearfulness;
}

void VirtualAgent::setAppreciativeness(float appreciativeness)
{
    this->appreciativeness = appreciativeness;
}

void VirtualAgent::setExcitability(float excitability)
{
    this->excitability = excitability;
}

// setters - emotional feelings
void VirtualAgent::setHappiness(float happiness)
{
    this->happiness = happiness;
}

void VirtualAgent::setPride(float pride)
{
    this->pride = pride;
}

void VirtualAgent::setLove(float love)
{
    this->love = love;
}

void VirtualAgent::setHate(float hate)
{
    this->hate = hate;
}

void VirtualAgent::setFear(float fear)
{
    this->fear = fear;
}

void VirtualAgent::setAnger(float anger)
{
    this->anger = anger;
}

void VirtualAgent::setGratitude(float gratitude)
{
    this->gratitude = gratitude;
}

void VirtualAgent::setExcitement(float excitement)
{
    this->excitement = excitement;
}

// setters - others
void VirtualAgent::setNovelty(bool novelty)
{
    this->novelty = novelty;
}

void VirtualAgent::setLearning(bool learning)
{
    this->learning = learning;
}

void VirtualAgent::setLearnedTricks(bool learnedTricks)
{
    this->learnedTricks = learnedTricks;
}

void VirtualAgent::setRequestedSchema(bool requestedSchema)
{
    this->requestedSchema = requestedSchema;
}

void VirtualAgent::setAvatarAskedToTry(bool askedToTry)
{
    this->askedToTry = askedToTry;
}

// getters - isX form feelings
float VirtualAgent::getHunger() const
{
    return this->hunger;
}

float VirtualAgent::getThirst() const
{
    return this->thirst;
}

float VirtualAgent::getEnergy() const
{
    return this->energy;
}

float VirtualAgent::getFitness() const
{
    return this->fitness;
}

float VirtualAgent::getPeeUrgency() const
{
    return this->pee_urgency;
}

float VirtualAgent::getPooUrgency() const
{
    return this->poo_urgency;
}

// getters - traits
float VirtualAgent::getAggressiveness() const
{
    return this->aggressiveness;
}

float VirtualAgent::getCuriosity() const
{
    return this->curiosity;
}

float VirtualAgent::getPlayfulness() const
{
    return this->playfulness;
}

float VirtualAgent::getFriendliness() const
{
    return this->friendliness;
}

float VirtualAgent::getFearfulness() const
{
    return this->fearfulness;
}

float VirtualAgent::getAppreciativeness() const
{
    return this->appreciativeness;
}

float VirtualAgent::getExcitability() const
{
    return this->excitability;
}

// getters - emotional feelings
float VirtualAgent::getHappiness() const
{
    return this->happiness;
}

float VirtualAgent::getPride() const
{
    return this->pride;
}

float VirtualAgent::getLove() const
{
    return this->love;
}

float VirtualAgent::getHate() const
{
    return this->hate;
}

float VirtualAgent::getFear() const
{
    return this->fear;
}


float VirtualAgent::getAnger() const
{
    return this->anger;
}

float VirtualAgent::getGratitude() const
{
    return this->gratitude;
}

float VirtualAgent::getExcitement() const
{
    return this->excitement;
}

// getters - others
bool VirtualAgent::hasNovelty() const
{
    return this->novelty;
}

bool VirtualAgent::isLearning() const
{
    return this->learning;
}

bool VirtualAgent::hasLearnedTricks() const
{
    return this->learnedTricks;
}

bool VirtualAgent::hasRequestedSchema() const
{
    return this->requestedSchema;
}

bool VirtualAgent::avatarAskedToTry() const
{
    return this->askedToTry;
}

