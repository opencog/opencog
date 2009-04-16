/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualEntity.h
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

#ifndef VIRTUAL_ENTITY_H
#define VIRTUAL_ENTITY_H

#include <map>
#include <string>

namespace VirtualWorldData
{

enum VirtualEntityType {
    PET,
    OBJECT,
    AVATAR
};

/*
 * Virtual Entity structure - base class for pets, avatars and virtual world
 * objects.
 */
struct VirtualEntity {

public:

    VirtualEntity();
    VirtualEntity(const std::string & name, const VirtualEntityType type);
    ~VirtualEntity();

    // setters
    void setName(const std::string & name);
    void setType(VirtualEntityType type);

    void setNoisy(bool noise);
    void setSmall(bool small);
    void setEdible(bool edible);
    void setMovable(bool movable);
    void setDrinkable(bool drinkable);
    void setPeePlace(bool pee_place);
    void setPooPlace(bool poo_place);
    void setPickupable(bool pickupable);

    void setMoving(bool moving);

    // getters - in an isX form
    const std::string & getName() const;
    VirtualEntityType getType() const;

    bool isNoisy() const;
    bool isSmall() const;
    bool isEdible() const;
    bool isMovable() const;
    bool isPeePlace() const;
    bool isPooPlace() const;
    bool isDrinkable() const;
    bool isPickupable() const;

    bool isMoving() const;

    // type related getters - in an isX form
    bool isPet() const;
    bool isAvatar() const;
    bool isObject() const;

private:
    void init();

    std::string name;
    VirtualEntityType type;

    bool noisy;
    bool small;
    bool edible;
    bool movable;
    bool drinkable;
    bool pee_place;
    bool poo_place;
    bool pickupable;

    bool moving;

}; // struct VirtualEntity

/*
 *
 */
struct VirtualAgent : public VirtualEntity {

public:
    VirtualAgent();
    VirtualAgent(const std::string & name);
    ~VirtualAgent();

    // setters - feelings
    void setHunger(float hunger);
    void setThirst(float thirst);
    void setEnergy(float energy);
    void setFitness(float fitness);
    void setPeeUrgency(float peeUrgency);
    void setPooUrgency(float pooUrgency);

    // setters - traits
    void setAggressiveness(float aggressiveness);
    void setCuriosity(float curiosity);
    void setPlayfulness(float playfulness);
    void setFriendliness(float friendliness);
    void setFearfulness(float fearfulness);
    void setAppreciativeness(float appreciativeness);
    void setExcitability(float excitability);

    // setters - emotional feelings
    void setHappiness(float happiness);
    void setPride(float pride);
    void setLove(float love);
    void setHate(float hate);
    void setFear(float fear);
    void setAnger(float anger);
    void setGratitude(float gratitude);
    void setExcitement(float excitement);

    // setters - others
    void setNovelty(bool novelty);
    void setLearning(bool learning);
    void setLearnedTricks(bool learnedTricks);
    void setRequestedSchema(bool resquestedSchema);
    void setAvatarAskedToTry(bool askedToTry);

    // getters - isX form feelings
    float getHunger() const;
    float getThirst() const;
    float getEnergy() const;
    float getFitness() const;
    float getPeeUrgency() const;
    float getPooUrgency() const;

    // getters - traits
    float getAggressiveness() const;
    float getCuriosity() const;
    float getPlayfulness() const;
    float getFriendliness() const;
    float getFearfulness() const;
    float getAppreciativeness() const;
    float getExcitability() const;

    // getters - emotional feelings
    float getHappiness() const;
    float getPride() const;
    float getLove() const;
    float getHate() const;
    float getFear() const;
    float getAnger() const;
    float getGratitude() const;
    float getExcitement() const;

    // getters - others
    bool hasNovelty() const;
    bool isLearning() const;
    bool hasLearnedTricks() const;
    bool hasRequestedSchema() const;
    bool avatarAskedToTry() const;

private:

    void init();

    // feelings
    float hunger;
    float thirst;
    float energy;
    float fitness;
    float pee_urgency;
    float poo_urgency;

    // emotional feelings
    float excitement;
    float happiness;
    float gratitude;
    float pride;
    float anger;
    float love;
    float hate;
    float fear;

    // traits
    float aggressiveness;
    float curiosity;
    float playfulness;
    float friendliness;
    float fearfulness;
    float appreciativeness;
    float excitability;

    // other
    bool novelty;
    bool learning;
    bool learnedTricks;
    bool requestedSchema;
    bool askedToTry;

}; // struct VirtualAgent
}  // namespace

#endif
