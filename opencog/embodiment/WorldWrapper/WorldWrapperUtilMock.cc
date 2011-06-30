/*
 * opencog/embodiment/WorldWrapper/WorldWrapperUtilMock.cc
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

#include <opencog/util/functional.h>
#include <opencog/util/foreach.h>
#include <opencog/util/Logger.h>

#include "WorldWrapperUtilMock.h"
#include <math.h>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <ext/functional>

using namespace PetCombo;
using namespace boost::assign;
using namespace opencog;

using namespace opencog::world;

typedef combo_tree::iterator pre_it;
typedef combo_tree::sibling_iterator sib_it;

WorldWrapperUtilMock::PetPerception WorldWrapperUtilMock::petPerception;
WorldWrapperUtilMock::WorldPerception WorldWrapperUtilMock::worldPerception;
WorldWrapperUtilMock::EntityPerception WorldWrapperUtilMock::entityPerception;
WorldWrapperUtilMock::BoolPetPerception WorldWrapperUtilMock::boolPetPerception;

bool WorldWrapperUtilMock::initialized = false;

void WorldWrapperUtilMock::initializeMaps()
{

    if (initialized) {
        return;
    }

    // adding VirtualAgent pointers to float member functions
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_hunger, &VirtualWorldData::VirtualAgent::getHunger));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_thirst, &VirtualWorldData::VirtualAgent::getThirst));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_energy, &VirtualWorldData::VirtualAgent::getEnergy));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_fitness, &VirtualWorldData::VirtualAgent::getFitness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_pee_urgency, &VirtualWorldData::VirtualAgent::getPeeUrgency));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_poo_urgency, &VirtualWorldData::VirtualAgent::getPooUrgency));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_aggressiveness, &VirtualWorldData::VirtualAgent::getAggressiveness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_curiosity, &VirtualWorldData::VirtualAgent::getCuriosity));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_playfulness, &VirtualWorldData::VirtualAgent::getPlayfulness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_friendliness, &VirtualWorldData::VirtualAgent::getFriendliness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_fearfulness, &VirtualWorldData::VirtualAgent::getFearfulness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_appreciativeness, &VirtualWorldData::VirtualAgent::getAppreciativeness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_excitability, &VirtualWorldData::VirtualAgent::getExcitability));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_happiness, &VirtualWorldData::VirtualAgent::getHappiness));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_pride, &VirtualWorldData::VirtualAgent::getPride));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_love, &VirtualWorldData::VirtualAgent::getLove));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_hate, &VirtualWorldData::VirtualAgent::getHate));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_fear, &VirtualWorldData::VirtualAgent::getFear));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_anger, &VirtualWorldData::VirtualAgent::getAnger));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_gratitude, &VirtualWorldData::VirtualAgent::getGratitude));
    WorldWrapperUtilMock::petPerception.insert(
        PetPerceptionPair(id::get_excitement, &VirtualWorldData::VirtualAgent::getExcitement));

    // adding VirtualAgent pointers to bool member functions
    WorldWrapperUtilMock::boolPetPerception.insert(
        BoolPetPerceptionPair(id::has_novelty, &VirtualWorldData::VirtualAgent::hasNovelty));
    WorldWrapperUtilMock::boolPetPerception.insert(
        BoolPetPerceptionPair(id::has_learned_tricks, &VirtualWorldData::VirtualAgent::hasLearnedTricks));
    WorldWrapperUtilMock::boolPetPerception.insert(
        BoolPetPerceptionPair(id::has_requested_schema, &VirtualWorldData::VirtualAgent::hasRequestedSchema));
    WorldWrapperUtilMock::boolPetPerception.insert(
        BoolPetPerceptionPair(id::is_learning, &VirtualWorldData::VirtualAgent::isLearning));
    WorldWrapperUtilMock::boolPetPerception.insert(
        BoolPetPerceptionPair(id::avatar_asked_to_try, &VirtualWorldData::VirtualAgent::avatarAskedToTry));

    // adding VirtualEntity pointers to member functions
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_pet, &VirtualWorldData::VirtualEntity::isPet));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_avatar, &VirtualWorldData::VirtualEntity::isAvatar));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_object, &VirtualWorldData::VirtualEntity::isObject));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_moving, &VirtualWorldData::VirtualEntity::isMoving));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_noisy, &VirtualWorldData::VirtualEntity::isNoisy));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_small, &VirtualWorldData::VirtualEntity::isSmall));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_edible, &VirtualWorldData::VirtualEntity::isEdible));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_movable, &VirtualWorldData::VirtualEntity::isMovable));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_pee_place, &VirtualWorldData::VirtualEntity::isPeePlace));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_poo_place, &VirtualWorldData::VirtualEntity::isPooPlace));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_drinkable, &VirtualWorldData::VirtualEntity::isDrinkable));
    WorldWrapperUtilMock::entityPerception.insert(
        EntityPerceptionPair(id::is_pickupable, &VirtualWorldData::VirtualEntity::isPickupable));

    // adding VirtualEntity pointers to member functions
    WorldWrapperUtilMock::worldPerception.insert(
        WorldPerceptionPair(id::near, &VirtualWorldData::VirtualWorldState::nearObjects));
    WorldWrapperUtilMock::worldPerception.insert(
        WorldPerceptionPair(id::next, &VirtualWorldData::VirtualWorldState::nextObjects));
    WorldWrapperUtilMock::worldPerception.insert(
        WorldPerceptionPair(id::is_owner, &VirtualWorldData::VirtualWorldState::ownerObjects));
    WorldWrapperUtilMock::worldPerception.insert(
        WorldPerceptionPair(id::inside_pet_fov, &VirtualWorldData::VirtualWorldState::isInsideFoVObjects));
    WorldWrapperUtilMock::worldPerception.insert(
        WorldPerceptionPair(id::is_moving_toward, &VirtualWorldData::VirtualWorldState::movingTowardObjects));

    initialized = true;
}

/* -----------------------------------------------------------------------------
 * Functions
 * -----------------------------------------------------------------------------
 */
vertex WorldWrapperUtilMock::evalIndefiniteObject(combo::indefinite_object io,
        VirtualWorldData::VirtualWorldState & vw,
        combo::variable_unifier& vu)
{
    return evalIndefiniteObject(get_enum(io), vw, vu);
}

vertex WorldWrapperUtilMock::evalIndefiniteObject(combo::pet_indefinite_object_enum ioe,
        VirtualWorldData::VirtualWorldState & vw,
        combo::variable_unifier& vu)
{
    initializeMaps();

    std::string res;
    const VirtualWorldData::IndefiniteObjects & indefObjects = vw.getIndefiniteObjects();

    indefinite_object io = instance(ioe);
    switch (ioe) {
    case id::nearest_object:
        res = indefObjects.nearest_object;
        break;
    case id::nearest_edible:
        res = indefObjects.nearest_edible;
        break;
    case id::nearest_movable:
        res = indefObjects.nearest_movable;
        break;
    case id::nearest_pickupable:
        res = indefObjects.nearest_pickupable;
        break;
    case id::nearest_drinkable:
        res = indefObjects.nearest_drinkable;
        break;
    case id::nearest_avatar:
        res = indefObjects.nearest_avatar;
        break;
    case id::nearest_pet:
        res = indefObjects.nearest_pet;
        break;
    case id::nearest_small:
        res = indefObjects.nearest_small;
        break;
    case id::nearest_moving:
        res = indefObjects.nearest_moving;
        break;
    case id::nearest_noisy:
        res = indefObjects.nearest_noisy;
        break;
    case id::nearest_poo_place:
        res = indefObjects.nearest_poo_place;
        break;
    case id::nearest_pee_place:
        res = indefObjects.nearest_pee_place;
        break;
    case id::random_object:
        res = indefObjects.random_object;
        break;
    case id::random_edible:
        res = indefObjects.random_edible;
        break;
    case id::random_movable:
        res = indefObjects.random_movable;
        break;
    case id::random_pickupable:
        res = indefObjects.random_pickupable;
        break;
    case id::random_drinkable:
        res = indefObjects.random_drinkable;
        break;
    case id::random_avatar:
        res = indefObjects.random_avatar;
        break;
    case id::random_pet:
        res = indefObjects.random_pet;
        break;
    case id::random_small:
        res = indefObjects.random_small;
        break;
    case id::random_moving:
        res = indefObjects.random_moving;
        break;
    case id::random_noisy:
        res = indefObjects.random_noisy;
        break;
    case id::random_poo_place:
        res = indefObjects.random_poo_place;
        break;
    case id::random_pee_place:
        res = indefObjects.random_pee_place;
        break;
    case id::food_bowl:
        res = indefObjects.food_bowl;
        break;
    case id::water_bowl:
        res = indefObjects.water_bowl;
        break;
    case id::pet_home:
        res = indefObjects.pet_home;
        break;
    case id::pet_bowl:
        res = indefObjects.pet_bowl;
        break;
    case id::last_food_place:
        res = indefObjects.last_food_place;
        break;
    case id::exemplar_avatar:
        res = indefObjects.exemplar_avatar;
        break;
    default:
        res = "";
        break;
    }

    std::stringstream ss;
    ss << io;

    if (res == "") {
        res = id::null_obj;
    }

//    MAIN_LOGGER.debug(
//            "RunningComboProc - Analyzing '%s'. Result: '%s'.",
//            ss.str().c_str(), res.c_str());

    return vertex(res);
}

combo::vertex WorldWrapperUtilMock::evalPerception(const pre_it it,
        VirtualWorldData::VirtualWorldState & vw,
        combo::variable_unifier& vu)
{
    initializeMaps();

    // temporary tree used to copy perception to
    // not modify it
    //combo_tree tmp(it);

    perception p = get_perception(*it);
    pet_perception_enum pe = get_enum(p);
    switch (pe) {
    case id::exists_edible:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_edible, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_movable:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_movable, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_pickupable:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_pickupable, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_drinkable:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_drinkable, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_avatar:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_avatar, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_pet:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_pet, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_small:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_small, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_moving:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_moving, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_noisy:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_noisy, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_poo_place:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_poo_place, vw) !=
                                      vertex(id::null_obj)));
    case id::exists_pee_place:
        return combo::bool_to_vertex((evalIndefiniteObject(id::random_pee_place, vw) !=
                                      vertex(id::null_obj)));
    case id::exists:
        assert(it.number_of_children() == 1);
        {
            vertex vo = *it.begin();
            std::vector<definite_object> definite_objects =
                WorldWrapperUtilMock::getDefiniteObjects(vo, vw, vu);

            if (definite_objects.size() != 1) {
                // there is no definite object, so return false immediately
                return combo::bool_to_vertex(false);
            }

            const VirtualWorldData::VirtualEntity * entity = vw.findEntity(definite_objects[0]);
            return combo::bool_to_vertex(entity != NULL);
        }

    case id::is_null:
        OC_ASSERT(it.number_of_children() == 1);
        {

            combo::vertex v = *it.begin();
            if (is_indefinite_object(v)) {
                v = WorldWrapperUtilMock::evalIndefiniteObject(get_indefinite_object(v), vw);
            }

            return combo::bool_to_vertex(get_definite_object(v) == id::null_obj);
        }

        // types
    case id::is_pet:
    case id::is_object:
    case id::is_avatar:

        //moving state
    case id::is_moving:

        // object properties
    case id::is_noisy:
    case id::is_small:
    case id::is_edible:
    case id::is_movable:
    case id::is_pee_place:
    case id::is_poo_place:
    case id::is_drinkable:
    case id::is_pickupable:
        assert(it.number_of_children() == 1);
        {
            vertex vo = *it.begin();
            std::vector<definite_object> definite_objects =
                WorldWrapperUtilMock::getDefiniteObjects(vo, vw, vu);

            bool general_result = false;
            const VirtualWorldData::VirtualEntity * entity;
            foreach(combo::definite_object def_obj, definite_objects) {
                entity = vw.findEntity(def_obj);

                bool result = false;
                if (entity != NULL) {
                    EntityPerceptionIt e_it =
                        WorldWrapperUtilMock::entityPerception.find(pe);

                    if (e_it !=  WorldWrapperUtilMock::entityPerception.end()) {
//                            MAIN_LOGGER.debug(
//                                            "evalPerception - Found function pointer");
                        // pointer to member function
                        result = (entity->*(e_it->second))();

                    }
                }

                if (is_wild_card(vo)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }

            if (is_wild_card(vo)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

        // physiological feelings
    case id::get_hunger:
    case id::get_thirst:
    case id::get_energy:
    case id::get_fitness:
    case id::get_pee_urgency:
    case id::get_poo_urgency:

        // traits
    case id::get_curiosity:
    case id::get_playfulness:
    case id::get_fearfulness:
    case id::get_friendliness:
    case id::get_excitability:
    case id::get_aggressiveness:
    case id::get_appreciativeness:

        // emotional feelings
    case id::get_fear:
    case id::get_love:
    case id::get_hate:
    case id::get_anger:
    case id::get_pride:
    case id::get_gratitude:
    case id::get_happiness:
    case id::get_excitement:
        OC_ASSERT(it.number_of_children() == 1,
                         "WWUtil - get_X perception accept only one argument. Got '%d'.",
                         it.number_of_children());
        {
            vertex vo = *it.begin();
            if (is_indefinite_object(vo)) {
                vo = WorldWrapperUtilMock::evalIndefiniteObject(get_indefinite_object(vo), vw);
            }

            const VirtualWorldData::VirtualEntity * entity = vw.findEntity(get_definite_object(vo));

            float result = 0.0f;
            if (entity != NULL && entity->isPet()) {
                PetPerceptionIt p_it =
                    WorldWrapperUtilMock::petPerception.find(pe);

                if (p_it !=  WorldWrapperUtilMock::petPerception.end()) {
                    // pointer to member function - note type cast on entity
                    result = (((const VirtualWorldData::VirtualAgent *)entity)->*(p_it->second))();
                }
            }
            return result;
        }

    case id::is_learning:
    case id::has_novelty:
    case id::has_learned_tricks:
    case id::has_requested_schema:
    case id::avatar_asked_to_try: {
        vertex vo = *it.begin();
        if (is_indefinite_object(vo)) {
            vo = WorldWrapperUtilMock::evalIndefiniteObject(get_indefinite_object(vo), vw);
        }

        const VirtualWorldData::VirtualEntity * entity = vw.findEntity(get_definite_object(vo));

        bool result = false;
        if (entity != NULL && entity->isPet()) {
            BoolPetPerceptionIt bp_it =
                WorldWrapperUtilMock::boolPetPerception.find(pe);

            if (bp_it !=  WorldWrapperUtilMock::boolPetPerception.end()) {
                // pointer to member function - note type cast on entity
                result = (((const VirtualWorldData::VirtualAgent *)entity)->*(bp_it->second))();
            }
        }
        return combo::bool_to_vertex(result);
    }

    case id::get_current_action_repetition: {
        return ((float)vw.getCurrentActionRepetition());
    }
    break;

    case id::near:
    case id::next:
    case id::is_owner:
    case id::inside_pet_fov:
    case id::is_moving_toward: {
        sib_it sib_arg = it.begin();
        vertex vo1 = *sib_arg;
        std::vector<definite_object> vo1_def_obj =
            WorldWrapperUtilMock::getDefiniteObjects(vo1, vw, vu);

        vertex vo2 = *(++sib_arg);
        std::vector<definite_object> vo2_def_obj =
            WorldWrapperUtilMock::getDefiniteObjects(vo2, vw, vu);

        WorldPerceptionIt w_it =
            WorldWrapperUtilMock::worldPerception.find(pe);

        bool general_result = false;
        if (w_it !=  WorldWrapperUtilMock::worldPerception.end()) {

            foreach(definite_object vo1_do, vo1_def_obj) {
                foreach(definite_object vo2_do, vo2_def_obj) {
                    // checking - pointer to member function
                    bool result = (vw.*(w_it->second))(vo1_do, vo2_do);

                    if (is_wild_card(vo1)) {
                        vu.setVariableState(vo1_do, result);
                    }
                    if (is_wild_card(vo2)) {
                        vu.setVariableState(vo2_do, result);
                    }
                    if (result) {
                        general_result = true;
                    }
                }
            }
            if (is_wild_card(vo1) || is_wild_card(vo2)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
        }
        return combo::bool_to_vertex(general_result);
    }

    case id::has_said:
        OC_ASSERT(it.number_of_children() == 2,
                         "WWUtil - hasSaid - must have 2 children");

        {
            sib_it sib_arg = it.begin();
            vertex vo = *sib_arg;
            std::vector<definite_object> definite_objects =
                WorldWrapperUtilMock::getDefiniteObjects(vo, vw, vu);

            vertex vo2 = *(++sib_arg);
            const std::string& message = get_message(vo2).getContent();

            bool general_result = false;
            foreach(definite_object def_obj, definite_objects) {

                bool result = vw.hasSaidObjects(def_obj, message);

                if (is_wild_card(vo)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }
            if (is_wild_card(vo)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

    case id::is_there_relation:
        OC_ASSERT(it.number_of_children() == 3,
                         "WWUtil - is_there_relation perception needs three arguments. Got '%d'.",
                         it.number_of_children());
        {
            sib_it sib_arg = it.begin();
            vertex vr = *sib_arg;
            OC_ASSERT(is_definite_object(vr),
                             "is_there_relation - invalid relation.");
            std::string relation = get_definite_object(vr);

            vertex vo1 = *(++sib_arg);
            std::vector<definite_object> vo1_def_obj =
                WorldWrapperUtilMock::getDefiniteObjects(vo1, vw, vu);

            vertex vo2 = *(++sib_arg);
            std::vector<definite_object> vo2_def_obj =
                WorldWrapperUtilMock::getDefiniteObjects(vo2, vw, vu);

            bool general_result = false;
            foreach(definite_object vo1_do, vo1_def_obj) {
                foreach(definite_object vo2_do, vo2_def_obj) {
                    // checking - pointer to member function
                    bool result = vw.relationObjects(vo1_do, vo2_do, relation);

                    if (is_wild_card(vo1)) {
                        vu.setVariableState(vo1_do, result);
                    }
                    if (is_wild_card(vo2)) {
                        vu.setVariableState(vo2_do, result);
                    }
                    if (result) {
                        general_result = true;
                    }
                }
            }
            if (is_wild_card(vo1) || is_wild_card(vo2)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

    case id::is_agent_state:
        OC_ASSERT(( it.number_of_children() == 1 && is_contin(*it.begin()) ),
                         "is_agent_state accept one argument and it must be a contin. Got '%d' arguments.",
                         it.number_of_children());
        {
            int state = (int)ceil(get_contin(*it.begin()));
            return combo::bool_to_vertex(vw.isAgentState(state));
        }

    case id::is_last_agent_action:
        OC_ASSERT(it.number_of_children() >= 2,
                         "WWUtil - is_last_agent_action perception needs at least two arguments. Got '%d'.",
                         it.number_of_children());
        {
            sib_it sib_arg = it.begin();

            vertex vo1 = *sib_arg;
            std::vector<combo::definite_object> agent_definite_objects =
                WorldWrapperUtilMock::getDefiniteObjects(vo1, vw, vu);

            vertex vo2 = *(++sib_arg);
            OC_ASSERT(is_definite_object(vo2),
                             "WWUtil - is_last_agent_action 2nd parameter should be a definite_object");

            // action name comes with an ACTION_NAME_POSTFIX so it won't be
            // confused with combo actions - so first we need to remove this
            std::string action_name = get_action_name(get_definite_object(vo2));

            std::vector<std::string> parameters;
            for (++sib_arg; sib_arg != it.end(); sib_arg++) {
                vertex v_temp = *sib_arg;

                if (is_indefinite_object(v_temp)) {
                    v_temp = WorldWrapperUtilMock::evalIndefiniteObject(
                                 get_indefinite_object(v_temp), vw);
                }
                parameters.push_back(std::string(get_definite_object(v_temp)));
            }

            bool general_result = false;
            foreach(definite_object def_obj, agent_definite_objects) {

                bool result = vw.isLastAgentAction(def_obj, action_name, parameters);

                if (is_wild_card(vo1)) {
                    vu.setVariableState(def_obj, result);
                }
                if (result) {
                    general_result = true;
                }
            }
            if (is_wild_card(vo1)) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

    case id::is_proportional_next:
        OC_ASSERT(it.number_of_children() == 4,
                         "WWUtil - is_proportional_next perception needs four arguments. Got '%d'.",
                         it.number_of_children());
        {
        }


    case id::is_last_pet_schema:
        OC_ASSERT(it.number_of_children() >= 2,
                         "is_last_pet_schema perception needs at least one argument. Got '%d'.",
                         it.number_of_children());
        {
            sib_it sib_arg = it.begin();
            vertex vo1 = *sib_arg;
            std::string schemaName = get_action_name(get_definite_object(vo1));

            vo1 = *(++sib_arg);
            OC_ASSERT(is_action_result(vo1),
                             "WWUtil - is_last_pet_schema 2nd parameter should be an action_result");

            std::string schemaResult = "action_failure";
            if (get_action(vo1) == combo::id::action_success) {
                schemaResult = "action_success";
            }

            unsigned int wild_card_index = 0;
            unsigned int wild_card_itens = 1;
            bool has_wild_card = false;
            std::vector<std::vector<combo::definite_object> > parameters;

            int i;
            for (i = 0, ++sib_arg; sib_arg != it.end(); sib_arg++, i++) {
                vertex v_temp = *sib_arg;

                std::vector<combo::definite_object> v_temp_defs =
                    WorldWrapperUtilMock::getDefiniteObjects(v_temp, vw, vu);
                parameters.push_back(v_temp_defs);

                if (is_wild_card(v_temp) && !has_wild_card) {
                    has_wild_card = true;
                    wild_card_index = i;
                    wild_card_itens = v_temp_defs.size();
                }
            }

            // check first if there is any parameter to test
            if (parameters.size() == 0) {

                // empty param vector
                std::vector<std::string> param;
                return combo::bool_to_vertex(vw.isLastPetSchema(schemaName, schemaResult, param));
            }

            bool general_result = false;
            for (unsigned int index = 0; index < wild_card_itens; index++) {

                std::vector<std::string> param;
                std::string wild_card_candidate;
                for (unsigned int i = 0; i < parameters.size(); i++) {
                    std::vector<combo::definite_object> temp = parameters[i];

                    if (index < temp.size()) {
                        param.push_back(temp[index]);

                        // get the wild card candidate
                        if (has_wild_card && wild_card_index == i) {
                            wild_card_candidate = temp[index];
                        }

                    } else {
                        param.push_back(temp[temp.size() - 1]);
                    }
                }

                bool result = vw.isLastPetSchema(schemaName, schemaResult, param);

                if (has_wild_card) {
                    vu.setVariableState(wild_card_candidate, result);
                }
                if (result) {
                    general_result = true;
                }
            }
            if (has_wild_card) {
                vu.setUpdated(true);
                vu.setOneVariableActive(general_result);
            }
            return combo::bool_to_vertex(general_result);
        }

    default:
        break;
    }
    return vertex(id::null_obj);
}

std::vector<combo::definite_object> WorldWrapperUtilMock::getDefiniteObjects(
    combo::vertex& v,
    VirtualWorldData::VirtualWorldState & vw,
    combo::variable_unifier& vu)
{
    initializeMaps();

    std::vector<definite_object> definite_objects;

    if (is_indefinite_object(v)) {
        v = WorldWrapperUtilMock::evalIndefiniteObject(get_indefinite_object(v), vw);
        if (is_definite_object(v)) {
            definite_objects.push_back(get_definite_object(v));
        }

    } else if (is_wild_card(v)) {
        OC_ASSERT(!vu.empty(),
                         "WWUtil - evalPerception - unifier should not be empty.");

        combo::UnifierIt it;
        for (it = vu.begin(); it != vu.end(); it++) {
            if ((*it).second) {
                definite_objects.push_back((*it).first);
            }
        }

    } else {
        if (is_definite_object(v)) {
            definite_object obj = get_definite_object(v);
            if (obj == id::self) {
                definite_objects.push_back(vw.getPetId());
            } else if (obj == id::owner) {
                definite_objects.push_back(vw.getPetOwnerId());
            } else {
                definite_objects.push_back(obj);
            }
        }
    }
    return definite_objects;
}

