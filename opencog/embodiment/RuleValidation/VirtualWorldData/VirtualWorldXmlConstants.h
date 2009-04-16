/*
 * opencog/embodiment/RuleValidation/VirtualWorldData/VirtualWorldXmlConstants.h
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

#ifndef VIRTUAL_WORLD_XML_CONSTANTS
#define VIRTUAL_WORLD_XML_CONSTANTS

#define XML_TAG_LENGTH 512

// World definition
#define WORLD_DEFINITION_ELEM "world-definition"

// Structures, objects, pets, agents, humanoids
#define ENTITY_INFO_ELEM "entity-info"
#define ENTITY_OBJ_ELEM  "entity-object"
#define AGENT_OBJ_ELEM   "agent-object"

#define THIRST_ELEM "thirst"
#define HUNGER_ELEM "hunger"
#define ENERGY_ELEM "energy"
#define FITNESS_ELEM "fitness"
#define PEE_URGENCY_ELEM "pee_urgency"
#define POO_URGENCY_ELEM "poo_urgency"

#define FEAR_ELEM  "fear"
#define HATE_ELEM  "hate"
#define LOVE_ELEM  "love"
#define ANGER_ELEM "anger"
#define PRIDE_ELEM "pride"
#define GRATITUDE_ELEM "gratitude"
#define HAPPINESS_ELEM "happiness"
#define EXCITEMENT_ELEM "excitement"

#define AGGRESSIVENESS_ELEM "aggressiveness"
#define PLAYFULNESS_ELEM "playfulness"
#define CURIOSITY_ELEM "curiosity"
#define FRIENDLINESS_ELEM "friendliness"
#define FEARFULNESS_ELEM "fearfulness"
#define APPRECIATIVENESS_ELEM "appreciativeness"
#define EXCITABILITY_ELEM "excitability"

// Indefinite objects
#define INDEF_OBJ_ELEM "indefinite-object-info"

#define N_OBJ_ELEM "nearest_object"
#define N_EDIBLE_ELEM "nearest_edible  value"
#define N_MOVABLE_ELEM "nearest_movable value"
#define N_PICKUPABLE_ELEM "nearest_pickupable"
#define N_DRINKABLE_ELEM "nearest_drinkable"
#define N_AVATAR_ELEM "nearest_avatar value"
#define N_PET_ELEM "nearest_pet"
#define N_SMALL_ELEM "nearest_small"
#define N_MOVING_ELEM "nearest_moving"
#define N_NOISY_ELEM "nearest_noisy"
#define N_FRIENDLY_ELEM "nearest_friendly"
#define N_POO_PLACE_ELEM "nearest_poo_place"
#define N_PEE_PLACE_ELEM "nearest_pee_place"

#define R_OBJ_ELEM "random_object"
#define R_EDIBLE_ELEM "random_edible"
#define R_MOVABLE_ELEM "random_movable"
#define R_PICKUPABLE_ELEM "random_pickupable"
#define R_DRINKABLE_ELEM "random_drinkable"
#define R_AVATAR_ELEM "random_avatar"
#define R_PET_ELEM "random_pet"
#define R_SMALL_ELEM "random_small"
#define R_MOVING_ELEM "random_moving"
#define R_NOISY_ELEM "random_noisy"
#define R_FRIENDLY_ELEM "random_friendly"
#define R_PEE_PLACE_ELEM "random_poo_place"
#define R_POO_PLACE_ELEM "random_pee_place"

#define FOOD_BOWL_ELEM "food_bowl"
#define WATER_BOWL_ELEM "water_bowl"
#define PET_HOME_ELEM "pet_home"
#define PET_BOWL_ELEM "pet_bowl"
#define LAST_FOOD_PLACE_ELEM "last_food_place"
#define EXEMPLAR_AVATAR_ELEM "exemplar_avatar"

#define WORLD_STATE_INFO_ELEM "world-state-info"
#define NEAR_INFO_ELEM "near-info"
#define NEAR_ELEM "near"
#define NEXT_INFO_ELEM "next-info"
#define NEXT_ELEM "next"
#define OWNER_INFO_ELEM "owner-info"
#define OWNS_ELEM "owns"
#define MOVING_TOWARD_INFO_ELEM "moving-toward-info"
#define MOVING_TOWARD_ELEM "moving-toward"
#define INSIDE_FOV_INFO_ELEM "inside-fov-elem"
#define INSIDE_FOV_ELEM "inside-fov"
#define RELATIONS_INFO_ELEM "relations-info"
#define RELATION_ELEM "relation"
#define HAS_SAID_INFO_ELEM "has-said-info"
#define HAS_SAID_ELEM "has-said"
#define MESSAGE_ELEM "message"
#define LAST_AGENT_ACTION_INFO_ELEM "last-agent-action-info"
#define AGENT_ACTION_ELEM "agent-action"
#define LAST_PET_SCHEMA_INFO_ELEM "last-pet-schema-info"
#define PET_SCHEMA_ELEM "pet-schema"
#define PARAM_ELEM "param"

#define NAME_ATTR "name"
#define TYPE_ATTR "type"
#define NOISY_ATTR "noisy"
#define SMALL_ATTR "small"
#define EDIBLE_ATTR "edible"
#define MOVABLE_ATTR "movable"
#define DRINKABLE_ATTR "drinkable"
#define PEE_PLACE_ATTR "pee_place"
#define POO_PLACE_ATTR "poo_place"
#define PICKUPABLE_ATTR "pickupable"
#define MOVING_ATTR "moving"
#define NOVELTY_ATTR "novelty"
#define LEARNING_ATTR "learning"
#define ASKED_TO_TRY_ATTR "askedToTry"
#define LEARNED_TRICKS_ATTR "learnedTricks"
#define REQUESTED_SCHEMA_ATTR "requestedSchema"

#define PET_ID_ATTR "pet-id"
#define OWNER_ID_ATTR "owner-id"
#define PET_MODE_ATTR "pet-mode"
#define AGENT_STATE_ATTR "agent-state"
#define ACTION_REPETITION_ATTR "action-repetition"

#endif
