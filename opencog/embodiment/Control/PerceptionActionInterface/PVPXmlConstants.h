/*
 * opencog/embodiment/Control/PerceptionActionInterface/PVPXmlConstants.h
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

#ifndef PVPXMLCONSTANTS_H_
#define PVPXMLCONSTANTS_H_
/**
 * Define the contants used in the PVP xml messages.
 */

// xml elements
#define FROM_PROXY_ROOT_ELEMENT  "embodiment-msg"
#define ENTITY_ELEMENT           "entity"
#define POSITION_ELEMENT         "position"
#define VELOCITY_ELEMENT         "velocity"
#define ROTATION_ELEMENT         "rotation"
#define PARAMETER_ELEMENT        "param"
#define VECTOR_ELEMENT           "vector"
#define ACTION_PLAN_ELEMENT      "pet:action-plan"
#define ACTION_ELEMENT           "action"
#define MAP_INFO_ELEMENT         "map-info"
#define BLIP_ELEMENT             "blip"
#define PET_SIGNAL_ELEMENT       "pet-signal"
#define AVATAR_SIGNAL_ELEMENT    "avatar-signal"
#define INSTRUCTION_ELEMENT      "instruction"
#define AGENT_SENSOR_INFO_ELEMENT      "agent-sensor-info"
#define PERCEPTION_ELEMENT       "perception"
#define EMOTIONAL_FEELING_ELEMENT "pet:emotional-feeling"
#define FEELING_ELEMENT           "feeling"
#define PSI_DEMAND_ELEMENT       "pet:psi-demand"   
#define DEMAND_ELEMENT           "demand"
#define AGENT_SIGNAL_ELEMENT     "agent-signal"
#define PROPERTIES_ELEMENT       "properties"
#define PROPERTY_ELEMENT         "property"

// xml attributes
#define PET_ID_ATTRIBUTE                     "pet-id"
#define ENTITY_ID_ATTRIBUTE                  "entity-id"
#define NAME_ATTRIBUTE                       "name"
#define TIMESTAMP_ATTRIBUTE                  "timestamp"
#define DETECTOR_ATTRIBUTE                   "detector"
#define ID_ATTRIBUTE                         "id"
#define TYPE_ATTRIBUTE                       "type"
#define VALUE_ATTRIBUTE                      "value"
#define ACTION_PLAN_ID_ATTRIBUTE             "action-plan-id"
#define SEQUENCE_ATTRIBUTE                   "sequence"
#define AVATAR_ID_ATTRIBUTE                  "avatar-id"
#define X_ATTRIBUTE                          "x"
#define Y_ATTRIBUTE                          "y"
#define Z_ATTRIBUTE                          "z"
#define PITCH_ATTRIBUTE                      "pitch"
#define ROLL_ATTRIBUTE                       "roll"
#define YAW_ATTRIBUTE                        "yaw"
#define OWNER_ID_ATTRIBUTE                   "owner-id"
#define OWNER_NAME_ATTRIBUTE                 "owner-name"
#define OBJECT_ID_ATTRIBUTE                  "object-id"
#define STATUS_ATTRIBUTE                     "status"
#define REMOVE_ATTRIBUTE                     "remove"
#define LENGTH_ATTRIBUTE                     "length"
#define WIDTH_ATTRIBUTE                      "width"
#define VISIBILITY_STATUS_ATTRIBUTE          "visibility-status"
#define HEIGHT_ATTRIBUTE                     "height"
#define EDIBLE_ATTRIBUTE                     "edible"
#define DRINKABLE_ATTRIBUTE                  "drinkable"
#define PET_HOME_ATTRIBUTE                   "petHome"
#define FOOD_BOWL_ATTRIBUTE                  "foodBowl"
#define WATER_BOWL_ATTRIBUTE                 "waterBowl"
#define SENSOR_ATTRIBUTE           "sensor"
#define SUBJECT_ATTRIBUTE          "subject"
#define SIGNAL_ATTRIBUTE           "signal"
#define AGENT_ID_ATTRIBUTE       "agent-id"
#define AGENT_TYPE_ATTRIBUTE     "agent-type"
#define ENTITY_CLASS_ATTRIBUTE  "class"
#define COLOR_ATTRIBUTE         "color"
#define MATERIAL_ATTRIBUTE      "material"
#define TEXTURE_ATTRIBUTE       "texture"
#define IS_TOY_ATTRIBUTE        "isToy"

#define PRIORITY_ATTRIBUTE       "priority" // Note from Tristan: this is actually used only when actions are sent from Proxy to SL. 
#define GLOBAL_POS_X_ATTRIBUTE   "global-position-x"
#define GLOBAL_POS_Y_ATTRIBUTE   "global-position-y"
#define GLOBAL_POS_OFFSET_ATTRIBUTE   "global-position-offset"
#define GLOBAL_FLOOR_HEIGHT_ATTRIBUTE "global-floor-height"

// xml object types
#define PET_OBJECT_TYPE          "pet"
#define HUMANOID_OBJECT_TYPE     "humanoid"
#define STRUCTURE_OBJECT_TYPE    "structure"
#define AVATAR_OBJECT_TYPE       "avatar"
#define ACCESSORY_OBJECT_TYPE    "accessory"
#define ORDINARY_OBJECT_TYPE     "object"
#define UNKNOWN_OBJECT_TYPE      "unknown"

// known parameter names
#define POSITION_PARAMETER_NAME  "position"
#define ROTATE_PARAMETER_NAME    "rotate"

// xml action status (values for <pet-signal>'s status attribute)
#define DONE_ACTION_STATUS       "done"
#define ERROR_ACTION_STATUS      "error"

// instruction tags
#define SENTENCE_TYPE                   "sentence"
#define PARSED_SENTENCE_TYPE            "parsed-sentence"
#define CONTENT_TYPE_ATTRIBUTE          "content-type"
#define TARGET_MODE_ATTRIBUTE           "target-mode"

#endif /*PVPXMLCONSTANTS_H_*/
