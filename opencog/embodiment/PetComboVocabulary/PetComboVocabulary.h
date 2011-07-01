/*
 * opencog/embodiment/PetComboVocabulary/PetComboVocabulary.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#ifndef _PETCOMBOVOCABULARY_H
#define _PETCOMBOVOCABULARY_H

#include "pet_builtin_action.h"
#include "pet_perception.h"
#include "pet_action_symbol.h"
#include "pet_indefinite_object.h"

#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace combo {

// Some common definitions

namespace id {

static const definite_object self = "self";
static const definite_object owner = "owner";
static const definite_object null_obj = "null_obj";
// custom path are processed inside agent handler.
static const definite_object custom_path = "custom_path";
// custom object or position are defined inside agentModeHandler.
static const definite_object custom_object = "custom_object";
static const definite_object custom_position = "custom_position";
static const definite_object custom_message = "custom_message";
// a custom object which represents all the avaiable agents
static const definite_object all_agents = "all_agents";

}

}} // ~namespaces combo opencog

// this namespace is defined in order to break ambiguities
// and allow several vocabularies to coexist 
namespace PetCombo {

using namespace opencog::combo;

//return a pointer to class base instance
//of a given action or perception enum
builtin_action instance(pet_builtin_action_enum);
perception instance(pet_perception_enum);
action_symbol instance(pet_action_symbol_enum);
indefinite_object instance(pet_indefinite_object_enum);

//get the enum corresponding to
//builtin_action, perception, action_symbol or indefinite_object
pet_builtin_action_enum get_enum(builtin_action);
pet_perception_enum get_enum(perception);
pet_action_symbol_enum get_enum(action_symbol);
pet_indefinite_object_enum get_enum(indefinite_object);

//return wether an indefinite object is random,
//of course the type of the object must be pet_indefinite_object*
bool is_random(indefinite_object);
bool is_random(pet_indefinite_object_enum);

//for some bizard reason this makes a compiling error
//however I keep them because it can indiquate when it is required to
//use base_instance(action or perception) instead of enum which print
//the wrong thing (a digit)
std::ostream& operator<<(std::ostream& out, pet_builtin_action_enum e);
std::ostream& operator<<(std::ostream& out, pet_perception_enum e);
std::ostream& operator<<(std::ostream& out, pet_action_symbol_enum e);
std::ostream& operator<<(std::ostream& out, pet_indefinite_object_enum e);

std::istream& operator>>(std::istream& in, vertex& v);
std::istream& operator>>(std::istream& in, combo_tree& tr);

//this is added to be sure that the operators == and != between
//combo operator type and enum is used
//That means that different vocabularies will not be able to coexist
//with the same code file but will be able within the same library
//using namespace PetCombo;
bool operator==(builtin_action, pet_builtin_action_enum);
bool operator==(pet_builtin_action_enum, builtin_action);
bool operator!=(builtin_action, pet_builtin_action_enum);
bool operator!=(pet_builtin_action_enum, builtin_action);
bool operator==(perception, pet_perception_enum);
bool operator==(pet_perception_enum, perception);
bool operator!=(perception, pet_perception_enum);
bool operator!=(pet_perception_enum, perception);
bool operator==(action_symbol, pet_action_symbol_enum);
bool operator==(pet_action_symbol_enum, action_symbol);
bool operator!=(action_symbol, pet_action_symbol_enum);
bool operator!=(pet_action_symbol_enum, action_symbol);
bool operator==(indefinite_object, pet_indefinite_object_enum);
bool operator==(pet_indefinite_object_enum, indefinite_object);
bool operator!=(indefinite_object, pet_indefinite_object_enum);
bool operator!=(pet_indefinite_object_enum, indefinite_object);

}//~namespace PetCombo

#endif

