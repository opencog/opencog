/*
 * opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h
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

#ifndef _AVATARCOMBOVOCABULARY_H
#define _AVATARCOMBOVOCABULARY_H

#include "avatar_builtin_action.h"
#include "avatar_perception.h"
#include "avatar_action_symbol.h"
#include "avatar_indefinite_object.h"

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
namespace AvatarCombo {

using namespace opencog::combo;

//return a pointer to class base instance
//of a given action or perception enum
builtin_action get_instance(avatar_builtin_action_enum);
perception get_instance(avatar_perception_enum);
action_symbol get_instance(avatar_action_symbol_enum);
indefinite_object get_instance(avatar_indefinite_object_enum);

//get the enum corresponding to
//builtin_action, perception, action_symbol or indefinite_object
avatar_builtin_action_enum get_enum(builtin_action);
avatar_perception_enum get_enum(perception);
avatar_action_symbol_enum get_enum(action_symbol);
avatar_indefinite_object_enum get_enum(indefinite_object);

//return wether an indefinite object is random,
//of course the type of the object must be avatar_indefinite_object*
bool is_random(indefinite_object);
bool is_random(avatar_indefinite_object_enum);

// Overload the stream operators
std::ostream& operator<<(std::ostream&, avatar_builtin_action_enum);
std::ostream& operator<<(std::ostream&, avatar_perception_enum);
std::ostream& operator<<(std::ostream&, avatar_action_symbol_enum);
std::ostream& operator<<(std::ostream&, avatar_indefinite_object_enum);

// Overload the operators to parse input, and create vertexes and combos.
// XXX This design is real dicy, as its easy for the compiler to use
// the wrong opator>>(), and to get confused about which of serval to
// use... the problem is that the below, which are really 
// AvatarCombo::operator>>() collide with opencog::combo::operator>>()
std::istream& operator>>(std::istream&, vertex&);
std::istream& operator>>(std::istream&, combo_tree&);

// This is added to be sure that the operators == and != between
// combo operator type and enum is used
// That means that different vocabularies will be able to coexist
// with the same code file and within the same library -- just say
// using namespace AvatarCombo.
bool operator==(builtin_action, avatar_builtin_action_enum);
bool operator==(avatar_builtin_action_enum, builtin_action);
bool operator!=(builtin_action, avatar_builtin_action_enum);
bool operator!=(avatar_builtin_action_enum, builtin_action);
bool operator==(perception, avatar_perception_enum);
bool operator==(avatar_perception_enum, perception);
bool operator!=(perception, avatar_perception_enum);
bool operator!=(avatar_perception_enum, perception);
bool operator==(action_symbol, avatar_action_symbol_enum);
bool operator==(avatar_action_symbol_enum, action_symbol);
bool operator!=(action_symbol, avatar_action_symbol_enum);
bool operator!=(avatar_action_symbol_enum, action_symbol);
bool operator==(indefinite_object, avatar_indefinite_object_enum);
bool operator==(avatar_indefinite_object_enum, indefinite_object);
bool operator!=(indefinite_object, avatar_indefinite_object_enum);
bool operator!=(avatar_indefinite_object_enum, indefinite_object);

} // ~namespace AvatarCombo

#endif

