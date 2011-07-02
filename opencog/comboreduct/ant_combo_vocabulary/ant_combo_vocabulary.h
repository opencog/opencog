/*
 * opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _ANT_COMBO_VOCABULARY_H
#define _ANT_COMBO_VOCABULARY_H

#include "ant_builtin_action.h"
#include "ant_perception.h"
#include "ant_action_symbol.h"
#include "ant_indefinite_object.h"
#include <opencog/comboreduct/combo/vertex.h>

//this namespace is defined in order to break ambiguities
//and make coexist several vocabularies
namespace ant_combo {

using namespace opencog::combo;

//return a pointer to class base instance
//of a given action or perception enum
builtin_action instance(ant_builtin_action_enum);
perception instance(ant_perception_enum);
action_symbol instance(ant_action_symbol_enum);
indefinite_object instance(ant_indefinite_object_enum);

//get the enum corresponding to a builtin_action, perception or action_symbol
ant_builtin_action_enum get_enum(builtin_action);
ant_perception_enum get_enum(perception);
ant_action_symbol_enum get_enum(action_symbol);
ant_indefinite_object_enum get_enum(indefinite_object);
    
//this is added to be sure that the operators == and != between
//combo operator type and enum are used
//if not then it would easily lead to buggy situations because enum are cast to
//pointer
bool operator==(builtin_action, ant_builtin_action_enum);
bool operator==(ant_builtin_action_enum, builtin_action);
bool operator!=(builtin_action, ant_builtin_action_enum);
bool operator!=(ant_builtin_action_enum, builtin_action);
bool operator==(perception, ant_perception_enum);
bool operator==(ant_perception_enum, perception);
bool operator!=(perception, ant_perception_enum);
bool operator!=(ant_perception_enum, perception);
bool operator==(action_symbol, ant_action_symbol_enum);
bool operator==(ant_action_symbol_enum, action_symbol);
bool operator!=(action_symbol, ant_action_symbol_enum);
bool operator!=(ant_action_symbol_enum, action_symbol);
bool operator==(indefinite_object, ant_indefinite_object_enum);
bool operator==(ant_indefinite_object_enum, indefinite_object);
bool operator!=(indefinite_object, ant_indefinite_object_enum);
bool operator!=(ant_indefinite_object_enum, indefinite_object);

} // ~namespace ant_combo

namespace opencog { namespace combo {

std::istream& operator>>(std::istream& in, combo::vertex& v);
std::istream& operator>>(std::istream& in, combo::combo_tree& tr);

} // ~namespace combo
} // ~namespace opencog

#endif
