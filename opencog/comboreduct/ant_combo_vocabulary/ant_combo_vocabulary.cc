/*
 * opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.cc
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
#include "ant_combo_vocabulary.h"

namespace ant_combo {

builtin_action instance(ant_builtin_action_enum e) {
    return ant_builtin_action::instance(e);
}
perception instance(ant_perception_enum e){
    return ant_perception::instance(e);
}
action_symbol instance(ant_action_symbol_enum e) {
    return ant_action_symbol::instance(e);
}
indefinite_object instance(ant_indefinite_object_enum e) {
    return ant_indefinite_object::instance(e);
}

ant_builtin_action_enum get_enum(builtin_action ba) {
    return dynamic_cast<const ant_builtin_action*>(ba)->get_enum();
}
ant_perception_enum get_enum(perception p) {
    return dynamic_cast<const ant_perception*>(p)->get_enum();
}
ant_action_symbol_enum get_enum(action_symbol as) {
    return dynamic_cast<const ant_action_symbol*>(as)->get_enum();
}
ant_indefinite_object_enum get_enum(indefinite_object as) {
    return dynamic_cast<const ant_indefinite_object*>(as)->get_enum();
}

bool operator==(builtin_action b, ant_builtin_action_enum e) {
    return instance(e)==b;
}
bool operator==(ant_builtin_action_enum e, builtin_action b) {
    return instance(e)==b;
}
bool operator!=(builtin_action b, ant_builtin_action_enum e) {
    return instance(e)!=b;
}
bool operator!=(ant_builtin_action_enum e, builtin_action b) {
    return instance(e)!=b;
}
bool operator==(perception p, ant_perception_enum e) {
    return instance(e)==p;
}
bool operator==(ant_perception_enum e, perception p) {
    return instance(e)==p;
}
bool operator!=(perception p, ant_perception_enum e) {
    return instance(e)!=p;
}
bool operator!=(ant_perception_enum e, perception p) {
    return instance(e)!=p;
}
bool operator==(action_symbol a, ant_action_symbol_enum e) {
    return instance(e)==a;
}
bool operator==(ant_action_symbol_enum e, action_symbol a) {
    return instance(e)==a;
}
bool operator!=(action_symbol a, ant_action_symbol_enum e) {
    return instance(e)!=a;
}
bool operator!=(ant_action_symbol_enum e, action_symbol a) {
    return instance(e)!=a;
}
bool operator==(indefinite_object i, ant_indefinite_object_enum e) {
    return instance(e)==i;
}
bool operator==(ant_indefinite_object_enum e, indefinite_object i) {
    return instance(e)==i;
}
bool operator!=(indefinite_object i, ant_indefinite_object_enum e) {
    return instance(e)!=i;
}
bool operator!=(ant_indefinite_object_enum e, indefinite_object i) {
    return instance(e)!=i;
}

}//~namespace ant_combo

namespace opencog { namespace combo {
  
std::istream& operator>>(std::istream& in, vertex& v) {
    return stream_to_vertex<ant_builtin_action, ant_perception, ant_action_symbol, ant_indefinite_object>(in, v);
}

std::istream& operator>>(std::istream& in, combo_tree& tr) {
    return stream_to_combo_tree<ant_builtin_action, ant_perception, ant_action_symbol, ant_indefinite_object>(in, tr);
}  
  
} // ~namespace combo
} // ~namespace opencog
