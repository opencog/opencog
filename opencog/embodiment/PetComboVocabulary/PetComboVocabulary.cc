/*
 * opencog/embodiment/PetComboVocabulary/PetComboVocabulary.cc
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

#include "PetComboVocabulary.h"

namespace PetCombo
{

builtin_action get_instance(pet_builtin_action_enum e)
{
    return pet_builtin_action::get_instance(e);
}
perception get_instance(pet_perception_enum e)
{
    return pet_perception::get_instance(e);
}
action_symbol get_instance(pet_action_symbol_enum e)
{
    return pet_action_symbol::get_instance(e);
}
indefinite_object get_instance(pet_indefinite_object_enum e)
{
    return pet_indefinite_object::get_instance(e);
}

pet_builtin_action_enum get_enum(builtin_action ba)
{
    return dynamic_cast<const pet_builtin_action*>(ba)->get_enum();
}
pet_perception_enum get_enum(perception p)
{
    return dynamic_cast<const pet_perception*>(p)->get_enum();
}
pet_action_symbol_enum get_enum(action_symbol p)
{
    return dynamic_cast<const pet_action_symbol*>(p)->get_enum();
}
pet_indefinite_object_enum get_enum(indefinite_object o)
{
    return dynamic_cast<const pet_indefinite_object*>(o)->get_enum();
}

bool is_random(indefinite_object o)
{
    return dynamic_cast<const pet_indefinite_object*>(o)->is_random();
}
bool is_random(pet_indefinite_object_enum oe)
{
    return dynamic_cast<const pet_indefinite_object*>(get_instance(oe))->is_random();
}

std::ostream& operator<<(std::ostream& out, pet_builtin_action_enum e)
{
    out << get_instance(e)->get_name();
    return out;
}
std::ostream& operator>>(std::ostream& out, pet_perception_enum e)
{
    out << get_instance(e)->get_name();
    return out;
}
std::ostream& operator>>(std::ostream& out, pet_action_symbol_enum e)
{
    out << get_instance(e)->get_name();
    return out;
}

std::istream& operator>>(std::istream& in, vertex& v)
{
    return stream_to_vertex<pet_builtin_action, pet_perception, pet_action_symbol, pet_indefinite_object>(in, v);
}

std::istream& operator>>(std::istream& in, combo_tree& tr)
{
    return stream_to_combo_tree<pet_builtin_action, pet_perception, pet_action_symbol, pet_indefinite_object>(in, tr);
}

bool operator==(builtin_action b, pet_builtin_action_enum e)
{
    return get_instance(e) == b;
}
bool operator==(pet_builtin_action_enum e, builtin_action b)
{
    return get_instance(e) == b;
}
bool operator!=(builtin_action b, pet_builtin_action_enum e)
{
    return get_instance(e) == b;
}
bool operator!=(pet_builtin_action_enum e, builtin_action b)
{
    return get_instance(e) == b;
}
bool operator==(perception p, pet_perception_enum e)
{
    return get_instance(e) == p;
}
bool operator==(pet_perception_enum e, perception p)
{
    return get_instance(e) == p;
}
bool operator!=(perception p, pet_perception_enum e)
{
    return get_instance(e) == p;
}
bool operator!=(pet_perception_enum e, perception p)
{
    return get_instance(e) == p;
}
bool operator==(action_symbol a, pet_action_symbol_enum e)
{
    return get_instance(e) == a;
}
bool operator==(pet_action_symbol_enum e, action_symbol a)
{
    return get_instance(e) == a;
}
bool operator!=(action_symbol a, pet_action_symbol_enum e)
{
    return get_instance(e) == a;
}
bool operator!=(pet_action_symbol_enum e, action_symbol a)
{
    return get_instance(e) == a;
}
bool operator==(indefinite_object i, pet_indefinite_object_enum e)
{
    return get_instance(e) == i;
}
bool operator==(pet_indefinite_object_enum e, indefinite_object i)
{
    return get_instance(e) == i;
}
bool operator!=(indefinite_object i, pet_indefinite_object_enum e)
{
    return get_instance(e) == i;
}
bool operator!=(pet_indefinite_object_enum e, indefinite_object i)
{
    return get_instance(e) == i;
}

}//~namespace PetCombo
