/*
 * opencog/embodiment/AvatarComboVocabulary/avatar_indefinite_object.cc
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

#include "avatar_indefinite_object.h"
#include <opencog/comboreduct/type_checker/type_tree.h>

using namespace opencog::combo;
using namespace avatar_indefinite_object_properties;

avatar_indefinite_object::avatar_indefinite_object()
{
    _random = false;
}

const indefinite_object_basic_description* avatar_indefinite_object::get_basic_description_array() const
{
    return iobd;
}

unsigned int avatar_indefinite_object::get_basic_description_array_count() const
{
    return sizeof(iobd) / sizeof(basic_description);
}

const avatar_indefinite_object* avatar_indefinite_object::init_indefinite_object()
{
    static avatar_indefinite_object* indefinite_objects =
        new avatar_indefinite_object[id::avatar_indefinite_object_count];
    for (unsigned int i = 0; i < id::avatar_indefinite_object_count; i++)
        indefinite_objects[i].set_indefinite_object((avatar_indefinite_object_enum)i);
    return indefinite_objects;
}

void avatar_indefinite_object::set_indefinite_object(avatar_indefinite_object_enum pioe)
{
    OC_ASSERT(pioe < id::avatar_indefinite_object_count);
    _enum = pioe;
    //fill the various properties using the arrays edited by the developer
    set_basic_description(pioe);
    //fill the properties
    unsigned int iopd_count =
        sizeof(iopd) / sizeof(indefinite_object_property_description);
    OC_ASSERT(
                     iopd_count == (unsigned int)id::avatar_indefinite_object_count,
                     "there must be entries for all indefinite objects.");
    bool found = false;
    for (unsigned int i = 0; i < iopd_count && !found; ++i) {
        if (iopd[i].indefinite_object == pioe) {
            found = true;
            _random = iopd[i].random;
        }
    }
    OC_ASSERT(found,
                     "avatar_perception with enum %d has not been found in iopd", pioe);
}

indefinite_object avatar_indefinite_object::get_instance(const std::string& name)
{
    //look up for avatar_indefinite_object_enum corresponding to that name
    bool found = false;
    indefinite_object as = NULL;
    for (unsigned int i = 0; i < id::avatar_indefinite_object_count && !found; i++) {
        as = avatar_indefinite_object::get_instance((avatar_indefinite_object_enum)i);
        found = as->get_name() == name;
    }
    return (found ? as : NULL);
}

indefinite_object avatar_indefinite_object::get_instance(avatar_indefinite_object_enum pioe)
{
    static const avatar_indefinite_object* indefinite_objects = init_indefinite_object();
    OC_ASSERT(pioe < id::avatar_indefinite_object_count);
    return static_cast<indefinite_object>(&indefinite_objects[pioe]);
}

const std::string& avatar_indefinite_object::get_name() const
{
    return _name;
}

const type_tree& avatar_indefinite_object::get_type_tree() const
{
    return _type_tree;
}

arity_t avatar_indefinite_object::arity() const
{
    return _arity;
}

type_tree avatar_indefinite_object::get_output_type_tree() const
{
    return _output_type;
}

const type_tree& avatar_indefinite_object::get_input_type_tree(arity_t i) const
{
    return argument_type_list_input_type(_arg_type_tree, _arity, i);
}

bool avatar_indefinite_object::is_random() const
{
    return _random;
}
