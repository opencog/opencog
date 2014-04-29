/*
 * opencog/embodiment/AvatarComboVocabulary/avatar_indefinite_object.h
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

#ifndef _AVATAR_INDEFINITE_OBJECT_H
#define _AVATAR_INDEFINITE_OBJECT_H

#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/indefinite_object.h>
#include "avatar_operator.h"

namespace opencog { namespace combo {

namespace id {
enum avatar_indefinite_object_enum {
    nearest_object,
    nearest_edible,
    nearest_movable,
    nearest_pickupable,
    nearest_drinkable,
    nearest_avatar,
    nearest_small,
    nearest_moving,
    nearest_noisy,

    random_object,
    random_edible,
    random_movable,
    random_pickupable,
    random_drinkable,
    random_avatar,
    random_small,
    random_moving,
    random_noisy,

    last_food_place,

    exemplar_avatar,

    avatar_indefinite_object_count
};
}

typedef id::avatar_indefinite_object_enum avatar_indefinite_object_enum;

/*********************************************************************
 *   Arrays containing indefinite_object name type and properties    *
 *                 to be edited by the developer                     *
 *********************************************************************/

namespace avatar_indefinite_object_properties {

//struct for description of name and type
typedef avatar_operator<avatar_indefinite_object_enum, id::avatar_indefinite_object_count>::basic_description indefinite_object_basic_description;

//struct for property description
struct indefinite_object_property_description {
    avatar_indefinite_object_enum indefinite_object;
    bool random;
};

static const indefinite_object_basic_description iobd[] = {
    //indefinite_object        name                   type
    { id::nearest_object,      "nearest_object",      "indefinite_object" },
    { id::nearest_edible,      "nearest_edible",      "indefinite_object" },
    { id::nearest_movable,     "nearest_movable",     "indefinite_object" },
    { id::nearest_pickupable,  "nearest_pickupable",  "indefinite_object" },
    { id::nearest_drinkable,   "nearest_drinkable",   "indefinite_object" },
    { id::nearest_avatar,      "nearest_avatar",      "indefinite_object" },
    { id::nearest_small,       "nearest_small",       "indefinite_object" },
    { id::nearest_moving,      "nearest_moving",      "indefinite_object" },
    { id::nearest_noisy,       "nearest_noisy",       "indefinite_object" },
    { id::random_object,       "random_object",       "indefinite_object" },
    { id::random_edible,       "random_edible",       "indefinite_object" },
    { id::random_movable,      "random_movable",      "indefinite_object" },
    { id::random_pickupable,   "random_pickupable",   "indefinite_object" },
    { id::random_drinkable,    "random_drinkable",    "indefinite_object" },
    { id::random_avatar,       "random_avatar",       "indefinite_object" },
    { id::random_small,        "random_small",        "indefinite_object" },
    { id::random_moving,       "random_moving",       "indefinite_object" },
    { id::random_noisy,        "random_noisy",        "indefinite_object" },
    { id::last_food_place,     "last_food_place",     "indefinite_object" },
    { id::exemplar_avatar,     "exemplar_avatar",     "indefinite_object" }

};

static const indefinite_object_property_description iopd[] = {
    //indefinite_object       random
    { id::nearest_object,     false },
    { id::nearest_edible,     false },
    { id::nearest_movable,    false },
    { id::nearest_pickupable, false },
    { id::nearest_drinkable,  false },
    { id::nearest_avatar,     false },
    { id::nearest_small,      false },
    { id::nearest_moving,     false },
    { id::nearest_noisy,      false },
    { id::random_object,      true },
    { id::random_edible,      true },
    { id::random_movable,     true },
    { id::random_pickupable,  true },
    { id::random_drinkable,   true },
    { id::random_avatar,      true },
    { id::random_small,       true },
    { id::random_moving,      true },
    { id::random_noisy,       true },
    { id::last_food_place,    false },
    { id::exemplar_avatar,    false }
};

}//~namespace avatar_perception_properties

//avatar_indefinite_object both derive
//from indefinite_object_base and avatar_operator
class avatar_indefinite_object : public avatar_operator<avatar_indefinite_object_enum, id::avatar_indefinite_object_count>, public indefinite_object_base
{

private:

    //private attribute
    bool _random; //indicate whether the indefinite object is random

    //private methods

    //ctor
    avatar_indefinite_object();

    const basic_description * get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const avatar_indefinite_object* init_indefinite_object();
    void set_indefinite_object(avatar_indefinite_object_enum);

public:
    //name
    const std::string& get_name() const;

    //type_tree
    const type_tree& get_type_tree() const;

    //helper methods for fast access type properties
    //number of arguments that takes the operator
    arity_t arity() const;
    //return the type node of the operator
    type_tree get_output_type_tree() const;

    //return the type tree of the input argument of index i
    //if the operator has arg_list(T) as last input argument
    //then it returns always T past that index
    const type_tree& get_input_type_tree(arity_t i) const;

    //return a pointer of the static action_symbol corresponding
    //to a given name string
    //if no such action_symbol exists then return NULL pointer
    static indefinite_object get_instance(const std::string& name);

    //return a pointer of the static avatar_perception_action corresponding
    //to a given avatar_perception_enum
    static indefinite_object get_instance(avatar_indefinite_object_enum);

    //is_random, property used by PetBrain
    bool is_random() const;

};

}} // ~namespaces combo opencog

#endif
