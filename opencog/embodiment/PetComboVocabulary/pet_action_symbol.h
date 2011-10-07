/*
 * opencog/embodiment/PetComboVocabulary/pet_action_symbol.h
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

#ifndef _PET_ACTION_SYMBOL_H
#define _PET_ACTION_SYMBOL_H

#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/action_symbol.h>
#include "pet_operator.h"

namespace opencog { namespace combo {

namespace id {
enum pet_action_symbol_enum {
    TOWARDS,
    AWAY,
    RIGHT_FOOT,
    LEFT_FOOT,
    RIGHT_HAND,
    LEFT_HAND,
    CROTCH,
    BUTT,
    NOSE,
    RIGHT_EAR,
    LEFT_EAR,
    NECK,
    RIGHT_SHOULDER,
    LEFT_SHOULDER,
    TWITCH,
    PERK,
    BACK,
    pet_action_symbol_count
};
}

typedef id::pet_action_symbol_enum pet_action_symbol_enum;

/*********************************************************************
 *      Arrays containing action_symbol name type and properties     *
 *                 to be edited by the developer                     *
 *********************************************************************/

namespace pet_action_symbol_properties {

//struct for description of name and type
typedef pet_operator<pet_action_symbol_enum, id::pet_action_symbol_count>::basic_description action_symbol_basic_description;

static const action_symbol_basic_description asbd[] = {
    //action_symbol          name                 type
    { id::TOWARDS,           "TOWARDS",           "action_symbol" },
    { id::AWAY,              "AWAY",              "action_symbol" },
    { id::RIGHT_FOOT,        "RIGHT_FOOT",        "action_symbol" },
    { id::LEFT_FOOT,         "LEFT_FOOT",         "action_symbol" },
    { id::RIGHT_HAND,        "RIGHT_HAND",        "action_symbol" },
    { id::LEFT_HAND,         "LEFT_HAND",         "action_symbol" },
    { id::CROTCH,            "CROTCH",            "action_symbol" },
    { id::BUTT,              "BUTT",              "action_symbol" },
    { id::NOSE,              "NOSE",              "action_symbol" },
    { id::RIGHT_EAR,         "RIGHT_EAR",         "action_symbol" },
    { id::LEFT_EAR,          "LEFT_EAR",          "action_symbol" },
    { id::NECK,              "NECK",              "action_symbol" },
    { id::RIGHT_SHOULDER,    "RIGHT_SHOULDER",    "action_symbol" },
    { id::LEFT_SHOULDER,     "LEFT_SHOULDER",     "action_symbol" },
    { id::TWITCH,            "TWITCH",            "action_symbol" },
    { id::PERK,              "PERK",              "action_symbol" },
    { id::BACK,              "BACK",              "action_symbol" }
};

}//~namespace pet_perception_properties

//pet_action_symbol both derive from action_symbol_base and pet_operator
class pet_action_symbol : public pet_operator<pet_action_symbol_enum, id::pet_action_symbol_count>, public action_symbol_base
{

private:

    //private methods

    //ctor
    pet_action_symbol();

    const basic_description * get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const pet_action_symbol* init_action_symbol();
    void set_action_symbol(pet_action_symbol_enum);

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
    static action_symbol get_instance(const std::string& name);

    //return a pointer of the static pet_perception_action corresponding
    //to a given pet_perception_enum
    static action_symbol get_instance(pet_action_symbol_enum);

};

}} // ~namespaces combo opencog

#endif
