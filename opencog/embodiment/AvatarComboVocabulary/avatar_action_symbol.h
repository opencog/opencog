/*
 * opencog/embodiment/AvatarComboVocabulary/avatar_action_symbol.h
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

#ifndef _AVATAR_ACTION_SYMBOL_H
#define _AVATAR_ACTION_SYMBOL_H

#include <opencog/util/numeric.h>

#include <moses/comboreduct/combo/action_symbol.h>
#include "avatar_operator.h"

namespace opencog { namespace combo {

namespace id {
enum avatar_action_symbol_enum {
    avatar_action_symbol_count
};
}

typedef id::avatar_action_symbol_enum avatar_action_symbol_enum;

/*********************************************************************
 *      Arrays containing action_symbol name type and properties     *
 *                 to be edited by the developer                     *
 *********************************************************************/

namespace avatar_action_symbol_properties {

//struct for description of name and type
typedef avatar_operator<avatar_action_symbol_enum, id::avatar_action_symbol_count>::basic_description action_symbol_basic_description;

static const action_symbol_basic_description asbd[] = {
    //action_symbol          name                 type
};

}//~namespace avatar_perception_properties

//avatar_action_symbol both derive from action_symbol_base and avatar_operator
class avatar_action_symbol : public avatar_operator<avatar_action_symbol_enum, id::avatar_action_symbol_count>, public action_symbol_base
{

private:

    //private methods

    //ctor
    avatar_action_symbol();

    const basic_description * get_basic_description_array() const;
    unsigned int get_basic_description_array_count() const;

    static const avatar_action_symbol* init_action_symbol();
    void set_action_symbol(avatar_action_symbol_enum);

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

    //return a pointer of the static avatar_perception_action corresponding
    //to a given avatar_perception_enum
    static action_symbol get_instance(avatar_action_symbol_enum);

};

}} // ~namespaces combo opencog

#endif
