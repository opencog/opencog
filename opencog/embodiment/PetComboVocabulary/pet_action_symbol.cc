/*
 * opencog/embodiment/PetComboVocabulary/pet_action_symbol.cc
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

#include "pet_action_symbol.h"
#include <opencog/comboreduct/combo/type_tree.h>

using namespace opencog::combo;
using namespace pet_action_symbol_properties;

pet_action_symbol::pet_action_symbol() { }

const action_symbol_basic_description* pet_action_symbol::get_basic_description_array() const
{
    return asbd;
}

unsigned int pet_action_symbol::get_basic_description_array_count() const
{
    return sizeof(asbd) / sizeof(basic_description);
}

const pet_action_symbol* pet_action_symbol::init_action_symbol()
{
    static pet_action_symbol* action_symbols =
        new pet_action_symbol[id::pet_action_symbol_count];
    for (unsigned int i = 0; i < id::pet_action_symbol_count; i++)
        action_symbols[i].set_action_symbol((pet_action_symbol_enum)i);
    return action_symbols;
}

void pet_action_symbol::set_action_symbol(pet_action_symbol_enum pase)
{
    OC_ASSERT(pase < id::pet_action_symbol_count);
    _enum = pase;
    //fill the various properties using the arrays edited by the developer
    set_basic_description(pase);
}

action_symbol pet_action_symbol::get_instance(const std::string& name)
{
    //look up for pet_action_symbol_enum corresponding to that name
    bool found = false;
    action_symbol as = NULL;
    for (unsigned int i = 0; i < id::pet_action_symbol_count && !found; i++) {
        as = pet_action_symbol::get_instance((pet_action_symbol_enum)i);
        found = as->get_name() == name;
    }
    return (found ? as : NULL);
}

action_symbol pet_action_symbol::get_instance(pet_action_symbol_enum pase)
{
    static const pet_action_symbol* action_symbols = init_action_symbol();
    OC_ASSERT(pase < id::pet_action_symbol_count);
    return static_cast<action_symbol>(&action_symbols[pase]);
}

const std::string& pet_action_symbol::get_name() const
{
    return _name;
}

const type_tree& pet_action_symbol::get_type_tree() const
{
    return _type_tree;
}

arity_t pet_action_symbol::arity() const
{
    return _arity;
}

type_tree pet_action_symbol::get_output_type_tree() const
{
    return _output_type;
}

const type_tree& pet_action_symbol::get_input_type_tree(arity_t i) const
{
    return argument_type_list_input_type(_arg_type_tree, _arity, i);
}
