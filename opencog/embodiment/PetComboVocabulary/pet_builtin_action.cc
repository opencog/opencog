/*
 * opencog/embodiment/PetComboVocabulary/pet_builtin_action.cc
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

#include "pet_builtin_action.h"
#include <opencog/comboreduct/combo/type_tree.h>

using namespace opencog::combo;
using namespace pet_builtin_action_properties;

pet_builtin_action::pet_builtin_action()
{
    _reversible = false;
    _always_succeeds = false;
    _reversal = NULL;
    _exists_additive_argument = false;
    _exists_zero_neutral_argument = false;
    _compound = false;
}

const action_basic_description* pet_builtin_action::get_basic_description_array() const
{
    return pet_builtin_action_properties::abd;
}

unsigned int pet_builtin_action::get_basic_description_array_count() const
{
    return sizeof(pet_builtin_action_properties::abd) / sizeof(basic_description);
}

const pet_builtin_action* pet_builtin_action::init_actions()
{
    pet_builtin_action* action_array =
        new pet_builtin_action[id::pet_builtin_action_count];
    for (unsigned int i = 0; i < id::pet_builtin_action_count; i++)
        action_array[i].set_action((pet_builtin_action_enum)i, action_array);
    return action_array;
}

void pet_builtin_action::set_action(pet_builtin_action_enum pbae,
                                    pet_builtin_action* action_array)
{
    OC_ASSERT(pbae < id::pet_builtin_action_count);
    _enum = pbae;
    //printf("_enum = %d\n", _enum);
    //fill the various properties using the arrays edited by the developer

    //basic properties
    set_basic_description(pbae);
    //standard and PetBrain properties specific to action
    unsigned int apd_count = sizeof(apd) / sizeof(action_property_description);
    OC_ASSERT(
                     apd_count == (unsigned int)id::pet_builtin_action_count,
                     "there must be entries for all actions.");
    bool found = false;
    for (unsigned int i = 0; i < apd_count && !found; ++i) {
        if (apd[i].action == pbae) {
            found = true;
            _compound = apd[i].compound;
            _idempotent = apd[i].idempotent;
            _reversible = apd[i].reversible;
            _always_succeeds = apd[i].always_succeeds;
            if (_reversible)
                _reversal = &action_array[apd[i].reversal];
            else _reversal = NULL;
        }
    }
    OC_ASSERT(found,
                     "pet_builtin_action with enum %d has not been found in apd", pbae);
    //standard properties specific to action argument
    unsigned int aapd_count =
        sizeof(aapd) / sizeof(action_argument_property_description);
    std::set<arity_t> arg_not_found; //contains all argument indices
    //set arg_not_found
    arity_t abs_arity = std::abs(_arity);
    for (arity_t a = 0; a < abs_arity; a++)
        arg_not_found.insert(a);
    //initialize all argument fast arrays
    _arg_additive.resize(abs_arity);
    _arg_zero_neutral.resize(abs_arity);
    _arg_modulo.resize(abs_arity);
    _arg_modulo_max.resize(abs_arity);
    _arg_modulo_min.resize(abs_arity);
    //fill all argument fast arrays
    for (unsigned int i = 0; i < aapd_count && !arg_not_found.empty(); ++i) {
        if (aapd[i].action == pbae) {
            //take the found index argument off arg_not_found
            arity_t index = aapd[i].argument_index;
            std::set<arity_t>::iterator it = arg_not_found.find(index);
            OC_ASSERT(it != arg_not_found.end(),
                             "Either action %s does not have argument %d or this argument has already been found", _name.c_str(), index);
            arg_not_found.erase(it);
            OC_ASSERT(index < abs_arity);
            _arg_additive[index] = aapd[i].additive;
            _arg_zero_neutral[index] = aapd[i].zero_neutral;
            _arg_modulo[index] = aapd[i].modular;
            _arg_modulo_min[index] = aapd[i].min_value;
            _arg_modulo_max[index] = aapd[i].max_value;
        }
    }
    OC_ASSERT(arg_not_found.empty(),
                     "Some argument are not found in aapd for action %s", _name.c_str());
    //check if exists additive, zero_neutral
    bool not_exists_additive = true;
    for (std::vector<bool>::const_iterator it = _arg_additive.begin();
            it != _arg_additive.end() && not_exists_additive; ++it)
        not_exists_additive = !*it;
    _exists_additive_argument = !not_exists_additive;
    bool not_exists_zero_neutral = true;
    for (std::vector<bool>::const_iterator it = _arg_zero_neutral.begin();
            it != _arg_zero_neutral.end() && not_exists_zero_neutral; ++it)
        not_exists_zero_neutral = !*it;
    _exists_zero_neutral_argument = !not_exists_zero_neutral;
    //preconditions
    unsigned int mp_count = sizeof(must_precede) / sizeof(action_precedence);
    for (unsigned int i = 0; i < mp_count; i++) {
        if (must_precede[i].action2 == _enum) {
            _preconditions.insert(&action_array[must_precede[i].action1]);
        }
    }
}

builtin_action pet_builtin_action::instance(const std::string& name)
{
    //look up for pet_builtin_action_enum corresponding to that name
    bool found = false;
    builtin_action a = NULL;
    for (unsigned int i = 0; i < id::pet_builtin_action_count && !found; i++) {
        a = pet_builtin_action::instance((pet_builtin_action_enum)i);
        found = a->get_name() == name;
    }
    return (found ? a : NULL);
}

builtin_action pet_builtin_action::instance(pet_builtin_action_enum pbae)
{
    static const pet_builtin_action* actions = init_actions();
    OC_ASSERT(pbae < id::pet_builtin_action_count);
    return static_cast<builtin_action>(&actions[pbae]);
}

const std::string& pet_builtin_action::get_name() const
{
    return _name;
}

const type_tree& pet_builtin_action::get_type_tree() const
{
    return _type_tree;
}

arity_t pet_builtin_action::arity() const
{
    return _arity;
}

type_tree pet_builtin_action::get_output_type_tree() const
{
    return _output_type;
}

const type_tree& pet_builtin_action::get_input_type_tree(arity_t i) const
{
    return argument_type_list_input_type(_arg_type_tree, _arity, i);
}

bool pet_builtin_action::is_reversible() const
{
    return _reversible;
}

bool pet_builtin_action::always_succeeds() const
{
    return _always_succeeds;
}

const builtin_action_base* pet_builtin_action::get_reversal() const
{
    return _reversal;
}

bool pet_builtin_action::is_idempotent() const
{
    return _idempotent;
}

bool pet_builtin_action::is_additive(arity_t i) const
{
    return _arg_additive[convert_index(_arity, i)];
}

bool pet_builtin_action::exists_additive_argument() const
{
    return _exists_additive_argument;
}

bool pet_builtin_action::is_zero_neutral(arity_t i) const
{
    return _arg_zero_neutral[convert_index(_arity, i)];
}

bool pet_builtin_action::exists_zero_neutral_argument() const
{
    return _exists_zero_neutral_argument;
}

bool pet_builtin_action::is_modulo(arity_t i) const
{
    return _arg_modulo[convert_index(_arity, i)];
}

double pet_builtin_action::modulo_min(arity_t i) const
{
    return _arg_modulo_min[convert_index(_arity, i)];
}

double pet_builtin_action::modulo_max(arity_t i) const
{
    return _arg_modulo_max[convert_index(_arity, i)];
}

const std::set<builtin_action> pet_builtin_action::preconditions() const
{
    return _preconditions;
}

bool pet_builtin_action::is_compound() const
{
    return _compound;
}
