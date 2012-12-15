/*
 * opencog/embodiment/AvatarComboVocabulary/avatar_perception.cc
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

#include "avatar_perception.h"
#include <opencog/comboreduct/type_checker/type_tree.h>

using namespace opencog::combo;
using namespace avatar_perception_properties;

avatar_perception::avatar_perception()
{
    _ultrametric = false;
    _transitive = false;
    _irreflexive = false;
    _reflexive = false;
    _symmetric = false;
    _identity_of_indiscernibles = false;
}

const perception_basic_description* avatar_perception::get_basic_description_array() const
{
    return avatar_perception_properties::pbd;
}

unsigned int avatar_perception::get_basic_description_array_count() const
{
    return sizeof(avatar_perception_properties::pbd) / sizeof(basic_description);
}

const avatar_perception* avatar_perception::init_perceptions()
{
    static avatar_perception* perceptions =
        new avatar_perception[id::avatar_perception_count];
    for (unsigned int i = 0; i < id::avatar_perception_count; i++)
        perceptions[i].set_perception((avatar_perception_enum)i);
    return perceptions;
}

void avatar_perception::set_perception(avatar_perception_enum ppe)
{
    OC_ASSERT(ppe < id::avatar_perception_count);
    _enum = ppe;
    //fill the various properties using the arrays edited by the developer
    set_basic_description(ppe);
    //standard properties specific to action
    unsigned int ppd_count = sizeof(ppd) / sizeof(perception_property_description);
    OC_ASSERT(
                     ppd_count == (unsigned int)id::avatar_perception_count,
                     "there must be entries for all perceptions.");
    bool found = false;
    for (unsigned int i = 0; i < ppd_count && !found; ++i) {
        if (ppd[i].perception == ppe) {
            found = true;
            _ultrametric = ppd[i].ultrametric;
            _transitive = ppd[i].transitive;
            _irreflexive = ppd[i].irreflexive;
            _reflexive = ppd[i].reflexive;
            _symmetric = ppd[i].symmetric;
            _identity_of_indiscernibles = ppd[i].identity_of_indiscernibles;
        }
    }
    OC_ASSERT(found,
                     "avatar_perception with enum %d has not been found in ppd", ppe);
}

perception avatar_perception::get_instance(const std::string& name)
{
    //look up for avatar_builtin_action_enum corresponding to that name
    bool found = false;
    perception p = NULL;
    for (unsigned int i = 0; i < id::avatar_perception_count && !found; i++) {
        p = avatar_perception::get_instance((avatar_perception_enum)i);
        found = p->get_name() == name;
    }
    return (found ? p : NULL);
}

perception avatar_perception::get_instance(avatar_perception_enum ppe)
{
    static const avatar_perception* perceptions = init_perceptions();
    OC_ASSERT(ppe < id::avatar_perception_count);
    return static_cast<perception>(&perceptions[ppe]);
}

const std::string& avatar_perception::get_name() const
{
    return _name;
}

const type_tree& avatar_perception::get_type_tree() const
{
    return _type_tree;
}

arity_t avatar_perception::arity() const
{
    return _arity;
}

type_tree avatar_perception::get_output_type_tree() const
{
    return _output_type;
}

const type_tree& avatar_perception::get_input_type_tree(arity_t i) const
{
    return argument_type_list_input_type(_arg_type_tree, _arity, i);
}

bool avatar_perception::is_ultrametric() const
{
    return _ultrametric;
}
bool avatar_perception::is_transitive() const
{
    return _transitive;
}
bool avatar_perception::is_irreflexive() const
{
    return _irreflexive;
}
bool avatar_perception::is_reflexive() const
{
    return _reflexive;
}
bool avatar_perception::is_symmetric() const
{
    return _symmetric;
}
bool avatar_perception::is_identity_of_indiscernibles() const
{
    return _identity_of_indiscernibles;
}

