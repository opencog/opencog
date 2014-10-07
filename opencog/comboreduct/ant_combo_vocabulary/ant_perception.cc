/*
 * opencog/comboreduct/ant_combo_vocabulary/ant_perception.cc
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
#include "ant_perception.h"
#include <opencog/comboreduct/type_checker/type_tree.h>

namespace opencog { namespace combo {

using namespace ant_perception_properties;

ant_perception::ant_perception() {
    _ultrametric = false;
    _transitive = false;
    _irreflexive = false;
    _reflexive = false;
    _symmetric = false;
    _identity_of_indiscernibles = false;
}

const perception_basic_description* ant_perception::get_basic_description_array() const {
    return ant_perception_properties::pbd;
}

unsigned int ant_perception::get_basic_description_array_count() const {
    return sizeof(ant_perception_properties::pbd)/sizeof(basic_description);
}

const ant_perception* ant_perception::init_perceptions() {
    static ant_perception* perceptions =
        new ant_perception[id::ant_perception_count];
    for(unsigned int i = 0; i < id::ant_perception_count; ++i)
        perceptions[i].set_perception((ant_perception_enum)i);
    return perceptions;
}

void ant_perception::set_perception(ant_perception_enum ape) {
    OC_ASSERT(ape<id::ant_perception_count);
    _enum = ape;
    //fill the various properties using the arrays edited by the developer

    //basic properties
    set_basic_description(ape);
    //standard properties specific to action
    unsigned int ppd_count = sizeof(ppd)/sizeof(perception_property_description);
    OC_ASSERT(
              ppd_count==(unsigned int)id::ant_perception_count,
              "there must be entries for all perceptions.");
    bool found = false;
    for(unsigned int i = 0; i < ppd_count && !found; ++i) {
        if(ppd[i].perception==ape) {
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
              "ant_perception with enum %d has not been found in ppd", ape);
}

const ant_perception* ant_perception::get_instance(const std::string& name) {
    //look up for ant_builtin_action_enum corresponding to that name
    bool found = false;
    const ant_perception* p = NULL;
    for(unsigned int i = 0; i<id::ant_perception_count && !found; ++i) {
        p = ant_perception::get_instance((ant_perception_enum)i);
        found = p->get_name()==name;
    }
    return (found? p : NULL);
}

const ant_perception* ant_perception::get_instance(ant_perception_enum ape) {
    static const ant_perception* perceptions=ant_perception::init_perceptions();
    OC_ASSERT(ape<id::ant_perception_count);
    return &perceptions[ape];
}

const std::string& ant_perception::get_name() const {
    return _name;
}

const type_tree& ant_perception::get_type_tree() const {
    return _type_tree;
}

arity_t ant_perception::arity() const {
    return _arity;
}

type_tree ant_perception::get_output_type_tree() const {
    return _output_type;
}

const type_tree& ant_perception::get_input_type_tree(arity_t i) const {
    return argument_type_list_input_type(_arg_type_tree, _arity, i);
}

bool ant_perception::is_ultrametric() const {
    return _ultrametric;
}
bool ant_perception::is_transitive() const {
    return _transitive;
}
bool ant_perception::is_irreflexive() const {
    return _irreflexive;
}
bool ant_perception::is_reflexive() const {
    return _reflexive;
}
bool ant_perception::is_symmetric() const {
    return _symmetric;
}
bool ant_perception::is_identity_of_indiscernibles() const {
    return _identity_of_indiscernibles;
}

}} // ~namespaces combo opencog
