/*
 * opencog/comboreduct/ant_combo_vocabulary/ant_indefinite_object.cc
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
#include "ant_indefinite_object.h"
#include <opencog/comboreduct/type_checker/type_tree.h>

namespace opencog { namespace combo {

using namespace ant_indefinite_object_properties;

ant_indefinite_object::ant_indefinite_object() { }

const indefinite_object_basic_description* ant_indefinite_object::get_basic_description_array() const {
  return iobd;
}

unsigned int ant_indefinite_object::get_basic_description_array_count() const {
  return sizeof(iobd)/sizeof(basic_description);
}

const ant_indefinite_object* ant_indefinite_object::init_indefinite_object() {
  static ant_indefinite_object* indefinite_objects =
    new ant_indefinite_object[id::ant_indefinite_object_count];
  for(unsigned int i = 0; i < id::ant_indefinite_object_count; ++i)
    indefinite_objects[i].set_indefinite_object((ant_indefinite_object_enum)i);
  return indefinite_objects;
}

void ant_indefinite_object::set_indefinite_object(ant_indefinite_object_enum pioe) {
  OC_ASSERT(pioe<id::ant_indefinite_object_count);
  _enum = pioe;
  //fill the various properties using the arrays edited by the developer
  set_basic_description(pioe);
}

indefinite_object ant_indefinite_object::get_instance(const std::string& name) {
  //look up for ant_indefinite_object_enum corresponding to that name
  bool found = false;
  indefinite_object as = NULL;
  for(unsigned int i = 0; i<id::ant_indefinite_object_count && !found; ++i) {
    as = ant_indefinite_object::get_instance((ant_indefinite_object_enum)i);
    found = as->get_name()==name;
  }
  return (found? as : NULL);
}

indefinite_object ant_indefinite_object::get_instance(ant_indefinite_object_enum pioe) {
  static const ant_indefinite_object* indefinite_objects=init_indefinite_object();
  OC_ASSERT(pioe<id::ant_indefinite_object_count);
  return static_cast<indefinite_object>(&indefinite_objects[pioe]);
}

const std::string& ant_indefinite_object::get_name() const {
  return _name;
}

const type_tree& ant_indefinite_object::get_type_tree() const {
  return _type_tree;
}

arity_t ant_indefinite_object::arity() const {
  return _arity;
}

type_tree ant_indefinite_object::get_output_type_tree() const {
  return _output_type;
}

const type_tree& ant_indefinite_object::get_input_type_tree(arity_t i) const {
  return argument_type_list_input_type(_arg_type_tree, _arity, i);
}

}} // ~namespaces combo opencog
