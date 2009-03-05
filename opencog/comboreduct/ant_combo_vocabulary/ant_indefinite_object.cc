#include "ComboReduct/ant_combo_vocabulary/ant_indefinite_object.h"
#include "ComboReduct/combo/type_tree.h"

using namespace combo;
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
  for(unsigned int i = 0; i < id::ant_indefinite_object_count; i++)
    indefinite_objects[i].set_indefinite_object((ant_indefinite_object_enum)i);
  return indefinite_objects;
}

void ant_indefinite_object::set_indefinite_object(ant_indefinite_object_enum pioe) {
  LADSUtil::cassert(TRACE_INFO, pioe<id::ant_indefinite_object_count);
  _enum = pioe;
  //fill the various properties using the arrays edited by the developer
  set_basic_description(pioe);
}

indefinite_object ant_indefinite_object::instance(const std::string& name) {
  //look up for pet_indefinite_object_enum corresponding to that name
  bool found = false;
  indefinite_object as = NULL;
  for(unsigned int i = 0; i<id::ant_indefinite_object_count && !found; i++) {
    as = ant_indefinite_object::instance((ant_indefinite_object_enum)i);
    found = as->get_name()==name;
  }
  return (found? as : NULL);
}

indefinite_object ant_indefinite_object::instance(ant_indefinite_object_enum pioe) {
  static const ant_indefinite_object* indefinite_objects=init_indefinite_object();
  LADSUtil::cassert(TRACE_INFO, pioe<id::ant_indefinite_object_count);
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
