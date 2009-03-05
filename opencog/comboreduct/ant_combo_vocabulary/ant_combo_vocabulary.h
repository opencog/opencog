#ifndef _ANT_COMBO_VOCABULARY_H
#define _ANT_COMBO_VOCABULARY_H

#include "ComboReduct/ant_combo_vocabulary/ant_builtin_action.h"
#include "ComboReduct/ant_combo_vocabulary/ant_perception.h"
#include "ComboReduct/ant_combo_vocabulary/ant_action_symbol.h"
#include "ComboReduct/ant_combo_vocabulary/ant_indefinite_object.h"
#include "ComboReduct/combo/vertex.h"

//this namespace is defined in order to break ambiguities
//and make coexist several vocabularies
namespace ant_combo {

  using namespace combo;

  //return a pointer to class base instance
  //of a given action or perception enum
  builtin_action instance(ant_builtin_action_enum);
  perception instance(ant_perception_enum);
  action_symbol instance(ant_action_symbol_enum);
  indefinite_object instance(ant_indefinite_object_enum);

  //get the enum corresponding to a builtin_action, perception or action_symbol
  ant_builtin_action_enum get_enum(builtin_action);
  ant_perception_enum get_enum(perception);
  ant_action_symbol_enum get_enum(action_symbol);
  ant_indefinite_object_enum get_enum(indefinite_object);

  std::istream& operator>>(std::istream& in, combo::vertex& v);
  std::istream& operator>>(std::istream& in, combo::combo_tree& tr);
    
}//~namespace ant_combo

//this is added to be sure that the operators == and != between
//combo operator type and enum are used
//if not then it would easily lead to buggy situations because enum are cast to
//pointer
bool operator==(combo::builtin_action, ant_combo::ant_builtin_action_enum);
bool operator==(ant_combo::ant_builtin_action_enum, combo::builtin_action);
bool operator!=(combo::builtin_action, ant_combo::ant_builtin_action_enum);
bool operator!=(ant_combo::ant_builtin_action_enum, combo::builtin_action);
bool operator==(combo::perception, ant_combo::ant_perception_enum);
bool operator==(ant_combo::ant_perception_enum, combo::perception);
bool operator!=(combo::perception, ant_combo::ant_perception_enum);
bool operator!=(ant_combo::ant_perception_enum, combo::perception);
bool operator==(combo::action_symbol, ant_combo::ant_action_symbol_enum);
bool operator==(ant_combo::ant_action_symbol_enum, combo::action_symbol);
bool operator!=(combo::action_symbol, ant_combo::ant_action_symbol_enum);
bool operator!=(ant_combo::ant_action_symbol_enum, combo::action_symbol);
bool operator==(combo::indefinite_object, ant_combo::ant_indefinite_object_enum);
bool operator==(ant_combo::ant_indefinite_object_enum, combo::indefinite_object);
bool operator!=(combo::indefinite_object, ant_combo::ant_indefinite_object_enum);
bool operator!=(ant_combo::ant_indefinite_object_enum, combo::indefinite_object);


#endif

