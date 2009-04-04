#ifndef _COMBO_DEFINITE_OBJECT_H
#define _COMBO_DEFINITE_OBJECT_H

#include <string>
#include <set>

namespace combo {

  typedef std::string definite_object;

  typedef std::set<definite_object> definite_object_set;
  typedef definite_object_set::iterator definite_object_set_it;
  typedef definite_object_set::const_iterator definite_object_set_const_it;

  //a definite object with suffix _action is a special definite_object
  //used to reflect a self or other agent action
  //the following methods allows to check wether a definite_object
  //is of action type and get the action name (without the suffix "_action")
  bool is_action_definite_object(const definite_object& d);
  std::string get_action_name(const definite_object& d);
  definite_object get_action_definite_object(const std::string& action_name);
}

#endif
