#include "ComboReduct/combo/definite_object.h"

#define ACTION_NAME_POSTFIX "_action"

namespace combo {

  bool is_action_definite_object(const definite_object& d) {
    return std::string::npos != d.rfind(ACTION_NAME_POSTFIX);
  }

  std::string get_action_name(const definite_object& d) {
    return d.substr(0, d.rfind(ACTION_NAME_POSTFIX));
  }

  definite_object get_action_definite_object(const std::string& action_name) {
    return definite_object(action_name + std::string(ACTION_NAME_POSTFIX));
  }

}
