#ifndef _COMBO_OPERATOR_BASE_H
#define _COMBO_OPERATOR_BASE_H

#include "ComboReduct/combo/common_def.h"
#include "ComboReduct/combo/type_tree_def.h"

namespace combo {

  //that abstract class contains common methods
  //of builtin_action_base and perception_base
  class operator_base {
  public:
    virtual ~operator_base() {}

    //get_name
    virtual const std::string& get_name() const = 0;

    //type_tree
    virtual const type_tree& get_type_tree() const = 0;

    //helper methods for fast access type properties
    //number of arguments that takes the operator
    virtual arity_t arity() const = 0;
    //return the type node of the operator
    virtual type_tree get_output_type_tree() const = 0;
    //return the type tree of the input argument of index i
    virtual const type_tree& get_input_type_tree(arity_t i) const = 0;
  };

}

#endif
