#ifndef _COMBO_INDEFINITE_OBJECT_H
#define _COMBO_INDEFINITE_OBJECT_H

#include "util/exceptions.h"

#include "comboreduct/combo/type_tree_def.h"
#include "comboreduct/combo/operator_base.h"

namespace combo {
  
  //indefinite_object inherits from operator_base
  //without additional properties
  class indefinite_object_base : public operator_base {
  public:
    virtual ~indefinite_object_base() {}
  };

  typedef const indefinite_object_base* indefinite_object;

  typedef std::set<indefinite_object> indefinite_object_set;
  typedef indefinite_object_set::iterator indefinite_object_set_it;
  typedef indefinite_object_set::const_iterator indefinite_object_set_const_it;

}//~namespace combo

std::ostream& operator<<(std::ostream&, combo::indefinite_object);

#endif

