#ifndef _COMBO_ACTION_SYMBOL_H
#define _COMBO_ACTION_SYMBOL_H

#include "comboreduct/combo/operator_base.h"
#include "comboreduct/combo/type_tree_def.h"

namespace combo {

  //action_symbol_base inherit operator_base
  //without additional properties
  class action_symbol_base : public operator_base {
  public:
    virtual ~action_symbol_base() {}
  };

  typedef const action_symbol_base* action_symbol;
  
}//~namespace combo


std::ostream& operator<<(std::ostream&,combo::action_symbol);


#endif
