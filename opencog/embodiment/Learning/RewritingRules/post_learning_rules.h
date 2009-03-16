#ifndef _POST_LEARNING_RULES_H
#define _POST_LEARNING_RULES_H

#include "comboreduct/reduct/reduct.h"

namespace reduct {

  //add a drop action in front of a grab action
  struct post_learning_drop_before_grab 
    : public crule<post_learning_drop_before_grab> {
    void operator()(combo_tree& tr, combo_tree::iterator it) const;
  };
}//~namespace reduct

#endif
