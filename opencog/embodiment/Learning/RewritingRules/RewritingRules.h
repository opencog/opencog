#ifndef _REWRITING_RULES_H
#define _REWRITING_RULES_H

#include <comboreduct/reduct/reduct.h>
#include "hillclimbing_action_reduction.h"
#include "hillclimbing_perception_reduction.h"
#include "hillclimbing_full_reduction.h"
#include "post_learning_rewriting.h"

namespace reduct {
  
  //hillclimbing
  inline void hillclimbing_full_reduce(combo_tree& tr, combo_tree::iterator it) {
    hillclimbing_full_reduction()(tr,it);
  }

  inline void hillclimbing_full_reduce(combo_tree& tr) {
    hillclimbing_full_reduction()(tr);
  }

  inline void hillclimbing_perception_reduce(combo_tree& tr, combo_tree::iterator it) {
    hillclimbing_perception_reduction()(tr,it);
  }

  inline void hillclimbing_perception_reduce(combo_tree& tr) {
    hillclimbing_perception_reduction()(tr);
  }

  inline void hillclimbing_action_reduce(combo_tree& tr, combo_tree::iterator it) {
    hillclimbing_action_reduction()(tr,it);
  }

  inline void hillclimbing_action_reduce(combo_tree& tr) {
    hillclimbing_action_reduction()(tr);
  }

  //post_learning
  inline void post_learning_rewrite(combo_tree& tr, combo_tree::iterator it) {
    post_learning_rewriting()(tr,it);
  }

  inline void post_learning_rewrite(combo_tree& tr) {
    post_learning_rewriting()(tr);
  }


}//~namespace reduct

#endif
