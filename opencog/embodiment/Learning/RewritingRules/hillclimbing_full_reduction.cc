#include "comboreduct/reduct/reduct.h"
#include "hillclimbing_full_reduction.h"
#include "comboreduct/reduct/meta_rules.h"
#include "comboreduct/reduct/general_rules.h"
#include "comboreduct/reduct/action_rules.h"
#include "comboreduct/reduct/perception_rules.h"
#include "comboreduct/reduct/logical_rules.h"

namespace reduct {

  const rule& hillclimbing_full_reduction() {
    static iterative r;
    
    r =
      iterative(sequential(//general
			   downwards(level()),
			   //action
			   downwards(reduce_action_if()),
			   downwards(reduce_action_boolean_if_sub_cond()),
			   downwards(reduce_boolean_while_sub_cond()),
			   downwards(reduce_repeat_out_action_while()),
			   downwards(reduce_repeat_in_action_while()),
			   //action based on properties
			   downwards(reduce_idempotent()),
			   downwards(reduce_opposite()),
			   downwards(reduce_additive()),
			   downwards(reduce_zero_neutral()),
			   //action based on get_action_result
			   downwards(reduce_action_action_if_always_succeeds()),
			   downwards(reduce_action_while_always_fails()),
			   downwards(reduce_sequential_and_always_fails()),
			   //perception
			   downwards(reduce_nots()),
			   downwards(reduce_not_cond_action_boolean_if())
			   ));

    return r;
  }
}

