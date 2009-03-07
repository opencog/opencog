#include "comboreduct/reduct/reduct.h"
#include "comboreduct/reduct/meta_rules.h"
#include "comboreduct/reduct/general_rules.h"
#include "comboreduct/reduct/action_rules.h"

namespace reduct {

  const rule& action_reduction() {
    static iterative r;
    
    r =
      iterative(sequential(//general reduction
			   downwards(level()),
			   
			   //general action reduction
			   downwards(reduce_action_if()),
			   downwards(reduce_action_action_if()),
			   downwards(reduce_const_cond_action_if()),
			   downwards(reduce_const_action_seq()),
			   downwards(reduce_empty_arg_seq()),

			   downwards(reduce_repeat_out_action_while()),
			   downwards(reduce_repeat_in_action_while()),

			   //property action reduction
			   downwards(reduce_idempotent()),
			   downwards(reduce_opposite()),
			   downwards(reduce_additive()),
			   downwards(reduce_zero_neutral()),

                           // modular argument reduction
                           downwards(reduce_modular_argument()),
                           
                           // modular argument reduction
                           preconditions_check()

			   ));

    return r;
  }
}

