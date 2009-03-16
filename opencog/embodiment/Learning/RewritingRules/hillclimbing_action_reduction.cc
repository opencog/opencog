#include "comboreduct/reduct/reduct.h"
#include "hillclimbing_action_reduction.h"
#include "comboreduct/reduct/meta_rules.h"
#include "comboreduct/reduct/general_rules.h"
#include "comboreduct/reduct/action_rules.h"

namespace reduct {
  const rule& hillclimbing_action_reduction() {
    static iterative r;

    r =
      iterative(sequential(//general
			   downwards(level()),

			   //perception, that's because composite actions
			   //contain perceptions
			   downwards(reduce_not_cond_action_boolean_if()),

			   //action
			   downwards(reduce_action_action_if_always_succeeds()),
			   downwards(reduce_action_if()),
			   downwards(reduce_action_while_always_fails())
			   )
		);
    
    return r;
  }
}
