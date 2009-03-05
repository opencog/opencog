#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/reduct/meta_rules.h"
#include "ComboReduct/reduct/general_rules.h"
#include "ComboReduct/reduct/contin_rules.h"

namespace reduct {
  const rule& contin_reduction(LADSUtil::RandGen& rng) {
    static iterative r=
      iterative(sequential(downwards(level()),
			   upwards(eval_constants(rng)),
			       
			   downwards(reduce_plus_times_one_child()),
			   downwards(reduce_plus_zero()),
			   downwards(reduce_times_one_zero()),
			   downwards(reduce_sin()),
			   downwards(reduce_invert_constant()),

			   downwards(reduce_log_div_times()),
			   downwards(reduce_exp_times()),
			   downwards(reduce_exp_div()),
			   downwards(reduce_exp_log()),
			   downwards(reduce_times_div()),
			   downwards(reduce_sum_log()),
			   
			   upwards(reorder_commutative()),
			   downwards(reduce_factorize()),
			   downwards(reduce_factorize_fraction()),
			   downwards(reduce_fraction())));
    return r;
  }
} //~namespace reduct
