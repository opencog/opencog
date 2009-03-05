#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/reduct/meta_rules.h"
#include "ComboReduct/reduct/general_rules.h"
#include "ComboReduct/reduct/logical_rules.h"

namespace reduct {
  const rule& logical_reduction() {
    using namespace combo;
    static sequential r=
      sequential(downwards(reduce_nots(),id::boolean_type),
		 upwards(remove_dangling_junctors()),
		 iterative(sequential(upwards(eval_logical_identities()),
				      downwards(level()),
				      downwards(insert_ands(),id::boolean_type),
				      subtree_to_enf(),
				      downwards(reduce_ands(),id::boolean_type),
				      downwards(reduce_ors(),
						id::boolean_type))),
		 downwards(remove_unary_junctors(),id::boolean_type));
    return r;
  }
} //~namespace reduct
