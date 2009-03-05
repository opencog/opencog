#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/reduct/meta_rules.h"
#include "ComboReduct/reduct/general_rules.h"
#include "ComboReduct/reduct/logical_rules.h"
#include "ComboReduct/reduct/mixed_rules.h"
#include "ComboReduct/reduct/perception_rules.h"

namespace reduct {

  const rule& perception_reduction() {
    static assum_iterative r;
    
    r = assum_iterative(sequential(downwards(level()),
				   //simple perception rules
				   downwards(reduce_irreflexive()),
				   downwards(reduce_reflexive()),
				   downwards(reduce_identity_of_indiscernibles()),

				   //generate assumptions
				   downwards(reduce_and_assumptions(r)),
				   //the following is commented because due to
				   //the fact that there is no boolean rules here
				   //the double negation cannot be reduced
				   //and it leads to an infinit recursion
				   //downwards(reduce_or_assumptions(r))
				   downwards(reduce_ultrametric()),
				   downwards(reduce_transitive()),
				   downwards(reduce_symmetric()),
				   //reduce from assumptyions
				   downwards(reduce_from_assumptions(r)),
				   downwards(reduce_inequality_from_assumptions())
				   )
			);
    
    return r;
  }
}

