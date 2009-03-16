#include "comboreduct/reduct/reduct.h"
#include "hillclimbing_perception_reduction.h"
#include "comboreduct/reduct/meta_rules.h"
#include "comboreduct/reduct/general_rules.h"
#include "comboreduct/reduct/perception_rules.h"

namespace reduct {

  const rule& hillclimbing_perception_reduction() {
    static iterative r;
    
    r = iterative(sequential(//perception
			     downwards(reduce_reflexive()),
			     downwards(reduce_irreflexive()),
			     upwards(reorder_commutative())
			     ));
    
    return r;
  }
}

