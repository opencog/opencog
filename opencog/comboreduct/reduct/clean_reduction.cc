#include "comboreduct/reduct/reduct.h"
#include "comboreduct/reduct/meta_rules.h"
#include "comboreduct/reduct/general_rules.h"

namespace reduct {
  const rule& clean_reduction() {
    static downwards r=downwards(remove_null_vertices());
    return r;
  }
  /*const rule& clean_and_full_reduction() {
    static sequential r=sequential(clean_reduction(),full_reduction());
    return r;
    }*/
} //~namespace reduct
