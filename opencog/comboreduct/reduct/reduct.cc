#include "comboreduct/reduct/reduct.h"

namespace reduct {
  reduct::rule* new_clone(const reduct::rule& r) { return r.clone(); }
} //~namespace reduct
