#include "comboreduct/reduct/reduct.h"
#include "comboreduct/reduct/meta_rules.h"

#include "post_learning_rewriting.h"
#include "post_learning_rules.h"

namespace reduct {

  const rule& post_learning_rewriting() {
    static sequential r = 
      sequential(downwards(post_learning_drop_before_grab()));
    
    return r;
  }
}
