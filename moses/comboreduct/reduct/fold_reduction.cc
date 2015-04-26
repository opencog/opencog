/*
 * moses/comboreduct/reduct/fold_reduction.cc
 */

#include "reduct.h"
#include "meta_rules.h"
#include "general_rules.h"
#include "fold_rules.h"
#include <mutex>

namespace moses3 { namespace reduct {

const rule& fold_reduction()
{
    // A note about the locking below, and the *pr pointer.  It can (and
    // does!) happen that two different threads may enter this routine
    // simltaneously.  Because c++ will defer running static initializers
    // until they are needed, then, if we did not lock below, then both
    // threads will start running the static initializers (constructors). 
    // The faster thread would have returned a rule, while the slower 
    // thread clobbered it, causing destructors to run on that rule.
    // As a result, the faster thread was found to be accessing freed
    // memory!  Ouch.  So a lock is needed.  To avoid locking *every*
    // time, the 'static rule *pr' is used to avoid locking if the
    // initializers have run at least once.
    static rule *pr = NULL;
    if (pr != NULL) return *pr;

    static std::mutex m;
    std::lock_guard<std::mutex> static_ctor_lock(m);
    
    static iterative r = iterative(sequential(

                                   downwards(fold_unrolling())
                             ));
    if (pr == NULL) pr = &r;
    return *pr;
}

} // ~namespace reduct
} // ~namespace moses3
