/*
 * moses/comboreduct/reduct/perception_reduction.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://moses.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <mutex>
#include "reduct.h"
#include "meta_rules.h"
#include "general_rules.h"
#include "logical_rules.h"
#include "mixed_rules.h"
#include "perception_rules.h"

namespace moses { namespace reduct {

const rule& perception_reduction()
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

    static assum_iterative r =
       assum_iterative(sequential(downwards(level()),
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

    if (pr == NULL) pr = &r;
    return *pr;
}

} // ~namespace reduct
} // ~namespace moses

