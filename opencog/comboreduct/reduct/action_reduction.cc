/*
 * opencog/comboreduct/reduct/action_reduction.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
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
#include "action_rules.h"

namespace opencog { namespace reduct {

const rule& action_reduction()
{
    // A note about the locking below, and the *pr pointer.  It can (and
    // has!) happened that two different threads may enter this routine
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

    static iterative r =
        iterative(sequential(//general reduction
                             downwards(level()),
	   
                             //general action reduction
                             downwards(reduce_action_if()),
                             downwards(reduce_action_action_if()),
                             downwards(reduce_const_cond_action_if()),
                             downwards(reduce_const_action_seq()),
                             downwards(reduce_empty_arg_seq()),

                             downwards(reduce_repeat_out_action_while()),
                             downwards(reduce_repeat_in_action_while()),

                             //property action reduction
                             downwards(reduce_idempotent()),
                             downwards(reduce_opposite()),
                             downwards(reduce_additive()),
                             downwards(reduce_zero_neutral()),

                             // modular argument reduction
                             downwards(reduce_modular_argument()),
                   
                             // modular argument reduction
                             preconditions_check()

                             ));

    if (pr == NULL) pr = &r;
    return *pr;
}

} // ~namespace reduct
} // ~namespace opencog 

