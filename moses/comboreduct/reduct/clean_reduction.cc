/*
 * moses/comboreduct/reduct/clean_reduction.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#include "logical_rules.h"

namespace moses3 { namespace reduct {

const rule& clean_reduction()
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

    static sequential r= sequential(downwards(remove_null_vertices()),
                                    upwards(remove_dangling_junctors()));
    if (pr == NULL) pr = &r;
    return *pr;
}

} // ~namespace reduct
} // ~namespace moses3
