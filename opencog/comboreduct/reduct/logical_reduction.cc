/*
 * opencog/comboreduct/reduct/logical_reduction.cc
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
#include "mixed_rules.h"

namespace opencog { namespace reduct {

rule* logical_reduction::p_extra_simple = NULL;
rule* logical_reduction::p_simple = NULL;

void logical_reduction::do_init(void)
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
    if (p_extra_simple != NULL) return;

    static std::mutex m;
    std::lock_guard<std::mutex> static_ctor_lock(m);


    // extra_simple
    static downwards extra_simple = downwards(reduce_nots(), id::boolean_type,
                                              "extra_simple");

    // simple
    static sequential simple = 
        sequential(downwards(reduce_nots(), id::boolean_type),
                   iterative(sequential(upwards(eval_logical_identities()),
                                        downwards(level()),
                                        downwards(reduce_ands(), id::boolean_type),
                                        downwards(reduce_ors(), id::boolean_type))
                             ),
                   "simple");

    p_extra_simple = &extra_simple;
    p_simple = &simple;
}

logical_reduction::logical_reduction(void)
{
    do_init();
    p_medium = NULL;
    p_complexe = NULL;
}

logical_reduction::~logical_reduction()
{
    if (p_medium) delete p_medium;
    if (p_complexe) delete p_complexe;
    p_medium = NULL;
    p_complexe = NULL;
}

logical_reduction::logical_reduction(const logical_reduction& rhs) :
    p_medium(rhs.p_medium->clone()), p_complexe(rhs.p_complexe->clone())
{}

logical_reduction& logical_reduction::operator=(const logical_reduction& rhs)
{
    p_medium = rhs.p_medium->clone();
    p_complexe = rhs.p_complexe->clone();
    return *this;
}

logical_reduction::logical_reduction(const vertex_set& ignore_ops)
{
    using namespace opencog::combo;

    do_init();

    // medium
    sequential pre_subtree_to_enf = 
        sequential(upwards(eval_logical_identities()),
                   downwards(level()),
                   downwards(insert_ands(), id::boolean_type));

    sequential post_subtree_to_enf =
        sequential(downwards(reduce_ands(), id::boolean_type),
                   downwards(reduce_ors(), id::boolean_type));

    // Arghh .. XXX should use reduct_effort==3 for the complexe rule.
    int reduct_effort = 2;

    // Can't be static, due to ignore_ops argument
    p_medium = new sequential(
        downwards(simplify_predicates(reduct_effort, ignore_ops), id::boolean_type),
        downwards(reduce_nots(), id::boolean_type),
                   
        iterative(sequential(pre_subtree_to_enf,
                             subtree_to_enf(),
                             post_subtree_to_enf)),
        downwards(remove_unary_junctors(), id::boolean_type),
        "medium");

    // complexe
    p_complexe = new
        iterative(sequential(*p_medium,
                             reduce_remove_subtree_equal_tt()),
                  "complexe");

}

// effort 0 (extra simple) to 3 (complex)
const rule& logical_reduction::operator()(int effort)
{
    using namespace opencog::combo;

    switch (effort) {
    case 0: return *p_extra_simple;
    case 1: return *p_simple;
    default: break;
    }

    // The higher reduction efforts require the use of a different
    // constructor, the ctor that had initialized medium.
    OC_ASSERT(p_medium, "Error: logical reduction effort 2 and greater "
                      "requires ignore_ops");

    switch (effort) {
    case 2: return *p_medium;
    case 3: return *p_complexe;
    default:
        OC_ASSERT(0, "Error: logical reduction: no such effort");
        exit(1); // function does not return.
    }

}

} // ~namespace reduct
} // ~namespace opencog
