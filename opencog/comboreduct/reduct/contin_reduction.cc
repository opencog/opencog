/*
 * opencog/comboreduct/reduct/contin_reduction.cc
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
#include "reduct.h"
#include "meta_rules.h"
#include "general_rules.h"
#include "contin_rules.h"

// Note: the rule names are useful when debugging the reduct engine
// (uncomment META_RULE_DEBUG in meta_rules.cc)

namespace opencog { namespace reduct {
const rule& contin_reduction(const opencog::combo::vertex_set& ignore_ops, 
                             opencog::RandGen& rng) {
    // rules that do not involve factorizing or distributing
    static sequential seq_without_factorize_distribute =
        sequential(// these 2 ones below are added first because the
                   // representation building puts lots of 0*#n
                   downwards(reduce_times_one_zero()), 
                   downwards(reduce_plus_zero()),

                   downwards(level()),
                   upwards(eval_constants(rng)),
                           
                   downwards(reduce_plus_times_one_child()),
                   downwards(reduce_plus_zero()),
                   downwards(reduce_times_one_zero()),
                   when(downwards(reduce_sin()),
                        ignore_ops.find(id::sin) == ignore_ops.end()),
                   when(downwards(reduce_invert_constant()),
                        ignore_ops.find(id::div) == ignore_ops.end()),
                   when(downwards(reduce_log_div_times()),
                        ignore_ops.find(id::log) == ignore_ops.end()),
                   when(downwards(reduce_exp_times()),
                        ignore_ops.find(id::exp) == ignore_ops.end()),
                   when(downwards(reduce_exp_div()),
                        ignore_ops.find(id::exp) == ignore_ops.end()
                        && ignore_ops.find(id::div) == ignore_ops.end()),
// the following rules is not valid if log has the semantics log(abs(x))
// the macro ABS_LOG is defined in file vertex.h
#ifndef ABS_LOG
                   when(downwards(reduce_exp_log()),
                        ignore_ops.find(id::sin) == ignore_ops.end()
                        && ignore_ops.find(id::log) == ignore_ops.end()),
#endif
                   when(downwards(reduce_times_div()), 
                        ignore_ops.find(id::div) == ignore_ops.end()),
                   when(downwards(reduce_sum_log()),
                        ignore_ops.find(id::log) == ignore_ops.end()),
                   
                   upwards(reorder_commutative()),
                   when(downwards(reduce_fraction()),
                        ignore_ops.find(id::div) == ignore_ops.end()),
                   "seq_without_factorize_distribute"
                   );

    static iterative iter_without_factorize_distribute =
        iterative(seq_without_factorize_distribute,
                  "iter_without_factorize_distribute");

    static sequential complete_factorize = 
        sequential(downwards(reduce_factorize()),
                   when(downwards(reduce_factorize_fraction()),
                        ignore_ops.find(id::div) == ignore_ops.end()),
                   seq_without_factorize_distribute,
                   "complete_factorize");
    
    static downwards complete_distribute =
        downwards(reduce_distribute(iter_without_factorize_distribute));

    static iterative res =
        iterative(sequential(seq_without_factorize_distribute,
                             ignore_size_increase(complete_factorize),
                             // ignore_size_increase(sequential(complete_factorize,
                             //                                 complete_distribute)),
                             ignore_size_increase(sequential(complete_distribute,
                                                             complete_factorize))));
    return res;
}

} // ~namespace reduct
} // ~namespace opencog
