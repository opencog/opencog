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

namespace reduct {
  const rule& contin_reduction(opencog::RandGen& rng) {
    static iterative r=
      iterative(sequential(downwards(level()),
			   upwards(eval_constants(rng)),
			       
			   downwards(reduce_plus_times_one_child()),
			   downwards(reduce_plus_zero()),
			   downwards(reduce_times_one_zero()),
			   downwards(reduce_sin()),
			   downwards(reduce_invert_constant()),

			   downwards(reduce_log_div_times()),
			   downwards(reduce_exp_times()),
			   downwards(reduce_exp_div()),
			   downwards(reduce_exp_log()),
			   downwards(reduce_times_div()),
			   downwards(reduce_sum_log()),
			   
			   upwards(reorder_commutative()),
			   downwards(reduce_factorize()),
			   downwards(reduce_factorize_fraction()),
			   downwards(reduce_fraction())));
    return r;
  }
} //~namespace reduct
