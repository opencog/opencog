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
#include "reduct.h"
#include "meta_rules.h"
#include "general_rules.h"
#include "logical_rules.h"
#include "mixed_rules.h"

namespace opencog { namespace reduct {


logical_reduction::logical_reduction(void)
{
    medium = NULL;
    complexe = NULL;
}

logical_reduction::~logical_reduction()
{
	if (medium) delete medium;
	if (complexe) delete complexe;
}

logical_reduction::logical_reduction(const vertex_set& ignore_ops,
                                     opencog::RandGen& rng)
{
    using namespace opencog::combo;

    // medium
    static sequential pre_subtree_to_enf = 
        sequential(upwards(eval_logical_identities()),
                   downwards(level()),
                   downwards(insert_ands(), id::boolean_type));

    static sequential post_subtree_to_enf =
        sequential(downwards(reduce_ands(), id::boolean_type),
                   downwards(reduce_ors(), id::boolean_type));

    // Can't be static, due to ignore_ops and rng arguments
    medium = new
        sequential(downwards(simplify_predicates(ignore_ops, rng), id::boolean_type),
                   downwards(reduce_nots(), id::boolean_type),
                   
                   iterative(sequential(pre_subtree_to_enf,
                                        subtree_to_enf(),
                                        post_subtree_to_enf)),
                   downwards(remove_unary_junctors(), id::boolean_type),
                   "medium");

    // complexe
    complexe = new
        iterative(sequential(*medium,
                             reduce_remove_subtree_equal_tt()),
                  "complexe");

}

// effort 0 (extra simple) to 3 (complex)
const rule& logical_reduction::operator()(int effort)
{
    using namespace opencog::combo;

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

    switch (effort) {
    case 0: return extra_simple;
    case 1: return simple;
    default: break;
    }

    // The higher reduction efforts require the use of a different
    // constructor.
    OC_ASSERT(medium, "Error: logical reduction effort 2 and greater "
                      "requires ignore_ops and rng");

    switch (effort) {
    case 2: return *medium;
    case 3: return *complexe;
    default:
        OC_ASSERT(0, "Error: logical reduction: no such effort");
        exit(1); // function does not return.
    }
}

} // ~namespace reduct
} // ~namespace opencog
