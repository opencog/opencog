/*
 * opencog/comboreduct/reduct/perception_reduction.cc
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
#include "logical_rules.h"
#include "mixed_rules.h"
#include "perception_rules.h"

namespace opencog { namespace reduct {

const rule& perception_reduction() {
    static assum_iterative r;

    r = assum_iterative(sequential(downwards(level()),
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

    return r;
}

} // ~namespace reduct
} // ~namespace opencog

