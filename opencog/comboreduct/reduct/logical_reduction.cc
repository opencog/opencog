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

namespace reduct {

const rule& logical_reduction() {
    using namespace combo;
    static sequential r=
        sequential(downwards(reduce_nots(),id::boolean_type),
                   upwards(remove_dangling_junctors()),
                   iterative(sequential(upwards(eval_logical_identities()),
                                        downwards(level()),
                                        downwards(insert_ands(),
                                                  id::boolean_type),
                                        subtree_to_enf(),
                                        downwards(reduce_ands(),
                                                  id::boolean_type),
                                        downwards(reduce_ors(),
                                                  id::boolean_type))),
                   downwards(remove_unary_junctors(),id::boolean_type));
    return r;
}

} //~namespace reduct
