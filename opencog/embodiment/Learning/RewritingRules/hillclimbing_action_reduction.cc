/*
 * opencog/embodiment/Learning/RewritingRules/hillclimbing_action_reduction.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#include <opencog/comboreduct/reduct/reduct.h>
#include "hillclimbing_action_reduction.h"
#include <opencog/comboreduct/reduct/meta_rules.h>
#include <opencog/comboreduct/reduct/general_rules.h>
#include <opencog/comboreduct/reduct/action_rules.h>

namespace opencog { namespace reduct {
const rule& hillclimbing_action_reduction()
{
    static iterative r;

    r =
        iterative(sequential(//general
                      downwards(level()),

                      //perception, that's because composite actions
                      //contain perceptions
                      downwards(reduce_not_cond_action_boolean_if()),

                      //action
                      downwards(reduce_action_action_if_always_succeeds()),
                      downwards(reduce_action_if()),
                      downwards(reduce_action_while_always_fails())
                  )
                 );

    return r;
}

} // ~namespace reduct
} // ~namespace opencog
