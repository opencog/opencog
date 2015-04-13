/** moses_main.cc ---
 *
 * Copyright (C) 2012 Pouling Holdings
 *
 * Author: Linas Vepstas <linasvepstas@gmailcom>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/util/Logger.h>

#include "../metapopulation/metapopulation.h"
#include "distributed_moses.h"
#include "local_moses.h"
#include "mpi_moses.h"

#include "moses_main.h"

namespace opencog { namespace moses {

#define strform(x) #x
#define stringify(x) strform(x)

#ifdef MOSES_GIT_DESCRIBE
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH) " (git-describe "
    stringify(MOSES_GIT_DESCRIBE) ")";

#else
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH);

#endif

void run_moses(metapopulation& metapop,
               deme_expander& dex,
               const moses_parameters& moses_params,
               moses_statistics& stats)
{
    // Run moses, either on localhost, or distributed.
    if (moses_params.local)
        local_moses(metapop, dex, moses_params, stats);
    else if (moses_params.mpi)
        mpi_moses(metapop, dex, moses_params, stats);
    else
        distributed_moses(metapop, dex, moses_params, stats);
}

/**
 * This twiddles the trmination criteria for deciding when to finish
 * the search. Not sure why this code is here, instead of being earlier,
 * and getting set up during the option sepcification stage ... 
 */
void adjust_termination_criteria(const behave_cscore& c_scorer,
                                 optim_parameters& opt_params,
                                 moses_parameters& moses_params)
{
    // Update terminate_if_gte and max_score criteria. An explicit
    // user-specified max score always over-rides the inferred score.
    score_t target_score = c_scorer.best_possible_score();
    if (very_best_score != moses_params.max_score) {
        target_score = moses_params.max_score;
        logger().info("Target score = %g", target_score);
    } else {
        logger().info("Inferred target score = %g", target_score);
    }

    // negative min_improv is interpreted as percentage of
    // improvement, if so then don't substract anything, since in that
    // scenario the absolute min improvent can be arbitrarily small
    score_t actual_min_improv = std::max(c_scorer.min_improv(), (score_t)0);
    target_score -= actual_min_improv;
    logger().info("Subtract %g (minimum significant improvement) "
                  "from the target score to deal with float imprecision = %g",
                  actual_min_improv, target_score);

    opt_params.terminate_if_gte = target_score;
    moses_params.max_score = target_score;

    // update minimum score improvement
    opt_params.set_min_score_improv(c_scorer.min_improv());
}

} // ~namespace moses
} // ~namespace opencog

