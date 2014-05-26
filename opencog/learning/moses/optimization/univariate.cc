/*
 * opencog/learning/moses/optimization/univariate.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
 *            Nil Geisweiller
 *            Xiaohui Liu
 *            Linas Vepstas
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

#include <opencog/util/oc_assert.h>
#include <opencog/util/selection.h>

#include "../eda/termination.h"
#include "../eda/replacement.h"
#include "../eda/logging.h"
#include "../eda/local_structure.h"
#include "../eda/optimize.h"
#include "../moses/neighborhood_sampling.h"

#include "univariate.h"

namespace opencog { namespace moses {

/////////////////////////////
// Univariate Optimization //
/////////////////////////////

//return # of evaluations actually performed
unsigned univariate_optimization::operator()(deme_t& deme,
                    const iscorer_base& iscorer,
                    unsigned max_evals, time_t max_time)
{
    unsigned pop_size = opt_params.pop_size(deme.fields());
    unsigned max_gens_total = information_theoretic_bits(deme.fields());
    unsigned max_gens_improv = opt_params.max_gens_improv(deme.fields());
    unsigned n_select = double(pop_size) * eda_params.selection_ratio;
    unsigned n_generate = double(pop_size) * eda_params.replacement_ratio;

    // Adjust parameters based on the maximal # of evaluations allowed
    if (max_evals < pop_size) {
        pop_size = max_evals;
        max_gens_total = 0;
    } else {
        max_gens_total = std::min(max_gens_total,
                                  (max_evals - pop_size) / n_generate);
    }

    // Create the initial sample
    // Generate the initial sample to populate the deme
    deme.resize(pop_size);
    generate_initial_sample(deme.fields(), pop_size, deme.begin(), deme.end());

    if (eda_params.is_tournament_selection()) {
        cout_log_best_and_gen logger;
        return optimize
               (deme, n_select, n_generate, max_gens_total, iscorer,
                terminate_if_gte_or_no_improv<composite_score>
                (composite_score(opt_params.terminate_if_gte,
                                  worst_composite_score.get_complexity(),
                                  0),
                 max_gens_improv),
                tournament_selection((unsigned)eda_params.selection),
                univariate(), local_structure_probs_learning(),
                // rtr_replacement(deme.fields(),
                //                      opt_params.rtr_window_size(deme.fields())),
                replace_the_worst(),
                logger);
    } else { //truncation selection
        OC_ASSERT(false,
                  "Trunction selection not implemented."
                  " Tournament should be used instead.");
        return 42;
        /*
        return optimize(deme,n_select,n_generate,args.max_gens,score,
          terminate_if_gte_or_no_improv(opt_params.terminate_if_gte,
                   max_gens_improv),
          //truncation selection goes here
          univariate(),local_structure_probs_learning(),
          replace_the_worst(),cout_log_best_and_gen());
        */
    }
}

} // ~namespace moses
} // ~namespace opencog

