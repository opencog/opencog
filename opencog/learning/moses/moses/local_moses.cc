/*
 * opencog/learning/moses/moses/local_moses.cc
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

#include "local_moses.h"

namespace opencog {
namespace moses {

using namespace combo;

/**
 * expand_deme -- Run one deme-creation and optimization step.
 *
 * A single step consists of representation-building, to create
 * a deme, followed by optimization, (according to the specified
 * optimizer and scoring function), and finally, a merger of
 * the unique (and possibly non-dominated) trees back into the
 * metapopulation, for potential use as exemplars for future demes.
 *
 * @param max_evals    the max evals
 *
 * @return return true if expansion has succeeded, false otherwise
 *
 */
bool expand_deme(metapopulation& mp,
     int max_evals, moses_statistics& stats)
{
    if (mp.empty())
        return true;

    // Attempt to create a non-empty representation, by looping
    // over exemplars until we find one that expands.
    // XXX When would one never expand?  Wouldn't that be a bug?
    while (1) {
        bscored_combo_tree_ptr_set_cit exemplar = mp.select_exemplar();

        // Should have found something by now.
        if (exemplar == mp.end()) {
            logger().warn(
                "WARNING: All exemplars in the metapopulation have "
                "been visited, but it was impossible to build a "
                "representation for any of them.  Perhaps the reduct "
                "effort for knob building is too high.");
            return true;
        }

        // if create_deme returned true, we are good to go.
        if (mp._dex.create_deme(*exemplar)) break;

        OC_ASSERT(false, "Exemplar failed to expand!\n");
    }

    size_t evals_this_deme = mp._dex.optimize_deme(max_evals);
    stats.n_evals += evals_this_deme;
    stats.n_expansions++;

    bool done = mp.merge_deme(mp._dex._deme, mp._dex._rep, evals_this_deme);

    if (logger().isInfoEnabled()) {
        logger().info()
           << "Expansion " << stats.n_expansions
           << " total number of evaluations so far: " << stats.n_evals;
        mp.log_best_candidates();
    }

    mp._dex.free_deme();

    // Might be empty, if the eval fails and throws an exception
    return done || mp.empty();
}

/**
 * The main function of MOSES, non-distributed version.
 *
 * This is termed "local" because it runs only on the local CPU node.
 * That is, it does not assume nor make use of any distrirubted
 * processing capabilities.  It does assume that the local environment
 * might possibly be an SMP system (i.e. generic CPU's with more-or-less
 * tightly coupled memory).
 *
 * @param mp the metapopulation
 * @param pa the parameters to run moses
 */
void local_moses(metapopulation& mp,
                 const moses_parameters& pa,
                 moses_statistics& stats)
{
    logger().info("MOSES starts, max_evals=%d max_gens=%d",
                  pa.max_evals, pa.max_gens);

    optim_stats *os = dynamic_cast<optim_stats *> (&mp._dex._optimize);

    // Print legend for the columns of the stats.
    print_stats_header(os);

    while ((stats.n_evals < pa.max_evals)
           && (pa.max_gens != stats.n_expansions)
           && (mp.best_score() < pa.max_score))
    {
        // Run a generation
        bool done = expand_deme(mp, pa.max_evals - stats.n_evals, stats);

        // Print stats in a way that makes them easy to graph.
        // (columns of tab-seprated numbers)
        if (logger().isInfoEnabled()) {
            stringstream ss;
            ss << "Stats: " << stats.n_expansions;
            ss << "\t" << stats.n_evals;    // number of evaluations so far
            ss << "\t" << mp.size();       // size of the metapopulation
            ss << "\t" << mp.best_score(); // score of the highest-ranked exemplar.
            ss << "\t" << get_complexity(mp.best_composite_score()); // as above.
            if (os) {
                ss << "\t" << os->field_set_size;  // number of bits in the knobs
                ss << "\t" << os->nsteps;  // number of iterations of optimizer
                ss << "\t" << os->over_budget;  // exceeded max_evals
            }
            logger().info(ss.str());
        }

        // In iterative hillclimbing, it is possible (but not likely)
        // that the metapop gets empty and expand returns false.
        // Alternately, the candidate callback may urge a premature
        // termination.
        if (done) break;
    }

    logger().info("MOSES ends");
}

} // ~namespace moses
} // ~namespace opencog

