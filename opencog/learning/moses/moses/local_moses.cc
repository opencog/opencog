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
static bool expand_deme(metapopulation& mp,
                 deme_expander& dex,
                 int max_evals, time_t max_time,
                 moses_statistics& stats)
{
    if (mp.empty())
        return true;

    // Attempt to create a non-empty representation, by looping
    // over exemplars until we find one that expands.
    // XXX When would one never expand?  Wouldn't that be a bug?
    while (1) {
        scored_combo_tree_ptr_set_cit exemplar = mp.select_exemplar();

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
        if (dex.create_demes(exemplar->get_tree(), stats.n_expansions))
            break;

        logger().error() << "Exemplar: " << exemplar->get_tree();
        OC_ASSERT(false, "Exemplar failed to expand!\n");
    }

    dex.optimize_demes(max_evals, max_time);
    stats.n_evals += dex.total_evals();
    stats.n_expansions++;

    bool done = mp.merge_demes(dex._demes, dex._reps);

    if (logger().isInfoEnabled()) {
        logger().info() << "Expansion " << stats.n_expansions << " done";
        logger().info() << "Total number of evaluations so far: " << stats.n_evals;
        mp.log_best_candidates();
    }
    dex.free_demes();

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
                 deme_expander& dex,
                 const moses_parameters& pa,
                 moses_statistics& stats)
{
    logger().info("MOSES starts, max_evals=%d max_gens=%d max_time=%d",
                  pa.max_evals, pa.max_gens, pa.max_time);

    optim_stats *os = dynamic_cast<optim_stats *> (&dex._optimize);

    // Print legend for the columns of the stats.
    print_stats_header(os, mp.diversity_enabled());

    struct timeval start;
    gettimeofday(&start, NULL);
    stats.elapsed_secs = 0.0;

    while ((stats.n_evals < pa.max_evals)
           and (pa.max_gens != stats.n_expansions)
           and (mp.best_score() < pa.max_score)
           and (stats.elapsed_secs < pa.max_time))
    {
        // Run a generation
        bool done = expand_deme(mp, dex,
                                pa.max_evals - stats.n_evals, 
                                pa.max_time - stats.elapsed_secs, 
                                stats);

        struct timeval stop, elapsed;
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);
        start = stop;
        stats.elapsed_secs += elapsed.tv_sec + 1.0e-6*elapsed.tv_usec;

        // Print stats in a way that makes them easy to graph.
        // (columns of tab-seprated numbers)
        if (logger().isInfoEnabled()) {

            composite_score best(mp.best_composite_score());
            std::stringstream ss;
            ss << "Stats: " << stats.n_expansions
               << "\t" << stats.n_evals    // number of evaluations so far
               << "\t" << ((int) stats.elapsed_secs)  // wall-clock time.
               << "\t" << mp.size()        // size of the metapopulation
               << "\t" << best.get_score() // score of the highest-ranked exemplar.
               << "\t" << best.get_complexity(); // as above.
            if (os) {
                ss << "\t" << os->field_set_size  // number of bits in the knobs
                   << "\t" << os->nsteps  // number of iterations of optimizer
                   << "\t" << os->over_budget;  // exceeded max_evals
            }
            if (mp.diversity_enabled()) {
                // diversity stats over all metapopulation
                auto ds = mp.gather_diversity_stats(-1);
                ss << "\t" << ds.count // number of pairs of candidates
                   << "\t" << ds.mean  // average distance
                   << "\t" << ds.std   // average dst dev
                   << "\t" << ds.min   // min distance
                   << "\t" << ds.max;  // max distance

                // diversity stats over all best n candidates of the metapopulation
                // TODO use the option of the output
                auto best_ds = mp.gather_diversity_stats(pa.max_cnd_output);
                ss << "\t" << best_ds.count // number of pairs of candidates
                   << "\t" << best_ds.mean  // average distance
                   << "\t" << best_ds.std   // average dst dev
                   << "\t" << best_ds.min   // min distance
                   << "\t" << best_ds.max;  // max distance                
            }
            logger().info(ss.str());
        }

        // I find this particularly useful for studying diversity but
        // it could be relaxed and printed whatever
        if (logger().isDebugEnabled() and mp.diversity_enabled()) {
            std::stringstream ss;
            ss << pa.max_cnd_output << " best candidates of the metapopulation (with scores and visited status):" << std::endl;
            mp.ostream_metapop(ss, pa.max_cnd_output);
            logger().debug(ss.str());
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

