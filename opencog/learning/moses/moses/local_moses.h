/*
 * opencog/learning/moses/moses/local_moses.h
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
#ifndef _MOSES_LOCAL_MOSES_H
#define _MOSES_LOCAL_MOSES_H

#include "metapopulation.h"
#include "moses_params.h"
#include "../optimization/optimization.h"

namespace opencog {
namespace moses {

using namespace combo;

/**
 * expand -- Run one deme-creation and optimization step.
 *
 * A single step consists of representation-building, to create
 * a deme, followed by optimization, (according to the specified
 * optimizer and scoring function), and finally, a merger of
 * the unique (and possibly non-dominated) trees back into the
 * metapopulation, for potential use as exemplars for futre demes.
 *
 * @param max_evals    the max evals
 *
 * @return return true if expansion has succeeded, false otherwise
 *
 */
template<typename Scoring, typename BScoring, typename Optimization>
bool expand_deme(metapopulation<Scoring, BScoring, Optimization>& mp,
     int max_evals)
{
    if (mp.empty())
        return false;

    // Attempt to create a non-empty representation, by looping
    // over exemplars until we find one that expands.
    // XXX When would one never expand?  Wouldn't that be a bug?
    while (1) {
        bscored_combo_tree_set::const_iterator exemplar = mp.select_exemplar();

        // Should have found something by now.
        if (exemplar == mp.end()) {
            logger().warn(
                "WARNING: All exemplars in the metapopulation have "
                "been visited, but it was impossible to build a "
                "representation for any of them.  Perhaps the reduct "
                "effort for knob building is too high.");
            return false;
        }

        // if create_deme returned true, we are good to go.
        if (mp._dex.create_deme(exemplar)) break;

        OC_ASSERT(false, "Exemplar failed to expand!\n");
    }

    mp.n_expansions() ++;
    size_t evals_this_deme = mp._dex.optimize_deme(max_evals);
    mp.n_evals() += evals_this_deme;

    bool done = mp.merge_deme(mp._dex._deme, mp._dex._rep, evals_this_deme);

    if (logger().isInfoEnabled()) {
        logger().info()
           << "Expansion " << mp.n_expansions()
           << " total number of evaluations so far: " << mp.n_evals();
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
template<typename Scoring, typename BScoring, typename Optimization>
void local_moses(metapopulation<Scoring, BScoring, Optimization>& mp,
           const moses_parameters& pa = moses_parameters())
{
    // Logger
    logger().info("MOSES starts, max_evals=%d max_gens=%d",
                  pa.max_evals, pa.max_gens);
    // ~Logger
    int gen_idx = 0;

    optim_stats *os = dynamic_cast<optim_stats *> (&mp._dex._optimize);

    // Print legend for the columns of the stats.
    if (logger().isInfoEnabled()) {
        stringstream ss;
        ss << "Stats:# \n"
           << "Stats:# Stats are tab-separated, ready for graphing.  Column explanation:\n"
           << "Stats:# \n"
           << "Stats:# gen is the generation number.\n"
           << "Stats:# num_evals is the number of scoring function evaluations so far.\n"
           << "Stats:# metapop_size is the size of the metapopulation.\n"
           << "Stats:# best_score is the highest raw score seen, of all exemplars.\n"
           << "Stats:# complexity is in bits, of the highest-composite score exemplar.\n";
        if (os) {
           ss << "Stats:# field_set_size is the ESTIMATED number of bits in all the knobs.\n"
              << "Stats:# optim_steps is the number of steps the optimizer took.\n"
              << "Stats:# over_budget is bool, T if search exceeded scoring func eval budget.\n";
        }
        ss << "Stats:# \n"
           << "Stats:# gen\tnum_evals\tmetapop_size\tbest_score\tcomplexity";
        if (os) {
            ss << "\tfield_set_size\toptim_steps\tover_budget";
        }
        ss << "Stats:# \n";
        ss << endl;
        logger().info(ss.str());
    }

    while ((int(mp.n_evals()) < pa.max_evals) && (pa.max_gens != gen_idx++))
    {
        logger().info("Deme generation: %i", gen_idx);

        // Run a generation
        bool done = expand_deme(mp, pa.max_evals - mp.n_evals());

        // Print stats in a way that makes them easy to graph.
        // (columns of tab-seprated numbers)
        if (logger().isInfoEnabled()) {
            stringstream ss;
            ss << "Stats: " << gen_idx;
            ss << "\t" << mp.n_evals();    // number of evaluations so far
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

        if (mp.best_score() >= pa.max_score || mp.empty())
            break;
    }

    logger().info("MOSES ends");
}

} // ~namespace moses
} // ~namespace opencog

#endif
