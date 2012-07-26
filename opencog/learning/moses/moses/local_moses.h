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
        bool done = mp.expand(pa.max_evals - mp.n_evals());

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
