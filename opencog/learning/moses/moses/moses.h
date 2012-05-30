/*
 * opencog/learning/moses/moses/moses.h
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
#ifndef _MOSES_MOSES_H
#define _MOSES_MOSES_H

#include <map>
#include <boost/program_options/variables_map.hpp>
#include "metapopulation.h"
#include "../optimization/optimization.h"

namespace opencog {
 namespace moses {

using namespace combo;

/// A map between hostname and number of jobs allocated.
typedef std::map<string, unsigned> jobs_t;

/**
 * parameters to decide how to run moses
 */
struct moses_parameters
{
    moses_parameters(const boost::program_options::variables_map& _vm =
                           boost::program_options::variables_map(),
                     const jobs_t& _jobs = jobs_t(),
                     bool _local = true,
                     int _max_evals = 10000,
                     int _max_gens = -1,
                     score_t _max_score = 0)
        : local(_local), jobs(_jobs), vm(_vm),
          max_evals(_max_evals), max_gens(_max_gens), max_score(_max_score)
    {}

    // Distributed solver control.
    bool local;
    const jobs_t& jobs;
    const boost::program_options::variables_map& vm;

    // total maximun number of evals
    int max_evals;
    // the max number of demes to create and optimize, if negative,
    // then no limit
    int max_gens;
    // the max score
    score_t max_score;
};

/**
 * the main function of MOSES
 *
 * @param mp the metapopulation
 * @param pa the parameters to run moses
 */
template<typename Scoring, typename BScoring, typename Optimization>
void moses(metapopulation<Scoring, BScoring, Optimization>& mp,
           const moses_parameters& pa = moses_parameters())
{
    // Logger
    logger().info("MOSES starts, max_evals=%d max_gens=%d", 
                  pa.max_evals, pa.max_gens);
    // ~Logger
    int gen_idx = 0;

    optim_stats *os = dynamic_cast<optim_stats *> (&mp.optimize);

    // Print legend for the columns of the stats.
    if (logger().isInfoEnabled()) {
        stringstream ss;
        ss << "Stats:# gen\tnum_evals\tmetapop_size\tbest_score\tcomplexity";
        if (os) {
            ss << "\toptim_steps\tover_budget";
        }
        ss << endl;
        logger().info(ss.str());
    }

    while ((mp.n_evals() < pa.max_evals) && (pa.max_gens != gen_idx++))
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
