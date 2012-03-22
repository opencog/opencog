/** moses-exec.h ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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

#ifndef _OPENCOG_MOSES_EXEC_H
#define _OPENCOG_MOSES_EXEC_H

#include <iostream>
#include <fstream>
#include <memory>
#include <stdio.h>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/combo/combo.h>

// for operator>> to combo
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

#include "../moses/moses.h"
#include "../moses/distributed_moses.h"
#include "../optimization/optimization.h"
#include "../moses/metapopulation.h"
#include "../moses/scoring_functions.h"
#include "../moses/scoring.h"
#include "moses_exec_def.h"

// enable a cache on bscore (takes more memory)
#define ENABLE_BCACHE

namespace opencog { namespace moses {

using namespace boost::program_options;
using boost::lexical_cast;
using namespace std;
using namespace reduct;
using namespace ant_combo;

// default number of samples to describe a problem
static const unsigned int default_nsamples = 20;

// problems
static const string it="it"; // regression based on input table
static const string pre="pre"; // regression based on input table by
                               // maximizing precision (or negative
                               // predictive value)
static const string ip="ip"; // find interesting patterns
static const string cp="cp"; // regression based on combo program to fit
static const string pa="pa"; // even parity
static const string dj="dj"; // disjunction
static const string mux="mux"; // multiplex
static const string sr="sr"; // simple regression of f(x)_o = sum_{i={1,o}} x^i
static const string ann_it="ann-it"; // regression based on input table using ann
static const string ann_cp="ann-cp"; // regression based on combo program using ann
static const string ann_xor="ann-xor"; // binary-xor problem using ann
static const string ann_pole1="ann-pole1"; // pole balancing problem using ann
static const string ann_pole2="ann-pole2"; // double pole balancing problem ann

// optimization algorithms
static const string un="un"; // univariate
static const string sa="sa"; // star-shaped search
static const string hc="hc"; // local search

// used by the main function, it is included in the library for its
// convenience
int moses_exec(int argc, char** argv);
// helper for the function above, note that the first still represents
// the name of the supposed executable
int moses_exec(const vector<string>& argv);

// Parameters controlling output printing and display.
struct metapop_moses_results_parameters
{
    metapop_moses_results_parameters(const variables_map& _vm,
                                     long _result_count,
                                     bool _output_score,
                                     bool _output_complexity,
                                     bool _output_bscore,
                                     bool _output_dominated,
                                     bool _output_eval_number,
                                     bool _output_with_labels,
                                     const string& _opt_algo,
                                     bool _enable_cache,
                                     const vector<string>& _labels,
                                     const string& _output_file,
                                     const jobs_t& _jobs,
                                     bool _only_local,
                                     bool _output_python,
                                     bool _hc_widen_search,
                                     bool _hc_single_step,
                                     bool _hc_crossover,
                                     bool _hc_single_step) :
        vm(_vm), result_count(_result_count), output_score(_output_score),
        output_complexity(_output_complexity),
        output_bscore(_output_bscore),
        output_dominated(_output_dominated),
        output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels),
        opt_algo(_opt_algo),
        enable_cache(_enable_cache), labels(_labels),
        output_file(_output_file),
        jobs(_jobs), only_local(_only_local),
        output_python(_output_python),
        hc_widen_search(_hc_widen_search),
        hc_single_step(_hc_single_step),
        hc_crossover(_hc_crossover) {}

    const variables_map& vm;
    long result_count;
    bool output_score;
    bool output_complexity;
    bool output_bscore;
    bool output_dominated;
    bool output_eval_number;
    bool output_with_labels;
    string opt_algo;
    bool enable_cache;
    const vector<string>& labels;
    string output_file;
    const jobs_t& jobs;
    bool only_local;
    bool output_python;

    // Not really results, but we have nowhere else to put these...
    bool hc_widen_search;
    bool hc_single_step;
    bool hc_crossover;
};

/**
 * 1) create a metapopulation
 * 2) run moses
 * 3) print the results
 */
template<typename Score, typename BScore, typename Optimization>
void metapop_moses_results_a(RandGen& rng,
                           Optimization& opt,
                           const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const Score& sc,
                           const BScore& bsc,
                           const metapop_parameters& meta_params,
                           const moses_parameters& moses_params,
                           const metapop_moses_results_parameters& pa)
{
    // instantiate metapop
    metapopulation<Score, BScore, Optimization>
        metapop(rng, bases, tt, si_ca, si_kb, sc, bsc, opt, meta_params);

    // run moses, either on localhost, or distributed.
    if (pa.only_local) {
        unsigned n_threads = pa.jobs.find(localhost)->second;
        setting_omp(n_threads);
        moses::moses(metapop, moses_params);
    }
    else
        moses::distributed_moses(metapop, pa.vm, pa.jobs, moses_params);

    // output result
    {
        stringstream ss;
        metapop.ostream(ss,
                        pa.result_count, pa.output_score,
                        pa.output_complexity,
                        pa.output_bscore,
                        pa.output_dominated,
                        pa.output_python); 

        if (pa.output_eval_number)
            ss << number_of_evals_str << ": " << metapop.n_evals() << std::endl;;
        string res = (pa.output_with_labels && !pa.labels.empty()?
                      ph2l(ss.str(), pa.labels) : ss.str());
        if (pa.output_file.empty())
            std::cout << res;
        else {
            ofstream of(pa.output_file.c_str());
            of << res;
            of.close();
        }
    }

    // Log the best candidate
    {
        stringstream ss;
        metapop.ostream(ss, 1, true, true);
        string res = (pa.output_with_labels && !pa.labels.empty()?
                      ph2l(ss.str(), pa.labels) : ss.str());
        if (res.empty())
            logger().info("No candidate is good enough to be returned. Yeah that's bad!");
        else
            logger().info("Best candidate (preceded by its score and complexity): %s", res.c_str());
    }
}

/**
 * like above but takes the algo type instead of the algo template
 */
template<typename Score, typename BScore>
void metapop_moses_results_b(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const Score& sc,
                           const BScore& bsc,
                           const optim_parameters& opt_params,
                           const metapop_parameters& meta_params,
                           const moses_parameters& moses_params,
                           const metapop_moses_results_parameters& pa)
{
    if (pa.opt_algo == un) { // univariate
        univariate_optimization unopt(rng, opt_params);
        metapop_moses_results_a(rng, unopt,
                              bases, tt, si_ca, si_kb, sc, bsc,
                              meta_params, moses_params, pa);
    }
    else if (pa.opt_algo == sa) { // simulated annealing
        simulated_annealing annealer(rng, opt_params);
        metapop_moses_results_a(rng, annealer,
                              bases, tt, si_ca, si_kb, sc, bsc,
                              meta_params, moses_params, pa);
    }
    else if (pa.opt_algo == hc) { // exhaustive neighborhood search
        hc_parameters hc_params(pa.hc_widen_search,
                                pa.hc_single_step,
                                pa.hc_crossover);
        hill_climbing climber(rng, opt_params, hc_params);
        metapop_moses_results_a(rng, climber,
                              bases, tt, si_ca, si_kb, sc, bsc,
                              meta_params, moses_params, pa);
#ifdef GATHER_STATS
        climber.hiscore /= climber.hicount;
        for (unsigned i=0; i< climber.scores.size(); i++) {
            climber.scores[i] /= climber.counts[i];
            logger().info() << "Avg Scores: "
                << i << "\t"
                << climber.hiscore << "\t"
                << climber.counts[i] << "\t"
                << climber.scores[i];
        }
#endif
    }
    else {
        std::cerr << "Unknown optimization algo " << pa.opt_algo
                  << ". Supported algorithms are un (for univariate),"
                  << " sa (for star-shaped search) and hc (for local search)"
                  << std::endl;
        exit(1);
    }
}

/**
 * like above, but assumes that the score is bscore based
 */
template<typename BScore>
void metapop_moses_results(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const BScore& bsc,
                           optim_parameters opt_params,
                           const metapop_parameters& meta_params,
                           moses_parameters moses_params,
                           const metapop_moses_results_parameters& pa)
{
    bscore_based_score<BScore> bb_score(bsc);

    // update terminate_if_gte and max_score criteria
    score_t bps = bb_score.best_possible_score(),
        target_score = std::min(moses_params.max_score, bps);
    opt_params.terminate_if_gte = target_score;
    // update minimum score improvement
    opt_params.set_min_score_improv(bb_score.min_improv());
    moses_params.max_score = target_score;
    logger().info("Target score = %f", target_score);

    if (pa.enable_cache) {
        static const unsigned initial_cache_size = 1000000;
#ifdef ENABLE_BCACHE
        typedef prr_cache_threaded<BScore> BScoreCache;
        typedef adaptive_cache<BScoreCache> BScoreACache;
        typedef bscore_based_score<BScoreACache> Score;
        BScoreCache bscore_cache(initial_cache_size, bsc);
        BScoreACache bscore_acache(bscore_cache);
#define BSCORE bscore_acache
#else
        typedef bscore_based_score<BScore> Score;
#define BSCORE bsc
#endif
        typedef adaptive_cache<prr_cache_threaded<Score> > ScoreACache;
        Score score(BSCORE);
        prr_cache_threaded<Score> score_cache(initial_cache_size, score);
        ScoreACache score_acache(score_cache);
        metapop_moses_results_b(rng, bases, tt, si_ca, si_kb,
                              score_acache, BSCORE,
                              opt_params, meta_params, moses_params, pa);
        // log the number of cache failures
        if (pa.only_local) { // do not print if using distributed moses
            logger().info("Score cache hits=%u misses=%u",
                          score_acache.get_hits(),
                          score_acache.get_failures());
        }
    }
    else {
        metapop_moses_results_b(rng, bases, tt, si_ca, si_kb, bb_score, bsc,
                              opt_params, meta_params, moses_params, pa);
    }
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_EXEC_H
