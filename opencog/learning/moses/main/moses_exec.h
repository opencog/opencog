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

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>

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
static const string cp="cp"; // regression based on combo program to learn
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
static const string sa="sa"; // simulation annealing
static const string hc="hc"; // hillclimbing

// used by the main function, it is included in the library for its
// convenience
int moses_exec(int argc, char** argv);
// helper for the function above, note that the first still represents
// the name of the supposed executable
int moses_exec(const vector<string>& argv);

struct metapop_moses_results_parameters {
    metapop_moses_results_parameters(long _result_count,
                                     bool _output_score,
                                     bool _output_complexity,
                                     bool _output_bscore,
                                     bool _output_eval_number,
                                     bool _output_with_labels,
                                     bool _enable_cache,
                                     const vector<string>& _labels,
                                     string _output_file,
                                     const jobs_t& _jobs,
                                     bool _only_local,
                                     bool _hc_terminate_if_improvement) : 
        result_count(_result_count), output_score(_output_score), 
        output_complexity(_output_complexity),
        output_bscore(_output_bscore), output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels),
        enable_cache(_enable_cache), labels(_labels),
        output_file(_output_file),
        jobs(_jobs), only_local(_only_local),
        hc_terminate_if_improvement(_hc_terminate_if_improvement) {}
    long result_count;
    bool output_score;
    bool output_complexity;
    bool output_bscore;
    bool output_eval_number;
    bool output_with_labels;
    bool enable_cache;
    const vector<string>& labels;
    string output_file;
    const jobs_t& jobs;
    bool only_local;
    bool hc_terminate_if_improvement;
};

/**
 * 1) create a metapopulation
 * 2) run moses
 * 3) print the results
 */
template<typename Score, typename BScore, typename Optimization>
void metapop_moses_results(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const Score& sc,
                           const BScore& bsc,
                           const Optimization& opt,
                           const metapop_parameters& meta_params,
                           const moses_parameters& moses_params,
                           const variables_map& vm,
                           const metapop_moses_results_parameters& pa) {
    // instantiate metapop
    metapopulation<Score, BScore, Optimization> 
        metapop(rng, bases, tt, si_ca, si_kb, sc, bsc, opt, meta_params);

    // run moses
    if(pa.only_local) {
        unsigned n_threads = pa.jobs.find(localhost)->second;
        setting_omp(n_threads);
        moses::moses(metapop, moses_params);
    } else moses::distributed_moses(metapop, vm, pa.jobs, moses_params);

    // output result
    {
        stringstream ss;
        metapop.ostream(ss,
                        pa.result_count, pa.output_score,
                        pa.output_complexity,
                        pa.output_bscore);
        if(pa.output_eval_number)
            ss << number_of_evals_str << ": " << metapop.n_evals() << std::endl;;
        string res = (pa.output_with_labels && !pa.labels.empty()?
                      ph2l(ss.str(), pa.labels) : ss.str());
        if(pa.output_file.empty())
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
        logger().info(string("Best candidate (preceded by its score and complexity): ") + res);
    }
}

/**
 * like above but takes the algo type instead of the algo template
 */
template<typename Score, typename BScore>
void metapop_moses_results(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const Score& sc,
                           const BScore& bsc,
                           const string& opt_algo,
                           const optim_parameters& opt_params,
                           const metapop_parameters& meta_params,
                           const moses_parameters& moses_params,
                           const variables_map& vm,
                           const metapop_moses_results_parameters& pa) {    
    if(opt_algo == un) { // univariate
        metapop_moses_results(rng, bases, tt, si_ca, si_kb, sc, bsc,
                              univariate_optimization(rng, opt_params),
                              meta_params, moses_params, vm, pa);
    } else if(opt_algo == sa) { // simulation annealing
        metapop_moses_results(rng, bases, tt, si_ca, si_kb, sc, bsc,
                              simulated_annealing(rng, opt_params),
                              meta_params, moses_params, vm, pa);
    } else if(opt_algo == hc) { // hillclimbing
        hc_parameters hc_params(pa.hc_terminate_if_improvement);
        metapop_moses_results(rng, bases, tt, si_ca, si_kb, sc, bsc,
                              iterative_hillclimbing(rng, opt_params, hc_params),
                              meta_params, moses_params, vm, pa);
    } else {
        std::cerr << "Unknown optimization algo " << opt_algo 
                  << ". Supported algorithms are un (for univariate),"
                  << " sa (for simulation annealing) and hc (for hillclimbing)"
                  << std::endl;
        exit(1);
    }
}

/**
 * like above but assumes that the score is bscore based
 */
template<typename BScore>
void metapop_moses_results(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const BScore& bsc,
                           const string& opt_algo,
                           optim_parameters opt_params,
                           const metapop_parameters& meta_params,
                           moses_parameters moses_params,
                           const variables_map& vm,
                           const metapop_moses_results_parameters& pa) {
    bscore_based_score<BScore> bb_score(bsc);
    
    // update terminate_if_gte and max_score criteria
    score_t bps = bb_score.best_possible_score();
    opt_params.terminate_if_gte = std::min(opt_params.terminate_if_gte, bps);
    moses_params.max_score = std::min(moses_params.max_score, bps);
    logger().info("Target score (maximum reachable score) = %f", bps);

    if(pa.enable_cache) {
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
        metapop_moses_results(rng, bases, tt, si_ca, si_kb,
                              score_acache, BSCORE, opt_algo,
                              opt_params, meta_params, moses_params, vm, pa);
        // log the number of cache failures
        if(pa.only_local) { // do not print if using distributed moses
            logger().info("Number of cache failures for score = %u",
                          score_acache.get_failures());
        }            
    } else {
        metapop_moses_results(rng, bases, tt, si_ca, si_kb, bb_score, bsc,
                              opt_algo, opt_params, meta_params, moses_params,
                              vm, pa);
    }
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_EXEC_H
