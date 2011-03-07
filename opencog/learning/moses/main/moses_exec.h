/** moses-exec.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
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
#include <boost/assign/list_of.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/iostreamContainer.h>

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>

// for operator>> to combo 
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h> 

#include <opencog/learning/feature-selection/feature_optimization.h>
#include <opencog/learning/feature-selection/feature_scorer.h>

#include "../moses/moses.h"
#include "../moses/distributed_moses.h"
#include "../moses/optimization.h"
#include "../moses/scoring_functions.h"
#include "../moses/scoring.h"
#include "moses_exec_def.h"

using namespace boost::program_options;
using boost::lexical_cast;
using boost::assign::list_of;
using namespace std;
using namespace moses;
using namespace reduct;
using namespace opencog;
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

// feature selection algorithms
static const string is="is"; // incremental selection

// feature selection scorer
static const string en="en"; // entropy

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
                                     bool _output_score_complexity_old_moses,
                                     bool _output_bscore,
                                     bool _output_eval_number,
                                     bool _output_with_labels,
                                     const vector<string>& _labels,
                                     string _output_file,
                                     const jobs_t& _jobs) : 
        result_count(_result_count), output_score(_output_score), 
        output_complexity(_output_complexity),
        output_score_complexity_old_moses(_output_score_complexity_old_moses),
        output_bscore(_output_bscore), output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels), labels(_labels),
        output_file(_output_file),
        jobs(_jobs) {}
    long result_count;
    bool output_score;
    bool output_complexity;
    bool output_score_complexity_old_moses;
    bool output_bscore;
    bool output_eval_number;
    bool output_with_labels;
    const vector<string>& labels;
    string output_file;
    const jobs_t& jobs;
};

/**
 * 1) create a metapopulation
 * 2) run moses
 * 3) print the results
 */
template<typename Score, typename BScore, typename Optimization>
void metapop_moses_results(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
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
    if(pa.jobs.empty()) {
        moses::moses(metapop, moses_params);
    } else moses::distributed_moses(metapop, vm, pa.jobs, moses_params);

    // output result
    stringstream ss;
    metapop.ostream(ss,
                    pa.result_count, pa.output_score,
                    pa.output_complexity,
                    pa.output_score_complexity_old_moses,
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

/**
 * like above but takes the algo type instead of the algo template
 */
template<typename Score, typename BScore>
void metapop_moses_results(RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
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
        metapop_moses_results(rng, bases, tt, si_ca, si_kb, sc, bsc,
                              iterative_hillclimbing(rng, opt_params),
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
                           const combo::type_tree& tt,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const BScore& bsc,
                           unsigned long cache_size,
                           const string& opt_algo,
                           const optim_parameters& opt_params,
                           const metapop_parameters& meta_params,
                           const moses_parameters& moses_params,
                           const variables_map& vm,
                           const metapop_moses_results_parameters& pa) {
    if(cache_size > 0) {
        typedef lru_cache<BScore> BScoreCache;
        typedef bscore_based_score<BScoreCache> Score;
        typedef lru_cache<Score> ScoreCache;
        BScoreCache bscore_cache(cache_size, bsc);
        Score score(bscore_cache);
        ScoreCache score_cache(cache_size, score);
        metapop_moses_results(rng, bases, tt, si_ca, si_kb,
                              score_cache, bscore_cache, opt_algo,
                              opt_params, meta_params, moses_params, vm, pa);
        // log the number of cache failures
        if(pa.jobs.empty()) { // do not print the cache if using distributed moses
            logger().info("Number of cache failures for score = %u"
                          " and bscore = %u",
                          score_cache.get_failures(), bscore_cache.get_failures());
        }
    } else {
        bscore_based_score<BScore> score(bsc);
        metapop_moses_results(rng, bases, tt, si_ca, si_kb, score, bsc,
                              opt_algo, opt_params, meta_params, moses_params,
                              vm, pa);
    }
}

#endif // _OPENCOG_MOSES_EXEC_H
