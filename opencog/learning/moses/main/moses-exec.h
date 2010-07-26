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

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>

// for operator>> to combo 
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h> 

#include "../moses/moses.h"
#include "../moses/optimization.h"
#include "../moses/scoring_functions.h"
#include "../moses/scoring.h"

using namespace boost::program_options;
using boost::lexical_cast;
using namespace std;
using namespace moses;
using namespace reduct;
using opencog::logger;
using namespace ant_combo;

// default number of samples to describe a problem
static const unsigned int default_nsamples = 20;

// problems
static const string it="it"; // regression based on input table
static const string cp="cp"; // regression based on combo program to learn
static const string pa="pa"; // even parity
static const string dj="dj"; // disjunction
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

/**
 * 1) create a metapopulation
 * 2) run moses
 * 3) print the results
 */
template<typename Score, typename BScore, typename Optimization>
void metapop_moses_results(opencog::RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
                           const reduct::rule& si,
                           bool reduce_all,
                           const Score& sc,
                           const BScore& bsc,
                           bool count_base,
                           const Optimization& opt,
                           int max_evals,
                           int max_gens,
                           const vertex_set& ignore_ops,
                           long result_count,
                           bool output_complexity,
                           bool output_bscore) {
    metapopulation<Score, BScore, Optimization> 
        metapop(rng, bases, tt, si, reduce_all, sc, bsc, count_base, opt);
    moses::moses(metapop, max_evals, max_gens, 0, ignore_ops);
    metapop.print_best(result_count, output_complexity, output_bscore);
}

/**
 * like above but takes the algo type instead of the algo template
 */
template<typename Score, typename BScore>
void metapop_moses_results(opencog::RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
                           const reduct::rule& si,
                           bool reduce_all,
                           const Score& sc,
                           const BScore& bsc,
                           bool count_base,
                           const string& opt_algo,
                           int max_evals,
                           int max_gens,
                           const vertex_set& ignore_ops,
                           long result_count,
                           bool output_complexity,
                           bool output_bscore) {
    if(opt_algo == un) { // univariate
        metapop_moses_results(rng, bases, tt, si, reduce_all, 
                              sc, bsc, count_base,
                              univariate_optimization(rng),
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity, output_bscore);
    } else if(opt_algo == sa) { // simulation annealing
        metapop_moses_results(rng, bases, tt, si, reduce_all, 
                              sc, bsc, count_base,
                              simulated_annealing(rng),
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity, output_bscore);
    } else if(opt_algo == hc) { // hillclimbing
        metapop_moses_results(rng, bases, tt, si, reduce_all,
                              sc, bsc, count_base,
                              iterative_hillclimbing(rng),
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity, output_bscore);
    } else {
        std::cerr << "Unknown optimization algo " << opt_algo << ". Supported algorithms are un (for univariate), sa (for simulation annealing) and hc (for hillclimbing)" << std::endl;
        exit(1);
    }
}

/**
 * like above but assumes that the score is bscore based
 */
template<typename BScore>
void metapop_moses_results(opencog::RandGen& rng,
                           const std::vector<combo_tree>& bases,
                           const combo::type_tree& tt,
                           const reduct::rule& si,
                           bool reduce_all,
                           const BScore& bsc,
                           unsigned long cache_size,
                           bool count_base,
                           const string& opt_algo,
                           int max_evals,
                           int max_gens,
                           const vertex_set& ignore_ops,
                           long result_count,
                           bool output_complexity,
                           bool output_bscore) {
    if(cache_size > 0) {
        // until lry_cache is fixed (can handle expection half-way),
        // simple_cache is used
        // typedef opencog::lru_cache<BScore> BScoreCache;
        typedef opencog::simple_cache<BScore> BScoreCache;
        typedef bscore_based_score<BScoreCache> Score;
        // typedef opencog::lru_cache<Score> ScoreCache;
        typedef opencog::simple_cache<Score> ScoreCache;        
        BScoreCache bscore_cache(cache_size, bsc);
        Score score(bscore_cache);
        ScoreCache score_cache(cache_size, score);
        metapop_moses_results(rng, bases, tt, si,
                              reduce_all, score_cache, bscore_cache,
                              count_base, opt_algo,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity, output_bscore);
    } else {
        bscore_based_score<BScore> score(bsc);
        metapop_moses_results(rng, bases, tt, si,
                              reduce_all, score, bsc,
                              count_base, opt_algo,
                              max_evals, max_gens, ignore_ops,
                              result_count, output_complexity, output_bscore);
    }
}

#endif // _OPENCOG_MOSES_EXEC_H
