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

// used by the main function, it is included in the library for its
// convenience
int moses_exec(int argc, char** argv);
// helper for the function above, note that the first still represents
// the name of the supposed executable
int moses_exec(const vector<string>& argv);

/**
 * Run moses
 * (This could be moved to moses.h, except for a header-include order
 * clash with distributed_moses.)
 */
template<typename Score, typename BScore, typename Optimization>
void run_moses(metapopulation<Score, BScore, Optimization> &metapop,
                             const moses_parameters& moses_params)
{
    // Run moses, either on localhost, or distributed.
    if (moses_params.only_local)
        moses::moses(metapop, moses_params);
    else
        moses::distributed_moses(metapop, moses_params);
}

/// Print metapopulation results to stdout, logfile, etc.
struct metapop_printer
{
    metapop_printer(long _result_count,
                    bool _output_score,
                    bool _output_complexity,
                    bool _output_bscore,
                    bool _output_dominated,
                    bool _output_eval_number,
                    bool _output_with_labels,
                    const vector<string>& _labels,
                    const string& _output_file,
                    bool _output_python) :
        result_count(_result_count), output_score(_output_score),
        output_complexity(_output_complexity),
        output_bscore(_output_bscore),
        output_dominated(_output_dominated),
        output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels),
        labels(_labels),
        output_file(_output_file),
        output_python(_output_python) {}

    /**
     * Print metapopulation summary.
     */
    template<typename Score, typename BScore, typename Optimization>
    void operator()(metapopulation<Score, BScore, Optimization> &metapop) const
    {
        stringstream ss;
        metapop.ostream(ss,
                        result_count,
                        output_score,
                        output_complexity,
                        output_bscore,
                        output_dominated,
                        output_python); 
    
        if (output_eval_number)
            ss << number_of_evals_str << ": " << metapop.n_evals() << std::endl;;
        string res = (output_with_labels && !labels.empty()?
                      ph2l(ss.str(), labels) : ss.str());
        if (output_file.empty())
            std::cout << res;
        else {
            ofstream of(output_file.c_str());
            of << res;
            of.close();
        }
    
        // Log the best candidate
        stringstream ssb;
        metapop.ostream(ssb, 1, true, true);
        string resb = (output_with_labels && !labels.empty()?
                      ph2l(ssb.str(), labels) : ssb.str());
        if (resb.empty())
            logger().info("No candidate is good enough to be returned. Yeah that's bad!");
        else
            logger().info("Best candidate (preceded by its score and complexity): %s", res.c_str());
    
    #ifdef GATHER_STATS
        metapop.optimize.hiscore /= metapop.optimize.hicount;
        for (unsigned i=0; i< metapop.optimize.scores.size(); i++) {
            metapop.optimize.scores[i] /= metapop.optimize.counts[i];
            logger().info() << "Avg Scores: "
                << i << "\t"
                << metapop.optimize.hiscore << "\t"
                << metapop.optimize.counts[i] << "\t"
                << metapop.optimize.scores[i];
        }
    #endif
    }
private:
    long result_count;
    bool output_score;
    bool output_complexity;
    bool output_bscore;
    bool output_dominated;
    bool output_eval_number;
    bool output_with_labels;
    const vector<string>& labels;
    string output_file;
    bool output_python;
};


/**
 * Create metapopulation, run moses, print results.
 */
template<typename Score, typename BScore, typename Printer>
void metapop_moses_results_b(const std::vector<combo_tree>& bases,
                             const opencog::combo::type_tree& tt,
                             const reduct::rule& si_ca,
                             const reduct::rule& si_kb,
                             const Score& sc,
                             const BScore& bsc,
                             const optim_parameters& opt_params,
                             const metapop_parameters& meta_params,
                             const moses_parameters& moses_params,
                             const Printer& printer)
{
    if (opt_params.opt_algo == hc) { // exhaustive neighborhood search
        hill_climbing climber(opt_params);

        metapopulation<Score, BScore, hill_climbing>
            metapop(bases, tt, si_ca, si_kb, sc, bsc, climber, meta_params);

        run_moses(metapop, moses_params);
        printer(metapop);
    }
    else if (opt_params.opt_algo == sa) { // simulated annealing
        simulated_annealing annealer(opt_params);

        metapopulation<Score, BScore, simulated_annealing>
            metapop(bases, tt, si_ca, si_kb, sc, bsc, annealer, meta_params);

        run_moses(metapop, moses_params);
        printer(metapop);
    }
    else if (opt_params.opt_algo == un) { // univariate
        univariate_optimization unopt(opt_params);

        metapopulation<Score, BScore, univariate_optimization>
            metapop(bases, tt, si_ca, si_kb, sc, bsc, unopt, meta_params);

        run_moses(metapop, moses_params);
        printer(metapop);
    }
    else {
        std::cerr << "Unknown optimization algo " << opt_params.opt_algo
                  << ". Supported algorithms are un (for univariate),"
                  << " sa (for star-shaped search) and hc (for local search)"
                  << std::endl;
        exit(1);
    }
}

/**
 * like above, but assumes that the score is bscore based
 */
template<typename BScore, typename Printer>
void metapop_moses_results(const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& type_sig,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           const BScore& bsc,
                           optim_parameters opt_params,
                           const metapop_parameters& meta_params,
                           moses_parameters moses_params,
                           const Printer& printer)
{
    bscore_based_score<BScore> bb_score(bsc);

    // update terminate_if_gte and max_score criteria
    score_t bps = bb_score.best_possible_score();
    score_t target_score = std::min(moses_params.max_score, bps);
    opt_params.terminate_if_gte = target_score;
    // update minimum score improvement
    opt_params.set_min_score_improv(bb_score.min_improv());
    moses_params.max_score = target_score;
    logger().info("Target score = %f", target_score);

    if (meta_params.enable_cache) {
        static const unsigned initial_cache_size = 1000000;
        
        if(meta_params.include_dominated) {
            typedef bscore_based_score<BScore> Score;
            typedef adaptive_cache<prr_cache_threaded<Score> > ScoreACache;
            Score score(bsc);
            prr_cache_threaded<Score> score_cache(initial_cache_size, score);
            ScoreACache score_acache(score_cache, "scores");
            metapop_moses_results_b(bases, type_sig, si_ca, si_kb,
                                    score_acache, bsc,
                                    opt_params, meta_params, moses_params,
                                    printer);
        }
        else {
            // We put the cache on the bscore as well because then it
            // is reused later (for metapopulation merging)
            typedef prr_cache_threaded<BScore> BScoreCache;
            typedef adaptive_cache<BScoreCache> BScoreACache;
            typedef bscore_based_score<BScoreACache> Score;
            BScoreCache bscore_cache(initial_cache_size, bsc);
            BScoreACache bscore_acache(bscore_cache, "behavioural scores");
            typedef adaptive_cache<prr_cache_threaded<Score> > ScoreACache;
            Score score(bscore_acache);
            prr_cache_threaded<Score> score_cache(initial_cache_size, score);
            ScoreACache score_acache(score_cache, "scores");
            metapop_moses_results_b(bases, type_sig, si_ca, si_kb,
                                    score_acache, bscore_acache,
                                    opt_params, meta_params, moses_params,
                                    printer);
        }
        return;
    }

    metapop_moses_results_b(bases, type_sig, si_ca, si_kb, bb_score, bsc,
                         opt_params, meta_params, moses_params, printer);
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_EXEC_H
