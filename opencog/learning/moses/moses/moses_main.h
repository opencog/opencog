/** moses_main.h ---
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

#ifndef _OPENCOG_MOSES_MAIN_H
#define _OPENCOG_MOSES_MAIN_H

#include <iostream>
#include <fstream>
#include <memory>
#include <stdio.h>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/combo/combo.h>

#include "distributed_moses.h"
#include "local_moses.h"
#include "mpi_moses.h"
#include "metapopulation.h"
#include "scoring.h"

namespace opencog { namespace moses {

using namespace std;

extern const char * version_string;
typedef bscore_based_cscore<bscore_base> cscorer_t;

/**
 * Run moses
 */
void run_moses(metapopulation& metapop,
               const moses_parameters& moses_params,
               moses_statistics& stats);

/// Print metapopulation results to stdout, logfile, etc.
struct metapop_printer
{
    metapop_printer(long _result_count,
                    bool _output_score,
                    bool _output_penalty,
                    bool _output_bscore,
                    bool _output_dominated,
                    bool _output_eval_number,
                    bool _output_with_labels,
                    const vector<string>& _labels,
                    const string& _output_file,
                    bool _output_python,
                    bool _is_mpi) :
        result_count(_result_count), output_score(_output_score),
        output_penalty(_output_penalty),
        output_bscore(_output_bscore),
        output_dominated(_output_dominated),
        output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels),
        labels(_labels),
        output_file(_output_file),
        output_python(_output_python),
        is_mpi(_is_mpi) {}

    /**
     * Print metapopulation summary.
     */
    void operator()(metapopulation &metapop,
                   moses_statistics& stats) const
    {
        // We expect the mpi worker processes to have an empty
        // metapop at this point.  So don't print alarming output
        // messages.  In fact, the mpi workers should not even have
        // a printer at all, or use a null_printer.  Unfortunately,
        // the current code structure makes this hard to implement.
        // XXX TODO this should be fixed, someday...
        if (is_mpi && metapop.size() == 0)
            return;

        stringstream ss;
        metapop.ostream(ss,
                        result_count,
                        output_score,
                        output_penalty,
                        output_bscore,
                        output_dominated,
                        output_python); 
    
        if (output_eval_number)
            ss << number_of_evals_str << ": " << stats.n_evals << std::endl;;

#if 0
// XXX TEMPORARY HACK for perf measurements, remove me after June 2012
        extern std::vector<CTable> gtables;

        enum_table_bscore etb(gtables[0]);
        bscore_based_cscore<enum_table_bscore> straight(etb);

        enum_graded_bscore etg(gtables[0]);
        bscore_based_cscore<enum_graded_bscore> regrade(etg);

        enum_effective_bscore etf(gtables[0]);
        bscore_based_cscore<enum_effective_bscore> effect(etf);

        // const bscored_combo_tree& candidate = begin();
        auto cit = metapop.begin();
    
        for (int i=0; i<10; i++) {
            if (cit == metapop.end()) break;
            composite_score sc = straight(get_tree(*cit));
            ss << "Sraight: " << sc 
               << "\n Graded: " << get_composite_score(*cit);

            ss << "\nRegrade: " << regrade(get_tree(*cit));
            ss << "\nEffecti: " << effect(get_tree(*cit));
            ss << std::endl;
            cit++;
        }
#endif

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
            logger().warn("No candidate is good enough to be returned. Yeah that's bad!");
        else
            logger().info("Best candidates (preceded by its score):\n%s", res.c_str());
    
    #ifdef GATHER_STATS
        metapop._dex._optimize.hiscore /= metapop._dex._optimize.hicount;
        metapop._dex._optimize.num_improved /= metapop._dex._optimize.count_improved;
        logger().info() << "Avg number of improved scores = "
                        << metapop._dex._optimize.num_improved;
        logger().info() << "Avg improved as percentage= "
                        << 100.0 * metapop._dex._optimize.num_improved /
                               metapop._dex._optimize.scores.size();

        for (unsigned i=0; i< metapop._dex._optimize.scores.size(); i++) {
            metapop._dex._optimize.scores[i] /= metapop._dex._optimize.counts[i];
            logger().info() << "Avg Scores: "
                << i << "\t"
                << metapop._dex._optimize.hiscore << "\t"
                << metapop._dex._optimize.counts[i] << "\t"
                << metapop._dex._optimize.scores[i];
        }
    #endif
    }
private:
    long result_count;
    bool output_score;
    bool output_penalty;
    bool output_bscore;
    bool output_dominated;
    bool output_eval_number;
    bool output_with_labels;
    const vector<string>& labels;
    string output_file;
    bool output_python;
    bool is_mpi;
};


/**
 * Create metapopulation, run moses, print results.
 */
template<typename Printer>
void metapop_moses_results_b(const std::vector<combo_tree>& bases,
                             const opencog::combo::type_tree& tt,
                             const reduct::rule& si_ca,
                             const reduct::rule& si_kb,
                             const cscore_base& sc,
                             const bscore_base& bsc,
                             const optim_parameters& opt_params,
                             const metapop_parameters& meta_params,
                             const moses_parameters& moses_params,
                             Printer& printer)
{
    moses_statistics stats;
    optimizer_base *optimizer = NULL;

    if (opt_params.opt_algo == hc) { // exhaustive neighborhood search
        optimizer = new hill_climbing(opt_params);
    }
    else if (opt_params.opt_algo == sa) { // simulated annealing
        optimizer = new simulated_annealing(opt_params);
    }
    else if (opt_params.opt_algo == un) { // univariate
        optimizer = new univariate_optimization(opt_params);
    }
    else {
        std::cerr << "Unknown optimization algo " << opt_params.opt_algo
                  << ". Supported algorithms are un (for univariate),"
                  << " sa (for star-shaped search) and hc (for local search)"
                  << std::endl;
        exit(1);
    }

    metapopulation metapop(bases, tt, si_ca, si_kb, sc, bsc,
                           *optimizer, meta_params);

    run_moses(metapop, moses_params, stats);
    printer(metapop, stats);
    delete optimizer;
}

/**
 * like above, but assumes that the score is bscore based
 */
template<typename Printer>
void metapop_moses_results(const std::vector<combo_tree>& bases,
                           const opencog::combo::type_tree& type_sig,
                           const reduct::rule& si_ca,
                           const reduct::rule& si_kb,
                           bscore_base& bscorer,
                           optim_parameters opt_params,
                           const metapop_parameters& meta_params,
                           moses_parameters moses_params,
                           Printer& printer)
{
    cscorer_t c_scorer(bscorer);

    // update terminate_if_gte and max_score criteria
    score_t bps = c_scorer.best_possible_score();
    score_t target_score = std::min(moses_params.max_score, bps);
    opt_params.terminate_if_gte = target_score;
    // update minimum score improvement
    opt_params.set_min_score_improv(c_scorer.min_improv());
    moses_params.max_score = target_score;
    logger().info("Target score = %f", target_score);

    if (meta_params.cache_size > 0) {
        // WARNING: adaptive_cache is not thread safe (and therefore
        // deactivated for now)
        
        // static const unsigned initial_cache_size = 1000000;
        unsigned initial_cache_size = meta_params.cache_size;
        
        // When the include_dominated flag is set, then trees are merged
        // into the metapop based only on the score (and complexity),
        // not on the behavioral score. So we can throw away the 
        // behavioral score after computng it (we don't need to cche it).
        // typedef adaptive_cache<prr_cache_threaded<CScorer> > ScoreACache;
        cscorer_t cscorer(bscorer);
        prr_cache_threaded<cscorer_t> score_cache(initial_cache_size, cscorer,
                                                 "composite scores");
        // ScoreACache score_acache(score_cache);
        metapop_moses_results_b(bases, type_sig, si_ca, si_kb,
                                score_cache /*score_acache*/, bscorer,
                                opt_params, meta_params, moses_params,
                                printer);
    } else
        metapop_moses_results_b(bases, type_sig, si_ca, si_kb, c_scorer, bscorer,
                                opt_params, meta_params, moses_params, printer);
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_MAIN_H
