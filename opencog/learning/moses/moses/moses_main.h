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

#include <opencog/util/Logger.h>

#include <opencog/comboreduct/combo/combo.h>

#include "../deme/deme_expander.h"
#include "../metapopulation/metapopulation.h"

#include "../optimization/hill-climbing.h"
#include "../optimization/star-anneal.h"
#include "../optimization/univariate.h"
#include "../scoring/behave_cscore.h"
#include "distributed_moses.h"
#include "moses_params.h"

namespace opencog { namespace moses {

using namespace std;

extern const char * version_string;

/**
 * Run moses
 */
void run_moses(metapopulation&,
               deme_expander&,
               const moses_parameters&,
               moses_statistics&);

/// Print metapopulation results to stdout, logfile, etc.
struct metapop_printer
{
    metapop_printer() {}
    metapop_printer(long _result_count,
                    bool _output_score,
                    bool _output_penalty,
                    bool _output_bscore,
                    bool _output_only_best,
                    bool _output_eval_number,
                    bool _output_with_labels,
                    const vector<string>& _ilabels,
                    const string& _output_file,
                    bool _output_python,
                    bool _is_mpi) :
        result_count(_result_count), output_score(_output_score),
        output_penalty(_output_penalty),
        output_bscore(_output_bscore),
        output_only_best(_output_only_best),
        output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels),
        ilabels(_ilabels),
        output_file(_output_file),
        output_python(_output_python),
        is_mpi(_is_mpi) {}

    /**
     * Print metapopulation summary.
     */
    void operator()(metapopulation &metapop,
                    deme_expander& dex,
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
        if (output_python) {
            // Python boilerplate
            // XXX FIXME -- I doubt that this actually works.
            ss << "#!/usr/bin/env python" << std::endl
               << "from operator import *" << std::endl
               << std::endl
               << "#These functions allow multiple args instead of lists." << std::endl
               << "def ors(*args):" << std::endl
               << "    return any(args)" << std::endl
               << std::endl
               << "def ands(*args):" << std::endl
               << "    return all(args)" << std::endl << std::endl
               << "def moses_eval(i):" << std::endl << "    return ";
        }

        const scored_combo_tree_set& tree_set = metapop.best_candidates();

        // search for the top score... we need this, if printing only
        // the high-scorers.
        score_t best_score = very_worst_score;
        if (output_only_best) {
            for (const scored_combo_tree& sct : tree_set) {
                score_t sc = sct.get_score();
                if (best_score < sc) best_score = sc;
            }
        }

        int cnt = 0;
        for (const scored_combo_tree& sct : tree_set) {
            if (result_count < ++cnt) break;
            if (best_score <= sct.get_score()) {
                if (output_python) {
                    ss << "#score: " << sct.get_score() << std::endl;
                    ostream_combo_tree (ss, sct.get_tree(), combo::fmt::python);
                } else {
                    ss << sct.get_score() << " "
                       << sct.get_weight() << " "
                       << sct.get_tree();
                    if (output_score)
                       ss << " " << sct.get_composite_score();
                    if (output_bscore)
                       ss << " " << sct.get_bscore();
                    ss << std::endl;
                }
            }
        }
    
        if (output_eval_number)
            ss << number_of_evals_str << ": " << stats.n_evals << std::endl;;

        string res = (output_with_labels && !ilabels.empty()?
                      ph2l(ss.str(), ilabels) : ss.str());
        if (output_file.empty())
            std::cout << res;
        else {
            ofstream of(output_file.c_str());
            of << res;
            of.close();
        }
    
        // Log the best candidate
        stringstream ssb;
        metapop.ostream_metapop(ssb, 1);
        string resb = (output_with_labels && !ilabels.empty()?
                       ph2l(ssb.str(), ilabels) : ssb.str());
        if (resb.empty())
            logger().warn("No candidate is good enough to be returned. Yeah that's bad!");
        else
            logger().info("Best candidates:\n%s", res.c_str());
    
    #ifdef GATHER_STATS
        dex._optimize.hiscore /= dex._optimize.hicount;
        dex._optimize.num_improved /= dex._optimize.count_improved;
        logger().info() << "Avg number of improved scores = "
                        << dex._optimize.num_improved;
        logger().info() << "Avg improved as percentage= "
                        << 100.0 * dex._optimize.num_improved /
                               dex._optimize.scores.size();

        for (unsigned i=0; i< dex._optimize.scores.size(); i++) {
            dex._optimize.scores[i] /= dex._optimize.counts[i];
            logger().info() << "Avg Scores: "
                << i << "\t"
                << dex._optimize.hiscore << "\t"
                << dex._optimize.counts[i] << "\t"
                << dex._optimize.scores[i];
        }
    #endif
    }

private:
    long result_count;
    bool output_score;
    bool output_penalty;
    bool output_bscore;
    bool output_only_best;
    bool output_eval_number;
    bool output_with_labels;
public:
    vector<string> ilabels;
private:
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
                             behave_cscore& sc,
                             const optim_parameters& opt_params,
                             const hc_parameters& hc_params,
                             const deme_parameters& deme_params,
                             const metapop_parameters& meta_params,
                             const moses_parameters& moses_params,
                             Printer& printer)
{
    moses_statistics stats;
    optimizer_base* optimizer = nullptr;

    if (opt_params.opt_algo == hc) { // exhaustive neighborhood search
        optimizer = new hill_climbing(opt_params, hc_params);
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

    // This seems kind of cheesy ... shouldn't the exemplars 
    // already be simplified, by now?
    std::vector<combo_tree> simple_bases;
    for (const combo_tree& xmplr: bases) {
        combo_tree siba(xmplr);
        si_ca(siba);
        simple_bases.push_back(siba);
    }

    deme_expander dex(tt, si_ca, si_kb, sc, *optimizer, deme_params);
    metapopulation metapop(simple_bases, sc, meta_params);

    run_moses(metapop, dex, moses_params, stats);
    printer(metapop, dex, stats);
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
                           behave_cscore& c_scorer,
                           optim_parameters opt_params,
                           hc_parameters hc_params,
                           const deme_parameters& deme_params,
                           const metapop_parameters& meta_params,
                           moses_parameters moses_params,
                           Printer& printer)
{
    // Update terminate_if_gte and max_score criteria. An explicit
    // user-specified max score always over-rides the inferred score.
    score_t target_score = c_scorer.best_possible_score();
    if (very_best_score != moses_params.max_score) {
        target_score = moses_params.max_score;
        logger().info("Target score = %g", target_score);
    } else {
        logger().info("Inferred target score = %g", target_score);
    }

    // negative min_improv is interpreted as percentage of
    // improvement, if so then don't substract anything, since in that
    // scenario the absolute min improvent can be arbitrarily small
    score_t actual_min_improv = std::max(c_scorer.min_improv(), (score_t)0);
    target_score -= actual_min_improv;
    logger().info("Subtract %g (minimum significant improvement) "
                  "from the target score to deal with float imprecision = %g",
                  actual_min_improv, target_score);

    opt_params.terminate_if_gte = target_score;
    moses_params.max_score = target_score;

    // update minimum score improvement
    opt_params.set_min_score_improv(c_scorer.min_improv());

    metapop_moses_results_b(bases, type_sig, si_ca, si_kb,
                                c_scorer,
                                opt_params, hc_params,
                                deme_params, meta_params,
                                moses_params, printer);
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_MAIN_H
