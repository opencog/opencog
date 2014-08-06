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
#include "../scoring/ss_bscore.h"
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
                    bool _output_cscore,
                    bool _output_bscore,
                    bool _output_only_best,
                    bool _output_ensemble,
                    bool _output_eval_number,
                    bool _output_with_labels,
                    bool _output_demeID,
                    const std::vector<std::string>& _ilabels,
                    const std::string& _output_file,
                    bool _output_python,
                    bool _is_mpi) :
        result_count(_result_count),
        output_score(_output_score),
        output_cscore(_output_cscore),
        output_bscore(_output_bscore),
        output_only_best(_output_only_best),
        output_ensemble(_output_ensemble),
        output_eval_number(_output_eval_number),
        output_with_labels(_output_with_labels),
        output_demeID(_output_demeID),
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
        // A number of external tools read the printed results, and
        // parse them. The floating-point scores must be printed with
        // reasonable accuracy, else these tools fail.
        ss << std::setprecision(moses::io_score_precision);
        if (output_ensemble) {
            const scored_combo_tree_set& tree_set = metapop.get_ensemble().get_ensemble();
            if (output_python) {
                // Python boilerplate
                ss << "#!/usr/bin/env python\n"
                   << "from operator import *\n\n"
                   << "#These functions allow multiple args instead of lists.\n"
                   << "def ors(*args):\n"
                   << "    return any(args)\n\n"
                   << "def ands(*args):\n"
                   << "    return all(args)\n\n"
                   << "#score: " << metapop.best_score() << std::endl
                   << "def moses_eval(i):\n"
                   << "    sum = 0.0 \\\n";
                // XXX this is close to what we want but maybe borken.
                // FIXME untested.
                for (const scored_combo_tree& sct : tree_set)
                    ss << "      + " << sct.get_weight()
                       << " * " << sct.get_tree() << "\\\n";
                ss << "\n    return (0.0 < val)\n";
            } else {

                // For ensembles, output as usual: score followed by tree
                const ensemble& ensm(metapop.get_ensemble());
                ss << ensm.flat_score() << " "
                   << ensm.get_weighted_tree();

                // if (output_bscore)
                //    ss << " " <<
                //    metapop._cscorer.get_bscore(metapop.get_ensemble().get_weighted_tree());
                ss << std::endl;
            }

        } else {
            // scored_combo_tree_ptr_set keeps the trees in penalized-
            // score-sorted order. We want this, so that we can print
            // them out in this order.  Note, however, the printed scores
            // are the raw scores, not the penalized scores, so the
            // results can seem out-of-order.
            scored_combo_tree_ptr_set tree_set;
            if (output_only_best) {
                for (const scored_combo_tree& sct : metapop.best_candidates())
                   tree_set.insert(new scored_combo_tree(sct));
            } else {
                tree_set = metapop.get_trees();
            }

            int cnt = 0;
            for (const scored_combo_tree& sct : tree_set) {
                if (result_count < ++cnt) break;
                if (output_python) {
                    // Python boilerplate
                    ss << "#!/usr/bin/env python\n"
                       << "from operator import *\n\n"
                       << "#These functions allow multiple args instead of lists.\n"
                       << "def ors(*args):\n"
                       << "    return any(args)\n\n"
                       << "def ands(*args):\n"
                       << "    return all(args)\n\n"
                       << "#score: " << sct.get_score() << std::endl
                       << "def moses_eval(i):\n"
                       << "    return ";
                    ostream_combo_tree (ss, sct.get_tree(), combo::fmt::python);
                } else {
                    ostream_scored_combo_tree(ss, sct, output_score,
                                              output_cscore, output_demeID,
                                              output_bscore);
                }
            }
        }

        if (output_eval_number)
            ss << number_of_evals_str << ": " << stats.n_evals << std::endl;;

        // OK, this is kind-of cheesy, but it goes and replaces $1 $2 $3
        // etc with the strings from the ilabels vector. Its just a pure
        // string search-n-replace.
        string res = (output_with_labels && !ilabels.empty()?
                      ph2l(ss.str(), ilabels) : ss.str());
        if (output_file.empty())
            std::cout << res;
        else {
            ofstream of(output_file.c_str());
            of << res;
            of.close();
        }

        // Also log the thing, if logging is enabled.
        if (logger().isInfoEnabled()) {
            if (output_ensemble) {
                stringstream ssb;
                for (const auto& cand : metapop.get_ensemble().get_ensemble()) {
                    ssb << cand.get_weight() << " " << cand.get_tree();
                }

                string resb = (output_with_labels && !ilabels.empty()?
                               ph2l(ssb.str(), ilabels) : ssb.str());
                if (resb.empty())
                    logger().warn("Ensemble was empty!");
                else
                    logger().info("Final ensemble, consisting of %d members:\n%s",
                        metapop.get_ensemble().get_ensemble().size(), res.c_str());
            } else {
                // Log the single best candidate
                stringstream ssb;
                metapop.ostream_metapop(ssb, 1);
                string resb = (output_with_labels && !ilabels.empty()?
                               ph2l(ssb.str(), ilabels) : ssb.str());
                if (resb.empty())
                    logger().warn("No candidate is good enough to be returned. Yeah that's bad!");
                else
                    logger().info("Best candidates:\n%s", res.c_str());
            }
        }

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
    bool output_cscore;
    bool output_bscore;
    bool output_only_best;
    bool output_ensemble;
    bool output_eval_number;
    bool output_with_labels;
    bool output_demeID;
public:
    std::vector<std::string> ilabels;
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
                             const subsample_deme_filter_parameters& filter_params,
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

    deme_expander dex(tt, si_ca, si_kb, sc, *optimizer, deme_params, filter_params);
    metapopulation metapop(simple_bases, sc, meta_params, filter_params);

    run_moses(metapop, dex, moses_params, stats);
    printer(metapop, dex, stats);
    delete optimizer;
}

/**
 * Adjust the termination criteria.
 */
void adjust_termination_criteria(const behave_cscore& sc,
                                 optim_parameters& opt_params,
                                 moses_parameters& moses_params);

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
                           const subsample_deme_filter_parameters& filter_params,
                           const metapop_parameters& meta_params,
                           moses_parameters moses_params,
                           Printer& printer)
{
    adjust_termination_criteria(c_scorer, opt_params, moses_params);

    if (filter_params.n_subsample_fitnesses > 1
        and filter_params.low_dev_pressure > 0.0)
    {
        // Enable SS-fitness
        const bscore_base& bscorer = c_scorer.get_bscorer();
        ss_bscore ss_bscorer(bscorer,
                             filter_params.n_subsample_fitnesses,
                             filter_params.low_dev_pressure,
                             filter_params.by_time);
        behave_cscore ss_cscorer(ss_bscorer);
        metapop_moses_results_b(bases, type_sig, si_ca, si_kb,
                                ss_cscorer,
                                opt_params, hc_params,
                                deme_params, filter_params, meta_params,
                                moses_params, printer);
    } else {
        metapop_moses_results_b(bases, type_sig, si_ca, si_kb,
                                c_scorer,
                                opt_params, hc_params,
                                deme_params, filter_params, meta_params,
                                moses_params, printer);
    }
}

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_MAIN_H
