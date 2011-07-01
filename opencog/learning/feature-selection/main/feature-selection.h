/** feature-selection.h --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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


#ifndef _OPENCOG_FEATURE_SELECTION_H
#define _OPENCOG_FEATURE_SELECTION_H

#include <boost/assign/std/vector.hpp> // for 'operator+=()'

#include <opencog/learning/moses/eda/field_set.h>
#include <opencog/learning/moses/eda/instance_set.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/comboreduct/combo/table.h>

#include "../feature_scorer.h"
#include "../moses_based_scorer.h"

using namespace eda;
using namespace moses;
using namespace opencog::combo;
using namespace boost::assign; // bring 'operator+=()' into scope

// optimization algorithms
static const string un="un"; // univariate
static const string sa="sa"; // simulation annealing
static const string hc="hc"; // hillclimbing

static const string default_log_file_prefix = "feature-selection";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

// program option names and abbreviations
// for their meanings see options_description in moses-exec.cc
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> opt_algo_opt("opt-algo", "a");
static const pair<string, string> input_data_file_opt("input-file", "i");
static const pair<string, string> initial_feature_opt("initial-feature", "f");
static const pair<string, string> max_evals_opt("max-evals", "m");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "F");
static const pair<string, string> log_file_dep_opt_opt("log-file-dep-opt", "L");
static const pair<string, string> cache_size_opt("cache-size", "s");
static const pair<string, string> complexity_penalty_intensity_opt("complexity-penalty-intensity", "p");
static const pair<string, string> confidence_penalty_intensity_opt("confidence-penalty-intensity", "c");
static const pair<string, string> resources_opt("resources", "R");
static const pair<string, string> max_score_opt("max-score", "A");

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

// parameters of feature-selection, see desc.add_options() in
// feature-selection.cc for their meaning
struct feature_selection_parameters {
    std::string algorithm;
    unsigned int max_evals;
    std::string input_file;
    std::string output_file;
    std::vector<std::string> initial_features;
    unsigned long cache_size;
    double cpi; // complexity penalty intensity
    double confi; //  confidence intensity
    double resources; // resources of the learning algo that will take
                      // in input the feature set
    double max_score;
};

template<typename IT, typename OT, typename Optimize, typename Scorer>
void feature_selection(IT& it, const OT& ot,
                       const field_set& fields,
                       instance_set<composite_score>& deme,
                       eda::instance& init_inst,
                       Optimize& optimize, const Scorer& scorer,
                       const feature_selection_parameters& fs_params) {
    // optimize feature set
    unsigned ae; // actual number of evaluations to reached the best candidate
    unsigned evals = optimize(deme, init_inst, scorer, fs_params.max_evals, &ae);
    // get the best one
    std::sort(deme.begin(), deme.end(),
              std::greater<scored_instance<composite_score> >());
    eda::instance best_inst = *deme.begin_instances();
    composite_score best_score = *deme.begin_scores();
    // get the best feature set
    std::set<arity_t> best_fs = get_feature_set(fields, best_inst);
    // set the input table accordingly
    it.set_consider_args_from_zero(best_fs);
    // Logger
    {
        // log set of selected feature set
        stringstream ss;
        ss << "The following set of features has been selected: ";
        ostreamContainer(ss, it.get_considered_labels(), ",");
        logger().info(ss.str());
    }
    {
        // log its score
        stringstream ss;
        ss << "with composite score: " << best_score;
        logger().info(ss.str());
    }
    {
        // Log the actual number of evaluations
        logger().info("Total number of evaluations performed: %u", evals);
        logger().info("Actual number of evaluations to reach the best feature set: %u", ae);
    }
    // ~Logger
    // print the filtered table
    if(fs_params.output_file.empty())
        ostreamTable(std::cout, it, ot);
    else
        ostreamTable(fs_params.output_file, it, ot);
}

eda::instance initial_instance(const feature_selection_parameters& fs_params,
                               const field_set& fields) {
    eda::instance res(fields.packed_width());
    vector<std::string> labels = readInputLabels(fs_params.input_file);
    vector<std::string> vif; // valid initial features, used for logging
    foreach(const std::string& f, fs_params.initial_features) {
        size_t idx = std::distance(labels.begin(), find(labels, f));
        if(idx < labels.size()) { // feature found
            *(fields.begin_bits(res) + idx) = true;
            // for logging
            vif += f;
        }
        else // feature not found
            logger().warn("No such a feature #%s in file %s. It will be ignored as initial feature.", f.c_str(), fs_params.input_file.c_str());
    }
    // Logger
    if(vif.empty())
        logger().info("The search will start with the empty feature set");
    else {
        stringstream ss;
        ss << "The search will start with the following feature set: ";
        ostreamContainer(ss, vif, ",");
        logger().info(ss.str());
    }
    // ~Logger
    return res;
}

template<typename IT, typename OT, typename Optimize>
void feature_selection(IT& it, const OT& ot,
                       Optimize& optimize,
                       const feature_selection_parameters& fs_params) {
    arity_t arity = it.get_arity();
    field_set fields(field_set::disc_spec(2), arity);
    instance_set<composite_score> deme(fields);
    // determine the initial instance given the initial feature set
    eda::instance init_inst = initial_instance(fs_params, fields);
    // define feature set quality scorer
    typedef MICSScorer<IT, OT, set<arity_t> > FSScorer;
    FSScorer fs_sc(it, ot,
                   fs_params.cpi, fs_params.confi, fs_params.resources);
    typedef moses_based_scorer<FSScorer> MBScorer;
    MBScorer mb_sc(fs_sc, fields);
    // possibly wrap in a cache
    if(fs_params.cache_size > 0) {
        typedef prr_cache<MBScorer> ScorerCache;
        ScorerCache sc_cache(fs_params.cache_size, mb_sc);
        feature_selection(it, ot, fields, deme, init_inst, optimize,
                          sc_cache, fs_params);
        // Logger
        logger().info("Number of cache failures = %u",
                      sc_cache.get_failures());
        // ~Logger
    } else {
        feature_selection(it, ot, fields, deme, init_inst, optimize,
                          mb_sc, fs_params);
    }
}

template<typename IT, typename OT>
void feature_selection(IT& it, const OT& ot,
                       const feature_selection_parameters& fs_params,
                       RandGen& rng) {
    optim_parameters op_param(20, fs_params.max_score);
    if(fs_params.algorithm == un) {
        OC_ASSERT(false, "TODO");
    } else if(fs_params.algorithm == sa) {
        OC_ASSERT(false, "TODO");        
    } else if(fs_params.algorithm == hc) {
        hc_parameters hc_param(false); // do not terminate if improvement
        iterative_hillclimbing hc(rng, op_param, hc_param);
        feature_selection(it, ot, hc, fs_params);            
    }
}

#endif // _OPENCOG_FEATURE-SELECTION_H
