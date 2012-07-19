/**
 * main/feature-selection.h --- 
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

#include <boost/range/algorithm/sort.hpp>

#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/representation/field_set.h>
#include <opencog/learning/moses/representation/instance_set.h>
#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/comboreduct/combo/table.h>

#include "../feature_scorer.h"
#include "../feature_max_mi.h"
#include "../feature_optimization.h"
#include "../moses_based_scorer.h"

namespace opencog {

using namespace moses;
using namespace combo;

// Feature selection algorithms
static const string un="un"; // moses based univariate
static const string sa="sa"; // moses based simulation annealing
static const string hc="hc"; // moses based hillclimbing
static const string inc="inc"; // incremental_selection (see
                               // feature_optimization.h)
static const string mmi="mmi"; // max_mi_selection (see
                               // feature_max_mi.h)

// parameters of feature-selection, see desc.add_options() in
// feature-selection.cc for their meaning
struct feature_selection_parameters
{
    std::string algorithm;
    unsigned int max_evals;
    std::string input_file;
    int target_feature;
    std::vector<int> ignore_features;
    std::vector<std::string> force_features_str;
    std::string output_file;
    unsigned target_size;
    double threshold;
    unsigned jobs;
    double inc_target_size_epsilon;
    double inc_red_intensity;
    unsigned inc_interaction_terms;
    double hc_max_score;
    double hc_confi; //  confidence intensity
    unsigned long hc_cache_size;
    double hc_fraction_of_remaining;
    std::vector<std::string> hc_initial_features;
};

typedef std::set<arity_t> feature_set;

/**
 * Add forced features to table.
 *
 * @todo update type_tree (if ever needed)
 */
Table add_force_features(const Table& table,
                         const feature_selection_parameters& fs_params);

/**
 * update fs_params.target_feature so that it keeps the same relative
 * position with the selected features.
 */
int update_target_feature(const Table& table,
                          const feature_selection_parameters& fs_params);

void write_results(const Table& table,
                   const feature_selection_parameters& fs_params);

template<typename Optimize, typename Scorer>
feature_set moses_select_features(const field_set& fields,
                                  instance_set<composite_score>& deme,
                                  instance& init_inst,
                                  Optimize& optimize, const Scorer& scorer,
                                  const feature_selection_parameters& fs_params)
{
    // optimize feature set
    unsigned ae; // actual number of evaluations to reached the best candidate
    unsigned evals = optimize(deme, init_inst, scorer, fs_params.max_evals, &ae);

    // get the best one
    boost::sort(deme, std::greater<scored_instance<composite_score> >());
    instance best_inst = evals > 0 ? *deme.begin_instances() : init_inst;
    composite_score best_score =
        evals > 0 ? *deme.begin_scores() : worst_composite_score;

    // get the best feature set
    feature_set selected_features = get_feature_set(fields, best_inst);
    // Logger
    {
        // log its score
        stringstream ss;
        ss << "Selected feature set has composite score: ";
        if (evals > 0)
            ss << best_score;
        else
            ss << "Unknown";
        logger().info(ss.str());
    }
    {
        // Log the actual number of evaluations
        logger().info("Total number of evaluations performed: %u", evals);
        logger().info("Actual number of evaluations to reach the best feature set: %u", ae);
    }
    // ~Logger
    return selected_features;
}

/** For the MOSES algo, generate the intial instance */
instance initial_instance(const feature_selection_parameters& fs_params,
                          const field_set& fields);

// run feature selection given an moses optimizer
template<typename Optimize>
feature_set moses_select_features(CTable& ctable,
                                  Optimize& optimize,
                                  const feature_selection_parameters& fs_params) {
    arity_t arity = ctable.get_arity();
    field_set fields(field_set::disc_spec(2), arity);
    instance_set<composite_score> deme(fields);
    // determine the initial instance given the initial feature set
    instance init_inst = initial_instance(fs_params, fields);
    // define feature set quality scorer
    typedef MICScorerCTable<set<arity_t> > FSScorer;
    FSScorer fs_sc(ctable, fs_params.hc_confi);
    typedef moses_based_scorer<FSScorer> MBScorer;
    MBScorer mb_sc(fs_sc, fields);
    // possibly wrap in a cache
    if(fs_params.hc_cache_size > 0) {
        typedef prr_cache_threaded<MBScorer> ScorerCache;
        ScorerCache sc_cache(fs_params.hc_cache_size, mb_sc);
        feature_set selected_features =
            moses_select_features(fields, deme, init_inst, optimize,
                                  sc_cache, fs_params);
        // Logger
        logger().info("Number of cache failures = %u", sc_cache.get_failures());
        // ~Logger
        return selected_features;
    } else {
        return moses_select_features(fields, deme, init_inst, optimize,
                                     mb_sc, fs_params);
    }
}

feature_set incremental_select_features(CTable& ctable,
                                        const feature_selection_parameters& fs_params);

feature_set max_mi_select_features(CTable& ctable,
                                   const feature_selection_parameters& fs_params);

/**
 * Select the features according to the method described in fs_params.
 */
feature_set select_features(CTable& ctable,
                            const feature_selection_parameters& fs_params);

/**
 * Like above but using Table instead of CTable
 */
feature_set select_features(Table& table,
                            const feature_selection_parameters& fs_params);

/**
 * Select the features, and output the table with the selected features
 */
void feature_selection(Table& table,
                       const feature_selection_parameters& fs_params);

} // ~namespace opencog
    
#endif // _OPENCOG_FEATURE-SELECTION_H
