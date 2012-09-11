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
static const string inc="inc"; // incremental_selection (see
                               // feature_optimization.h)
static const string mmi="mmi"; // max_mi_selection (see
                               // feature_max_mi.h)

// Feature selection scorers
static const string mi="mi";    // Mutual Information (see feature_scorer.h)
static const string pre="pre";  // Precision (see
                                // opencog/learning/moses/moses/scoring.h)
    
// parameters of feature-selection, see desc.add_options() in
// feature-selection.cc for their meaning
struct feature_selection_parameters
{
    std::string algorithm;
    std::string scorer;
    unsigned int max_evals;
    std::string input_file;
    // std::vector<std::string> ignore_features_str;
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
    float pre_penalty;
    float pre_min_activation;
    float pre_max_activation;
    bool pre_positive;
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
feature_set optimize_deme_select_features(const field_set& fields,
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
                          const field_set& fields,
                          const std::vector<std::string>& labels);

// run feature selection given a moses optimizer and a scorer, create
// a deme a define the wrap the scorer for that deme. Possibly add
// cache as well.
template<typename Optimize, typename Scorer>
feature_set create_deme_select_features(const CTable& ctable,
                                        Optimize& optimize,
                                        const Scorer& scorer,
                                        const feature_selection_parameters& fs_params)
{
    arity_t arity = ctable.get_arity();
    field_set fields(field_set::disc_spec(2), arity);
    instance_set<composite_score> deme(fields);
    // determine the initial instance given the initial feature set
    instance init_inst = initial_instance(fs_params, fields, ctable.get_labels());
    // define moses based scorer
    typedef deme_based_scorer<Scorer> DBScorer;
    DBScorer db_sc(scorer, fields);
    // possibly wrap in a cache
    if(fs_params.hc_cache_size > 0) {
        typedef prr_cache_threaded<DBScorer> ScorerCache;
        ScorerCache sc_cache(fs_params.hc_cache_size, db_sc);
        feature_set selected_features =
            optimize_deme_select_features(fields, deme, init_inst, optimize,
                                          sc_cache, fs_params);
        // Logger
        logger().info("Number of cache misses = %u", sc_cache.get_misses());
        // ~Logger
        return selected_features;
    } else {
        return optimize_deme_select_features(fields, deme, init_inst, optimize,
                                             db_sc, fs_params);
    }
}

template<typename FeatureSet>
struct fs_scorer : public unary_function<FeatureSet, double> {
    fs_scorer(const CTable& ctable,
              const feature_selection_parameters& fs_params)
        : _ptr_mi_scorer(nullptr), _ptr_pre_scorer(nullptr) {
        if (fs_params.scorer == mi) { // mutual information
            _ptr_mi_scorer =
                new MICScorerCTable<FeatureSet>(ctable, fs_params.hc_confi);
        } else if (fs_params.scorer == pre) { // precision (see
            // opencog/learning/moses/moses/scoring.h)
            _ptr_pre_scorer =
                new pre_scorer<FeatureSet>(ctable,
                                           fs_params.pre_penalty,
                                           fs_params.pre_min_activation,
                                           fs_params.pre_max_activation,
                                           fs_params.pre_positive);
        }
    }
    ~fs_scorer() {
        delete _ptr_mi_scorer;
        delete _ptr_pre_scorer;
    }
    double operator()(const FeatureSet& fs) const {
        if (_ptr_mi_scorer)
            return _ptr_mi_scorer->operator()(fs);
        else if (_ptr_pre_scorer)
            return _ptr_pre_scorer->operator()(fs);
        else {
            OC_ASSERT(false);
            return 0.0;
        }
    }
protected:
    MICScorerCTable<FeatureSet>* _ptr_mi_scorer;
    pre_scorer<FeatureSet>* _ptr_pre_scorer;
};
    
// run feature selection given a moses optimizer
template<typename Optimize>
feature_set moses_select_features(const CTable& ctable,
                                  Optimize& optimize,
                                  const feature_selection_parameters& fs_params) {
    fs_scorer<set<arity_t> > fs_sc(ctable, fs_params);
    return create_deme_select_features(ctable, optimize, fs_sc, fs_params);
}

feature_set incremental_select_features(const CTable& ctable,
                                        const feature_selection_parameters& fs_params);

feature_set max_mi_select_features(const CTable& ctable,
                                   const feature_selection_parameters& fs_params);

/**
 * Select the features according to the method described in fs_params.
 */
feature_set select_features(const CTable& ctable,
                            const feature_selection_parameters& fs_params);

/**
 * Like above but using Table instead of CTable
 */
feature_set select_features(const Table& table,
                            const feature_selection_parameters& fs_params);

/**
 * Select the features, and output the table with the selected features
 */
void feature_selection(const Table& table,
                       const feature_selection_parameters& fs_params);

} // ~namespace opencog
    
#endif // _OPENCOG_FEATURE-SELECTION_H
