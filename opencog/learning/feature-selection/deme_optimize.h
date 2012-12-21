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
#include <opencog/comboreduct/table/table.h>

#include "../feature_scorer.h"
#include "../stochastic_max_dependency.h"
#include "../incremental.h"
#include "../moses_based_scorer.h"

namespace opencog {

using namespace moses;
using namespace combo;

// Feature selection algorithms
static const string inc="inc"; // incremental_selection (see
                               // feature_optimization.h)
static const string smd="smd"; // stochastic_max_dependency (see
                               // stochastic_max_dependency.h)

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
    std::string input_file;
    std::string target_feature_str;
    std::vector<std::string> ignore_features_str;
    std::vector<std::string> force_features_str;
    std::vector<std::string> initial_features;
    std::string output_file;
    unsigned target_size;
    double threshold;
    unsigned jobs;

    // incremental selection paramters
    double inc_target_size_epsilon;
    double inc_red_intensity;
    unsigned inc_interaction_terms;

    // hill-climbing parameters
    // actually, these are generic for all moses optimizers,
    // not just hill-climbing...
    unsigned int hc_max_evals;
    time_t max_time;
    double hc_max_score;
    double hc_confi; //  confidence intensity
    unsigned long hc_cache_size;
    double hc_fraction_of_remaining;

    // precision scorer parameters
    float pre_penalty;
    float pre_min_activation;
    float pre_max_activation;
    bool pre_positive;

    // stochastic max-dependency parameters
    unsigned smd_top_size;
};

typedef std::set<arity_t> feature_set;

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
    unsigned evals = optimize(deme, init_inst, scorer, 
                             fs_params.hc_max_evals, fs_params.max_time,
                             &ae);

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

/**
 * Convert the initial features into feature_set (set of indices)
 */
feature_set initial_features(const vector<string>& labels,
                             const feature_selection_parameters& fs_params);

/** For the MOSES algo, generate the intial instance */
instance initial_instance(const feature_selection_parameters& fs_params,
                          const field_set& fields,
                          const std::vector<std::string>& labels);

// A wrapper, simply so that optimizer gets the iscorer_base base class.
// The only reason for this wrapper is that both iscorer_base, and
// prr_cache_threaded both define operator(), and I need it to be clear
// which operator() needs to be called. Otherwise, I guess multiple
// inheritance would have worked!?
template<typename DBScorer>
struct iscorer_cache : public iscorer_base
{
    iscorer_cache(size_t n, const DBScorer& sc) :
        _cache(n, sc) {}

    result_type operator()(const argument_type& x) const
    {
        return _cache.operator()(x);
    }
    unsigned get_misses() const { return _cache.get_misses(); }
    unsigned get_hits() const { return _cache.get_hits(); }
    prr_cache_threaded<DBScorer> _cache;
};

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
    instance init_inst = initial_instance(fs_params, fields, ctable.get_input_labels());
    // define moses based scorer
    typedef deme_based_scorer<Scorer> DBScorer;
    DBScorer db_sc(scorer, fields);
    // possibly wrap in a cache
    if(fs_params.hc_cache_size > 0) {
        // typedef prr_cache_threaded<DBScorer> ScorerCache;
        typedef iscorer_cache<DBScorer> ScorerCache;
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
struct fs_scorer : public unary_function<FeatureSet, double>
{
    fs_scorer(const CTable& ctable,
              const feature_selection_parameters& fs_params)
        : _ptr_mi_scorer(nullptr), _ptr_pre_scorer(nullptr)
    {
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
    double operator()(const FeatureSet& fs) const
    {
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

feature_set smd_select_features(const CTable& ctable,
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
