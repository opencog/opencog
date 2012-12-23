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

#include "../scorers/mutual_info.h"
#include "../scorers/moses_optim.h"
#include "../scorers/moses_matrix.h"  // for pre_scorer


namespace opencog {

// using namespace moses;

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

    // stochastic max-dependency selection parameters
    unsigned smd_top_size;

    // hill-climbing selection parameters
    // actually, these are generic for all moses optimizers,
    // not just hill-climbing...
    unsigned int hc_max_evals;
    time_t max_time;
    double hc_max_score;
    unsigned long hc_cache_size;
    double hc_fraction_of_remaining;

    // MI scorer parameters
    double mi_confi; //  confidence intensity

    // precision scorer parameters
    float pre_penalty;
    float pre_min_activation;
    float pre_max_activation;
    bool pre_positive;

};

typedef std::set<arity_t> feature_set;

void write_results(const Table& table,
                   const feature_selection_parameters& fs_params);

/**
 * Convert the initial features into feature_set (set of indices)
 */
feature_set initial_features(const vector<string>& labels,
                             const feature_selection_parameters& fs_params);

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


template<typename FeatureSet>
struct fs_scorer : public unary_function<FeatureSet, double>
{
    fs_scorer(const CTable& ctable,
              const feature_selection_parameters& fs_params)
        : _ptr_mi_scorer(nullptr), _ptr_pre_scorer(nullptr)
    {
        if (fs_params.scorer == mi) { // mutual information
            _ptr_mi_scorer =
                new MICScorerCTable<FeatureSet>(ctable, fs_params.mi_confi);
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
