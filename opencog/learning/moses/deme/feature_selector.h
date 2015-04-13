/*
 * opencog/learning/moses/moses/feature_selector.h
 *
 * Copyright (C) 2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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

#ifndef _OPENCOG_FEATURE_SELECTOR_H
#define _OPENCOG_FEATURE_SELECTOR_H

#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/learning/feature-selection/main/feature-selection.h>

namespace opencog {
namespace moses {

/**
 * Parameters of feature selector
*/
struct feature_selector_parameters
{
    feature_selector_parameters() :
        increase_target_size(true),
        ignore_xmplr_features(true),
        restrict_incorrect(true),
        restrict_true(false),
        init_xmplr_features(false),
        xmplr_as_feature(false),
        subsampling_ratio(1.0),
        subsampling_by_time(false),
        n_demes(1),
        diversity_pressure(0.0),
        diversity_cap(0),
        diversity_interaction(0)
    {}

    feature_selection_parameters fs_params;

    /**
     * The target_size is increase at each generation by the number of
     * features in the exemplar.
     */
    bool increase_target_size;

    /**
     * Ignore the features present in the exemplar when doing feature
     * selection.
     */
    bool ignore_xmplr_features;

    /**
     * Ignore features when doing feature selection.
     */
    std::set<arity_t> ignored_features;

    /**
     * Only consider the rows when the combo tree is predicting an
     * incorrect result.
    */
    bool restrict_incorrect;

    /**
     * Only consider the rows when the combo tree outputs true (this
     * is useful for precision fitness function). That option is
     * mutually exclusive with restrict_incorrect.
     */
    bool restrict_true;

    /**
     * Prune the exemplar of all not selected features
     */
    bool prune_xmplr;

    /**
     * Use the features in the exemplar as initial feature set
     */
    bool init_xmplr_features;

    /**
     * Use exemplar as feature, so feature selection can search good
     * features sets combined with the exemplar feature.
     */
    bool xmplr_as_feature;

    /**
     * Ratio size of the subsampled data set (this is used to
     * introduce some randomness in feature selection).
     */
    double subsampling_ratio;

    /**
     * If set to true then the subsampling_ratio concerns
     * timestamps. That is a subset of timestamps is selected (of size
     * subsampling_ratio * #timestamps) and all rows timestamped at
     * those timestamps are kept.
     */
    bool subsampling_by_time;

    /**
     * Number of feature sets to select out of feature selection and
     * demes to spawn.
     */
    unsigned n_demes;

    /**
     * When selecting several feature sets (n_demes > 1) a diversity
     * pressure can be applied before selecting the best feature sets.
     *
     * The penalized score of a feature set fs_i is:
     *
     * q(fs_i) - diversity_pressure * aggregate_{j=0}^{i-1}(mi(fs_i, fs_j))
     *
     * Where the feature sets fs_i are ordered by their penalized
     * scores. aggregate can be either generalized mean or sum, or
     * max.
     *
     * Alternatively if jaccard is enabled, then the calculation of
     * the penalized score of a feature set fs_i is:
     *
     * q(fs_i) - diversity_pressure * aggregate_{j=0}^{i-1}(J(fs_i, fs_j))
     */
    double diversity_pressure;

    /**
     * Set a cap regarding the population of feature sets. Right have
     * feature selection optimization, only diversity_cap found
     * feature sets are kept, or all if diversity_cap == 0.
     */
    size_t diversity_cap;

    /**
     * Number of interactions to consider when computing mi. A
     * negative value means all interactions.
     */
    int diversity_interaction;
    
     /**
     * If enabled then expensive multiple mi over feature set
     * activations is replaced by one cheap calculation of the Jaccard
     * index over feature sets.
     */
    bool diversity_jaccard;

    /**
     * Map between feature name and probability of being inserted in
     * the deme, regardless of whether it has been selected or not.
     */
    std::map<std::string,float> enforce_features;
};

// used for diversity ranking
typedef std::multimap<composite_score,
                      feature_set,
                      std::greater<composite_score>> csc_feature_set_pop;

/**
 * Struct in charge of selecting features maximize the amount of
 * information in combination with a candidate.
 */
struct feature_selector
{
    feature_selector(const combo::CTable& ctable,
                     const feature_selector_parameters& festor_params);

    feature_selector(const combo::Table& table,
                     const feature_selector_parameters& festor_params);

    /// Return feature set population that is good when combined with
    /// the exemplar tr.
    feature_set_pop operator()(const combo::combo_tree& xmplr);

    // Return a set of features randomly choosen given
    // enforce_features, a map from feature to probability of being
    // enforced
    feature_set sample_enforced_features() const;

    // Parameters
    feature_selector_parameters params;

    const combo::CTable& _ctable;
protected:
    /// Overwrite some parameters
    void preprocess_params(const combo::combo_tree& xmplr);

    /// Build ctable used for feature selection (possibly different
    /// than _ctable)
    combo::CTable build_fs_ctable(const combo::combo_tree& xmplr) const;

    /// Select top n feature sets
    feature_set_pop select_top_feature_sets(const feature_set_pop& fss) const;

    /// remove useless features (like the exemplar feature if any)
    void remove_useless_features(feature_set_pop& fss) const;

    // Rank feature sets, penalized by diversity
    csc_feature_set_pop rank_feature_sets(const feature_set_pop& fs_pop) const;

    void log_stats_top_feature_sets(const feature_set_pop& top_fs) const;

    // compute (or approximate) the mutual information between 2
    // feature sets.
    //
    // If params.diversity_interaction is negative then it returns the
    // mutual information between fs_l and fs_r.
    //
    // Otherwise it returns the average of mutual informations between
    // all subsets of fs_l and fs_r of size
    // params.diversity_interaction + 1
    double mi(const feature_set& fs_l, const feature_set& fs_r) const;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTOR_H
