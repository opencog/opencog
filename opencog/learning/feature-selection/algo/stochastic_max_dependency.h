/** stochastic_max_dependency.h ---
 *
 * Copyright (C) 2010,2012 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Authors: Nil Geisweiller <nilg@laptop>
 *          Linas Vepstas <linasvepstas@gmail.com>
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


#ifndef _OPENCOG_FEATURE_MAX_MI_ALGO_H
#define _OPENCOG_FEATURE_MAX_MI_ALGO_H

#include <functional>

#include <opencog/util/numeric.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/functional.h>
#include <opencog/util/oc_omp.h>

#include "../main/feature-selection.h" // needed for feature_set, feature_selection_parameters

namespace opencog {

feature_set smd_select_features(const CTable& ctable,
                                const feature_selection_parameters& fs_params);

/**
 * This algo was was originally written to maximize the mutual
 * information between a target variable, and a collection of
 * input features. It has since been re-written to use any scoring
 * system, and not just mutual information.  This is a basic algorithm,
 * described in textbooks on machine learning.
 *
 * This algo resembles a stochastic version of the algorithm coded
 * in the Section 2.1 of the paper entitled "Feature Selection Based
 * on Mutual Information: Criteria of Max-dependency, Max-relevance,
 * and Min-redundancy." by H Peng, 2005.  He calls it "max-dependency",
 * so we'll call this stochastic_max_dependency.
 *
 * Returns a set S of features using the following algo:
 *
 * 0) set<FeatureSet> tops = empty set
 * 1) For each FeatureSet fs in tops:
 * 2)     Add one feature, compute new score. Repeat 1)
 * 3) Discard all but top_size of highest scorers
 * 4) tops = the highest scorers.
 * 5) If score hasn't improved by threshold,
 *    then skip next step.
 * 6) Repeat, until tops holds FeatureSets of at most 
 *    'num_features' features.
 * 7) return highest scorer from tops.
 *
 * @param features       The initial set of features to be selected from
 * @param scorer         The function to score a set of features.
 * @param num_features   The maximum number of features to return.
 * @param threshold      Minimum desired improvement in mutual
 *                       information from each addition of a feature
 *                       to the feature set.  If improvement is not seen
 *                       then search is halted.
 * @param top_size       Number of top-correlated feature sets to explore.
 *
 * @return               The set of selected features
 */
template<typename Scorer, typename FeatureSet>
FeatureSet stochastic_max_dependency_selection(const FeatureSet& features,
                                               const FeatureSet& init_features,
                                               const Scorer& scorer,
                                               unsigned num_features,
                                               double threshold = 0.0,
                                               unsigned top_size = 100)
{
    if (logger().isDebugEnabled()) {
        logger().debug() << "Call stochastic_max_dependency_selection(num_features="
                         << num_features
                         << ", threshold=" << threshold
                         << ", top_size=" << top_size
                         <<")";
    }

    typedef typename FeatureSet::value_type feature_id;
    typedef std::map<double, FeatureSet> ranks_t;
    typedef boost::shared_mutex shared_mutex;
    typedef boost::shared_lock<shared_mutex> shared_lock;
    typedef boost::unique_lock<shared_mutex> unique_lock;
    shared_mutex mutex;

    // Start with the initial feature set
    double init_sc = scorer(init_features);
    std::vector<std::pair<double, FeatureSet>> tops{{init_sc, init_features}};
    double previous_high_score = init_sc;

    if (features.size() < num_features)
        num_features = features.size();

    // Randomize order, so that different redundant features
    // get a chance to show up in the final list. Copy set to
    // vector, then shuffle the vector.
    // LOL: I don't think this shuffling changes anything!!!
    // ?? Of course it does; just print out the final list and compare!
    // Think about it: which redundant feautres are discarded, and
    // which ones are kept?  The first one encountered is kept, the
    // rest are discarded.  Shuffling just changes which one is the
    // first one found.  Oh, wait, no. The aglorithm has been changed,
    // its different than what it used to be. So, right, shuffling no
    // longer has an effect... WTF.  This can't be correct...
    std::vector<feature_id> shuffle(features.begin(), features.end());
    auto shr = [&](ptrdiff_t i) { return randGen().randint(i); };
    random_shuffle(shuffle.begin(), shuffle.end(), shr);

    // Repeat, until we've gotton the highest-ranked FeatureSet
    // that has at most 'num_features' in it.
    for (unsigned i = init_features.size() + 1; i <= num_features; ++i) {

        ranks_t ranks;

        for (const auto& pr : tops) {
            const FeatureSet &fs = pr.second;
            // Add one feature at a time to fs, and score the result.
            // Rank the result.
            OMP_ALGO::for_each(shuffle.cbegin(), shuffle.cend(),
                               [&](feature_id fid) {
                                   if (fs.end() == fs.find(fid)) {
                                       
                                       // define new feature set
                                       FeatureSet prod = fs;
                                       prod.insert(fid);
                                   
                                       // score it
                                       double sc = scorer(prod);
                                   
                                       // insert it in ranks
                                       std::pair<double, FeatureSet> pdf(sc, prod);
                                       unique_lock lock(mutex);
                                       ranks.insert(pdf);
                                   }
                               });
        }

        // Discard all but the highest scorers.  When done, 'tops'
        // will hold FeatureSets with exactly 'i' elts each.
        tops.clear();
        auto rb = ranks.rbegin(),
            re = std::next(rb, std::min(top_size, (unsigned)ranks.size()));
        tops.insert(tops.begin(), rb, re);

        OC_ASSERT (!ranks.empty(), "Fatal Error: no ranked feature sets");

        // Get the highest score found.  If these are not getting better,
        // then stop looking for new features.
        double high_score = ranks.rbegin()->first;
        logger().debug("SMD: featureset size=%d highest score=%f", i, high_score);
        if (high_score - previous_high_score < threshold) {
            logger().debug("SMD: terminate, no improvment in score");
            break;
        }

        // Record the highest score found.
        previous_high_score = high_score;
    }

    // If we did this correctly, then the highest scorer is at the
    // end of the set.  Return that.
    OC_ASSERT(!tops.empty(), "top is empty, there must be a bug");
    FeatureSet res = tops.front().second;

    if (logger().isDebugEnabled()) {
        std::stringstream ss;
        ss << "Exit stochastic_max_dependency_selection(), selected: ";
        ostreamContainer(ss, res);
        ss << " Score = " << scorer(res);
        logger().debug() << ss.str();
    }
    return res;
}

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_MAX_MI_ALGO_H
