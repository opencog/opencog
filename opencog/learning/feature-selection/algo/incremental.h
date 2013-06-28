/** 
 * incremental.h ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@laptop>
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


#ifndef _OPENCOG_FEATURE_SELECTION_INCREMENTAL_ALGO_H
#define _OPENCOG_FEATURE_SELECTION_INCREMENTAL_ALGO_H

#include <functional>

#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/algorithm/max_element.hpp>

#include <opencog/util/algorithm.h>
#include <opencog/util/functional.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/numeric.h>
#include <opencog/util/oc_omp.h>

#include "../main/feature-selection.h"  // needed for feature_set, feature_selection_parameters

namespace opencog {

feature_set_pop incremental_select_feature_sets(const CTable& ctable,
                                                const feature_selection_parameters& fs_params);

/**
 * Returns a set S of features following the algo:
 * 0) res = empty set
 * 1.a) Select all relevant features (that score above threshold), called rel
 * 1.b) Select all redundant features among rel, called red
 * 1.c) res += rel - red
 * 2) remove rel from the initial set features, call this tf
 * 3.a) select all pairs of relevant features from tf, called rel
 * 3.b) select all redundant features among rel, called red
 * 4) follow the same pattern with triplets, etc, until max_interaction_terms.
 * 5) return res
 *
 * @param features       The initial set of features to be selected from
 * @param scorer         The function to score a set of features, it is
 *                       assumed that the codomain of the scorer is [0,1],
 *                       0 for the lowest score and 1 for the higest score.
 * @param threshold      The threshold to select a set of feature, in [0,1]
 * @param max_interaction_terms The maximum size of each feature set tested in the scorer
 * @param red_threshold  If > 0.0 it modulates the intensity of the
 *                       threshold of redundant_features(), precisely
 *                       red_threshold * threshold
 *                       Otherwise redundant features are ignored.
 *
 * @return               The set of selected features
 *
 * @note                 It is strongly suggested to wrap a cache around the
 *                       Scorer because the algorithm may do redundant
 *                       evaluations. Or you may use directly
 *                       cached_incremental_selection defined below
 */
template<typename Scorer, typename FeatureSet>
FeatureSet incremental_selection(const FeatureSet& features,
                                 const Scorer& scorer,
                                 double threshold,
                                 unsigned max_interaction_terms = 1,
                                 double red_threshold = -1.0)
{
    FeatureSet rel; // set of relevant features for a given iteration
    FeatureSet res; // set of relevant non-redundant features to return

    typedef boost::shared_mutex shared_mutex;
    typedef boost::unique_lock<shared_mutex> unique_lock;
    shared_mutex mutex;

#if DEBUG
    for (unsigned i = 1; i <= max_interaction_terms; ++i) {
        std::set<FeatureSet> ps = powerset(features, i, true);
        typename std::set<FeatureSet>::const_iterator psit;
        for (const FeatureSet& fs : ps) {
            printContainer(fs, ", ", "fs=[", "]");
            std::cout << "\t" << scorer(fs) << std::endl;
        }
        std::cout << "============================" << std::endl;
    }
#endif

    for (unsigned i = 1; i <= max_interaction_terms; ++i) {

        struct timeval start;
        gettimeofday(&start, NULL);

        // Define the set of set of features to test for relevancy
        FeatureSet tf = opencog::set_difference(features, rel);
        std::set<FeatureSet> fss = powerset(tf, i, true);

        logger().debug("Iteration %d feature set size=%d powerset size=%d",
            i, tf.size(), fss.size());

        // Add the set of relevant features for that iteration in rel
        rel.clear();
        auto fss_view = random_access_view(fss);
        auto filter_relevant = [&](const FeatureSet* fs) {
            if (scorer(*fs) > threshold) {
                unique_lock lock(mutex);
                /// @todo this lock can be more granular
                rel.insert(fs->begin(), fs->end());
            }};
        OMP_ALGO::for_each(fss_view.begin(), fss_view.end(), filter_relevant);
        logger().debug("Iteration %d relevant features=%d",
                       i, rel.size());

        if (0.0 < red_threshold) {
            // Define the set of set of features to test for redundancy
            std::set<FeatureSet> nrfss = powerset(rel, i+1, true);
            // determine the set of redundant features and add then in red
            FeatureSet red;
            auto filter_redundant = [&](const FeatureSet* fs) {
                // Test whether the feature set tested is disjoint
                // from red. WARNING: this is to speed up the
                // computation but it introduces some undeterminism
                // when run in multi-thread.
                // bool fs_red_disjoint = [&]() {
                //     shared_lock lock(mutex);
                //     return has_empty_intersection(*fs, red); }();
                // if (fs_red_disjoint)
                {
                    FeatureSet rfs = redundant_features(*fs, scorer,
                                                        threshold
                                                        * red_threshold);
                    unique_lock lock(mutex);
                    /// @todo the lock could be more granular
                    red.insert(rfs.begin(), rfs.end());
                }};
            auto nrfss_view = random_access_view(nrfss);
            OMP_ALGO::for_each(nrfss_view.begin(), nrfss_view.end(),
                               filter_redundant);

            logger().debug("Iteration %d redundant features=%d",
                i, red.size());
            // add in res the relevant non-redundant features
            boost::set_difference(rel, red, std::inserter(res, res.begin()));
        } else {
            res.insert(rel.begin(), rel.end());
        }
        logger().debug("Iteration %d finished with %d features\n",
            i, res.size());

        struct timeval stop, elapsed;
        gettimeofday(&stop, NULL);
        timersub(&stop, &start, &elapsed);
        logger().debug("Elapsed time %d seconds\n", elapsed.tv_sec);

        double rate = 1.0e6 * elapsed.tv_sec + elapsed.tv_usec;
        rate /= (double) res.size();
        rate /= (double) res.size();
        logger().debug("Rate %f microseconds per feature squared\n", rate);
    }

    // Log what it is that we actually got.
    if (logger().isInfoEnabled()) {
        std::stringstream ss;
        ss << "Exit incremental_selection(), selected: ";
        ostreamContainer(ss, res);
        // Do not print score.  Why?
        // 1) Its totally misleading, since this computes the score of
        //    all of the terms, interacting together.
        // 2) This can be a huge performance killer for the smd scorer.
        //    i.e this can takes vast amounts of cpu time,
        // 3) The contin MI scorer does not support interaction terms > 1
        // ss << " Score = " << scorer(res);
        logger().info() << ss.str();
    }

    return res;
}

/**
 * like incremental_selection but automatically wrap a cache around the scorer
 */
template<typename Scorer, typename FeatureSet>
FeatureSet cached_incremental_selection(const FeatureSet& features,
                                        const Scorer& scorer,
                                        double threshold,
                                        unsigned max_interaction_terms = 1,
                                        double red_threshold = -1.0)
{
    logger().debug() << "cached_incremental_selection(), num feats="
                     << features.size();
    /// @todo replace by lru_cache once thread safe fixed
    prr_cache_threaded<Scorer> scorer_cache(std::pow((double)features.size(),
                                                     (int)max_interaction_terms),
                                            scorer);
    return incremental_selection(features, scorer_cache, threshold,
                                 max_interaction_terms, red_threshold);
}


/**
 * like incremental_selection but take the number of feature to select
 * instead of a threshold. It searches it by bisection.
 */
template<typename Scorer, typename FeatureSet>
FeatureSet adaptive_incremental_selection(const FeatureSet& features,
                                          const Scorer& scorer,
                                          unsigned features_size_target,
                                          unsigned max_interaction_terms = 1,
                                          double red_threshold = -1.0,
                                          double min = 0, double max = 1,
                                          double epsilon = 0.001)
{
    double mean = (min+max)/2;
    if (logger().isDebugEnabled()) {
        logger().debug() << "Call adaptive_incremental_selection(size="
                         << features_size_target
                         << ", terms=" << max_interaction_terms
                         << ", red thresh=" << red_threshold
                         << ", min=" << min
                         << ", max=" << max
                         << ", epsi=" << epsilon
                         <<") so selection-thres=" << mean;
    }

    FeatureSet res = incremental_selection(features, scorer, mean,
                                           max_interaction_terms, red_threshold);
    unsigned rsize = res.size();
    // Logger
    logger().debug("Selected %d features", rsize);
    // ~Logger
    if (isWithin(min, max, epsilon) || rsize == features_size_target)
        return res;
    else {
        double nmin = rsize < features_size_target? min : mean;
        double nmax = rsize < features_size_target? mean : max;
        return adaptive_incremental_selection(features, scorer,
                                              features_size_target,
                                              max_interaction_terms,
                                              red_threshold,
                                              nmin, nmax, epsilon);
    }
}

/**
 * like adaptive_incremental_selection but wrapping the scorer with a cache
 */
template<typename Scorer, typename FeatureSet>
FeatureSet cached_adaptive_incremental_selection(const FeatureSet& features,
                                                 const Scorer& scorer,
                                                 unsigned features_size_target,
                                                 unsigned max_interaction_terms = 1,
                                                 double red_threshold = -1.0,
                                                 double min = 0, double max = 1,
                                                 double epsilon = 0.01)
{
    logger().debug() << "cached_adaptive_incremental_selection(),"
                     << " target=" << features_size_target
                     << " iterms=" << max_interaction_terms
                     << " num feats=" << features.size();
    /// @todo replace by lru_cache once thread safe fixed
    prr_cache_threaded<Scorer> scorer_cache(std::pow((double)features.size(),
                                                     (int)max_interaction_terms),
                                            scorer);
    FeatureSet f = adaptive_incremental_selection(features, scorer_cache,
                                                  features_size_target,
                                                  max_interaction_terms,
                                                  red_threshold,
                                                  min, max, epsilon);
    return f;
}

/**
 * Return a set of redundant features of a given set of features. It
 * looks for a subset of features that do not manage to raise the score
 * above a given threshold.
 */
template<typename Scorer, typename FeatureSet>
FeatureSet redundant_features(const FeatureSet& features, const Scorer& scorer,
                              double threshold)
{
    typedef FeatureSet FS;
    // Take the best powerset(features) of size i, return its
    // complementary with features if it good enough. We start with
    // lower i because the fewer the features the better.
    for (unsigned i = 1; i < features.size(); i++) {
        auto sfs = powerset(features, i, true);
        auto mit = boost::max_element(sfs, [&](const FS& fsl, const FS& fsr) {
                return scorer(fsl) < scorer(fsr); });
        if (scorer(features) - scorer(*mit) < threshold)
            return set_difference(features, *mit);
    }
    return FS();
}

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_INCREMENTAL_ALGO_H
