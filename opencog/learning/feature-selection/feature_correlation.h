/** feature_correlation.h ---
 *
 * Copyright (C) 2010,2012 OpenCog Foundation
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


#ifndef _OPENCOG_FEATURE_CORRELATION_ALGO_H
#define _OPENCOG_FEATURE_CORRELATION_ALGO_H

#include <functional>

#include <opencog/util/numeric.h>
#include <opencog/util/foreach.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/functional.h>
#include <opencog/util/oc_omp.h>

namespace opencog {

/**
 * Returns a set S of features following the algo:
 * 0) res = empty set
 * 1) Select single individual features with the highest mutual
 *    information (MI)
 * 2) Add one feature, compute new MI; keep only highest MI results.
 * 3) Repeat, until desired number of features is obtained.
 * 4) return res
 *
 * @param features       The initial set of features to be selected from
 * @param scorer         The function to score a set of features.
 * @param num_features   The desired number of features to return.
 *
 * @return               The set of selected features
 */
template<typename Scorer, typename FeatureSet>
FeatureSet correlation_selection(const FeatureSet& features,
                                 const Scorer& scorer,
                                 unsigned num_features)
{
    FeatureSet res; // set of features to return

    typedef boost::shared_mutex shared_mutex;
    typedef boost::shared_lock<shared_mutex> shared_lock;
    typedef boost::unique_lock<shared_mutex> unique_lock;
    shared_mutex mutex;

#if DEBUG
    for (unsigned i = 1; i <= max_interaction_terms; ++i) {
        std::set<FeatureSet> ps = powerset(features, i, true);
        typename std::set<FeatureSet>::const_iterator psit;
        for (psit = ps.begin(); psit != ps.end(); psit++) {
            const FeatureSet &fs = *psit;
            std::cout << "fs";
            typename FeatureSet::const_iterator fi;
            for (fi = fs.begin(); fi != fs.end(); fi++) {
                std::cout << "-" << *fi;
            }
            double mi = scorer(fs);
            std::cout << "\t" << mi << std::endl;
        }
        std::cout << "============================" << std::endl;
    }
#endif

    for (unsigned i = 1; i <= num_features; ++i) {
        std::map<double, FeatureSet> hi_scorers

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

        res.insert(rel.begin(), rel.end());
    }
    return res;
}

#if 0
/**
 * like incremental_selection but take the number of feature to select
 * instead of a threshold. It searches it by bisection.
 */
template<typename Scorer, typename FeatureSet>
FeatureSet adaptive_incremental_selection(const FeatureSet& features,
                                          const Scorer& scorer,
                                          unsigned features_size_target,
                                          unsigned max_interaction_terms = 1,
                                          double red_threshold = 0,
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
                                                 double red_threshold = 0,
                                                 double min = 0, double max = 1,
                                                 double epsilon = 0.01)
{
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

#endif


} // ~namespace opencog

#endif // _OPENCOG_FEATURE_CORRELATION_ALGO_H
