/** 
 * simple.h ---
 *
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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


#ifndef _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H
#define _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H

#include <mutex>

#include <opencog/util/numeric.h>
#include <opencog/util/oc_omp.h>

#include "../main/feature-selection.h"  // needed for feature_set, feature_selection_parameters

namespace opencog {

feature_set_pop simple_select_feature_sets(const CTable& ctable,
                                           const feature_selection_parameters& fs_params);

/**
 * Returns a set S of features following the algo:
 *
 * @param features       The initial set of features to be selected from
 * @param scorer         The function to score a set of features.
 * @param threshold      The threshold to select a set of feature
 * @param red_threshold  If >0 it modulates the intensity of the
 *                       threshold of redundant_features(), precisely
 *                       red_threshold * threshold
 *                       Otherwise redundant features are ignored.
 *
 * @return               The set of selected features
 */
template<typename Scorer, typename FeatureSet>
FeatureSet simple_selection(const FeatureSet& features,
                            const Scorer& scorer,
                            int num_desired,
                            double threshold,
                            double red_threshold = 0)
{
    std::multimap<double, FeatureSet, std::greater<double> > sorted_flist;

    // build vector of singleton feature sets
    std::vector<FeatureSet> singletons; 
    for (auto feat : features)
        singletons.push_back(FeatureSet({feat}));

    // compute score of all singletons and insert to sorted_flist
    // those above threshold
    std::mutex sfl_mutex;       // mutex for sorted_flist
    OMP_ALGO::for_each(singletons.begin(), singletons.end(),
                       [&](const FeatureSet& singleton) {
                           double sc = scorer(singleton);
                           if (threshold <= sc) {
                               std::unique_lock<std::mutex> lock(sfl_mutex);
                               sorted_flist.insert({sc, singleton});
                           }
                       });

    // select num_desired best features from sorted_flist as final
    // feature set
    FeatureSet final;
    for (const auto& pr : sorted_flist) {
        final.insert(*pr.second.begin());
        num_desired --;
        if (num_desired <= 0) break;
    }

    // TODO: if there is a non-negative red_threshold, then remove any
    // pair-wise redundant features. 

    return final;
}


} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H
