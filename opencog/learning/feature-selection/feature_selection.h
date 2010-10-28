/** feature_selection.h --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
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


#ifndef _OPENCOG_FEATURE_SELECTION_H
#define _OPENCOG_FEATURE_SELECTION_H

#include <functional>

#include <opencog/util/foreach.h>

namespace opencog {

/**
 * Returns a set S of features following the algo:
 * 1) Select all features that score above 'threshold'
 * 2) remove the selected features from the initial set 'features', called 'tf'
 * 3) select all pairs of features from 'ft' that score above 'threshold'
 * 4) follow the same pattern but with triplets, etc, until max_size.
 *
 * @todo add some code to ignore redundant features, like before
 * returning 'res', check all pairs of features of 'res' and remove
 * the ones of each pair that does not improve the score enough.
 *
 * @param features    The initial set of features to be selected from
 * @param scorer      The function to score a set of features
 * @param threshold   The threshold to select a set of feature
 * @param max_size    The maximum size of each feature set tested in the scorer
 *
 * @return            The set of selected features
 */
template<typename Scorer, typename FeatureSet>
FeatureSet incremental_selection(const FeatureSet& features, const Scorer& scorer,
                                 double threshold, unsigned int max_size = 1) {
    FeatureSet res; // the set of features to return
    for(unsigned int i = 1; i <= max_size; i++) {
        FeatureSet tf; // the set of features to test
        std::set_difference(features.begin(), features.end(),
                            res.begin(), res.end(), std::inserter(tf, tf.begin()));
        std::set<FeatureSet> fss = powerset(tf, i, true);
        foreach(const FeatureSet& fs, fss)
            if(scorer(fs) > threshold)
                res.insert(fs.begin(), fs.end());
    }
    return res;
}

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_H
