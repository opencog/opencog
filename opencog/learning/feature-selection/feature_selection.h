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

#include <opencog/util/foreach.h>

namespace opencog {

/**
 * Returns a set S of features such that if feature Fi is in S then
 * scorer(..., Fi, ...) > threshold. The algorithm goes as follows:
 * all features are tried seperatly first then all pairs, etc, up to
 * all set of size max_size.
 *
 * @param features    The initial set of features to select from
 * @param scorer      The function to score a set of features
 * @param threshold   The threshold to select a feature
 * @param max_size    The maximum size of feature set being tested in the scorer
 *
 * @return            The set of selected features, a subset of 'features'
 */
template<Scorer, FeatureSet>
FeatureSet incremental_selection(const FeatureSet& features, const Scorer& scorer,
                                 double threshold, unsigned int max_size = 1) {
    FeatureSet res;
    std::set<FeatureSet> fss = powerset(features, max_size);
    foreach(const FeatureSet& fs, fss)
        if(scorer(fs) > threshold)
            res.insert(fs.begin(), fs.end());
    return res;
}

} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_H
