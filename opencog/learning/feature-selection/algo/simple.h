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

#include <opencog/util/numeric.h>

#include "../main/feature-selection.h"  // needed for feature_set, feature_selection_parameters

namespace opencog {

feature_set simple_select_features(const CTable& ctable,
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
    std::multimap<double, FeatureSet> sorted_flist;

    for (auto feat : features) {
        FeatureSet fs;
        fs.insert(feat);
        double sc = scorer(fs);
        if (threshold <= sc) {
            auto scf = std::make_pair(sc, fs);
            sorted_flist.insert(scf);
        }
    }

    FeatureSet final;
    // for (auto pr : sorted_flist) {
    for (auto pr = sorted_flist.rbegin(); pr != sorted_flist.rend(); pr++) {
        // std::cout << "sc="<< pr->first <<" fe=" << *pr->second.begin() << std::endl;
        final.insert(*pr->second.begin());
        num_desired --;
        if (num_desired <= 0) break;
    }

    // TODO: if there is a non-negative red_threshold, then remove any
    // pair-wise redundant features. 

    return final;
}


} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_SIMPLE_ALGO_H
