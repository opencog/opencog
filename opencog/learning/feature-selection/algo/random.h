/** 
 * random.h ---
 *
 * Copyright (C) 2014 Aidyia Limited
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


#ifndef _OPENCOG_FEATURE_SELECTION_RANDOM_ALGO_H
#define _OPENCOG_FEATURE_SELECTION_RANDOM_ALGO_H

#include <opencog/util/random.h>

#include "../main/feature-selection.h"  // needed for feature_set, feature_selection_parameters

namespace opencog {

feature_set_pop random_select_feature_sets(const CTable& ctable,
                                           const feature_selection_parameters& fs_params);

/**
 * Returns a random subset of features.  The features are
 * choosen with a uniform probability.
 *
 * @param features       The initial set of features to be selected from
 *
 * @return               The set of selected features
 */
template<typename FeatureSet>
FeatureSet random_selection(const FeatureSet& features,
                            size_t num_desired)
{
    // Pick features randomly, until num_desired have been chosen.
    FeatureSet final;
    while (0 < num_desired) {
        final.insert(randset(features));
        num_desired --;
    }

    return final;
}


} // ~namespace opencog

#endif // _OPENCOG_FEATURE_SELECTION_RANDOM_ALGO_H
