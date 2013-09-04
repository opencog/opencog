/** algo/incremental.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include <boost/range/irange.hpp>

#include "incremental.h"

namespace opencog {

using namespace std;

feature_set_pop incremental_select_feature_sets(const CTable& ctable,
                                                const feature_selection_parameters& fs_params)
{
    auto ir = boost::irange(0, ctable.get_arity());
    feature_set all_features(ir.begin(), ir.end());
    typedef MutualInformation<feature_set> FeatureScorer;
    FeatureScorer fsc(ctable);

    logger().info() << "incremental_select_features, targ sz="
                    << fs_params.target_size
                    << " thresh=" << fs_params.threshold;
    if (fs_params.threshold <= 0 && fs_params.target_size <= 0) {
        // Nothing happened, return all features by default
        feature_set_pop::value_type sfs(fsc(all_features), all_features);
        return {sfs};
    }
    
    feature_set fs = 0 < fs_params.target_size ?
        cached_adaptive_incremental_selection(all_features, fsc,
                                              fs_params.target_size,
                                              fs_params.inc_interaction_terms,
                                              fs_params.inc_red_intensity,
                                              0, 1,
                                              fs_params.inc_target_size_epsilon)
        : cached_incremental_selection(all_features, fsc,
                                       fs_params.threshold,
                                       fs_params.inc_interaction_terms,
                                       fs_params.inc_red_intensity);

    feature_set_pop::value_type sfs(fsc(fs), fs);
    return {sfs};
}

} // ~namespace opencog
