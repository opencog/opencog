/*
 * opencog/learning/moses/moses/feature_selector.cc
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

#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/learning/feature-selection/main/feature-selection.h>

namespace opencog {
namespace moses {    

/**
 * Parameters of feature selector
*/
struct feature_selector_parameters {
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
    std::set<arity_t> ignore_features;

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
     * Probability of discarding a row
     */
    float subsampling_pbty;
};

/**
 * Struct in charge of selecting features maximize the amount of
 * information in combination with a candidate.
 */
struct feature_selector
{
    typedef std::set<combo::arity_t> feature_set;

    feature_selector(const combo::CTable& ctable,
                     const feature_selector_parameters& festor_params);

    feature_selector(const combo::Table& table,
                     const feature_selector_parameters& festor_params);

    /// Return a feature set that is good when combined with the
    /// exemplar tr.
    feature_set operator()(const combo::combo_tree& tr);

    // Parameters
    feature_selector_parameters params;

    const combo::CTable& _ctable;
};

} // ~namespace moses
} // ~namespace opencog
