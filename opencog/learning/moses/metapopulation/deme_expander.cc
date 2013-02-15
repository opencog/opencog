/** deme_expander.cc --- 
 *
 * Copyright (C) 2013 OpenCog Foundation
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

#include "deme_expander.h"

namespace opencog {
namespace moses {

string_seq deme_expander::fs_to_names(const set<arity_t>& fs,
                                      const string_seq& ilabels) const
{
    vector<string> fs_names;
    for (arity_t i : fs)
        fs_names.push_back(ilabels[i]);
    return fs_names;
}

void deme_expander::log_selected_features(const feature_set& selected_features,
                                          const feature_set& xmplr_features,
                                          const string_seq& ilabels) const {
    logger().info() << "Selected " << selected_features.size()
                    << " features for representation";
    auto xmplr_sf = set_intersection(selected_features, xmplr_features);
    auto new_sf = set_difference(selected_features, xmplr_features);
    ostreamContainer(logger().info() << "From which were in the exemplar: ",
                     fs_to_names(xmplr_sf, ilabels), ",");
    ostreamContainer(logger().info() << "From which are new: ",
                     fs_to_names(new_sf, ilabels), ",");
}

void deme_expander::prune_xmplr(combo_tree& xmplr,
                                const feature_set& selected_features) const
{
    // remove literals of non selected features from the exemplar
    for (auto it = xmplr.begin(); it != xmplr.end();) {
        if (is_argument(*it)) {
            arity_t feature = get_argument(*it).abs_idx_from_zero();
            auto fit = selected_features.find(feature);
            if (fit == selected_features.end())
                it = xmplr.erase(it); // not selected, prune it
            else
                ++it;
        }
        else
            ++it;
    }
    simplify_knob_building(xmplr);
    // if the exemplar is empty use a seed
    if (xmplr.empty()) {
        type_node otn = get_type_node(get_signature_output(_type_sig));
        xmplr = type_to_exemplar(otn);
    }
}

bool deme_expander::create_deme(const combo_tree& exemplar)
{
    using namespace reduct;

    OC_ASSERT(_rep == NULL);
    OC_ASSERT(_deme == NULL);

    combo_tree xmplr = exemplar;

    // [HIGHLY EXPERIMENTAL]. Limit the number of features used to
    // build the exemplar to a more manageable number.  Basically,
    // this is 'on-the-fly' feature selection.  This differs from an
    // ordinary, one-time only, up-front round of feature selection by
    // using only those features which score well with the current
    // exemplar.
    operator_set ignore_ops = _params.ignore_ops;
    if (_params.fstor) {
        // copy, any change in the parameters will not be remembered
        feature_selector festor = *_params.fstor;

        // get the set of features of the exemplar
        auto xmplr_features = get_argument_abs_idx_from_zero_set(xmplr);

        // get labels corresponding to all the features
        const auto& ilabels = festor._ctable.get_input_labels();

        // names of the exemplar features
        vector<string> xmplr_feature_names;
        for (arity_t i : xmplr_features)
                xmplr_feature_names.push_back(ilabels[i]);

        // Use the features of the exemplar as initial feature set to
        // seed the feature selection algorithm. That way the new
        // features will be selected to combine well with the
        // exemplar.
        if (festor.params.init_xmplr_features) {
            auto& pif = festor.params.fs_params.initial_features;
            pif.insert(pif.end(), xmplr_feature_names.begin(), xmplr_feature_names.end());
            // we increase the size to output new features (not the
            // ones already in the exemplar)
            festor.params.increase_target_size = true;
        }

        // If the combo tree is already using N features, we want to find
        // and additional M features which might make it better.  So bump
        // up the count.  Of course, the feat selector might not find any
        // of the existing args in the exemplar; but we want to avoid the
        // case where the feat sel is returning only those features already
        // in the exemplar.
        if (festor.params.increase_target_size) {
            festor.params.fs_params.target_size += xmplr_features.size();
        }

        // Alternatively one can ignore the features in the exemplar
        // during feature selection.
        festor.params.ignore_features = festor.params.ignore_xmplr_features ?
            xmplr_features : set<arity_t>();

        // return the set of selected features as column index
        // (left most column corresponds to 0)
        auto sf_pop = festor(xmplr);
        auto selected_features = sf_pop.begin()->second;

        log_selected_features(selected_features, xmplr_features, ilabels);

        if (festor.params.prune_xmplr) {
            auto xmplr_nsf = set_difference(xmplr_features, selected_features);
            ostreamContainer(logger().debug() <<
                             "Prune the exemplar from non-selected features: ",
                             fs_to_names(xmplr_nsf, ilabels));
            prune_xmplr(xmplr, selected_features);
        }
        else {
            logger().debug("Do not prune the exemplar from non-selected features");
            // Insert exemplar features as they are not pruned
            selected_features.insert(xmplr_features.begin(), xmplr_features.end());
        }
        
        // add the complement of the selected features to ignore_ops
        unsigned arity = festor._ctable.get_arity();
        for (unsigned i = 0; i < arity; i++)
            if (selected_features.find(i) == selected_features.end())
                ignore_ops.insert(argument(i + 1));
    }

    if (logger().isDebugEnabled())
        logger().debug() << "Attempt to build rep from exemplar: " << xmplr;

    // Build a representation by adding knobs to the exemplar,
    // creating a field set, and a mapping from field set to knobs.
    _rep = new representation(simplify_candidate,
                              simplify_knob_building,
                              xmplr, _type_sig,
                              ignore_ops,
                              _params.perceptions,
                              _params.actions,
                              _params.linear_contin,
                              _params.perm_ratio);

    // If the representation is empty, try the next
    // best-scoring exemplar.
    if (_rep->fields().empty()) {
        delete(_rep);
        _rep = NULL;
        logger().warn("The representation is empty, perhaps the reduct "
                      "effort for knob building is too high.");
    }

    if (!_rep) return false;

    // Create an empty deme.
    _deme = new deme_t(_rep->fields());

    return true;
}

int deme_expander::optimize_deme(int max_evals, time_t max_time)
{
    if (logger().isDebugEnabled()) {
        logger().debug()
           << "Optimize deme; max evaluations allowed: "
           << max_evals;
    }

    complexity_based_scorer cpx_scorer =
        complexity_based_scorer(_cscorer, *_rep, _params.reduce_all);
    return _optimize(*_deme, cpx_scorer, max_evals, max_time);
}

void deme_expander::free_deme()
{
    delete _deme;
    delete _rep;
    _deme = NULL;
    _rep = NULL;
}

} // ~namespace moses
} // ~namespace opencog
