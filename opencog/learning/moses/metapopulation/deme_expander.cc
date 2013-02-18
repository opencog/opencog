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

void deme_expander::log_selected_feature_sets(const feature_set_pop& sf_pop,
                                              const feature_set& xmplr_features,
                                              const string_seq& ilabels) const {
    for (const auto& sf : sf_pop) {
        logger().info() << "Selected " << sf.second.size()
                        << " features for representation";
        auto xmplr_sf = set_intersection(sf.second, xmplr_features);
        auto new_sf = set_difference(sf.second, xmplr_features);
        ostreamContainer(logger().info() << "From which were in the exemplar: ",
                         fs_to_names(xmplr_sf, ilabels), ",");
        ostreamContainer(logger().info() << "From which are new: ",
                         fs_to_names(new_sf, ilabels), ",");
    }
}

combo_tree deme_expander::prune_xmplr(const combo_tree& xmplr,
                                      const feature_set& selected_features) const
{
    combo_tree res = xmplr;
    // remove literals of non selected features from the exemplar
    for (auto it = res.begin(); it != res.end();) {
        if (is_argument(*it)) {
            arity_t feature = get_argument(*it).abs_idx_from_zero();
            auto fit = selected_features.find(feature);
            if (fit == selected_features.end())
                it = res.erase(it); // not selected, prune it
            else
                ++it;
        }
        else
            ++it;
    }
    simplify_knob_building(res);
    // if the exemplar is empty use a seed
    if (res.empty()) {
        type_node otn = get_type_node(get_signature_output(_type_sig));
        res = type_to_exemplar(otn);
    }
    return res;
}

bool deme_expander::create_demes(const combo_tree& exemplar)
{
    using namespace reduct;

    OC_ASSERT(_reps.empty());
    OC_ASSERT(_demes.empty());

    combo_tree xmplr = exemplar;

    // [HIGHLY EXPERIMENTAL]. Limit the number of features used to
    // build the exemplar to a more manageable number.  Basically,
    // this is 'on-the-fly' feature selection.  This differs from an
    // ordinary, one-time only, up-front round of feature selection by
    // using only those features which score well with the current
    // exemplar.
    vector<operator_set> ignore_ops_seq;
    vector<combo_tree> xmplr_seq;
    if (_params.fstor) {
        // copy, any change in the parameters will not be remembered
        feature_selector festor = *_params.fstor;

        // return the set of selected features as column index
        // (left most column corresponds to 0)
        auto sf_pop = festor(exemplar);

        // get the set of features of the exemplar
        auto xmplr_features = get_argument_abs_idx_from_zero_set(exemplar);

        // get labels corresponding to all the features
        const auto& ilabels = festor._ctable.get_input_labels();
    
        log_selected_feature_sets(sf_pop, xmplr_features, ilabels);

        for (auto& sf : sf_pop) {
            // Either prune the exemplar, or add all exemplars
            // features to the feature sets
            if (festor.params.prune_xmplr) {
                auto xmplr_nsf = set_difference(xmplr_features,
                                                sf.second);
                ostreamContainer(logger().debug() <<
                                 "Prune the exemplar from non-selected features: ",
                                 fs_to_names(xmplr_nsf, ilabels));
                xmplr_seq.push_back(prune_xmplr(exemplar, sf.second));
            }
            else {
                logger().debug("Do not prune the exemplar from "
                               "non-selected features");
                // Insert exemplar features as they are not pruned
                sf.second.insert(xmplr_features.begin(), xmplr_features.end());
            }

            // add the complement of the selected features to ignore_ops
            unsigned arity = festor._ctable.get_arity();
            vertex_set ignore_ops;
            for (unsigned i = 0; i < arity; i++)
                if (sf.second.find(i) == sf.second.end())
                    ignore_ops.insert(argument(i + 1));
            ignore_ops_seq.push_back(ignore_ops);
        }
    }
    else {                      // no feature selection within moses
        ignore_ops_seq.push_back(_params.ignore_ops);
        xmplr_seq.push_back(exemplar);
    }

    for (unsigned i = 0; i < xmplr_seq.size(); i++) {
        if (logger().isDebugEnabled())
            logger().debug() << "Attempt to build rep from exemplar: "
                             << xmplr_seq[i];

        // Build a representation by adding knobs to the exemplar,
        // creating a field set, and a mapping from field set to knobs.
        _reps.push_back(new representation(simplify_candidate,
                                           simplify_knob_building,
                                           xmplr_seq[i], _type_sig,
                                           ignore_ops_seq[i],
                                           _params.perceptions,
                                           _params.actions,
                                           _params.linear_contin,
                                           _params.perm_ratio));

        // If the representation is empty, try the next
        // best-scoring exemplar.
        if (_reps.back().fields().empty()) {
            _reps.pop_back();
            logger().warn("The representation is empty, perhaps the reduct "
                          "effort for knob building is too high.");
        }
    }
    if (_reps.empty()) return false;

    // Create empty demes
    for (const auto& rep : _reps)
        _demes.push_back(new deme_t(rep.fields()));

    return true;
}

int deme_expander::optimize_demes(int max_evals, time_t max_time)
{
    if (logger().isDebugEnabled()) {
        logger().debug()
           << "Optimize deme; max evaluations allowed: "
           << max_evals;
    }

    complexity_based_scorer cpx_scorer =
        complexity_based_scorer(_cscorer, _reps.front() /* TODO */,
                                _params.reduce_all);
    return _optimize(_demes.front() /* TODO */, cpx_scorer, max_evals, max_time);
}

void deme_expander::free_demes()
{
    _demes.clear();
    _reps.clear();
}

} // ~namespace moses
} // ~namespace opencog
