/** feature_selector.cc --- 
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

#include "feature_selector.h"

namespace opencog {
namespace moses {

feature_selector::feature_selector(const combo::CTable& ctable,
                                   const feature_selector_parameters& festor_params)
    : params(festor_params), _ctable(ctable) {}

feature_selector::feature_selector(const combo::Table& table,
                                   const feature_selector_parameters& festor_params)
    : params(festor_params), _ctable(table.compressed()) {}

feature_set feature_selector::operator()(const combo::combo_tree& tr) const
{
    /////////////////////////////////////////////
    // Build ctable used for feature selection //
    /////////////////////////////////////////////

    // set labels and signature
    auto labels = _ctable.get_labels();
    auto sig = _ctable.get_signature();
    auto cto = get_type_node(get_signature_output(sig));
    if (params.exemplar_as_feature) {
        // update labels and signature with the exemplar feature
        labels.push_back("__exemplar_feature__");
        auto head = sig.begin();
        // this assumes that the ctable output type equals the
        // exemplar output type.
        sig.append_child(head, cto);
    }
    combo::CTable fs_ctable(_ctable.get_labels(), _ctable.get_signature());

    // define interpreter visitor
    interpreter_visitor iv(tr);
    auto ai = boost::apply_visitor(iv);

    // define visitor to initialize the features to ignore
    std::vector<init_at_visitor> iavs(params.ignore_features.begin(),
                                      params.ignore_features.end());

    // add each considered row
    for (const combo::CTable::value_type& vct : _ctable) {
        vertex predicted_out = ai(vct.first.get_variant());

        // determine whether the row should be considered
        bool consider_row = true;
        if (params.restrict_incorrect) {
            // Use only rows where the model is just plain wrong.  The
            // feature selector will then look for any features that
            // correlate well with these incorrect answers. The hope
            // is that these will be used to build a more accurate
            // models.
            Counter<vertex, unsigned> cnt = vct.second;
            vertex actual_out = cnt.most_frequent();
            consider_row = predicted_out != actual_out;
        }
        else if (params.restrict_true) {
            // Use only rows where the model output is true, that
            // may be useful the prediction bscore is used because
            // positive answers indicates where the exemplar focus
            // is.
            consider_row = predicted_out == combo::id::logical_true;
        }

        // add row
        if (consider_row) {
            auto inputs = vct.first;

            // initialize all values of the ignored features
            if (params.ignore_exemplar_features)
                for (auto& iav : iavs)
                    boost::apply_visitor(iav, inputs.get_variant());

            // append exemplar feature at the end
            if (params.exemplar_as_feature) {
                if (cto == id::boolean_type)
                    inputs.push_back(get_builtin(predicted_out));
                else if (cto == id::contin_type)
                    inputs.push_back(get_contin(predicted_out));
                else
                    OC_ASSERT(false, "Not implemented");
            }

            fs_ctable.insert({inputs, vct.second});
        }
    }

    ////////////////////////////
    // Call feature selection //
    ////////////////////////////

    if (params.exemplar_as_feature) {
        // TODO add exemplar_feature as initial feature (to speed
        // things up a bit)
        ++params.fs_params.target_size;
    }

    auto self = select_features(fs_ctable, params.fs_params);

    if (params.exemplar_as_feature)
        --params.fs_params.target_size;

    // remove last feature if it's the feature exemplar
    if (params.exemplar_as_feature) {
        size_t xmplar_f_pos = _ctable.get_labels().size();
        auto xmplar_f_it = self.find(xmplar_f_pos);
        OC_ASSERT(xmplar_f_it != self.end(),
                  "The exemplar feature has not been selected, that's weird");
        self.erase(xmplar_f_it);
    }

    return self;
}

} // ~namespace moses
} // ~namespace opencog
