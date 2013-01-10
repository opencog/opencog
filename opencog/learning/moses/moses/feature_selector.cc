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
    // Build ctable used for feature selection
    combo::CTable fs_ctable(_ctable.get_labels(), _ctable.get_signature());
    // define interpreter visitor
    interpreter_visitor iv(tr);
    auto ai = boost::apply_visitor(iv);
    // define visitor to initialize the features to ignore
    std::vector<init_at_visitor> iavs(params.ignore_features.begin(),
                                      params.ignore_features.end());
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

        // add the row
        if (consider_row) {
            if (params.ignore_exemplar_features) {
                auto inputs = vct.first;
                for (auto& iav : iavs)
                    boost::apply_visitor(iav, inputs.get_variant());
                fs_ctable.insert({inputs, vct.second});
            }
            else fs_ctable.insert(vct);
        }
    }

    // Call feature selection
    return select_features(fs_ctable, params.fs_params);
}

} // ~namespace moses
} // ~namespace opencog
