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
#include <opencog/comboreduct/table/table_io.h>

// Name given to the feature corresponding to the output of the
// exemplar
#define EXEMPLAR_FEATURE_NAME "__exemplar_feature__"

namespace opencog {
namespace moses {

feature_selector::feature_selector(const combo::CTable& ctable,
                                   const feature_selector_parameters& festor_params)
    : params(festor_params), _ctable(ctable) {}

feature_selector::feature_selector(const combo::Table& table,
                                   const feature_selector_parameters& festor_params)
    : params(festor_params), _ctable(table.compressed()) {}

feature_set feature_selector::operator()(const combo::combo_tree& tr)
{
    /////////////////////////////////////////////
    // Build ctable used for feature selection //
    /////////////////////////////////////////////

    // set labels and signature
    auto labels = _ctable.get_labels();
    auto sig = _ctable.get_signature();
    auto cto = get_type_node(get_signature_output(sig));
    if (params.xmplr_as_feature) {
        // update labels and signature with the exemplar feature
        labels.push_back(EXEMPLAR_FEATURE_NAME);
        auto head = sig.begin();
        // this assumes that the ctable output type equals the
        // exemplar output type.
        sig.append_child(head, cto);
        logger().debug("Append exemplar feature");
    }
    combo::CTable fs_ctable(labels, sig);

    // define interpreter visitor
    interpreter_visitor iv(tr);
    auto ai = boost::apply_visitor(iv);

    // define visitor to initialize the features to ignore
    if (!params.ignore_features.empty())
        ostreamContainer(logger().debug() << "Ignore features: ",
                         params.ignore_features);
    std::vector<init_at_visitor> iavs(params.ignore_features.begin(),
                                      params.ignore_features.end());

    // add each considered row
    for (const combo::CTable::value_type& vct : _ctable) {
        vertex predicted_out = ai(vct.first.get_variant());

        // determine whether the row should be considered
        bool consider_row = true;
        if (params.restrict_true) {
            // Use only rows where the model output is true, that
            // may be useful the prediction bscore is used because
            // positive answers indicates where the exemplar focus
            // is.
            consider_row = predicted_out == combo::id::logical_true;
        }
        if (consider_row && params.restrict_incorrect) {
            // Use only rows where the model is just plain wrong.  The
            // feature selector will then look for any features that
            // correlate well with these incorrect answers. The hope
            // is that these will be used to build a more accurate
            // models.
            //
            // Nil: it's not exactly "plain wrong", plain wrong is
            // more like the least frequent answer has count zero and
            // the model predicts that. One could relax that
            // constraint by specifying to which degree the model
            // should be wrong so that the row is considered.
            Counter<vertex, unsigned> cnt = vct.second;
            vertex actual_out = cnt.most_frequent();
            consider_row = predicted_out != actual_out;
        }

        // subsample
        if (consider_row && params.subsampling_pbty > 0)
            consider_row = params.subsampling_pbty <= randGen().randfloat();

        // add row
        if (consider_row) {
            auto inputs = vct.first;

            // initialize all values of the ignored features
            if (params.ignore_xmplr_features)
                for (auto& iav : iavs)
                    boost::apply_visitor(iav, inputs.get_variant());

            // append exemplar feature at the end
            if (params.xmplr_as_feature) {
                if (cto == id::boolean_type)
                    inputs.push_back(get_builtin(predicted_out));
                else if (cto == id::contin_type)
                    inputs.push_back(get_contin(predicted_out));
                else
                    OC_ASSERT(false, "Not implemented");
            }

            fs_ctable[inputs] += vct.second;
        }
    }
    logger().debug("CTable size for feature selection = %u",
                   fs_ctable.size());

    if (logger().isFineEnabled()) {
        logger().fine("fs_ctable:");
        stringstream ss;
        ostreamCTable(ss, fs_ctable);
        logger().fine() << ss.str();
    }

    ////////////////////////////
    // Call feature selection //
    ////////////////////////////

    if (params.xmplr_as_feature) {
        params.fs_params.initial_features.push_back(EXEMPLAR_FEATURE_NAME);
        ++params.fs_params.target_size;
    }

    auto self = select_features(fs_ctable, params.fs_params);

    // remove last feature if it's the feature exemplar
    if (params.xmplr_as_feature) {
        size_t xmplar_f_pos = fs_ctable.get_arity() - 1;
        auto xmplar_f_it = self.find(xmplar_f_pos);
        if (xmplar_f_it == self.end()) {
            logger().debug("The exemplar feature has not been selected, "
                           "that exemplar must be pretty bad! As a result "
                           "one more feature is returned by feature selection "
                           "(as we obviously can't remove the exemplar feature).");
        } else {
            self.erase(xmplar_f_it);
            logger().debug("Remove the exemplar feature");
        }
    }

    return self;
}

} // ~namespace moses
} // ~namespace opencog
