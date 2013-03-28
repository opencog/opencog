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

#include <boost/range/algorithm/max_element.hpp>

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

void feature_selector::preprocess_params(const combo::combo_tree& xmplr)
{
    // Note that this is gonna overwrite some parameters. It is OK
    // because feature_selector being copied before being called by
    // deme_expander::create_deme

    // get the set of features of the exemplar
    auto xmplr_features = get_argument_abs_idx_from_zero_set(xmplr);

    // get labels corresponding to all the features
    const auto& ilabels = _ctable.get_input_labels();

    // names of the exemplar features
    vector<string> xmplr_feature_names;
    for (arity_t i : xmplr_features)
        xmplr_feature_names.push_back(ilabels[i]);

    // Use the features of the exemplar as initial feature set to
    // seed the feature selection algorithm. That way the new
    // features will be selected to combine well with the
    // exemplar.
    if (params.init_xmplr_features) {
        auto& pif = params.fs_params.initial_features;
        pif.insert(pif.end(),
                   xmplr_feature_names.begin(),
                   xmplr_feature_names.end());
        // we increase the size to output new features (not the
        // ones already in the exemplar)
        params.increase_target_size = true;
    }

    // If the combo tree is already using N features, we want to
    // find an additional M features which might make it better.
    // So bump up the count.  Of course, the feat selector might
    // not find any of the existing args in the exemplar; but we
    // want to avoid the case where the feat sel is returning only
    // those features already in the exemplar.
    if (params.increase_target_size) {
        params.fs_params.target_size += xmplr_features.size();
    }

    // Alternatively to params.increase_target_size, one can ignore
    // the features in the exemplar during feature selection.
    params.ignored_features = params.ignore_xmplr_features ?
        xmplr_features : set<arity_t>();

    // Or one can use the output of the exemplar as an initial feature
    if (params.xmplr_as_feature) {
        params.fs_params.initial_features.push_back(EXEMPLAR_FEATURE_NAME);
        ++params.fs_params.target_size;
    }
}

CTable feature_selector::build_fs_ctable(const combo_tree& xmplr) const {
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
    interpreter_visitor iv(xmplr);
    auto ai = boost::apply_visitor(iv);

    // define visitor to initialize the features to ignore
    if (!params.ignored_features.empty())
        ostreamContainer(logger().debug() << "Ignore features: ",
                         params.ignored_features);
    std::vector<init_at_visitor> iavs(params.ignored_features.begin(),
                                      params.ignored_features.end());

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

    return fs_ctable;
}

feature_set_pop feature_selector::select_top_feature_sets(const feature_set_pop& fss) const
{
    unsigned res_size = std::min(params.n_demes, (unsigned)fss.size());
    feature_set_pop res(fss.begin(), std::next(fss.begin(), res_size));
    return res;
}

void feature_selector::remove_useless_features(feature_set_pop& sf_pop) const
{
    // remove last feature if it's the feature exemplar
    if (params.xmplr_as_feature) {
        size_t xmplar_f_pos = _ctable.get_arity();
        for (auto& sf : sf_pop) {
            auto xmplar_f_it = sf.second.find(xmplar_f_pos);
            if (xmplar_f_it == sf.second.end()) {
                logger().debug("The exemplar feature has not been selected, "
                               "that exemplar must be pretty bad! As a result "
                               "one more feature is returned by feature selection "
                               "(as we obviously can't remove the exemplar feature).");
            } else {
                sf.second.erase(xmplar_f_it);
                logger().debug("Remove the exemplar feature");
            }
        }
    }
}

feature_set_pop feature_selector::operator()(const combo::combo_tree& xmplr)
{
    // Overwrite some parameters (it's OK to be overwritten because
    // that struct, festor, has been been passed by copy in
    // deme_expander::create_deme
    preprocess_params(xmplr);

    // Build ctable, possibly different than _ctable, used for feature
    // selection
    CTable fs_ctable = build_fs_ctable(xmplr);

    // Feature selection itself. Returns a population of feature sets
    feature_set_pop sf_pop = select_feature_sets(fs_ctable, params.fs_params);

    // Select the top params.n_demes feature sets
    feature_set_pop top_sfs = select_top_feature_sets(sf_pop);

    return top_sfs;
}

csc_feature_set_pop feature_selector::rank_feature_sets(const feature_set_pop& fs_pop) const {
    csc_feature_set_pop res;    // hold all the feature sets ranked by diversity

    // initialize csc_fs_pop (the temporary structure holding all the
    // feature sets not yet ranked by diversity)
    typedef pair<composite_score, feature_set> csc_feature_set;
    vector<csc_feature_set> csc_fs_seq;
    for (const auto& fs : fs_pop)
        csc_fs_seq.emplace_back(composite_score(fs.first, fs.second.size()),
                                fs.second);

    // csc_feature_set less_than function
    auto csc_fs_lt = [](const csc_feature_set& csc_fs_l,
                        const csc_feature_set& csc_fs_r) {
        return csc_fs_l.first < csc_fs_r.first;
    };

    while (!csc_fs_seq.empty()) {
        // assign to all elements of csc_fs_seq the right diversity penality
        for (csc_feature_set& csc_fs : csc_fs_seq) {
            // compute all mis between csc_fs and current res
            vector<double> mis;
            for (const auto resv : res)  // WARNING: highly pessimized
                mis.push_back(mi(resv.second, csc_fs.second));

            // aggregate the results (here max)
            double agg_mis = *boost::max_element(mis);

            // compute and update the diversity penalty
            double dp = params.diversity_pressure * agg_mis;
            csc_fs.first.set_diversity_penalty(dp);
        }

        // insert the best candidate in res and delete it from csc_fs_pop
        auto mit = boost::max_element(csc_fs_seq, csc_fs_lt);
        res.insert(*mit);
        csc_fs_seq.erase(mit);
    }

    return res;
}

double feature_selector::mi(const feature_set& fs_l,
                            const feature_set& fs_r) const
{
    if (params.diversity_pressure < 0)
        return mutualInformationBtwSets(_ctable, fs_l, fs_r);
    else {
        // Compute the average mutual informations between subsets of
        // size diversity_interaction + 1 of fs_l and fs_r
        unsigned sss = params.diversity_pressure + 1;
        double mi = 0.0;
        // determine powersets of fs_l and fs_r, each containing
        // feature sets of size diversity_interaction + 1
        auto pfs_l = powerset(fs_l, sss, true);
        auto pfs_r = powerset(fs_r, sss, true);
        for (const feature_set& sub_fs_l : pfs_l)
            for (const feature_set& sub_fs_r : pfs_r)
                mi += mutualInformationBtwSets(_ctable, sub_fs_l, sub_fs_r);
        return mi / (pfs_l.size() * pfs_r.size());
    }
}

} // ~namespace moses
} // ~namespace opencog
