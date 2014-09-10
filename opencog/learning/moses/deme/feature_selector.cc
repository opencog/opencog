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

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <boost/range/algorithm/max_element.hpp>

#include "feature_selector.h"
#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/table/table_io.h>
#include <opencog/util/oc_omp.h>
#include <opencog/util/random.h>
#include <opencog/util/lazy_random_selector.h>
#include <opencog/util/jaccard_index.h>

// Name given to the feature corresponding to the output of the
// exemplar
#define EXEMPLAR_FEATURE_NAME "__exemplar_feature__"

namespace opencog {
namespace moses {

namespace ba = boost::accumulators;

feature_selector::feature_selector(const combo::CTable& ctable,
                                   const feature_selector_parameters& festor_params)
    : params(festor_params), _ctable(ctable) {}

feature_selector::feature_selector(const combo::Table& table,
                                   const feature_selector_parameters& festor_params)
    : params(festor_params), _ctable(table.compressed()) {}

// Do some sanity checking and mangling on the parameters.
void feature_selector::preprocess_params(const combo::combo_tree& xmplr)
{
    // Note that this is going to overwrite some parameters. This is OK
    // because this struct is being copied before being called by
    // deme_expander::create_deme

    // Get the set of features of the exemplar.
    auto xmplr_features = get_argument_abs_idx_from_zero_set(xmplr);

    // Get labels corresponding to all the features
    const auto& ilabels = _ctable.get_input_labels();

    // Names of the exemplar features
    std::vector<std::string> xmplr_feature_names;
    for (arity_t i : xmplr_features)
        xmplr_feature_names.push_back(ilabels[i]);

    // Some of the feature-selection ealgos, such as "inc" and "smd",
    // wortk with a pool of features at a time.  These work best if
    // the feature pool is seeded with the features in the exemplar.
    // That way, any new features selected by these two algos will 
    // combine well with the exemplar.  This is un-needed/unwanted,
    // for the "simple" selector.
    if (params.init_xmplr_features) {
        auto& pif = params.fs_params.initial_features;
        pif.insert(pif.end(),
                   xmplr_feature_names.begin(),
                   xmplr_feature_names.end());
        // We need to increase the size to make room for the new
        // features (i.e. for those not already in the exemplar)
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

    // Instead of using params.increase_target_size, one can ignore
    // the features in the exemplar during feature selection. That
    // way, fs will never pick features in the exemplar.
    params.ignored_features = params.ignore_xmplr_features ?
        xmplr_features : std::set<arity_t>();

    // Or one can use the output of the exemplar as an initial feature
    // Huh ???
    if (params.xmplr_as_feature) {
        params.fs_params.initial_features.push_back(EXEMPLAR_FEATURE_NAME);
        ++params.fs_params.target_size;
    }
}

/// XXX TODO Explain what this function does. Why does it create a second
/// table that is different than _ctable? How are these two different
/// tables used?  Why do we need two different tables?  WTF.  This is 
/// confusing.
CTable feature_selector::build_fs_ctable(const combo_tree& xmplr) const
{
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
            consider_row = (predicted_out == combo::id::logical_true);
        }
        if (consider_row && params.restrict_incorrect) {
            // Use only rows where the model is just plain wrong.  The
            // feature selector will then look for any features that
            // correlate well with these incorrect answers. The hope
            // is that these will be used to build a more accurate
            // models.
            //
            // For compressed tables, it can happen that sometimes the
            // same row has multiple different outcomes (different
            // values for the output feature -- i.e. the uncompressed
            // table had multiple rows with the same input features).
            // In this case, "plain wrong" is a shade of grey.  Below,
            // we use most_frequent() to determine wrongness, but this
            // could be relaxed.
            //
            const TimedCounter& tc = vct.second;
            vertex actual_out = tc.most_frequent();
            consider_row = predicted_out != actual_out;
        }

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

    // subsample
    if (params.subsampling_ratio < 1.0) {
        if (params.subsampling_by_time) {
            std::set<TTable::value_type> timestamps = fs_ctable.get_timestamps(),
                rm_timestamps;
            unsigned timestamps_size = timestamps.size(),
                rm_size = (1.0 - params.subsampling_ratio) * timestamps_size;
            dorepeat(rm_size)
                rm_timestamps.insert(randset_erase(timestamps));
            fs_ctable.remove_rows_at_times(rm_timestamps);
        } else {
            std::set<unsigned> rm_row_idxs;
            unsigned fs_ctable_usize = fs_ctable.uncompressed_size(),
                rm_size = (1.0 - params.subsampling_ratio) * fs_ctable_usize;
            lazy_random_selector rm_selector(fs_ctable_usize);
            dorepeat(rm_size)
                rm_row_idxs.insert(rm_selector.select());
            fs_ctable.remove_rows(rm_row_idxs);
        }
    }

    logger().debug("CTable size for feature selection = %u",
                   fs_ctable.size());

    if (logger().isFineEnabled()) {
        logger().fine("fs_ctable:");
        std::stringstream ss;
        ostreamCTable(ss, fs_ctable);
        logger().fine() << ss.str();
    }

    return fs_ctable;
}

feature_set_pop feature_selector::select_top_feature_sets(const feature_set_pop& fss) const
{
    unsigned res_size = std::min(params.n_demes, (unsigned)fss.size());
    if (params.diversity_pressure > 0.0) {
        csc_feature_set_pop ranked_fss = rank_feature_sets(fss);
        feature_set_pop res;
        unsigned i = 0;
        for (auto it = ranked_fss.begin(); i < res_size; ++i, ++it)
            res.insert({it->first.get_penalized_score(), it->second});
        return res;
    } else {
        return feature_set_pop(fss.begin(), std::next(fss.begin(), res_size));
    }
}

void feature_selector::log_stats_top_feature_sets(const feature_set_pop& top_fs) const
{
    logger().info() << "Number of demes selected: " << top_fs.size();

    // There aren't any stats, if there's just one feature set!
    // (which is the case when diversity is turned off).
    if (1 == top_fs.size())
        return;

    // Accumulator to gather statistics about mutual information
    // between feature set candidates
    typedef ba::accumulator_set<double,
                                ba::stats<ba::tag::count,
                                          ba::tag::mean,
                                          ba::tag::variance,
                                          ba::tag::min,
                                          ba::tag::max> > accumulator_t;
    accumulator_t diversity_acc, score_acc;

    // Stats about score and diversity
    for (auto i_it = top_fs.cbegin(); i_it != top_fs.cend(); ++i_it) {
        score_acc(i_it->first);
        for (auto j_it = top_fs.cbegin(); j_it != i_it; ++j_it) {
            float sim = params.diversity_jaccard ?
                jaccardIndex(i_it->second, j_it->second)
                : mi(i_it->second, j_it->second);
            diversity_acc(sim);
        }
    }

    logger().info() << "Feature sets score stats (accounting for diversity) = "
                    << "(count: " << ba::count(score_acc)
                    << ", mean: " << ba::mean(score_acc)
                    << ", std dev: " << sqrt(ba::variance(score_acc))
                    << ", min: " << ba::min(score_acc)
                    << ", max: " << ba::max(score_acc) << ")";

    logger().info() << "feature sets diversity stats (MI) = "
                    << "(count: " << ba::count(diversity_acc)
                    << ", mean: " << ba::mean(diversity_acc)
                    << ", std dev: " << sqrt(ba::variance(diversity_acc))
                    << ", min: " << ba::min(diversity_acc)
                    << ", max: " << ba::max(diversity_acc) << ")";
}

void feature_selector::remove_useless_features(feature_set_pop& sf_pop) const
{
    if (not params.xmplr_as_feature)
        return;

    // Remove last feature if it's the feature exemplar
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

feature_set_pop feature_selector::operator()(const combo::combo_tree& xmplr)
{
    // Overwrite some parameters (it's OK to be overwritten because
    // that struct, festor, has been been passed by copy in
    // deme_expander::create_deme
    preprocess_params(xmplr);

    // Build a ctable, possibly different than _ctable, used for feature
    // selection
    CTable fs_ctable = build_fs_ctable(xmplr);

    // Feature selection itself. Returns a population of feature sets
    feature_set_pop sf_pop = select_feature_sets(fs_ctable, params.fs_params);

    // Cap, retain best feature sets
    if (0 < params.diversity_cap and params.diversity_cap < sf_pop.size())
        sf_pop.erase(std::next(sf_pop.begin(), params.diversity_cap), sf_pop.end());

    // Select the top params.n_demes feature sets
    feature_set_pop top_sfs = select_top_feature_sets(sf_pop);

    // Display stats about diversity of the top feature sets
    log_stats_top_feature_sets(top_sfs);

    return top_sfs;
}

csc_feature_set_pop feature_selector::rank_feature_sets(const feature_set_pop& fs_pop) const {
    logger().info() << "Ranking feature sets (acounting for feature diversity)";

    csc_feature_set_pop res;    // to put all feature sets ranked by diversity

    // holds the last feature set inserted (to compute the next mis)
    csc_feature_set_pop::const_iterator last_fs_cit = res.end();

    // initialize csc_fs_pop (the temporary structure holding all the
    // feature sets not yet ranked by diversity).  Each element will
    // contain the current diversity penalty between itself and the
    // ranked features sets (in res). Because the way the diversity
    // penalties are aggregated (max) we only need to compute the
    // diversity between the elements and the lastly added feature set
    // in res.
    typedef std::pair<composite_score, feature_set> csc_feature_set;
    std::vector<csc_feature_set> csc_fs_seq;
    for (const auto& fs : fs_pop)
        csc_fs_seq.emplace_back(composite_score(fs.first, fs.second.size()),
                                fs.second);

    // csc_feature_set less_than function
    auto csc_fs_lt = [](const csc_feature_set& csc_fs_l,
                        const csc_feature_set& csc_fs_r) {
        return csc_fs_l.first < csc_fs_r.first;
    };

    // Similarity to penalty function
    auto sim_to_penalty = [&](double sim) {
        return params.diversity_pressure * sim;
    };

    while (!csc_fs_seq.empty()) {

        if (last_fs_cit != res.end()) {
            // assign to all elements of csc_fs_seq the right diversity penality
            OMP_ALGO::for_each(csc_fs_seq.begin(), csc_fs_seq.end(),
                               [&](csc_feature_set& csc_fs) {
                // compute penalty between csc_fs and the last inserted
                // feature set

                // feature set similarity measure
                float sim = params.diversity_jaccard ?
                    jaccardIndex(last_fs_cit->second, csc_fs.second)
                    : mi(last_fs_cit->second, csc_fs.second);
                float last_dp = sim_to_penalty(sim);

                // // DEBUG
                // ostreamContainer(logger().debug() << "last_fs_cit->second = ",
                //                  last_fs_cit->second);
                // logger().debug() << "With composite score: " << last_fs_cit->first;
                // ostreamContainer(logger().debug() << "Feature set: ",
                //                  csc_fs.second);
                // logger().debug() << "With composite score (BEFORE): " << csc_fs.first;
                // logger().debug() << "fsmi = " << fsmi;
                // logger().debug() << "last_dp = " << last_dp;
                // // ~DEBUG

                // aggregate the results (here max)
                float agg_dp = std::max(csc_fs.first.get_diversity_penalty(),
                                        last_dp);

                // compute and update the diversity penalty
                csc_fs.first.set_diversity_penalty(agg_dp);

                // // DEBUG
                // logger().debug() << "With composite score (AFTER): " << csc_fs.first;
                // // ~DEBUG

            });
        }

        // insert the best candidate in res and delete it from csc_fs_pop
        auto mit = OMP_ALGO::max_element(csc_fs_seq.begin(), csc_fs_seq.end(),
                                         csc_fs_lt);
        last_fs_cit = res.insert(*mit);
        csc_fs_seq.erase(mit);
    }

    logger().info() << "Feature sets ranked";

    if (logger().isFineEnabled()) {
        for (const csc_feature_set& cfs : res) {
            ostreamContainer(logger().fine() << "Feature set: ", cfs.second);
            logger().fine() << "With composite score: " << cfs.first;
        }
    }

    return res;
}

double feature_selector::mi(const feature_set& fs_l,
                            const feature_set& fs_r) const
{
    if (params.diversity_interaction < 0)
        return mutualInformationBtwSets(_ctable, fs_l, fs_r);
    else {
        // Compute the average mutual informations between subsets of
        // size diversity_interaction + 1 of fs_l and fs_r
        unsigned sss = params.diversity_interaction + 1;
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

feature_set feature_selector::sample_enforced_features() const {
    // Sample the enforced features
    std::vector<std::string> sampled_features;
    for (auto& p : params.enforce_features) {
        if (biased_randbool(p.second)) {
            sampled_features.push_back(p.first);
        }
    }

    if (logger().isDebugEnabled()) {
        std::stringstream ss;
        ostreamContainer(ss << "Enforce features: ", sampled_features);
        logger().debug() << ss.str();
    }

    // Convert that into their indexes
    const auto& ilabels = _ctable.get_input_labels();
    auto vec_idx = get_indices(sampled_features, ilabels);
    return feature_set(vec_idx.begin(), vec_idx.end());
}

} // ~namespace moses
} // ~namespace opencog
