/** deme_expander.cc ---
 *
 * Copyright (C) 2013 OpenCog Foundation
 * Copyright (C) 2014 Aidyia Limited
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

#include <string>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>

#include <opencog/util/random.h>

#include "deme_expander.h"

namespace opencog {
namespace moses {

deme_expander::deme_expander(const type_tree& type_signature,
                             const reduct::rule& si_ca,
                             const reduct::rule& si_kb,
                             behave_cscore& sc,
                             optimizer_base& opt,
                             const deme_parameters& pa,
                             const subsample_deme_filter_parameters& fp) :
    _optimize(opt), _type_sig(type_signature), simplify_candidate(si_ca),
    simplify_knob_building(si_kb), _cscorer(sc), _params(pa), _filter_params(fp)
{
    random_shuffle_gen = [&](ptrdiff_t i) { return randGen().randint(i); };
}

string_seq deme_expander::fs_to_names(const std::set<arity_t>& fs,
                                      const string_seq& ilabels) const
{
    std::vector<std::string> fs_names;
    for (arity_t i : fs)
        fs_names.push_back(ilabels[i]);
    return fs_names;
}

void deme_expander::log_selected_feature_sets(const feature_set_pop& sf_pop,
                                              const feature_set& xmplr_features,
                                              const string_seq& ilabels) const
{
    unsigned sfps = sf_pop.size(), sfi = 0;

    OC_ASSERT(sfps == _demeIDs.size(), "The code is probably not robust enough");
    for (const auto& sf : sf_pop) {
        logger().info() << "Breadth-first expansion for deme : "
                        << _demeIDs[sfi];
        logger().info() << "Selected " << sf.second.size()
                        << " features for representation building";
        auto xmplr_sf = set_intersection(sf.second, xmplr_features);
        auto new_sf = set_difference(sf.second, xmplr_features);
        logger().info() << "Of these, " << xmplr_sf.size()
                        << " are already in the exemplar, and " << new_sf.size()
                        << " are new.";

        ostreamContainer(logger().info() << "Selected features which are in the exemplar: ",
                         fs_to_names(xmplr_sf, ilabels), ",");
        ostreamContainer(logger().info() << "Selected features which are new: ",
                         fs_to_names(new_sf, ilabels), ",");
        sfi++;
    }
}

std::vector<std::set<TTable::value_type>> deme_expander::subsample_by_time() const
{
    std::vector<std::set<TTable::value_type>> ignore_timestamps_per_ss_deme;
    unsigned n_ss_demes = _filter_params.n_subsample_demes;

    if (n_ss_demes > 1) {       // subsampling deme is enabled
        std::set<TTable::value_type> timestamps =
            _cscorer.get_ctable().get_timestamps();
        unsigned tt_size = timestamps.size();
        std::vector<TTable::value_type> timestamps_vec(timestamps.begin(),
                                                       timestamps.end());

        if (!_filter_params.contiguous_time) {
            logger().debug() << "Random subsampling by time";
            boost::random_shuffle(timestamps_vec, this->random_shuffle_gen);
        }

        // Divide the vector into n_ss_demes equal parts
        unsigned seg_size = tt_size / n_ss_demes;
        for (auto from = timestamps_vec.cbegin(), to = from + seg_size;
             from != timestamps_vec.cbegin() + n_ss_demes * seg_size;
             from += seg_size, to += seg_size)
        {
            ignore_timestamps_per_ss_deme.emplace_back(from, to);
            if (_filter_params.contiguous_time) {
                logger().debug() << "Discard segment from " << *from
                                 << " to " << *(from + (seg_size - 1))
                                 << " (included)";
            }
        }
    }
    else {                      // subsampling deme is disabled
        ignore_timestamps_per_ss_deme.emplace_back();
    }
    return ignore_timestamps_per_ss_deme;
}

std::vector<std::set<unsigned>> deme_expander::subsample_by_row() const
{
    std::vector<std::set<unsigned>> ignore_row_idxs_per_ss_deme;
    unsigned n_ss_demes = _filter_params.n_subsample_demes;

    if (n_ss_demes > 1) {       // subsampling deme is enabled
        unsigned usize = _cscorer.get_ctable_usize();

        // Create a vector [0, usize)
        auto rng = boost::irange(0U, usize);
        std::vector<unsigned> row_idxs(rng.begin(), rng.end());

        // Randomly shuffle the order
        boost::random_shuffle(row_idxs, random_shuffle_gen);

        // Divide the vector into n_ss_demes equal parts
        unsigned seg_size = usize / n_ss_demes;
        for (auto it = row_idxs.cbegin();
             it != row_idxs.cbegin() + n_ss_demes * seg_size;
             it += seg_size)
            ignore_row_idxs_per_ss_deme.emplace_back(it, it + seg_size);
    } else {
        ignore_row_idxs_per_ss_deme.emplace_back();
    }

    return ignore_row_idxs_per_ss_deme;
}

void deme_expander::create_demeIDs(int n_expansions)
{
    // Define the demeIDs of the demes to be spawned
    _demeIDs.clear();
    if (_params.fstor && _params.fstor->params.n_demes > 1) {
        for (unsigned i = 0; i < _params.fstor->params.n_demes; i++)
            _demeIDs.emplace_back(n_expansions + 1, i);
    } else
        _demeIDs.emplace_back(n_expansions + 1);
}

bool deme_expander::create_representations(const combo_tree& exemplar)
{
    // 'On-the-fly' (dynamic, boosted) feature selection.
    // This limits the number of features that will be used to build
    // the deme to a smaller, more manageable number.  This is extremely
    // useful when the dataset has thousands of features; pruning these
    // to a few hundred or a few dozen sharply reduces the number of knobs
    // in the representation.  Fewer knobs means a much smaller search
    // space, which can be explored more quickly.  It also means a smaller
    // rep, which can be reduced and evaluated more quickly.
    //
    // Typically (depending on the options) the feature selection
    // focuses on those features that correlate well with the incorrect
    // predictions of the current exemplar.  The idea is that these can
    // be used to fix the exemplar.
    //
    // This differs sharply from static, one-time-only, up-front feature
    // pre-selection, which only limits the total number of features that
    // MOSES can work with.
    std::vector<operator_set> ignore_ops_seq, considered_args_seq;
    std::vector<combo_tree> xmplr_seq;
    if (_params.fstor) {
        // Copy, as any change in the parameters will not be remembered.
        feature_selector festor = *_params.fstor;

        // Return multiple sets of selected features.  Each feature set
        // is a collection of integer-valued column indexes; with zero
        // denoting the left-most column corresponds.
        auto pop_of_selected_feats = festor(exemplar);

        // Get the set of features used in the exemplar.
        auto xmplr_features = get_argument_abs_idx_from_zero_set(exemplar);

        // Get feature labels (column labels) corresponding to all the features.
        const auto& ilabels = festor._ctable.get_input_labels();

        if (festor.params.n_demes > 1)
            logger().info() << "Breadth-first deme expansion "
                            << "(same exemplar, multiple feature sets): "
                            << festor.params.n_demes << " demes";

        log_selected_feature_sets(pop_of_selected_feats, xmplr_features, ilabels);

        logger().debug() << "Post-treatment: possibly enforce some features and prune"
                         << " exemplars for each deme";

        // pop_of_selected_feats is a set of feature sets. We will
        // create a representation, and a deme, for each distinct
        // feature set.
        unsigned sfi = 0;
        for (auto& selected_feats : pop_of_selected_feats) {
            logger().debug() << "Deme " << _demeIDs[sfi] << ":";

            // Either prune the exemplar, or add all of the features used
            // by the exemplar, to each feature set.  We need to do this
            // because ... ??? why?  In particular, why would we ever
            // want to prune the exemplar? I can't figure this out.
            if (festor.params.prune_xmplr) {
                auto xmplr_nsf = set_difference(xmplr_features,
                                                selected_feats.second);
                if (xmplr_features.empty())
                    logger().debug() << "No feature to prune in the "
                                     << "exemplar for deme " << _demeIDs[sfi];
                else {
                    ostreamContainer(logger().debug() <<
                                     "Prune the exemplar from non-selected "
                                     "features for deme " << _demeIDs[sfi]
                                     << ": ",
                                     fs_to_names(xmplr_nsf, ilabels));
                }
                xmplr_seq.push_back(prune_xmplr(exemplar, selected_feats.second));
            }
            else {
                logger().debug() << "Do not prune the exemplar from "
                                 << "non-selected features "
                                 << "for deme " << _demeIDs[sfi];
                // Insert exemplar features as they are not pruned
                selected_feats.second.insert(xmplr_features.begin(), xmplr_features.end());
                xmplr_seq.push_back(exemplar);
            }

            if (!festor.params.enforce_features.empty()) {
                // Enforce a set of features
                feature_set enforced_features = festor.sample_enforced_features();
                selected_feats.second.insert(
                    enforced_features.begin(), enforced_features.end());
            }

            // add the complement of the selected features to ignore_ops
            unsigned arity = festor._ctable.get_arity();
            std::set<arity_t> ignore_cols;
            std::set<arity_t>::iterator end = selected_feats.second.end();
            vertex_set ignore_ops, considered_args;

            for (unsigned i = 0; i < arity; i++) {
                argument arg(i + 1);
                if (selected_feats.second.find(i) == end) {
                    ignore_cols.insert(i);
                    ignore_ops.insert(arg);
                }
                else {
                    considered_args.insert(arg);
                }
            }
            _ignore_cols_seq.push_back(ignore_cols);
            ignore_ops_seq.push_back(ignore_ops);
            considered_args_seq.push_back(considered_args);

            sfi++;
        }
    }
    else {                      // no dynamic feature selection
        ignore_ops_seq.push_back(_params.ignore_ops);
        xmplr_seq.push_back(exemplar);
    }

    for (unsigned i = 0; i < xmplr_seq.size(); i++) {
        if (logger().isDebugEnabled()) {
            logger().debug() << "Attempt to build rep from exemplar "
                             << "for deme " << _demeIDs[i]
                             << " : " << xmplr_seq[i];
            if (!considered_args_seq.empty())
                ostreamContainer(logger().debug() << "Using arguments: ",
                                 considered_args_seq[i]);
        }

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

    return not _reps.empty();
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

/**
 * Create one or more demes.
 *
 * Recall that a single deme consists of:
 * -- A "representation", which is an exemplar decordated with knobs,
 * -- A "field_set", which maps knobs in the representation, to a linear
 *    array of knob setting locations,
 * -- A collection of scored instances, where each instance is a linear
 *    array of knob settings.
 *
 * By default, only one deme is created, by (randomly) using all
 * possible features in knobs randomly attached to the given exemplar.
 *
 * If dynamic feature selection is enabled, then only the selected
 * features will be used in creating the representation.
 *
 * More than one deme will be created if the feature-selection n_demes
 * (aka fs-demes) option is set to a value greater than one. This causes
 * the feature selector to return multiple different sets of features.
 * In this case, a different representation is built for each feature
 * set, and thus, a different deme.
 *
 * XXX TODO I honestly just don't see the utility of this multi-deme
 * creation mechanism.  Feature selection is a very crude mechanism,
 * and the representation is just randomly peppered with knobs. By
 * experience, almost all knobs are completely worthless, except for
 * a small handful (maybe half a dozen, maybe a few dozen, depending on
 * the particular problem type. As a ratio, I've typically seen that
 * 90% to 99.5% of all knobs are useless/ineffective/bad. There are
 * some explicit graphs demonstrating this in the diary archive folder.)
 * Given that almost all of the selected features end up attached to
 * useless knobs, and are thus discarded after deme optimization and
 * merging, I just completely fail to see how a minor twiddle in the
 * dynamic feature set makes any difference.  I mean, if you want to
 * explore more stuff, why not just use a slightly larger feature set
 * size?  For example, just use the union of the top N feature sets.
 * This would give roughly the same thing, wich a lot less code
 * complexity, it seems to me.  So unless we have some very explicit
 * evidence suggesting that this mechanism does something positive,
 * I suggest that it should be removed. -- Linas June 2014
 */
bool deme_expander::create_demes(const combo_tree& exemplar, int n_expansions)
{
    using namespace reduct;

    OC_ASSERT(_ignore_cols_seq.empty());
    OC_ASSERT(_reps.empty());
    OC_ASSERT(_demes.empty());

    create_demeIDs(n_expansions);

    create_representations(exemplar);

    // Create empty demes with their IDs
    for (unsigned i = 0; i < _reps.size(); i++) {
        if (_filter_params.n_subsample_demes > 1) {
            _demes.emplace_back();
            for (unsigned j = 0; j < _filter_params.n_subsample_demes; j++)
                _demes.back().emplace_back(_reps[i].fields(),
                                           demeID_t(n_expansions, i, j));
        } else {
            _demes.emplace_back(1, deme_t(_reps[i].fields(), _demeIDs[i]));
        }
    }

    return true;
}


void deme_expander::optimize_demes(int max_evals, time_t max_time)
{
    int max_evals_per_deme = max_evals / _demes.size();
    // We set to 1 n_ss_demes in case n_subsample_demes was set to 0
    unsigned n_ss_demes =
        std::max(_filter_params.n_subsample_demes, 1U);
    max_evals_per_deme /= n_ss_demes;
    for (unsigned i = 0; i < _demes.size(); i++)
    {
        if (n_ss_demes > 1)
            logger().debug("Subsample data and optimize SS-demes");

        // For each subsample-deme hold set of row indexes to remove
        std::vector<std::set<unsigned>> ignore_row_idxs_per_ss_deme =
            subsample_by_row();

        // For each subsample-deme hold set of timestamps to remove
        std::vector<std::set<TTable::value_type>> ignore_timestamps_per_ss_deme =
            subsample_by_time();

        for (unsigned j = 0; j < n_ss_demes; j++)
        {
            if (logger().isDebugEnabled()) {
                std::stringstream ss;
                ss << "Optimize deme " << _demes[i][j].getID() << "; "
                   << "max evaluations allowed: " << max_evals_per_deme;
                logger().debug(ss.str());
            }

            if (_params.fstor) {
                // Attempt to compress the CTable further (to optimize and
                // update max score)
                _cscorer.ignore_cols(_ignore_cols_seq[i]);

                // Compute the max target for that deme (if features have been
                // dynamically selected, it might be less that the global target;
                // that is, the deme might not be able to reach the best score.)
                //
                // TODO: DO NOT CHANGE THE MAX SCORE IF USER SET IT: BUT THAT
                // OPTION ISN'T GLOBAL WHAT TO DO?
                //
                // But why would we want to over-ride the best-possible score?
                // The point here is to figure out when to stop working on the
                // deme, and not when to stop working onthe problem as a whole,
                // right?
                score_t deme_target_score = _cscorer.best_possible_score();
                logger().info("Inferred target score for that deme = %g",
                              deme_target_score);
                logger().info("It appears there is an algorithmic bug in "
                              "precision_bscore::best_possible_bscore. "
                              "Till not fixed we shall not rely on it to "
                              "terminate deme search. Except I think this "
                              "is fixed now. It needs review and testing.");

                // TODO: re-enable that once best_possible_bscore is fixed
                // I think its now fixed, but I'm not sure.  It needs to be
                // reviewed and tested.
#if THIS_IS_DISABLED_UNTIL_ABOVE_IS_FIXED
                // negative min_improv is interpreted as percentage of
                // improvement, if so then don't substract anything, since in that
                // scenario the absolute min improvent can be arbitrarily small
                score_t actual_min_improv = std::max(_cscorer.min_improv(),
                                                     (score_t)0);
                deme_target_score -= actual_min_improv;
                logger().info("Subtract %g (minimum significant improvement) "
                              "from the target score to deal with float "
                              "imprecision = %g",
                              actual_min_improv, deme_target_score);

                // update max score optimizer
                _optimize.opt_params.terminate_if_gte = deme_target_score;
#endif // THIS_IS_DISABLED_UNTIL_ABOVE_IS_FIXED
            }

            if (n_ss_demes > 1) {
                if (_filter_params.by_time) {
                    OC_ASSERT(ignore_timestamps_per_ss_deme.size() == n_ss_demes,
                              "Apparently subsample_by_time() was not able to work properly. "
                              "Are you sure your data has timestamps, "
                              "and those are properly loaded (see option --timestamp-feature)");
                    _cscorer.ignore_rows_at_times(ignore_timestamps_per_ss_deme[j]);
                }
                else
                    _cscorer.ignore_rows(ignore_row_idxs_per_ss_deme[j]);
            }

            // Optimize
            complexity_based_scorer cpx_scorer =
                complexity_based_scorer(_cscorer, _reps[i], _params.reduce_all);
            _optimize(_demes[i][j], cpx_scorer, max_evals_per_deme, max_time);
        }

        if (n_ss_demes > 1)
            logger().debug("Done optimizing SS-demes");
    }

    if (_params.fstor) {
        // reset scorer to use all variables (important so that
        // behavioral score is consistent across generations
        // XXX FIXME this is a bug .. the user may have specified that
        // certain incdexes should be ignored, and this just wipes
        // those out...
        _cscorer.ignore_cols(std::set<arity_t>());
    }

    if (n_ss_demes > 1) {
        // reset scorer to use all rows (important so that the
        // ss_filter and the behavioral score is calculated over the
        // whole data set)
        _cscorer.ignore_rows(std::set<unsigned>());
    }
}

void deme_expander::free_demes()
{
    _ignore_cols_seq.clear();
    _demes.clear();
    _reps.clear();
}

unsigned deme_expander::total_evals()
{
    unsigned res = 0;
    for (const auto& ss_demes : _demes)
        for (const auto& deme : ss_demes)
            res += deme.n_evals;
    return res;
}

} // ~namespace moses
} // ~namespace opencog
