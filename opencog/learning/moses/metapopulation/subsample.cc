/** subsample.cc ---
 *
 * Copyright (C) 2014 Aidyia Limited
 *
 * Author: Author: Nil Geisweiller
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

#include <boost/range/algorithm/sort.hpp>

#include <boost/range/irange.hpp>

#include <opencog/util/lazy_random_selector.h>

#include "metapopulation.h"

namespace opencog {
namespace moses {

void metapopulation::recompute_scores_over_whole_dataset(
    std::vector<std::vector<deme_t>>& all_demes,
    const boost::ptr_vector<representation>& reps)
{
    // Only bother if subsampling filter is enabled
    if (_filter_params.n_subsample_demes > 1) {
        logger().fine() << "Re-evaluate scores over the whole dataset";

        for (unsigned i = 0; i < all_demes.size(); ++i) {
            for(deme_t& ss_deme : all_demes[i]) {
                if (logger().isFineEnabled())
                    logger().fine() << "SS-Deme: " << ss_deme.getID();

                for(auto& sc_inst : ss_deme) {
                    combo_tree tr = reps[i].get_candidate(sc_inst, true);

                    if (logger().isFineEnabled())
                        logger().fine() << "Re-evaluate candidate: " << tr;

                    // Evaluate the candidate over the whole dataset
                    // (normally restored by now)
                    composite_score sc = _cscorer.get_cscore(tr);

                    if (logger().isFineEnabled())
                        logger().fine() << "Composite score = " << sc;

                    // Overwrite the deme scored instance with the new
                    // (correct) score
                    select_tag()(sc_inst) = sc;
                }
            }
        }
    }
}

std::vector<bool> metapopulation::ss_filter(
    const std::vector<std::vector<deme_t>>& all_demes,
    const boost::ptr_vector<representation>& reps) const
{
    // Breadth first demes pass vector. By default they all pass the filter.
    std::vector<bool> bfpass(all_demes.size(), true);

    // If subsampling filter is enabled
    if (_filter_params.n_subsample_demes > 1) {
        // Select the best breadth first demes (best in the sense
        // that they have the highest agreement (score or activation
        // pattern, depending on user options)
        if (_filter_params.n_best_bfdemes > 0) {
            unsigned n_best_bfdemes = std::min(_filter_params.n_best_bfdemes,
                                               (unsigned)all_demes.size());

            logger().debug() << "Enter n best BFDemes filter, select "
                             << n_best_bfdemes << " BFDemes";

            // Compute aggregated agreement distances
            std::vector<float> ag_dsts(all_demes.size());
            for (unsigned i = 0; i < all_demes.size(); ++i) {
                logger().debug() << "BFDeme : " << i;

                tanimoto_acc_t acc;
                ss_tanimoto_stats(reps[i], all_demes[i], acc);

                float tanimoto_mean = boost::accumulators::mean(acc),
                    tanimoto_geo_mean = boost::accumulators::geometric_mean_mirror(acc),
                    tanimoto_max = boost::accumulators::max(acc);

                logger().debug() << "Tanimoto mean = " << tanimoto_mean
                                 << ", Tanimoto geometric mean = " << tanimoto_geo_mean
                                 << ", Tanimoto max = " << tanimoto_max;

                // Aggregate agreement distance
                float ag_dst = _filter_params.tanimoto_mean_weight * tanimoto_mean
                    + _filter_params.tanimoto_geo_mean_weight * tanimoto_geo_mean
                    + _filter_params.tanimoto_max_weight * tanimoto_max;

                ag_dsts[i] = ag_dst;

                logger().debug() << "Aggregated agreement distance = " << ag_dst;
            }

            // Select the indexes of worst bfdemes (with largest
            // aggredated agreement distances) to discard
            auto ir = boost::irange(0U, (unsigned)ag_dsts.size());
            std::vector<unsigned> worst_idxs(ir.begin(), ir.end());
            boost::sort(worst_idxs, [&ag_dsts](unsigned i1, unsigned i2) {
                    return ag_dsts[i1] < ag_dsts[i2]; });
            worst_idxs.erase(worst_idxs.begin(), worst_idxs.begin() + n_best_bfdemes);

            // Set to false the non selected BF demes
            for (unsigned idx : worst_idxs)
                bfpass[idx] = false;
        }
        // Select breadth first demes that pass the filters
        else {
            for (unsigned i = 0; i < all_demes.size(); ++i) {
                // Score variance filter
                if (_filter_params.std_dev_threshold
                    < std::numeric_limits<float>::max()) { // enabled
                    bfpass[i] =
                        ss_score_dev_filter(reps[i], all_demes[i]);
                    if (!bfpass[i]) continue;
                }

                // Tanimoto filter
                if (_filter_params.tanimoto_mean_threshold < 1.0
                    or _filter_params.tanimoto_geo_mean_threshold < 1.0
                    or _filter_params.tanimoto_max_threshold < 1.0) {
                    bfpass[i] =
                        ss_tanimoto_filter(reps[i], all_demes[i]);
                }
            }
        }
    }

    return bfpass;
}


bool metapopulation::ss_score_dev_filter(const representation& rep,
                                         const std::vector<deme_t>& ss_demes) const
{
    logger().fine() << "Enter ss_score_dev_filter";

    using namespace boost::accumulators;
    typedef accumulator_set<float,
                            stats<boost::accumulators::tag::count,
                                  boost::accumulators::tag::mean,
                                  boost::accumulators::tag::variance>> stat_acc_t;

    // We sample at most n_top_candidates tuples, because since we are
    // gonna use sampling without replacement for each deme, we can't
    // consider more than the top candidates set.
    unsigned n_top_candidates = _filter_params.n_top_candidates,
        n_tuples = _filter_params.n_tuples,
        base = n_top_candidates,
        n_possible_tuples = (unsigned)pow(base, ss_demes.size()),
        sample_tuples = std::min(n_possible_tuples, n_tuples);

    // Define a selector for all possible tuples
    lazy_random_selector selector(n_possible_tuples);

    stat_acc_t mean_std_acc;    // accumulator for mean standard deviation

    for (unsigned i_tuple = 0; i_tuple < sample_tuples; ++i_tuple) {
        if (logger().isFineEnabled())
            logger().fine() << "Tuple iteration: " << i_tuple;

        // Pick up a tuple
        unsigned rnd_tuple = selector.select();
        // Decompose tuple into tuple_idxs, where each element is a
        // coefficient in the representation of tuple in base
        // n_top_candidates
        std::vector<unsigned> tuple_idxs;
        for (unsigned i_deme = 0; i_deme < ss_demes.size(); ++i_deme) {
            tuple_idxs.push_back(rnd_tuple % base);
            rnd_tuple /= base;
        }

        stat_acc_t var_score_acc;         // accumulator for score variance

        for (unsigned i_deme = 0; i_deme < ss_demes.size(); ++i_deme) {
            const deme_t& ss_deme = ss_demes[i_deme];
            unsigned i_inst = tuple_idxs[i_deme];
            const scored_instance<composite_score>& inst = ss_deme[i_inst];
            const composite_score& inst_csc = inst.second;
            var_score_acc(inst_csc.get_penalized_score());
        }

        if (logger().isFineEnabled())
            logger().fine() << "Variance for tuple " << i_tuple << ": "
                            << boost::accumulators::variance(var_score_acc);

        mean_std_acc(sqrt(boost::accumulators::variance(var_score_acc)));
    }

    float mean_std = boost::accumulators::mean(mean_std_acc);
    bool pass = mean_std < _filter_params.std_dev_threshold;

    if (logger().isDebugEnabled())
    {
        std::stringstream ss;
        ss << "SS Filter: mean score standard deviation = " << mean_std
           << " over demes";
        for (const auto& deme : ss_demes)
            ss << " " << deme.getID();
        logger().debug() << ss.str();
        if (pass)
            logger().debug() << "Pass filter (below threshold "
                             << _filter_params.std_dev_threshold
                             << ")";
        else
            logger().debug() << "Not pass filter (above or equal to threshold "
                             << _filter_params.std_dev_threshold
                             << ")";
    }

    return pass;
}

float metapopulation::ss_average_agreement(const representation& rep,
                                           std::vector<deme_t>& ss_demes)
{
    logger().fine() << "Enter average_agreement";

    // We sample at most n_top_candidates tuples, because since we are
    // gonna use sampling without replacement for each deme, we can't
    // consider more than the top candidates set.
    unsigned n_top_candidates = _filter_params.n_top_candidates,
        n_tuples = _filter_params.n_tuples,
        base = n_top_candidates,
        exponent = ss_demes.size(),
        n_possible_tuples = (unsigned)pow(base, exponent),
        sample_tuples = std::min(n_possible_tuples, n_tuples);

    // Define a selector for all possible tuples
    lazy_random_selector selector(n_possible_tuples);

    float total_agreement = 0;

    for (unsigned i_tuple = 0; i_tuple < sample_tuples; ++i_tuple) {
        if (logger().isFineEnabled())
            logger().fine() << "Tuple iteration: " << i_tuple;

        // Pick up a tuple
        unsigned rnd_tuple = selector.select();
        // Decompose tuple into tuple_idxs, where each element is a
        // coefficient in the representation of tuple in base
        // n_top_candidates
        std::vector<unsigned> tuple_idxs;
        for (unsigned i_deme = 0; i_deme < exponent; ++i_deme) {
            tuple_idxs.push_back(rnd_tuple % base);
            rnd_tuple /= base;
        }

        // Generate a vector of candidates corresponding to each ss-deme
        std::vector<combo_tree> trs;
        for (unsigned i_deme = 0; i_deme < exponent; ++i_deme) {
            deme_t& deme = ss_demes[i_deme];

            // Get the candidate
            unsigned i_inst = tuple_idxs[i_deme];
            const scored_instance<composite_score>& inst = deme[i_inst];
            combo_tree tr = rep.get_candidate(inst, true);
            trs.push_back(tr);

            if (logger().isFineEnabled())
                logger().fine() << "SS-Deme: " << deme.getID()
                                << " candidate = " << tr;
        }

        for (const auto& io_row : _cscorer.get_ctable()) {
            const auto& irow = io_row.first;
            const auto& orow = io_row.second;

            int agreement = 0;
            for (const combo_tree& tr : trs) {
                auto iv = interpreter_visitor(tr);
                vertex result = boost::apply_visitor(iv, irow.get_variant());
                if (result == id::logical_true)
                    agreement++;
                else
                    agreement--;
            }
            float normalized_agreement = agreement/(float)exponent;
            total_agreement += abs(normalized_agreement) * orow.total_count();

            // if (logger().isFineEnabled())
            //     logger().fine() << "agreement = " << agreement
            //                     << ", normalized_agreement = " << normalized_agreement
            //                     << ", total_agreement = " << total_agreement;
        }
    }

    float mean_agreement =
        total_agreement / (sample_tuples * _cscorer.get_ctable_usize());

    logger().fine() << "mean_agreement = " << mean_agreement;

    return mean_agreement;
}

void metapopulation::ss_tanimoto_stats(const std::vector<combo_tree>& trs,
                                       metapopulation::tanimoto_acc_t& acc) const
{
    // Compute all weighted activations for all combo tree. Grouped by
    // input rows and/or timestamps.
    std::vector<std::vector<float>> weighted_activations(trs.size());
    std::vector<CTable> weighted_activations_grouped_by_timestamps(trs.size());

    for (const auto& io_row : _cscorer.get_ctable()) {
        const auto& irow = io_row.first;
        const auto& orow = io_row.second;
        for (unsigned i = 0; i < trs.size(); i++) {
            // Evaluate combo tree
            auto iv = interpreter_visitor(trs[i]);
            vertex result = boost::apply_visitor(iv, irow.get_variant());

            // Fill weighted activations
            float weight = result == id::logical_true ? orow.total_count() : 0.0;
            weighted_activations[i].push_back(weight);

            // Fill weighted activations grouped by timestamps
            for (const CTable::counter_t::value_type& tcv : orow) {
                CTable& ctable = weighted_activations_grouped_by_timestamps[i];
                CTable::key_type dummy_input;
                TimedValue timed_result({result, tcv.first.timestamp});
                ctable[dummy_input][timed_result] += tcv.second;
            }
        }
    }

#ifdef DOESNT_COMPILE
    // Log the CTable associated with the evaluations, to see the
    // patterns of activations grouped by timestamps
    if (logger().isFineEnabled()) {
        for (unsigned i = 0;
             i < weighted_activations_grouped_by_timestamps.size();
             ++i) {
            const CTable& ctable = weighted_activations_grouped_by_timestamps[i];
            const combo_tree& tr = trs[i];
            ostreamCTableTime(logger().fine()
                              << "CTable (grouped by time) of tree "
                              << tr << ":" << std::endl,
                              ctable.ordered_by_time());
        }
    }
#endif

    // Compute statistic of Tanimoto distance over all pairs of
    // vectors of weighted activations
    for (unsigned i = 0; i < trs.size(); i++)
        for (unsigned j = 0; j < i; j++)
            acc(tanimoto_distance<std::vector<float>, float>(weighted_activations[i],
                                                        weighted_activations[j]));
}

void metapopulation::ss_tanimoto_stats(const representation& rep,
                                       const std::vector<deme_t>& ss_demes,
                                       metapopulation::tanimoto_acc_t& acc) const
{
    // We sample at most n_top_candidates tuples, because since we are
    // gonna use sampling without replacement for each deme, we can't
    // consider more than the top candidates set.
    unsigned n_top_candidates = _filter_params.n_top_candidates,
        n_tuples = _filter_params.n_tuples,
        base = n_top_candidates,
        exponent = ss_demes.size(),
        n_possible_tuples = (unsigned)pow(base, exponent),
        sample_tuples = std::min(n_possible_tuples, n_tuples);

    // Define a selector for all possible tuples
    lazy_random_selector selector(n_possible_tuples);

    for (unsigned i_tuple = 0; i_tuple < sample_tuples; ++i_tuple) {
        if (logger().isFineEnabled())
            logger().fine() << "Tuple iteration: " << i_tuple;

        // Pick up a tuple
        unsigned rnd_tuple = selector.select();
        // Decompose tuple into tuple_idxs, where each element is a
        // coefficient in the representation of tuple in base
        // n_top_candidates
        std::vector<unsigned> tuple_idxs;
        for (unsigned i_deme = 0; i_deme < exponent; ++i_deme) {
            tuple_idxs.push_back(rnd_tuple % base);
            rnd_tuple /= base;
        }

        // Generate a vector of candidates corresponding to each ss-deme
        std::vector<combo_tree> trs;
        for (unsigned i_deme = 0; i_deme < exponent; ++i_deme) {
            const deme_t& deme = ss_demes[i_deme];

            // Get the candidate
            unsigned i_inst = tuple_idxs[i_deme];
            const scored_instance<composite_score>& inst = deme[i_inst];
            combo_tree tr = rep.get_candidate(inst, true);
            trs.push_back(tr);

            if (logger().isFineEnabled())
                logger().fine() << "SS-Deme: " << deme.getID()
                                << " candidate = " << tr;
        }

        ss_tanimoto_stats(trs, acc);
    }
}

bool metapopulation::ss_tanimoto_filter(const representation& rep,
                                        const std::vector<deme_t>& ss_demes) const
{
    logger().fine() << "Enter ss_tanimoto_filter";

    // Compute stats about tanimoto distances between top candidates
    tanimoto_acc_t acc;
    ss_tanimoto_stats(rep, ss_demes, acc);

    // Determine if breadth first deme has passed the tanimoto filter
    float tanimoto_mean = boost::accumulators::mean(acc),
        tanimoto_geo_mean = boost::accumulators::geometric_mean_mirror(acc),
        tanimoto_max = boost::accumulators::max(acc);

    bool mean_pass = tanimoto_mean < _filter_params.tanimoto_mean_threshold,
        geo_mean_pass = tanimoto_geo_mean < _filter_params.tanimoto_geo_mean_threshold,
        max_pass = tanimoto_max < _filter_params.tanimoto_max_threshold;

    // Debug log
    if (logger().isDebugEnabled())
    {
        {
            std::stringstream ss;
            ss << "SS Tanimoto filter: tanimoto mean = " << tanimoto_mean
               << ", tanimoto geometric mean = " << tanimoto_geo_mean
               << ", tanimoto max = " << tanimoto_max
               << " over demes";
            for (const auto& deme : ss_demes)
                ss << " " << deme.getID();
            logger().debug() << ss.str();
        }
        {
            std::stringstream ss;
            if (mean_pass)
                ss << "Pass mean filter (below threshold "
                   << _filter_params.tanimoto_mean_threshold << ")";
            else
                ss << "Not pass mean filter (above or equal to threshold "
                   << _filter_params.tanimoto_mean_threshold << ")";
            if (geo_mean_pass)
                ss << ", pass geometric mean filter (below threshold "
                   << _filter_params.tanimoto_geo_mean_threshold << ")";
            else
                ss << ", not pass geometric mean filter (above or equal to threshold "
                   << _filter_params.tanimoto_geo_mean_threshold << ")";
            if (max_pass)
                ss << ", pass max filter (below threshold "
                   << _filter_params.tanimoto_max_threshold << ")";
            else
                ss << ", not pass max filter (above or equal to threshold "
                   << _filter_params.tanimoto_max_threshold << ")";
            logger().debug(ss.str());
        }
    }

    return mean_pass and geo_mean_pass and max_pass;
}

}}
