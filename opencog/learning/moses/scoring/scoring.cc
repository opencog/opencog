/*
 * opencog/learning/moses/moses/scoring.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller, Linas Vepstas
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
#include "scoring.h"

#include <cmath>

#include <boost/range/irange.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/algorithm_ext/for_each.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

#include <opencog/util/numeric.h>
#include <opencog/util/KLD.h>
#include <opencog/util/MannWhitneyU.h>
#include <opencog/comboreduct/table/table_io.h>

namespace opencog { namespace moses {

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;
using boost::adaptors::filtered;
using boost::adaptors::reversed;
using boost::adaptors::transformed;
using boost::transform;
using namespace boost::phoenix;
using boost::phoenix::arg_names::arg1;
using namespace boost::accumulators;

// helper to log a combo_tree and its behavioral score
inline void log_candidate_pbscore(const combo_tree& tr,
                                  const penalized_bscore& pbs)
{
    if (logger().isFineEnabled())
        logger().fine() << "Evaluate candidate: " << tr << "\n"
                        << "\tBScored: " << pbs;
}

void bscore_base::set_complexity_coef(unsigned alphabet_size, float p)
{
    // Both p==0.0 and p==0.5 are singularities in the forumla.
    // See the explanation in the comment above ctruth_table_bscore.
    complexity_coef = 0.0;
    occam = (p > 0.0f && p < 0.5f);
    if (occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p);

    logger().info() << "BScore noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/complexity_coef;
}

void bscore_base::set_complexity_coef(score_t complexity_ratio)
{
    complexity_coef = 0.0;
    occam = (complexity_ratio > 0.0);
    if (occam)
        complexity_coef = 1.0 / complexity_ratio;

    logger().info() << "BScore complexity ratio = " << 1.0/complexity_coef;
}

////////////////////
// logical_bscore //
////////////////////
        
penalized_bscore logical_bscore::operator()(const combo_tree& tr) const
{
    combo::complete_truth_table tt(tr, arity);
    penalized_bscore pbs(
        make_pair<behavioral_score, score_t>(behavioral_score(target.size()), 0));

    boost::transform(tt, target, pbs.first.begin(), [](bool b1, bool b2) {
            return -score_t(b1 != b2); });

    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    return pbs;
}

behavioral_score logical_bscore::best_possible_bscore() const
{
    return behavioral_score(target.size(), 0);
}

score_t logical_bscore::min_improv() const
{
    return 0.5;
}

///////////////////
// contin_bscore //
///////////////////

// Note that this returns a POSITIVE number.
score_t contin_complexity_coef(unsigned alphabet_size, double stdev)
{
    return log(alphabet_size) * 2 * sq(stdev);
}

penalized_bscore contin_bscore::operator()(const combo_tree& tr) const
{
    // OTable target is the table of output we want to get.
    penalized_bscore pbs;

    // boost/range/algorithm/transform.
    // Take the input vectors cit, target, feed the elts to anon
    // funtion[] (which just computes square of the difference) and
    // put the results into bs.
    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);
    boost::transform(cti, target, back_inserter(pbs.first),
                     [&](const multi_type_seq& mts, const vertex& v) {
                         contin_t tar = get_contin(v),
                             res = get_contin(interpret_tr(mts.get_variant()));
                         return -err_func(res, tar);
                     });
    // add the Occam's razor feature
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    // Logger
    log_candidate_pbscore(tr, pbs);
    // ~Logger

    return pbs;
}

behavioral_score contin_bscore::best_possible_bscore() const
{
    return behavioral_score(target.size(), 0);
}

score_t contin_bscore::min_improv() const
{
    // The backwards compat version of this is 0.0.  But for
    // continuously-variable scores, this is crazy, as the
    // system falls into a state of tweaking the tenth decimal place,
    // Limit any such tweaking to 4 decimal places of precision.
    // (thus 1e-4 below).
    //
    // Note: positive min_improv is taken as an absolute score.
    // Negative min_improve is treated as a relative score.
    return -1.0e-4;
}
        
void contin_bscore::set_complexity_coef(unsigned alphabet_size, float stdev)
{
    occam = (stdev > 0.0);
    complexity_coef = 0.0;
    if (occam)
        complexity_coef = contin_complexity_coef(alphabet_size, stdev);

    logger().info() << "contin_bscore noise = " << stdev
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/complexity_coef;
}

///////////////////
// discriminator //
///////////////////

discriminator::discriminator(const CTable& ct)
    : _ctable(ct)
{
    _output_type = ct.get_output_type();
    if (_output_type == id::boolean_type) {
        // For boolean tables, sum the total number of 'T' values
        // in the output.
        sum_true = [](const CTable::counter_t& c)->score_t
        {
            return c.get(id::logical_true);
        };
        // For boolean tables, sum the total number of 'F' values
        // in the output.
        sum_false = [](const CTable::counter_t& c)->score_t
        {
            return c.get(id::logical_false);
        };
    } else if (_output_type == id::contin_type) {
        // For contin tables, we return the sum of the row values > 0
        sum_true = [](const CTable::counter_t& c)->score_t
        {
            score_t res = 0.0;
            for (const CTable::counter_t::value_type& cv : c)
                res += std::max(0.0, get_contin(cv.first) * cv.second);
            return res;
        };
        // For contin tables, we return the sum of the row values < 0
        sum_false = [](const CTable::counter_t& c)->score_t
        {
            score_t res = 0.0;
            for (const CTable::counter_t::value_type& cv : c)
                res += std::min(0.0, get_contin(cv.first) * cv.second);
            return res;
        };
    } else {
        OC_ASSERT(false, "Discriminator, unsupported output type");
        return;
    }

    _true_total = 0.0;
    _false_total = 0.0;
    for (const CTable::value_type& vct : _ctable) {
        // vct.first = input vector
        // vct.second = counter of outputs
        _true_total += sum_true(vct.second);
        _false_total += sum_false(vct.second);
    }
    logger().info() << "Discriminator: num_true=" << _true_total
                    << " num_false=" << _false_total;
}


discriminator::d_counts::d_counts()
{
    true_positive_sum = 0.0;
    false_positive_sum = 0.0;
    positive_count = 0.0;
    true_negative_sum = 0.0;
    false_negative_sum = 0.0;
    negative_count = 0.0;
};

discriminator::d_counts discriminator::count(const combo_tree& tr) const
{
    d_counts ctr;

    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);

    for (const CTable::value_type& vct : _ctable) {
        // vct.first = input vector
        // vct.second = counter of outputs

        double pos = sum_true(vct.second);
        double neg = sum_false(vct.second);
        unsigned totalc = vct.second.total_count();

        if (interpret_tr(vct.first.get_variant()) == id::logical_true)
        {
            ctr.true_positive_sum += pos;
            ctr.false_positive_sum += neg;
            ctr.positive_count += totalc;
        }
        else
        {
            ctr.true_negative_sum += neg;
            ctr.false_negative_sum += pos;
            ctr.negative_count += totalc;
        }
    }
    return ctr;
}

vector<discriminator::d_counts> discriminator::counts(const combo_tree& tr) const
{
    std::vector<d_counts> ctr_seq;

    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);

    for (const CTable::value_type& vct : _ctable) {
        // vct.first = input vector
        // vct.second = counter of outputs

        double pos = sum_true(vct.second);
        double neg = sum_false(vct.second);
        unsigned totalc = vct.second.total_count();

        d_counts ctr;
        if (interpret_tr(vct.first.get_variant()) == id::logical_true)
        {
            ctr.true_positive_sum = pos;
            ctr.false_positive_sum = neg;
            ctr.positive_count = totalc;
        }
        else
        {
            ctr.true_negative_sum = neg;
            ctr.false_negative_sum = pos;
            ctr.negative_count = totalc;
        }
        ctr_seq.push_back(ctr);
    }
    return ctr_seq;
}

//////////////////////////
// disciminating_bscore //
//////////////////////////

discriminating_bscore::discriminating_bscore(const CTable& ct,
                                             float min_threshold,
                                             float max_threshold,
                                             float hardness)
    : discriminator(ct),
      _ctable_usize(ct.uncompressed_size()),
      _min_threshold(min_threshold),
      _max_threshold(max_threshold),
      _hardness(hardness),
      _full_bscore(true)
{
    logger().info("Discriminating scorer, hardness = %f, "
                  "min_threshold = %f, "
                  "max_threshold = %f",
                  _hardness, _min_threshold, _max_threshold);

    // Verify that the thresholds are sane
    OC_ASSERT((0.0 < hardness) && (0.0 <= min_threshold) && (min_threshold <= max_threshold),
        "Discriminating scorer, invalid thresholds.  "
        "The hardness must be positive, the minimum threshold must be "
        "non-negative, and the maximum threshold must be greater "
        "than or equal to the minimum threshold.\n");

    // For boolean tables, the highest possible output is 1.0 (of course)
    if (_output_type == id::boolean_type) {
        _max_output = 1.0;
        _min_output = 0.0;
    }
    else // if (_output_type == id::contin_type)
    {
        // For contin tables, we search for the largest value in the table.
        _max_output = very_worst_score;
        _min_output = very_best_score;
        for (const auto& cr : _ctable) {
            const CTable::counter_t& c = cr.second;
            for (const auto& cv : c) {
                score_t val = get_contin(cv.first);
                _max_output = std::max(_max_output, val);
                _min_output = std::min(_min_output, val);
            }
        }
    }

    logger().info("Discriminating scorer, min_output = %f, "
                  "max_output = %f", _min_output, _max_output);
}

/// The best_possible_bscore is a pair of (best_score, fix_penalty).
/// The best_score is sum_over_variable, at the time that 
/// the sum_over_fixed exceeds the min_threshold.  The fix_penalty is
/// just the penalty applied to sum_over_fixed at this point.
/// Typically, the penalty is zero whenever something is greater than
/// the min_threshold; so the fix_penalty applies only if the min_threshold
/// wasn't actually reached.
behavioral_score discriminating_bscore::best_possible_bscore() const
{
    // create a list, maintained in sorted order by the best row
    // (meaning the row that contributes the most to the overall
    // score). We keep sum positive/negative of that row to not have
    // to recompute it again.
    typedef std::tuple<double, // row's sum positive
                       double, // row's sum negative
                       unsigned> // row's count
        pos_neg_cnt;

    typedef std::multimap<double, pos_neg_cnt> max_vary_t;

    max_vary_t max_vary;
    for (CTable::const_iterator it = _ctable.begin(); it != _ctable.end(); ++it)
    {
        const CTable::counter_t& c = it->second;

        double pos = sum_true(c);
        double neg = sum_false(c);
        unsigned total = c.total_count();

        double vary = get_variable(pos, neg, total);

        logger().fine() << "Disc: total=" << total << " pos=" << pos
                        << " neg=" << neg << " vary=" << vary;

        const pos_neg_cnt pnc(pos, neg, total);
        max_vary.insert({vary, pnc});
    }

    // Compute best variable part of the score till minimum fixed part
    // is reached. It is assumed that once the fixed part is reached
    // the variable part will only get lower so we can stop doing the
    // calculation.
    //
    // Then we select the best score obtained (accounting for both
    // variable and fixed parts).
    unsigned acc_cnt = 0;
    score_t acc_pos = 0.0,     // accumulation of positive
        acc_neg = 0.0,      // accumulation of negative
        best_sc = very_worst_score,      // best score
        best_vary = 0.0,    // best score varying component
        best_fixed = 0.0,   // the best score fixed component
        best_fixation_penalty = 0.0; // best score fixed component penalty

    logger().fine() << "Disc: min_thresh=" << _min_threshold;

    foreach (const pos_neg_cnt& pnc, max_vary | map_values | reversed) {
        acc_pos += std::get<0>(pnc);
        acc_neg += std::get<1>(pnc);
        acc_cnt += std::get<2>(pnc);

        // compute current score (and its varying and fixed parts)
        score_t vary = get_variable(acc_pos, acc_neg, acc_cnt),
            fixed = get_fixed(acc_pos, acc_neg, acc_cnt),
            fixation_penalty = get_threshold_penalty(fixed),
            sc = vary + fixation_penalty;

        // update best score (and its varying and fixed parts)
        if (sc > best_sc) {
            best_sc = sc;
            best_vary = vary;
            best_fixed = fixed;
            best_fixation_penalty = fixation_penalty;
        }

        logger().fine() << "Disc: vary=" << vary << " fixed=" << fixed
                        << " score=" << sc;

        // halt if fixed has reached _min_threshold
        if (_min_threshold <= fixed)
            break;
    }

    logger().info("Discriminating scorer, best score = %f", best_sc);
    logger().info("Discriminating scorer, variable component of best score = %f", best_vary);
    logger().info("Discriminating scorer, fixed component of best score = %f", best_fixed);
    logger().info("Discriminating scorer, fixation penalty of best score = %f", best_fixation_penalty);

    return {best_vary, best_fixation_penalty};
}

score_t discriminating_bscore::min_improv() const
{
    return 1.0 / _ctable_usize;
}

// Note that the logarithm is always negative, so this method always
// returns a value that is zero or negative.
score_t discriminating_bscore::get_threshold_penalty(score_t value) const
{
    score_t dst = 0.0;
    if (value < _min_threshold)
        dst = 1.0 - value / _min_threshold;
    
    if (_max_threshold < value)
        dst = (value - _max_threshold) / (1.0 - _max_threshold);

    // Attempt to avoid insane values.
    if (0.99999999 < dst) return -_hardness * 18;
    return _hardness * log(1.0 - dst);
}

void discriminating_bscore::set_complexity_coef(unsigned alphabet_size, float p)
{
    complexity_coef = 0.0;
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    occam = p > 0.0f && p < 0.5f;
    if (occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p)
            / _ctable_usize;    // normalized by the size of the table
                                // because the precision is normalized
                                // as well

    logger().info() << "Discriminating scorer, noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/complexity_coef;
}

void discriminating_bscore::set_complexity_coef(score_t ratio)
{
    complexity_coef = 0.0;
    occam = (ratio > 0);

    // The complexity coeff is normalized by the size of the table,
    // because the precision is normalized as well.  So e.g.
    // max precision for boolean problems is 1.0.  However...
    // umm XXX I think the normalization here should be the
    // best-possible activation, not the usize, right?
    //
    // @todo Sounds good too, as long as it's constant, so you would
    // replace _ctable_usize by _ctable_usize * max_activation?
    if (occam)
        complexity_coef = 1.0 / (_ctable_usize * ratio);

    logger().info() << "Discriminating scorer, table size = " << _ctable_usize;
    logger().info() << "Discriminating scorer, complexity ratio = " << 1.0f/complexity_coef;
}


///////////////////
// recall_bscore //
///////////////////

recall_bscore::recall_bscore(const CTable& ct,
                  float min_precision,
                  float max_precision,
                  float hardness) 
    : discriminating_bscore(ct, min_precision, max_precision, hardness)
{
}

penalized_bscore recall_bscore::operator()(const combo_tree& tr) const
{
    d_counts ctr = count(tr);

    // Compute normalized precision and recall.
    score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
    score_t precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 0.0;

    score_t tp_fn = ctr.true_positive_sum + ctr.false_negative_sum;
    score_t recall = (0.0 < tp_fn) ? ctr.true_positive_sum / tp_fn : 0.0;

    // We are maximizing recall, so that is the first part of the score.
    penalized_bscore pbs;
    pbs.first.push_back(recall);
    
    score_t precision_penalty = get_threshold_penalty(precision);
    pbs.first.push_back(precision_penalty);
    if (logger().isFineEnabled()) 
        logger().fine("recall_bcore: precision = %f  recall=%f  precision penalty=%e",
                     precision, recall, precision_penalty);
 
    // Add the Complexity penalty
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

/// Return the precision for this ctable row(s).
///
/// Since this scorer is trying to maximize recall while holding
/// precision fixed, We should return positive values as long as
/// this table has some positive entries in it. (Why? Because the
/// estimator sorts the table by the value returned here: so 
/// entries with a perfect score should return 1, thos with no 
/// positive entries should return 0, and everything else in the
/// middle.
///
/// For this ctable row collection, we have that:
/// pos == true_positives or false_negatives in this ctable row(s)
/// neg == false_positives or true_negatives in this ctable row(s)
/// cnt == pos + neg
/// How the "best possible model" will score this row depends
/// on what the precision threshold is.
score_t recall_bscore::get_fixed(score_t pos, score_t neg, unsigned cnt) const
{
    return pos / cnt;
}

/// Return the recall for this ctable row(s).
///
/// _true_total is the total number of rows that are T (or positive
/// weighted if contin)
score_t recall_bscore::get_variable(score_t pos, score_t neg, unsigned cnt) const
{
    return pos / _true_total;
}

///////////////////
// prerec_bscore //
///////////////////

prerec_bscore::prerec_bscore(const CTable& ct,
                  float min_recall,
                  float max_recall,
                  float hardness) 
    : discriminating_bscore(ct, min_recall, max_recall, hardness)
{
}

// Nearly identical to recall_bscore, except that the roles of precision
// and recall are switched.
penalized_bscore prerec_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;
    score_t precision = 1.0,
        recall = 0.0;
    if (_full_bscore) {
        // each element of the bscore correspond to a data point
        // contribution of the variable component of the score

        vector<d_counts> ctr_seq = counts(tr);
        score_t pos_total = 0.0;
        for (const d_counts& ctr : ctr_seq) {
            // here we actually store (tp-fp)/2 instead of tp, to have
            // a more expressive bscore (so that bad datapoints are
            // distict from non-positive ones)
            pbs.first.push_back((ctr.true_positive_sum - ctr.false_positive_sum)
                                / 2);
            pos_total += ctr.positive_count;
        }
        // divide all element by pos_total (so it sums up to precision - 1/2)
        boost::transform(pbs.first, pbs.first.begin(), arg1 / pos_total);

        // By using (tp-fp)/2 the sum of all the per-row contributions
        // is offset by -1/2 from the precision, as proved below
        //
        // 1/2 * (tp - fp) / (tp + fp)
        // = 1/2 * (tp - tp + tp - fp) / (tp + fp)
        // = 1/2 * (tp + tp) / (tp + fp) - (tp + fp) / (tp + fp)
        // = 1/2 * 2*tp / (tp + fp) - 1
        // = precision - 1/2
        //
        // So before adding the recall penalty we add +1/2 to
        // compensate that
        pbs.first.push_back(0.5);

        // compute precision and recall
        precision = boost::accumulate(pbs.first, 0);
        for (const d_counts& ctr : ctr_seq)
            recall += ctr.true_positive_sum;
        if (0.0 < _true_total)
            recall /= _true_total;
    } else {
        // the aggregated (across all datapoints) variable component
        // of the score is the first value of the bscore

        d_counts ctr = count(tr);

        // Compute normalized precision and recall.
        score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
        precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 1.0;
        recall = (0.0 < _true_total) ? ctr.true_positive_sum / _true_total : 0.0;

        // We are maximizing precision, so that is the first part of the score.
        pbs.first.push_back(precision);
    }

    // calculate recall_penalty
    score_t recall_penalty = get_threshold_penalty(recall);
    pbs.first.push_back(recall_penalty);

    // Log precision, recall and penalty
    if (logger().isFineEnabled()) 
        logger().fine("prerec_bscore: precision = %f  "
                      "recall=%f  recall penalty=%e",
                      precision, recall, recall_penalty);
 
    // Add the Complexity penalty
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

/// Return an approximation for the precision that this ctable row(s)
/// will contribute to the total precision.
score_t prerec_bscore::get_variable(score_t pos, score_t neg, unsigned cnt) const
{
    return pos / cnt;
}

/// Return the best-possible recall for this ctable row(s).
score_t prerec_bscore::get_fixed(score_t pos, score_t neg, unsigned cnt) const
{
    return pos / _true_total;
}

////////////////
// bep_bscore //
////////////////

bep_bscore::bep_bscore(const CTable& ct,
                       float min_diff,
                       float max_diff,
                       float hardness) 
    : discriminating_bscore(ct, min_diff, max_diff, hardness)
{
}

penalized_bscore bep_bscore::operator()(const combo_tree& tr) const
{
    d_counts ctr = count(tr);

    // Compute normalized precision and recall.
    score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
    score_t precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 0.0;

    score_t tp_fn = ctr.true_positive_sum + ctr.false_negative_sum;
    score_t recall = (0.0 < tp_fn) ? ctr.true_positive_sum / tp_fn : 0.0;

    score_t bep = (precision + recall) / 2;
    // We are maximizing bep, so that is the first part of the score.
    penalized_bscore pbs;
    pbs.first.push_back(bep);
    
    score_t bep_diff = fabs(precision - recall);
    score_t bep_penalty = get_threshold_penalty(bep_diff);
    pbs.first.push_back(bep_penalty);
    if (logger().isFineEnabled()) 
        logger().fine("bep = %f  diff=%f  bep penalty=%e",
                     bep, bep_diff, bep_penalty);
 
    // Add the Complexity penalty
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

/// Return the break-even-point for this ctable row.
score_t bep_bscore::get_variable(score_t pos, score_t neg, unsigned cnt) const
{
    // XXX TODO FIXME is this really correct?
    double best_possible_precision = pos / (cnt * _true_total);
    double best_possible_recall = 1.0 / _true_total;
    return (best_possible_precision + best_possible_recall) / 2;
}

/// Return the difference for this ctable row.
score_t bep_bscore::get_fixed(score_t pos, score_t neg, unsigned cnt) const
{
    // XXX TODO FIXME is this really correct?
    double best_possible_precision = pos / (cnt);
    double best_possible_recall = (0.0 < pos) ? 1.0 : 0.0;
    return fabs(best_possible_precision - best_possible_recall);
}

//////////////////
// f_one_bscore //
//////////////////

/// The F_1 bscore attempts to maximize the F_1 score.
/// The F_1 score is the harmonic mean of the precision and recall
/// (exactly as defined in standard textbooks, etc.)
/// While it might be nice to hold the ratio of precision to recall
/// within some given thresholds, this turns out to be complicated,
/// so we are not going to bother.
f_one_bscore::f_one_bscore(const CTable& ct)
    : discriminating_bscore(ct, 0.0, 1.0, 1.0e-20)
{
}

penalized_bscore f_one_bscore::operator()(const combo_tree& tr) const
{
    d_counts ctr = count(tr);

    // Compute normalized precision and recall.
    score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
    score_t precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 0.0;

    score_t tp_fn = ctr.true_positive_sum + ctr.false_negative_sum;
    score_t recall = (0.0 < tp_fn) ? ctr.true_positive_sum / tp_fn : 0.0;

    score_t f_one = 2 * precision * recall / (precision + recall);

    // We are maximizing f_one, so that is the first part of the score.
    penalized_bscore pbs;
    pbs.first.push_back(f_one);
    
    if (logger().isFineEnabled()) 
        logger().fine("f_one_bscore: precision = %f recall = %f f_one=%f",
                     precision, recall, f_one);
 
    // Add the Complexity penalty
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

/// Quasi-meaningless for f_one, but needed for automatic
// generation of best-possible score.
score_t f_one_bscore::get_fixed(score_t pos, score_t neg, unsigned cnt) const
{
    // XXX TODO FIXME is this really correct?
    return 1.0;
}

/// Return the f_one for this ctable row.
score_t f_one_bscore::get_variable(score_t pos, score_t neg, unsigned cnt) const
{
    // XXX TODO FIXME is this really correct?
    double best_possible_precision = pos / cnt;
    double best_possible_recall = 1.0;
    double f_one = 2 * best_possible_precision * best_possible_recall 
              / (best_possible_recall + best_possible_precision);
    f_one /= _true_total; // since we add these together.
    return f_one;
}

//////////////////////
// precision_bscore //
//////////////////////

/// NOTE: The "precision" bscore, below, does NOT correspond to the 
/// standard definition of "precision", as usually given in textbooks
/// or wikipedia, but is something similar but different in various
/// details. To get the standard, text-book definition of a "precision"
/// scorer, use the "prerec" class above.  Actually, chances are good
/// that you probably want the "recall" scorer, which maximizes recall
/// while holding precision at or above a minimum level.

precision_bscore::precision_bscore(const CTable& _ctable,
                                   float penalty_,
                                   float min_activation_,
                                   float max_activation_,
                                   bool positive_,
                                   bool worst_norm_)
    : orig_ctable(_ctable), wrk_ctable(orig_ctable),
      ctable_usize(orig_ctable.uncompressed_size()),
      min_activation(min_activation_), max_activation(max_activation_),
      penalty(penalty_), positive(positive_), worst_norm(worst_norm_),
    precision_full_bscore(true)
{
    output_type = wrk_ctable.get_output_type();
    if (output_type == id::boolean_type) {
        // For boolean tables, sum the total number of 'T' values
        // in the output.  Ths sum represents the best possible score
        // i.e. we found all of the true values correcty.  Count
        // 'F' is 'positive' is false.
        vertex target = bool_to_vertex(positive),
            neg_target = negate_vertex(target);
        sum_outputs = [this, target, neg_target](const CTable::counter_t& c)
            -> score_t
        {
            return ((int)c.get(target) - (int)c.get(neg_target)) * 0.5;
        };
    } else if (output_type == id::contin_type) {
        // For contin tables, we return the sum of the row values.
        sum_outputs = [this](const CTable::counter_t& c)->score_t
        {
            score_t res = 0.0;
            for (const CTable::counter_t::value_type& cv : c)
                res += get_contin(cv.first) * cv.second;
            return (positive? res : -res);
        };
    } else {
        OC_ASSERT(false, "Precision scorer, unsupported output type");
        return;
    }

    logger().fine("Precision scorer, penalty = %f, "
                  "min_activation = %f, "
                  "max_activation = %f",
                  penalty, min_activation, max_activation);

    // Verify that the penaly is sane
    OC_ASSERT((0.0 < penalty) && (0.0 < min_activation) && (min_activation <= max_activation),
        "Precision scorer, invalid activation bounds.  "
        "The penalty must be non-zero, the minimum activation must be "
        "greater than zero, and the maximum activation must be greater "
        "than or equal to the minimum activation.\n");

    // For boolean tables, the highest possible precision is 1.0 (of course)
    if (output_type == id::boolean_type)
        max_output = 1.0;
    else if (output_type == id::contin_type) {

        // For contin tables, we search for the largest value in the table.
        // (or smallest, if positive == false)
        max_output = very_worst_score;
        for (const auto& cr : wrk_ctable) {
            const CTable::counter_t& c = cr.second;
            for (const auto& cv : c) {
                score_t val = get_contin(cv.first);
                if (!positive) val = -val;
                max_output = std::max(max_output, val);
            }
        }
    }

    logger().fine("Precision scorer, max_output = %f", max_output);
}

void precision_bscore::set_complexity_coef(unsigned alphabet_size, float p)
{
    complexity_coef = 0.0;
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    occam = p > 0.0f && p < 0.5f;
    if (occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p)
            / ctable_usize;     // normalized by the size of the table
                                // because the precision is normalized
                                // as well

    logger().info() << "Precision scorer, noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/complexity_coef;
}

void precision_bscore::set_complexity_coef(score_t ratio)
{
    complexity_coef = 0.0;
    occam = (ratio > 0);

    if (occam)
        complexity_coef = 1.0 / ratio;

    logger().info() << "Precision scorer, complexity ratio = " << 1.0f/complexity_coef;
}

penalized_bscore precision_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;

    // Initial precision. No hits means perfect precision :)
    // Yes, zero hits is common, early on.
    score_t precision = 1.0;
    unsigned active = 0;   // total number of active outputs by tr
    score_t sao = 0.0;     // sum of all active outputs (in the boolean case)
    
    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);
    if (precision_full_bscore) {
        // compute active and sum of all active outputs
        for (const CTable::value_type& vct : wrk_ctable) {
            // for debugging, keep that around till we fix best_possible_bscore
            // {
            //     stringstream ss;
            //     ostreamCTableRow(ss, vct);
            //     logger().fine(ss.str());
            // }
            const auto& ct = vct.second;
            double sumo = 0.0;
            if (interpret_tr(vct.first.get_variant()) == id::logical_true) {
                sumo = sum_outputs(ct);
                sao += sumo;
                active += ct.total_count();
            }
            // for debugging, keep that around till we fix best_possible_bscore
            // logger().fine("sumo = %g, sao = %g, active = %u, is_true = %d",
            //               sumo, sao, active,
            //               interpret_tr(vct.first.get_variant()) == id::logical_true);
            pbs.first.push_back(sumo);
        }

        if (active > 0) {
            // normalize all components by active
            score_t iac = 1.0 / active; // inverse of activity to be faster
            boost::transform(pbs.first, pbs.first.begin(), arg1 * iac);

            // By using (tp-fp)/2 the sum of all the per-row contributions
            // is offset by -1/2 from the precision, as proved below
            //
            // 1/2 * (tp - fp) / (tp + fp)
            // = 1/2 * (tp - tp + tp - fp) / (tp + fp)
            // = 1/2 * (tp + tp) / (tp + fp) - (tp + fp) / (tp + fp)
            // = 1/2 * 2*tp / (tp + fp) - 1
            // = precision - 1/2
            //
            // So before adding the recall penalty we add +1/2 to
            // compensate for that
            pbs.first.push_back(0.5);
        }

    } else {

        // associate sum of worst outputs with number of observations for
        // that sum
        multimap<double, unsigned> worst_deciles;

        // compute active and sum of all active outputs
        for (const CTable::value_type& vct : wrk_ctable) {
            // vct.first = input vector
            // vct.second = counter of outputs
            if (interpret_tr(vct.first.get_variant()) == id::logical_true) {
                double sumo = sum_outputs(vct.second);
                unsigned totalc = vct.second.total_count();
                // For boolean tables, sao == sum of all true positives,
                // and active == sum of true+false positives.
                // For contin tables, sao = sum of contin values, and
                // active == count of rows.
                sao += sumo;
                active += totalc;
                if (worst_norm && sumo < 0)
                    worst_deciles.insert({sumo, totalc});
            }
        }

        // remove all observations from worst_norm so that only the worst
        // n_deciles or less remains and compute its average
        double avg_worst_deciles = 0.0;
        if (worst_norm and sao > 0) {
            unsigned worst_count = 0,
                n_deciles = active / 10;
            for (const auto& pr : worst_deciles) {
                worst_count += pr.second;
                avg_worst_deciles += pr.first;
                if (worst_count > n_deciles)
                    break;
            }
            avg_worst_deciles /= worst_count;
        }
        // normalize precision w.r.t. worst deciles
        if (avg_worst_deciles < 0) {
            logger().fine("precision before worst_norm = %f", precision);
            logger().fine("abs(avg_worst_deciles) = %f", -avg_worst_deciles);
            precision /= -avg_worst_deciles;
            if (avg_worst_deciles >= 0)
                logger().fine("Weird: worst_norm (%f) is positive, maybe the activation is really low", avg_worst_deciles);
        }

        pbs.first.push_back(precision);
    }

    if (0 < active)
        precision = (sao / active + 0.5) / max_output;
    
    // For boolean tables, activation sum of true and false positives
    // i.e. the sum of all positives.   For contin tables, the activation
    // is likewise: the number of rows for which the combo tree returned
    // true (positive).
    score_t activation = (score_t)active / ctable_usize;
    score_t activation_penalty = get_activation_penalty(activation);
    pbs.first.push_back(activation_penalty);
    if (logger().isFineEnabled())
        logger().fine("precision = %f  activation=%f  activation penalty=%e",
                      precision, activation, activation_penalty);

    // Add the Complexity penalty
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

behavioral_score precision_bscore::best_possible_bscore() const
{
    // @todo doesn't treat the case with worst_norm

    // For each row, compute the maximum precision it can get.
    // Typically, this is 0 or 1 for nondegenerate boolean tables.
    // Also store the sumo and total, so that they don't need to be
    // recomputed later.  Note that this routine could be performance
    // critical if used as a fitness function for feature selection
    // (which is the case).
    typedef std::multimap<contin_t,           // precision
                          std::pair<contin_t, // sum_outputs
                                    unsigned> // total count
                          > max_precisions_t;
    max_precisions_t max_precisions;
    for (CTable::const_iterator it = wrk_ctable.begin();
         it != wrk_ctable.end(); ++it) {
        const CTable::counter_t& c = it->second;
        double sumo = sum_outputs(c);
        unsigned total = c.total_count();
        double precision = sumo / total;

        // for debugging, keep that around till we fix best_possible_bscore
        // logger().fine("sumo = %g, total = %u, precision = %g",
        //               sumo, total, precision);

        auto lmnt = std::make_pair(precision, std::make_pair(sumo, total));
        max_precisions.insert(lmnt);
    }

    // Compute best precision till minimum activation is reached.
    // Note that the best precision (sao / active) can never increase
    // for each new mpv. However the activation penalty can, for that
    // reason we keep going until min_activation is reached. At the
    // end we keep the best score (considering both precision and
    // activation penalty at each step).
    unsigned active = 0;
    score_t sao = 0.0,
        best_sc = very_worst_score,
        best_precision = 0.0,
        best_activation = 0.0,
        best_activation_penalty = 0.0;
    
    reverse_foreach (const auto& mpv, max_precisions) {
        sao += mpv.second.first;
        active += mpv.second.second;

        // By using (tp-fp)/2 the sum of all the per-row contributions
        // is offset by -1/2 from the precision, as proved below
        //
        // 1/2 * (tp - fp) / (tp + fp)
        // = 1/2 * (tp - tp + tp - fp) / (tp + fp)
        // = 1/2 * (tp + tp) / (tp + fp) - (tp + fp) / (tp + fp)
        // = 1/2 * 2*tp / (tp + fp) - 1
        // = precision - 1/2
        //
        // So before adding the recall penalty we add 0.5 to
        // compensate for that
        score_t precision = (sao / active) / max_output + 0.5,
            activation = active / (score_t)ctable_usize,
            activation_penalty = get_activation_penalty(activation),
            sc = precision + activation_penalty;

        // update best score
        if (sc > best_sc) {
            best_sc = sc;
            best_precision = precision;
            best_activation = activation;
            best_activation_penalty = activation_penalty;
        }

        // for debugging, keep that around till we fix best_possible_bscore
        // logger().fine("sao = %g, active = %u, precision = %g, activation = %g, activation_penalty = %g, sc = %g, best_sc = %g, best_precision = %g, best_activation = %g, best_activation_penalty = %g", sao, active, precision, activation, activation_penalty, sc, best_sc, best_precision, best_activation, best_activation_penalty);

        // halt if min_activation is reached
        if (ctable_usize * min_activation <= active)
            break;
    }

    logger().fine("Precision scorer, best score = %f", best_sc);
    logger().fine("precision at best score = %f", best_precision);
    logger().fine("activation at best score = %f", best_activation);
    logger().fine("activation penalty at best score = %f", best_activation_penalty);

    /// @todo it's not really the best bscore but rather the best score
    return {best_sc};
}

// Note that the logarithm is always negative, so this method always
// returns a value that is zero or negative.
score_t precision_bscore::get_activation_penalty(score_t activation) const
{
    score_t dst = 0.0;
    if (activation < min_activation)
        dst = 1.0 - activation/min_activation;

    if (max_activation < activation)
        dst = (activation - max_activation) / (1.0 - max_activation);

    // logger().fine("activation penalty = %f", dst);
    return penalty * log(1.0 - dst);
}

score_t precision_bscore::min_improv() const
{
    return 1.0 / ctable_usize;
}

void precision_bscore::ignore_idxs(const std::set<arity_t>& idxs) const
{
    if (logger().isDebugEnabled())
    {
        stringstream ss;
        ss << "Compress CTable for optimization by ignoring features: ";
        ostreamContainer(ss, idxs, ",");
        logger().debug(ss.str());
    }

    // Get permitted idxs.
    auto irng = boost::irange(0, orig_ctable.get_arity());
    std::set<arity_t> all_idxs(irng.begin(), irng.end());
    std::set<arity_t> permitted_idxs = opencog::set_difference(all_idxs, idxs);

    // Filter orig_table with permitted idxs.
    wrk_ctable = orig_ctable.filtered_preverse_idxs(permitted_idxs);

    // for debugging, keep that around till we fix best_possible_bscore
    // fully_filtered_ctable = orig_ctable.filtered(permitted_idxs);

    logger().debug("Original CTable size = %u", orig_ctable.size());
    logger().debug("Working CTable size = %u", wrk_ctable.size());

    if (logger().isFineEnabled()) {
        {
            stringstream ss;
            ss << "wrk_ctable =" << endl;
            ostreamCTable(ss, wrk_ctable);
            logger().fine(ss.str());
        }
        // for debugging, keep that around till we fix best_possible_bscore
        // {
        //     stringstream ss;
        //     ss << "fully_filtered_ctable =" << endl;
        //     ostreamCTable(ss, fully_filtered_ctable);
        //     logger().fine(ss.str());
        // }
    }
}

combo_tree precision_bscore::gen_canonical_best_candidate() const
{
    // @todo doesn't treat the case with worst_norm

    // For each row, compute the maximum precision it can get.
    // Typically, this is 0 or 1 for nondegenerate boolean tables.
    // Also store the sumo and total, so that they don't need to be
    // recomputed later.  Note that this routine could be performance
    // critical if used as a fitness function for feature selection
    // (which is planned).
    typedef std::multimap<double, // precision
                          std::pair<CTable::const_iterator,
                                    unsigned> // total count
                          > precision_to_count_t;
    precision_to_count_t ptc;
    for (CTable::const_iterator it = wrk_ctable.begin();
         it != wrk_ctable.end(); ++it) {
        const CTable::counter_t& c = it->second;
        unsigned total = c.total_count();
        double precision = sum_outputs(c) / total;
        ptc.insert(std::make_pair(precision, std::make_pair(it, total)));
    }

    // Generate conjunctive clauses till minimum activation is
    // reached. Note that the best precision (sao / active) can never
    // increase for each new mpv.  Despite this, we keep going until
    // at least min_activation is reached. It's not clear this
    // actually gives the best candidate one can get if min_activation
    // isn't reached, but we don't want to go below min activation
    // anyway, so it's an acceptable inacurracy.  (It would be a
    // problem only if activation constraint is very loose.)
    //
    unsigned active = 0;
    combo_tree tr;
    auto head = tr.set_head(id::logical_or);
    reverse_foreach (const auto& v, ptc) {
        active += v.second.second;

        // build the disjunctive clause
        auto dch = tr.append_child(head, id::logical_and);
        arity_t idx = 1;
        for (const auto& input : v.second.first->first.get_seq<builtin>()) {
            argument arg(input == id::logical_true? idx++ : -idx++);
            tr.append_child(dch, arg);
        }

        // termination conditional
        if (ctable_usize * min_activation <= active)
            break;
    }
    return tr;
}

///////////////////////////
// precision_conj_bscore //
///////////////////////////

precision_conj_bscore::precision_conj_bscore(const CTable& _ctable,
                                             float hardness_,
                                             bool positive_)
    : ctable(_ctable), ctable_usize(ctable.uncompressed_size()),
      hardness(hardness_), positive(positive_)
{
    vertex target = bool_to_vertex(positive);
    sum_outputs = [target](const CTable::counter_t& c)->score_t {
        return c.get(target); };
}

void precision_conj_bscore::set_complexity_coef(unsigned alphabet_size, float p)
{
    complexity_coef = 0.0;
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    occam = p > 0.0f && p < 0.5f;
    if (occam)
        complexity_coef = discrete_complexity_coef(alphabet_size, p)
            / ctable_usize;     // normalized by the size of the table
                                // because the precision is normalized
                                // as well

    logger().info() << "Precision scorer, noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/complexity_coef;
}

void precision_conj_bscore::set_complexity_coef(score_t ratio)
{
    complexity_coef = 0.0;
    occam = (ratio > 0);

    if (occam)
        complexity_coef = 1.0 / ratio;

    logger().info() << "Precision scorer, complexity ratio = " << 1.0f/complexity_coef;
}

penalized_bscore precision_conj_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;

    // compute active and sum of all active outputs
    unsigned active = 0;   // total number of active outputs by tr
    score_t sao = 0.0;     // sum of all active outputs (in the boolean case)
    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);
    for (const CTable::value_type& vct : ctable) {
        // vct.first = input vector
        // vct.second = counter of outputs
        if (interpret_tr(vct.first.get_variant()) == id::logical_true) {
            double sumo = sum_outputs(vct.second);
            unsigned totalc = vct.second.total_count();
            // For boolean tables, sao == sum of all true positives,
            // and active == sum of true+false positives.
            sao += sumo;
            active += totalc;
        }
    }

    // Compute normalized precision.  No hits means perfect precision :)
    // Yes, zero hits is common, early on.
    score_t precision = 1.0;
    if (0 < active)
        precision = sao / active;
    pbs.first.push_back(precision);

    // Count the number of conjunctions (up to depth 2)
    unsigned conj_n = 0;
    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;
    pre_it head = tr.begin();
    if (*head == id::logical_and)
        ++conj_n;
    for (sib_it sib = head.begin(); sib != head.end(); ++sib)
        if (*sib == id::logical_and)
            ++conj_n;
    score_t conj_n_penalty = hardness * (-1.0 / (1.0 + conj_n));
    pbs.first.push_back(conj_n_penalty);

    if (logger().isFineEnabled())
        logger().fine("precision = %f  conj_n=%u  conj_n penalty=%e",
                     precision, conj_n, conj_n_penalty);

    // Add the Complexity penalty
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

behavioral_score precision_conj_bscore::best_possible_bscore() const
{
    return {1.0, 0.0};
}

score_t precision_conj_bscore::min_improv() const
{
    return 0.0;
    // return 1.0 / ctable_usize;
}

//////////////////////////////
// discretize_contin_bscore //
//////////////////////////////

// Note that this function returns a POSITIVE number, since p < 0.5
score_t discrete_complexity_coef(unsigned alphabet_size, double p)
{
    return -log((double)alphabet_size) / log(p/(1-p));
}

discretize_contin_bscore::discretize_contin_bscore(const OTable& ot,
                                                   const ITable& it,
                                                   const vector<contin_t>& thres,
                                                   bool wa)
    : target(ot), cit(it), thresholds(thres), weighted_accuracy(wa),
      classes(ot.size()), weights(thresholds.size() + 1, 1) {
    // enforce that thresholds is sorted
    boost::sort(thresholds);
    // precompute classes
    boost::transform(target, classes.begin(), [&](const vertex& v) {
            return this->class_idx(get_contin(v)); });
    // precompute weights
    multiset<size_t> cs(classes.begin(), classes.end());
    if (weighted_accuracy)
        for (size_t i = 0; i < weights.size(); ++i)
            weights[i] = classes.size() / (float)(weights.size() * cs.count(i));
}

behavioral_score discretize_contin_bscore::best_possible_bscore() const
{
    return behavioral_score(target.size(), 0);
}

score_t discretize_contin_bscore::min_improv() const
{
    // not necessarily right, just the backwards-compat behavior
    return 0.0;
}

size_t discretize_contin_bscore::class_idx(contin_t v) const
{
    if (v < thresholds[0])
        return 0;
    size_t s = thresholds.size();
    if (v >= thresholds[s - 1])
        return s;
    return class_idx_within(v, 1, s);
}

size_t discretize_contin_bscore::class_idx_within(contin_t v,
                                                  size_t l_idx,
                                                  size_t u_idx) const
{
    // base case
    if(u_idx - l_idx == 1)
        return l_idx;
    // recursive case
    size_t m_idx = l_idx + (u_idx - l_idx) / 2;
    contin_t t = thresholds[m_idx - 1];
    if(v < t)
        return class_idx_within(v, l_idx, m_idx);
    else
        return class_idx_within(v, m_idx, u_idx);
}

penalized_bscore discretize_contin_bscore::operator()(const combo_tree& tr) const
{
    /// @todo could be optimized by avoiding computing the OTable and
    /// directly using the results on the fly. On really big table
    /// (dozens of thousands of data points and about 100 inputs, this
    /// has overhead of about 10% of the overall time)
    OTable ct(tr, cit);
    penalized_bscore pbs(
        make_pair<behavioral_score, score_t>(behavioral_score(target.size()), 0));
    boost::transform(ct, classes, pbs.first.begin(), [&](const vertex& v, size_t c_idx) {
            return (c_idx != this->class_idx(get_contin(v))) * this->weights[c_idx];
        });

    // Add the Occam's razor feature
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    // Logger
    log_candidate_pbscore(tr, pbs);
    // ~Logger

    return pbs;    
}

/////////////////////////
// ctruth_table_bscore //
/////////////////////////
        
penalized_bscore ctruth_table_bscore::operator()(const combo_tree& tr) const
{
    //penalized_bscore pbs(
    //    make_pair<behavioral_score, score_t>(behavioral_score(target.size()), 0));
    penalized_bscore pbs;

    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);
    // Evaluate the bscore components for all rows of the ctable
    for (const CTable::value_type& vct : ctable) {
        const CTable::counter_t& c = vct.second;
        score_t sc = c.get(negate_vertex(interpret_tr(vct.first.get_variant())));
        pbs.first.push_back(-sc);
    }

    // Add the Occam's razor feature
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

behavioral_score ctruth_table_bscore::best_possible_bscore() const
{
    behavioral_score bs;
    transform(ctable | map_values, back_inserter(bs),
              [](const CTable::counter_t& c) {
                  // OK, this looks like magic, but here's what it does:
                  // CTable is a compressed table; multiple rows may
                  // have identical inputs, differing only in output.
                  // Clearly, in such a case, both outputs cannot be
                  // simultanously satisfied, but we can try to satisfy
                  // the one of which there is more.  Thus, we take
                  // the min of the two possiblities.
                  return -score_t(min(c.get(id::logical_true),
                                      c.get(id::logical_false)));
              });

    return bs;
}

score_t ctruth_table_bscore::min_improv() const
{
    return 0.5;
}


/////////////////////////
// enum_table_bscore //
/////////////////////////
        
penalized_bscore enum_table_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;

    // Evaluate the bscore components for all rows of the ctable
    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);
    for (const CTable::value_type& vct : ctable) {
        const CTable::counter_t& c = vct.second;
        // The number that are wrong equals total minus num correct.
        score_t sc = score_t(c.get(interpret_tr(vct.first.get_variant())));
        sc -= score_t(c.total_count());
        pbs.first.push_back(sc);
    }

    // Add the Occam's razor feature
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

behavioral_score enum_table_bscore::best_possible_bscore() const
{
    behavioral_score bs;
    transform(ctable | map_values, back_inserter(bs),
              [](const CTable::counter_t& c) {
                  // OK, this looks like magic, but here's what it does:
                  // CTable is a compressed table; multiple rows may
                  // have identical inputs, differing only in output.
                  // Clearly, in such a case, different outputs cannot be
                  // simultanously satisfied, but we can try to satisfy
                  // the one of which there is the most.
                  unsigned most = 0;
                  CTable::counter_t::const_iterator it = c.begin();
                  for (; it != c.end(); it++) {
                      if (most < it->second) most = it->second;
                  }
                  return score_t (most - c.total_count());
              });

    return bs;
}

score_t enum_table_bscore::min_improv() const
{
    return 0.5;
}

/////////////////////////
// enum_filter_bscore //
/////////////////////////
        
penalized_bscore enum_filter_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;

    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;

    pre_it it = tr.begin();
    if (is_enum_type(*it)) 
        return enum_table_bscore::operator()(tr);

    OC_ASSERT(*it == id::cond, "Error: unexpcected candidate!");
    sib_it predicate = it.begin();
    vertex consequent = *next(predicate);

    // Evaluate the bscore components for all rows of the ctable
    interpreter_visitor iv_tr(tr), iv_predicate(predicate);
    auto interpret_tr = boost::apply_visitor(iv_tr);
    auto interpret_predicate = boost::apply_visitor(iv_predicate);
    for (const CTable::value_type& vct : ctable) {
        const CTable::counter_t& c = vct.second;

        unsigned total = c.total_count();

        // The number that are wrong equals total minus num correct.
        score_t sc = score_t(c.get(interpret_tr(vct.first.get_variant())));
        sc -= score_t(total);

        // Punish the first predicate, if it is wrong.
        vertex pr = interpret_predicate(vct.first.get_variant());
        if (pr == id::logical_true) {
            if (total != c.get(consequent))
                sc -= punish * total;
        }

        pbs.first.push_back(sc);
    }

    // Add the Occam's razor feature
    if (occam)
        pbs.second = tree_complexity(tr) * complexity_coef;

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

/////////////////////////
// enum_graded_bscore //
/////////////////////////

/// OK, the goal here is to compute the "graded" tree complexity.
/// Much the same way as the score is graded below, we want to do
/// the same for the complexity, so that complex later predicates
/// don't (do?) dominate the the penalty.  Actually, this is
/// retro-graded: punish more complex, later predicates...
score_t enum_graded_bscore::graded_complexity(combo_tree::iterator it) const
{
    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;
    sib_it predicate = it.begin();
    score_t cpxy = 0.0;
    score_t weight = 1.0;
    while (1) {
        cpxy += weight * score_t(tree_complexity((pre_it) predicate));

        // Is it the last one, the else clause?
        if (is_enum_type(*predicate))
            break;

        // advance
        predicate = next(predicate, 2);
        weight /= grading;

    }
    return cpxy;
}
        
penalized_bscore enum_graded_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;

    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;

    pre_it it = tr.begin();
    if (is_enum_type(*it)) 
        return enum_table_bscore::operator()(tr);

    OC_ASSERT(*it == id::cond, "Error: unexpected candidate!");

    // Evaluate the bscore components for all rows of the ctable
    // TODO
    sib_it predicate = it.begin();
    for (const CTable::value_type& vct : ctable) {
        const CTable::counter_t& c = vct.second;

        unsigned total = c.total_count();
        score_t weight = 1.0;

        // The number that are wrong equals total minus num correct.
        score_t sc = -score_t(total);
        while (1) {
            // Is it the last one, the else clause?
            if (is_enum_type(*predicate)) {
                vertex consequent = *predicate;
                sc += c.get(consequent);
                sc *= weight;
                break;
            }
    
            // The first true predicate terminates.
            interpreter_visitor iv(predicate);
            vertex pr = boost::apply_visitor(iv, vct.first.get_variant());
            if (pr == id::logical_true) {
                vertex consequent = *next(predicate);
                sc += c.get(consequent);
                sc *= weight;
                break;
            }

            // advance
            predicate = next(predicate, 2);
            weight *= grading;
        }
        pbs.first.push_back(sc);
    }

    // Add the Occam's razor feature
    pbs.second = 0.0;
    if (occam) {
        // pbs.second = tree_complexity(tr) * complexity_coef;
        pbs.second = graded_complexity(it) * complexity_coef;
    }

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

score_t enum_graded_bscore::min_improv() const
{
    // Negative values are interpreted as percentages by the optimizer.
    // So -0.05 means "a 5% improvement".  Problem is, the grading
    // wrecks any sense of an absolute score improvement...
    return -0.05;
}

// Much like enum_graded_score, above, except that we exchange the 
// inner and outer loops.  This makes the algo slower and bulkier, but
// it does allow the effectiveness of predicates to be tracked.
//
penalized_bscore enum_effective_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;

    typedef combo_tree::sibling_iterator sib_it;
    typedef combo_tree::iterator pre_it;

    pbs.first = behavioral_score(_ctable_usize);

    // Is this just a constant? Then just add them up.
    pre_it it = tr.begin();
    if (is_enum_type(*it)) {
        behavioral_score::iterator bit = pbs.first.begin();
        for (const CTable::value_type& vct : ctable) {
            const CTable::counter_t& c = vct.second;

            // The number that are wrong equals total minus num correct.
            *bit++ = c.get(*it) - score_t(c.total_count());
        }
        return pbs;
    }

    OC_ASSERT(*it == id::cond, "Error: unexpcected candidate!");

    // Accumulate the score with multiple passes, so zero them out here.
    for (score_t& sc : pbs.first) sc = 0.0;

    // Are we done yet?
    vector<bool> done(_ctable_usize);
    vector<bool>::iterator dit = done.begin();
    for (; dit != done.end(); dit++) *dit = false;

    sib_it predicate = it.begin();
    score_t weight = 1.0;
    while (1) {

        // Is it the last one, the else clause?
        if (is_enum_type(*predicate)) {
            vertex consequent = *predicate;

            behavioral_score::iterator bit = pbs.first.begin();
            vector<bool>::iterator dit = done.begin();
            for (const CTable::value_type& vct : ctable) {
                if (*dit == false) {
                    const CTable::counter_t& c = vct.second;

                    // The number that are wrong equals total minus num correct.
                    score_t sc = -score_t(c.total_count());
                    sc += c.get(consequent);
                    *bit += weight * sc;
                }
                bit++;
                dit++;
            }
            break;
        }

        vertex consequent = *next(predicate);

        // Evaluate the bscore components for all rows of the ctable
        behavioral_score::iterator bit = pbs.first.begin();
        vector<bool>::iterator dit = done.begin();

        bool effective = false;
        interpreter_visitor iv(predicate);
        auto interpret_predicate = boost::apply_visitor(iv);
        for (const CTable::value_type& vct : ctable) {
            if (*dit == false) {
                vertex pr = interpret_predicate(vct.first.get_variant());
                if (pr == id::logical_true) {
                    const CTable::counter_t& c = vct.second;
                    int sc = c.get(consequent);
                    // A predicate is effective if it evaluates to true,
                    // and at least gets a right answr when it does...
                    if (0 != sc) effective = true;

                    // The number that are wrong equals total minus num correct.
                    sc -= c.total_count();
                    *bit += weight * score_t(sc);

                    *dit = true;
                }
            }
            bit++;
            dit++;
        }

        // advance
        predicate = next(predicate, 2);
        if (effective) weight *= grading;
    }

    // Add the Occam's razor feature
    pbs.second = 0.0;
    if (occam) {
        // pbs.second = tree_complexity(tr) * complexity_coef;
        pbs.second = graded_complexity(it) * complexity_coef;
    }

    log_candidate_pbscore(tr, pbs);

    return pbs;
}

//////////////////////////////////
// interesting_predicate_bscore //
//////////////////////////////////

interesting_predicate_bscore::interesting_predicate_bscore(const CTable& ctable_,
                                                           weight_t kld_w_,
                                                           weight_t skewness_w_,
                                                           weight_t stdU_w_,
                                                           weight_t skew_U_w_,
                                                           score_t min_activation_,
                                                           score_t max_activation_,
                                                           score_t penalty_,
                                                           bool positive_,
                                                           bool abs_skewness_,
                                                           bool decompose_kld_)
    : _ctable(ctable_),
      _kld_w(kld_w_), _skewness_w(skewness_w_), _abs_skewness(abs_skewness_),
      _stdU_w(stdU_w_), _skew_U_w(skew_U_w_), _min_activation(min_activation_),
      _max_activation(max_activation_), _penalty(penalty_), _positive(positive_),
      _decompose_kld(decompose_kld_)
{
    // Define counter (mapping between observation and its number of occurences)
    boost::for_each(_ctable | map_values, [this](const CTable::mapped_type& mv) {
            boost::for_each(mv, [this](const CTable::mapped_type::value_type& v) {
                    _counter[get_contin(v.first)] += v.second; }); });

    // Precompute pdf (probability distribution function)
    if (_kld_w > 0) {
        _pdf = _counter;
        _klds.set_p_pdf(_pdf);

        // Compute the skewness of the pdf
        accumulator_t acc;
        for (const auto& v : _pdf)
            acc(v.first, weight = v.second);
        _skewness = weighted_skewness(acc);
        logger().fine("interesting_predicate_bscore::_skewness = %f", _skewness);
    }
}

penalized_bscore interesting_predicate_bscore::operator()(const combo_tree& tr) const
{
    OTable pred_ot(tr, _ctable);

    vertex target = bool_to_vertex(_positive);
    
    unsigned total = 0;   // total number of observations (could be optimized)
    unsigned actives = 0; // total number of positive (or negative if
                          // positive is false) predicate values
    boost::for_each(_ctable | map_values, pred_ot,
                    [&](const CTable::counter_t& c, const vertex& v) {
                        unsigned tc = c.total_count();
                        if (v == target)
                            actives += tc;
                        total += tc;
                    });

    logger().fine("total = %u", total);
    logger().fine("actives = %u", actives);

    penalized_bscore pbs;
    behavioral_score &bs = pbs.first;

    // filter the output according to pred_ot
    counter_t pred_counter;     // map obvervation to occurence
                                // conditioned by predicate truth
    boost::for_each(_ctable | map_values, pred_ot,
                    [&](const CTable::counter_t& c, const vertex& v) {
                        if (v == target) {
                            for (const auto& mv : c)
                                pred_counter[get_contin(mv.first)] = mv.second;
                        }});

    logger().fine("pred_ot.size() = %u", pred_ot.size());
    logger().fine("pred_counter.size() = %u", pred_counter.size());

    if (pred_counter.size() > 1) { // otherwise the statistics are
                                   // messed up (for instance
                                   // pred_skewness can be inf)
        // compute KLD
        if (_kld_w > 0.0) {
            if (_decompose_kld) {
                _klds(pred_counter, back_inserter(bs));
                boost::transform(bs, bs.begin(), _kld_w * arg1);
            } else {
                score_t pred_klds = _klds(pred_counter);
                logger().fine("klds = %f", pred_klds);
                bs.push_back(_kld_w * pred_klds);
            }
        }

        if (_skewness_w > 0 || _stdU_w > 0 || _skew_U_w > 0) {
            
            // gather statistics with a boost accumulator
            accumulator_t acc;
            for (const auto& v : pred_counter)
                acc(v.first, weight = v.second);

            score_t diff_skewness = 0;
            if (_skewness_w > 0 || _skew_U_w > 0) {
                // push the absolute difference between the
                // unconditioned skewness and conditioned one
                score_t pred_skewness = weighted_skewness(acc);
                diff_skewness = pred_skewness - _skewness;
                score_t val_skewness = (_abs_skewness?
                                        abs(diff_skewness):
                                        diff_skewness);
                logger().fine("pred_skewness = %f", pred_skewness);
                if (_skewness_w > 0)
                    bs.push_back(_skewness_w * val_skewness);
            }

            score_t stdU = 0;
            if (_stdU_w > 0 || _skew_U_w > 0) {

                // compute the standardized Mann–Whitney U
                stdU = standardizedMannWhitneyU(_counter, pred_counter);
                logger().fine("stdU = %f", stdU);
                if (_stdU_w > 0.0)
                    bs.push_back(_stdU_w * abs(stdU));
            }
                
            // push the product of the relative differences of the
            // shift (stdU) and the skewness (so that if both go
            // in the same direction the value if positive, and
            // negative otherwise)
            if (_skew_U_w > 0)
                bs.push_back(_skew_U_w * stdU * diff_skewness);
        }
            
        // add activation_penalty component
        score_t activation = actives / (score_t) total;
        score_t activation_penalty = get_activation_penalty(activation);
        logger().fine("activation = %f", activation);
        logger().fine("activation penalty = %e", activation_penalty);
        bs.push_back(activation_penalty);
        
        // add the Occam's razor feature
        if (occam)
            pbs.second = tree_complexity(tr) * complexity_coef;
    } else {
        pbs.first.push_back(very_worst_score);
    }

    // Logger
    log_candidate_pbscore(tr, pbs);
    // ~Logger

    return pbs;
}

behavioral_score interesting_predicate_bscore::best_possible_bscore() const
{
    return behavioral_score(1, very_best_score);
}

void interesting_predicate_bscore::set_complexity_coef(unsigned alphabet_size,
                                                       float stdev)
{
    complexity_coef = 0.0;
    occam = stdev > 0;
    if (occam)
        complexity_coef = contin_complexity_coef(alphabet_size, stdev);

    logger().info() << "intersting_predicate_bscore noise = " << stdev
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/complexity_coef;
}

score_t interesting_predicate_bscore::get_activation_penalty(score_t activation) const
{
    score_t dst = max(max(_min_activation - activation, score_t(0))
                      / _min_activation,
                      max(activation - _max_activation, score_t(0))
                      / (1 - _max_activation));
    logger().fine("dst = %f", dst);
    return log(pow((1 - dst), _penalty));
}

score_t interesting_predicate_bscore::min_improv() const
{
    return 0.0;                 // not necessarily right, just the
                                // backwards-compatible behavior
}

//////////////////////////////
// multibscore_based_bscore //
//////////////////////////////

// main operator
penalized_bscore multibscore_based_bscore::operator()(const combo_tree& tr) const
{
    penalized_bscore pbs;
    for (const bscore_base& bsc : _bscorers) {
        penalized_bscore apbs = bsc(tr);
        boost::push_back(pbs.first, apbs.first);
        pbs.second += apbs.second;
    }
    return pbs;
}

behavioral_score multibscore_based_bscore::best_possible_bscore() const
{
    behavioral_score bs;
    for (const bscore_base& bsc : _bscorers) {
        boost::push_back(bs, bsc.best_possible_bscore());
    }
    return bs;
}

// return the min of all min_improv
score_t multibscore_based_bscore::min_improv() const
{
    /// @todo can be turned in to 1-line with boost::min_element
    // boost::min_element(_bscorers | boost::transformed(/*)
    score_t res = very_best_score;
    for (const bscore_base& bs : _bscorers)
        res = min(res, bs.min_improv());
    return res;
}

void multibscore_based_bscore::ignore_idxs(const std::set<arity_t>& idxs) const
{
    for (const bscore_base& bs : _bscorers)
        bs.ignore_idxs(idxs);
}

} // ~namespace moses
} // ~namespace opencog
