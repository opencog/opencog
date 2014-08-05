/*
 * opencog/learning/moses/moses/discriminating_bscore.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012,2013 Poulin Holdings LLC
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

#include "discriminating_bscore.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>

namespace opencog { namespace moses {

using namespace std;
using namespace combo;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;
using boost::adaptors::filtered;
using boost::adaptors::reversed;
using boost::adaptors::transformed;
using boost::transform;
using namespace boost::phoenix;
using boost::phoenix::arg_names::arg1;
using namespace boost::accumulators;

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
                res += std::max(0.0, get_contin(cv.first.value) * cv.second);
            return res;
        };
        // For contin tables, we return the sum of the row values < 0
        sum_false = [](const CTable::counter_t& c)->score_t
        {
            score_t res = 0.0;
            for (const CTable::counter_t::value_type& cv : c)
                res += std::min(0.0, get_contin(cv.first.value) * cv.second);
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
    // XXX Currently, this scorer does not return a true behavioral score
    _size = 2;

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
                score_t val = get_contin(cv.first.value);
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
    _complexity_coef = 0.0;
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    if (p > 0.0f and p < 0.5f)
        _complexity_coef = discrete_complexity_coef(alphabet_size, p)
            / _ctable_usize;    // normalized by the size of the table
                                // because the precision is normalized
                                // as well

    logger().info() << "Discriminating scorer, noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/_complexity_coef;
}

void discriminating_bscore::set_complexity_coef(score_t ratio)
{
    _complexity_coef = 0.0;

    // The complexity coeff is normalized by the size of the table,
    // because the precision is normalized as well.  So e.g.
    // max precision for boolean problems is 1.0.  However...
    // umm XXX I think the normalization here should be the
    // best-possible activation, not the usize, right?
    //
    // @todo Sounds good too, as long as it's constant, so you would
    // replace _ctable_usize by _ctable_usize * max_activation?
    if (ratio > 0)
        _complexity_coef = 1.0 / (_ctable_usize * ratio);

    logger().info() << "Discriminating scorer, table size = " << _ctable_usize;
    logger().info() << "Discriminating scorer, complexity ratio = " << 1.0f/_complexity_coef;
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
    // XXX Currently, this scorer does not return a true behavioral score
    _size = 2;
}

behavioral_score recall_bscore::operator()(const combo_tree& tr) const
{
    d_counts ctr = count(tr);

    // Compute normalized precision and recall.
    score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
    score_t precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 0.0;

    score_t tp_fn = ctr.true_positive_sum + ctr.false_negative_sum;
    score_t recall = (0.0 < tp_fn) ? ctr.true_positive_sum / tp_fn : 0.0;

    // We are maximizing recall, so that is the first part of the score.
    behavioral_score bs;
    bs.push_back(recall);

    score_t precision_penalty = get_threshold_penalty(precision);
    bs.push_back(precision_penalty);
    if (logger().isFineEnabled())
        logger().fine("recall_bcore: precision = %f  recall=%f  precision penalty=%e",
                     precision, recall, precision_penalty);

    log_candidate_bscore(tr, bs);
    return bs;
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
    // XXX TODO -- should not return the penalties as part of the bscore,
    // since this messes up boosting.
    _size = ct.size() + 2;
}

// Nearly identical to recall_bscore, except that the roles of precision
// and recall are switched.
behavioral_score prerec_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;
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
            bs.push_back((ctr.true_positive_sum - ctr.false_positive_sum)
                                / 2);
            pos_total += ctr.positive_count;
        }
        // divide all element by pos_total (so it sums up to precision - 1/2)
        boost::transform(bs, bs.begin(), arg1 / pos_total);

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
        bs.push_back(0.5);

        // compute precision and recall
        precision = boost::accumulate(bs, 0);
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
        bs.push_back(precision);
    }

    // calculate recall_penalty
    score_t recall_penalty = get_threshold_penalty(recall);
    bs.push_back(recall_penalty);

    // Log precision, recall and penalty
    if (logger().isFineEnabled())
        logger().fine("prerec_bscore: precision = %f  "
                      "recall=%f  recall penalty=%e",
                      precision, recall, recall_penalty);

    log_candidate_bscore(tr, bs);
    return bs;
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
    // XXX Currently, this scorer does not return a true behavioral score
    _size = 2;
}

behavioral_score bep_bscore::operator()(const combo_tree& tr) const
{
    d_counts ctr = count(tr);

    // Compute normalized precision and recall.
    score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
    score_t precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 0.0;

    score_t tp_fn = ctr.true_positive_sum + ctr.false_negative_sum;
    score_t recall = (0.0 < tp_fn) ? ctr.true_positive_sum / tp_fn : 0.0;

    score_t bep = (precision + recall) / 2;
    // We are maximizing bep, so that is the first part of the score.
    behavioral_score bs;
    bs.push_back(bep);

    score_t bep_diff = fabs(precision - recall);
    score_t bep_penalty = get_threshold_penalty(bep_diff);
    bs.push_back(bep_penalty);
    if (logger().isFineEnabled())
        logger().fine("bep = %f  diff=%f  bep penalty=%e",
                     bep, bep_diff, bep_penalty);

    log_candidate_bscore(tr, bs);
    return bs;
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
    // XXX Currently, this scorer does not return a true behavioral score
    _size = 1;
}

behavioral_score f_one_bscore::operator()(const combo_tree& tr) const
{
    d_counts ctr = count(tr);

    // Compute normalized precision and recall.
    score_t tp_fp = ctr.true_positive_sum + ctr.false_positive_sum;
    score_t precision = (0.0 < tp_fp) ? ctr.true_positive_sum / tp_fp : 0.0;

    score_t tp_fn = ctr.true_positive_sum + ctr.false_negative_sum;
    score_t recall = (0.0 < tp_fn) ? ctr.true_positive_sum / tp_fn : 0.0;

    score_t f_one = 2 * precision * recall / (precision + recall);

    // We are maximizing f_one, so that is the first part of the score.
    behavioral_score bs;
    bs.push_back(f_one);

    if (logger().isFineEnabled())
        logger().fine("f_one_bscore: precision = %f recall = %f f_one=%f",
                     precision, recall, f_one);

    log_candidate_bscore(tr, bs);
    return bs;
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

} // ~namespace moses
} // ~namespace opencog
