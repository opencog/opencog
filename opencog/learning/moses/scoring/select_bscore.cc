/*
 * opencog/learning/moses/moses/select_bscore.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012,2013 Poulin Holdings LLC
 * Copyright (C) 2014 Aidyia Limited
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

#include "select_bscore.h"

namespace opencog { namespace moses {

using namespace combo;

/**
 * The selection scorer will learn a window that selects a target range
 * of values in a continuous-valued dataset.
 *
 * The dataset is assumed to have a contin-valued output column. The
 * highest and lowest values are found in the output column, and the
 * window is then framed relative to these two boundaries.  That is,
 * if 'low_val' and 'hi_val' are the smallest and largest values in the
 * table, then the bottom of the selected window is located at
 *
 *    window_bottom = low_val + lower_percentile * (hi_val - low_val)
 *
 * and the top is located at
 *
 *    window_top = low_val + upper_percentile * (hi_val - low_val)
 *
 * If the rows are weighted, then the weighted equivalent of the above
 * boundaries is used.
 *
 * As currently implemented, this scorer behaves more-or-less exactly
 * the same as the plain boolean 'accuracy' scorer applied to the
 * booleanized version of the data.  That is, this scorer will provide
 * very similar results, and very similar performance, to what one
 * would get if one re-processed the contin data, converted the
 * selection range to an inside-or-outside boolean indicator, and then
 * used the plain 'it' boolean scorer to measure the accuracy of the
 * tree.
 *
 * The intended generalization, which would differentiate the two,
 * remains un-implemented.  The generalization would soften the edges
 * of the selection region, turning it into a trapezoid or a gaussian-
 * like bump function.  Such a softened-edge selector could not be
 * mimicked with a pure boolean accuracy scorer.
 */
select_bscore::select_bscore(const CTable& ctable,
                             double lower_percentile,
                             double upper_percentile,
                             double hardness,
                             bool positive) :
    bscore_ctable_base(ctable), _positive(positive)
{
    OC_ASSERT(id::contin_type == _wrk_ctable.get_output_type(),
        "The selection scorer can only be used with contin-valued tables!");

    // This is needed even if not boosted, as otherwise the
    // ignore_cols(), ignore_rows() methods strip out rows,
    // and cause mis-matches during scoring.
    _return_weighted_score = true;

    reset_weights();

    // Verify that the bounds are sane
    lower_percentile = fmax(0.0, lower_percentile);
    lower_percentile = fmin(1.0, lower_percentile);
    upper_percentile = fmax(0.0, upper_percentile);
    upper_percentile = fmin(1.0, upper_percentile);
    OC_ASSERT((lower_percentile < upper_percentile),
        "Selection scorer, invalid percentiles: %f and %f",
        lower_percentile, upper_percentile);

    // Maps are implicitly ordered, so the below has the effect of
    // putting the rows into sorted order, by output score.
    std::map<score_t, score_t> ranked_out;

    score_t total_weight = 0.0;
    for (const CTable::value_type& io_row : ctable) {
        // io_row.first = input vector
        // io_row.second = counter of outputs
        const CTable::counter_t& orow = io_row.second;
        for (const CTable::counter_t::value_type& tcv : orow) {
            score_t val = get_contin(tcv.first.value);
            if (not _positive) val = -val;
            score_t weight = tcv.second;
            ranked_out.insert(std::pair<score_t, score_t>(val, weight));
            total_weight += weight;
        }
    }

    logger().info() << "select_bscore: lower_percentile = " << lower_percentile
                    << " upper percentile = " << upper_percentile
                    << " total wieght = " << total_weight;

    // Now, carve out the selected range.
    upper_percentile *= total_weight;
    lower_percentile *= total_weight;
    score_t running_weight = 0.0;
    score_t last_val = very_worst_score;
    bool found_lower = false;
    bool found_upper = false;
    for (auto score_row : ranked_out) {
        score_t val = score_row.first;

        score_t weight = score_row.second;
        running_weight += weight;

        if (not found_lower and lower_percentile <= running_weight) {
            _lower_bound = val;
            found_lower = true;
        }
        if (not found_upper and upper_percentile <= running_weight) {
            _upper_bound = val;
            found_upper = true;
        }
        last_val = val;
    }
    if (not found_upper) {
        _upper_bound = last_val * (1.0 + 2.0*epsilon_score);
    }
    OC_ASSERT((_lower_bound < _upper_bound),
        "Selection scorer, invalid bounds: %f and %f",
        _lower_bound, _upper_bound);

    logger().info() << "select_bscore: lower_bound = " << _lower_bound
                    << " upper bound = " << _upper_bound;

    set_best_possible_bscore();
}

/**
 * selection scorer, indicates if a row is inside or outside of the
 * selection range.
 *
 * If the boolean tree correctly predicts that a row is inside/outside
 * the selection range, then scorer rewards this with a score of zero.
 * Otherwise, the score is minus the weight of that row.
 */
behavioral_score select_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;

    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);

    for (const CTable::value_type& io_row : _wrk_ctable) {
        // io_row.first = input vector
        // io_row.second = counter of outputs

        bool predict_inside = (id::logical_true == interpret_tr(io_row.first.get_variant()));
        score_t fail = 0.0;
        const CTable::counter_t& orow = io_row.second;
        for (const CTable::counter_t::value_type& tcv : orow) {
            score_t val = get_contin(tcv.first.value);
            if (not _positive) val = -val;
            score_t weight = tcv.second;
            bool inside = (_lower_bound <= val and val <= _upper_bound);
            if ((predict_inside and not inside) or (not predict_inside and inside))
                fail += weight;
        }
        bs.push_back(-fail);
    }

    // Report the score only relative to the best-possible score.
    bs -= _best_possible_score;

    return bs;
}

/// Scorer, suitable for use with boosting.
behavioral_score select_bscore::operator()(const scored_combo_tree_set& ensemble) const
{
    // Step 1: accumulate the weighted prediction of each tree in
    // the ensemble.
    behavioral_score hypoth (_size, 0.0);
    for (const scored_combo_tree& sct: ensemble) {
        const combo_tree& tr = sct.get_tree();
        score_t trweight = sct.get_weight();

        interpreter_visitor iv(tr);
        auto interpret_tr = boost::apply_visitor(iv);

        size_t i=0;
        for (const CTable::value_type& io_row : _wrk_ctable) {
            // io_row.first = input vector
            bool select = (interpret_tr(io_row.first.get_variant()) == id::logical_true);
            hypoth[i] += trweight * (2.0 * ((score_t) select) - 1.0);
            i++;
        }
    }

    // Step 2: compare the prediction of the ensemble to the desired
    // result. The array "hypoth" is positive to predict true, and
    // negative to predict false.  The resulting score is 0 if correct,
    // and -1 if incorrect.
    behavioral_score bs;
    size_t i = 0;
    for (const CTable::value_type& io_row : _wrk_ctable) {
        // io_row.first = input vector
        // io_row.second = counter of outputs

        bool predict_inside = (0 < hypoth[i]);
        score_t fail = 0.0;
        const CTable::counter_t& orow = io_row.second;
        for (const CTable::counter_t::value_type& tcv : orow) {
            score_t val = get_contin(tcv.first.value);
            if (not _positive) val = -val;
            score_t weight = tcv.second;
            bool inside = (_lower_bound <= val and val <= _upper_bound);
            if ((predict_inside and not inside) or (not predict_inside and inside))
                fail += weight;
        }
        bs.push_back(-fail);
        i++;
    }

    // Report the score only relative to the best-possible score.
    bs -= _best_possible_score;

    return bs;
}

behavioral_score select_bscore::best_possible_bscore() const
{
    return behavioral_score(_size, 0.0);
}

void select_bscore::set_best_possible_bscore()
{
    for (const CTable::value_type& io_row : _wrk_ctable) {
        // io_row.first = input vector
        // io_row.second = counter of outputs

        // Any row with a multiplicity of just one can be inherently
        // classified.  Thus, we push back 0.0 for that case. But
        // for rows with multiplicities greater than one, the situation
        // is trickier: some may be inside, and some outside the
        // selection range.
        score_t n_inside = 0.0;
        score_t n_outside = 0.0;
        const CTable::counter_t& orow = io_row.second;
        for (const CTable::counter_t::value_type& tcv : orow) {
            score_t val = get_contin(tcv.first.value);
            if (not _positive) val = -val;
            score_t weight = tcv.second;
            if (_lower_bound <= val and val <= _upper_bound)
                n_inside += weight;
            else
                n_outside += weight;
        }
        score_t sum = n_inside + n_outside;
        score_t split = fabs(n_inside - n_outside);
        _best_possible_score.push_back(-0.5 * fabs(sum - split));
    }

    logger().info() << "select_bscore: Best possible: "
                    << _best_possible_score;
}

behavioral_score select_bscore::worst_possible_bscore() const
{
    behavioral_score bs;
    for (const CTable::value_type& io_row : _wrk_ctable) {
        // io_row.first = input vector
        // io_row.second = counter of outputs

        // Any row with a multiplicity of just one can be inherently
        // mis-classified.  Thus, we push back -1 for that case. But
        // for rows with multiplicities greater than one, the situation
        // is trickier: some may be inside, and some outside the
        // selection range.
        score_t n_inside = 0.0;
        score_t n_outside = 0.0;
        const CTable::counter_t& orow = io_row.second;
        for (const CTable::counter_t::value_type& tcv : orow) {
            score_t val = get_contin(tcv.first.value);
            if (not _positive) val = -val;
            score_t weight = tcv.second;
            if (_lower_bound <= val and val <= _upper_bound)
                n_inside += weight;
            else
                n_outside += weight;
        }
        bs.push_back(-fabs(n_inside - n_outside));
    }
    return bs;
}

score_t select_bscore::get_error(const behavioral_score& bs) const
{
    return - sum_bscore(bs) / _ctable_weight;
}

// XXX This is not quite right, for weighted rows.  A row with a small
// weight could result in a much smaller min-improv.
// (But I think boosting should not affect min-improv, right?)
score_t select_bscore::min_improv() const
{
    return 1.0 / _ctable_weight;
}

void select_bscore::reset_weights()
{
    behavioral_score bs = worst_possible_bscore();
    double znorm = 0.0;
    for (size_t i=0; i<_size; i++) znorm -= bs[i];
    _effective_length = znorm;
    znorm /= _size;
    _weights = std::vector<double>(_size, znorm);

    logger().info() << "select_bscore::reset_weights() norm=" << znorm;
}

void select_bscore::update_weights(const std::vector<double>& rew)
{
    double znorm = 0.0;
    for (size_t i = 0; i < _size; i++) {
        _weights[i] *= rew[i];
        znorm += _weights[i];
    }

    // Normalization: sum of weights must equal 1.0
    znorm = _effective_length / znorm;
    for (size_t i=0; i<_size; i++) _weights[i] *= znorm;
}

} // ~namespace moses
} // ~namespace opencog
