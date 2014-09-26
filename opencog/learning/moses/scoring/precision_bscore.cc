/*
 * opencog/learning/moses/moses/precision_bscore.cc
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

#include <boost/range/irange.hpp>

#include <opencog/comboreduct/table/table_io.h>

#include "precision_bscore.h"

namespace opencog { namespace moses {

using namespace std;
using namespace combo;

//////////////////////
// precision_bscore //
//////////////////////

/// This scorer attempts to maximize precision while holding "activation"
/// at or above a given level.  The precision is computed in a somewhat
/// unusual fashion, and there is no standard texbook definition for
/// activation, so we define these below.  But first: please note that
/// there are also text-book-style scorers for maximizing precision or
/// recall, while holding the other constant.  These are more directly
/// sutiable for generating ROC curves.  For these, see the "prerec",
/// "recall", "fone" (F_1) and "bep" (break-even point) scorers, derived
/// from the discriminating scorer in discriminating_bscore.[cc|h]
///
/// This scorer supports both boolean-valued data tables, and contin-
/// valued tables.  The scoring is slightly different for each.  First,
/// the scoring of boolean-valued tables is described.
///
/// The combo tree given to the scorer below is always assumed to be a
/// binary-valued tree, which either selects a row, or does not.  If the
/// row is selected, then it is said to be "activated".  We then define:
///
///    tp == true-positive, a row that is 'activated' and row has value 1
///    fp == false-positive, a row that is 'activated' and row has value 0
///
/// and similarly
///
///    #tp == +1 if it is a tp row, else 0
///    #fp == -1 if it is an fp row, else 0
///
/// In addition, define:
///
///    TP == total number of tp rows
///    FP == total number of fp rows.
///    AC == TP + FP, the total number of "activated" rows.
///
/// Note that, with the above definition of #tp and #fp, that summing
/// #tp over all rows gives TP, but that summing #fp over all rows
/// gives -FP.
///
/// Each element of the behavioral score is then given a value of
///
///    0.5 * (#tp + #fp) / (TP+FP)    if the row is "activated"
///    0  (zero)                      if the row is not "activated"
///
/// If the above were to be summed over all rows, one would obtain
/// a total score of
///
///    0.5 * (TP-FP)/(TP+FP) == TP/(TP+FP) - 0.5
///                          == precision - 0.5
///
/// Notice the extra 0.5 -- to compensate for this, the behavioral
/// score gets an extra 'phantom' row tacked on, which has a value
/// of +0.5, so that the resulting composite score always contains
/// the precision.
///
/// This scorer also uses a penalty mechanism to make sure that the
/// activation stays above a minimum value. See get_activation_penaly()
/// for details.
///
/// This scorer also supports contin-valued tables.  In this case, the
/// combo tree is still treated as described above: it is boolean, and
/// either "activates" a row, or it does not.  However, the scoring is
/// now different.  Each row is now given the score
///
///    val/AC    if the row is activated
///    0.0       if the row is not activated
///
/// where "val" is the contin-value of the table row, and AC is the
/// total number of activated rows.
///
/// The above only descirbes the "precision_full_bscore" (default)
/// scoring; something else is done when precision_full_bscore is
/// set to false.  TBD XXX document that someday.


precision_bscore::precision_bscore(const CTable& ctable_,
                                   float activation_pressure_,
                                   float min_activation_,
                                   float max_activation_,
                                   float dispersion_pressure,
                                   float dispersion_exponent,
                                   bool exact_experts_,
                                   double bias_scale_,
                                   bool positive_,
                                   bool time_bscore_,
                                   TemporalGranularity granularity)
    : bscore_ctable_time_dispersion(ctable_, dispersion_pressure,
                                    dispersion_exponent, granularity),
    min_activation(bound(min_activation_, 0.0f, 1.0f)),
    max_activation(bound(max_activation_, 0.0f, 1.0f)),
    activation_pressure(activation_pressure_),
    bias_scale(bias_scale_),
    exact_experts(exact_experts_),
    positive(positive_),
    time_bscore(time_bscore_)
{
    reset_weights();

    // _target will be either id::logical_true, or id::logical_false
    _target = bool_to_vertex(positive);
    _neg_target = negate_vertex(_target);

    output_type = _wrk_ctable.get_output_type();
    OC_ASSERT(output_type == id::boolean_type,
        "Error: Precision scorer: output type must be boolean!");

    logger().fine("Precision scorer, "
                  "total table weight = %f, "
                  "activation_pressure = %f, "
                  "min_activation = %f, "
                  "max_activation = %f, "
                  "dispersion_pressure = %f",
                  _ctable_weight,
                  activation_pressure, min_activation, max_activation,
                  dispersion_pressure);

    // Verify that the activation parameters are sane
    OC_ASSERT((0.0 < activation_pressure) && (0.0 < min_activation)
              && (min_activation <= max_activation),
              "Precision scorer, invalid activation bounds.  "
              "The activation pressure must be non-zero, the minimum activation must be "
              "greater than zero, and the maximum activation must be greater "
              "than or equal to the minimum activation.\n");
}

/// For boolean tables, sum the total number of 'T' values
/// in the output.  This sum represents the best possible score
/// i.e. we found all of the true values correcty.  The 'F's are 
/// false positives.
score_t precision_bscore::sum_outputs(const CTable::counter_t& c) const
{
    return 0.5 * (c.get(_target) - c.get(_neg_target));
}

void precision_bscore::set_complexity_coef(unsigned alphabet_size, float p)
{
    _complexity_coef = 0.0;
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    if (p > 0.0f and p < 0.5f)
        _complexity_coef = discrete_complexity_coef(alphabet_size, p)
            / _ctable_weight;   // normalized by the size of the table
                                // because the precision is normalized
                                // as well

    logger().info() << "Precision scorer, noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/_complexity_coef;
}

void precision_bscore::set_complexity_coef(score_t ratio)
{
    _complexity_coef = 0.0;
    if (ratio > 0)
        _complexity_coef = 1.0 / ratio;

    logger().info() << "Precision scorer, complexity ratio = " << 1.0f/_complexity_coef;
}

/// scorer, the workhorse that does the actual scoring.
behavioral_score precision_bscore::do_score(std::function<bool(const multi_type_seq&)> select) const
{
    behavioral_score bs;
    behavioral_score ac;

    score_t active = 0.0;  // total weight of active outputs by tr
    score_t sao = 0.0;     // sum of all active outputs

    // 1. Compute active and sum of all active outputs
    //
    // 2. If time_bscore enabled, build map between timestamps and a
    // counter of a bool, representing tp and fp. Specifically, if the
    // combo output is true, then tp corresponds the target output being
    // true (resp. false if positive is false), and fp correspond to
    // the the target being false (resp. true if positive is false).
    //
    // 3. Otherwise build CTable holding the results for dispersion
    // penalty
    map<TTable::value_type, Counter<bool, count_t>> time2res;
    CTable ctable_res;
    for (const CTable::value_type& io_row : _wrk_ctable) {
        // const auto& irow = io_row.first;
        const multi_type_seq& irow = io_row.first;
        const auto& orow = io_row.second;

        score_t sumo = 0.0;  // (tp-fp)/2 if active, else 0 if not active
        score_t acto = 0.0;  // fp + tp if active, else 0 if not active
        bool selected = select(irow);
        if (selected) {
            sumo = sum_outputs(orow);
            sao += sumo;
            acto = orow.total_count();
            active += acto;
        }

        if (time_bscore) {
            for (const CTable::counter_t::value_type& tcv : orow) {
                bool target = vertex_to_bool(tcv.first.value);
                if (!positive) target = !target;
                count_t count = selected ? tcv.second : 0.0;
                time2res[tcv.first.timestamp][target] += count;
            }
        } else {
            bs.push_back(sumo);
            ac.push_back(acto);
        }

        // Build CTable of results
        if (_pressure > 0.0) {
            CTable::key_type dummy_input;
            for (const CTable::counter_t::value_type& tcv : orow) {
                auto tclass = get_timestamp_class(tcv.first.timestamp);
                vertex result = selected? id::logical_true : id::logical_false;
                TimedValue timed_result({result, tclass});
                ctable_res[dummy_input][timed_result] += tcv.second;
            }
        }
    }

    // Fill un-normalized bscore spread over the temporal axis
    if (time_bscore) {
        for (const auto& v : time2res) {
            count_t tp = v.second.get(true);
            count_t fp = v.second.get(false);
            score_t contribution = 0.5 * (tp - fp);
            bs.push_back(contribution);
            ac.push_back(tp+fp);
        }
    }

    if (0.0 < active) {
        // normalization is the inverse of activity
        score_t iac = 1.0 / active;

        // For the boosted scorer, the active rows are weighted too.
        // Its still tp+fp, except that the bscore for fp are negative...
        if (_return_weighted_score) {
            double wactive = 0.0;
            for (size_t i=0; i< _size; i++) {
                wactive += _weights[i] * ac[i];
            }
            iac = 1.0 / wactive;
        }

        // Normalize all components by active, where active = tp+fp
        for (auto& v : bs) v *= iac;

        // tp = true positive
        // fp = false positive
        //
        // By using (tp-fp)/2, the sum of all the per-row contributions
        // is offset by -1/2 from the precision, as proved below.
        //
        // 1/2 * (tp - fp) / (tp + fp)
        // = 1/2 * (tp - tp + tp - fp) / (tp + fp)
        // = 1/2 * [(tp + tp) / (tp + fp) - (tp + fp) / (tp + fp)]
        // = 1/2 * [2*tp / (tp + fp) - 1]
        // = precision - 1/2
        //
        // So before adding the recall penalty we add +1/2 to
        // compensate for that
        bs.push_back(0.5);
    }
    else // Add 0.0 to ensure the bscore has the same size
        bs.push_back(0.0);

    // Append the activation penalty only if we are not performing
    // boosting. Why? Because the boosted scorer only wants those
    // selectors that are exactly correct, even if they have a terrible
    // activation.  It will patch these together in the ensemble.  If
    // these perfect scorers were penalized, they'd be discarded in the
    // metapop, and we don't want that.  The member "_return_weighted_score"
    // is a synonym for "we're doing boosting now".
    if (not _return_weighted_score) {
        // For boolean tables, activation sum of true and false positives
        // i.e. the sum of all positives.   For contin tables, the activation
        // is likewise: the number of rows for which the combo tree returned
        // true (positive).
        score_t activation = active / _ctable_weight;
        score_t activation_penalty = get_activation_penalty(activation);
        bs.push_back(activation_penalty);

        if (logger().isFineEnabled()) {
            score_t precision = 0.0;
            if (0 < active) {
                // See above for explanation for the extra 0.5
                precision = sao / active + 0.5;
            }
            logger().fine("precision = %f  activation=%f  activation penalty=%e",
                          precision, activation, activation_penalty);
        }

        // Add time dispersion penalty
        if (_pressure > 0.0) {
            score_t dispersion_penalty =
                get_time_dispersion_penalty(ctable_res.ordered_by_time());
            bs.push_back(dispersion_penalty);
            logger().fine("dispersion_penalty = %f", dispersion_penalty);
        }
    }

    return bs;
}

/// scorer, for use with individual combo trees
behavioral_score precision_bscore::operator()(const combo_tree& tr) const
{
    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);

    std::function<bool(const multi_type_seq&)> selector;
    selector = [&](const multi_type_seq& irow)->bool
    {
        return id::logical_true == interpret_tr(irow.get_variant());
    };

    behavioral_score bs(do_score(selector));
    log_candidate_bscore(tr, bs);
    return bs;
}

behavioral_score precision_bscore::operator()(const scored_combo_tree_set& ensemble) const
{
    // The ensemble score must not use weights, but must be give the
    // result that the ensemble would give on regular data.  Thus,
    // disable weighting while we compute the ensemble score.
    OC_ASSERT(_return_weighted_score, "Unexpected call to ensemble scorer!");
    if (exact_experts) {
        _return_weighted_score = false;
        behavioral_score bs(exact_selection(ensemble));
        _return_weighted_score = true;
        return bs;
    }
    else {
        _return_weighted_score = false;
        behavioral_score bs(bias_selection(ensemble));
        _return_weighted_score = true;
        return bs;
    }
}

/// scorer, suitable for use with boosting, where the members of the
/// boosted ensemble are always correct in their selections. That is,
/// they are "exact" scorers.
behavioral_score precision_bscore::exact_selection(const scored_combo_tree_set& ensemble) const
{
    // For large tables, it is much more efficient to convert the
    // ensemble into a single tree, and just iterate on that, instead
    // of having multiple iterations on multiple trees.  But either way,
    // the two scorers here should be functionally equivalent.
// #define SCORE_MANY_TREES
#ifdef SCORE_MANY_TREES
    // Step 1: If any tree in the ensemble picks a row, that row is
    // picked by the ensemble as a whole.
    std::vector<bool> hypoth(_size, false);
    for (const scored_combo_tree& sct: ensemble) {
        const combo_tree& tr = sct.get_tree();

        interpreter_visitor iv(tr);
        auto interpret_tr = boost::apply_visitor(iv);

        size_t i=0;
        for (const CTable::value_type& io_row : _wrk_ctable) {
            // io_row.first = input vector
            const auto& irow = io_row.first;

            if (interpret_tr(irow.get_variant()) == id::logical_true) {
                hypoth[i] = true;
            }
            i++;
        }
    }

    // Step 2: tot up the active rows.
    size_t i = 0;
    std::function<bool(const multi_type_seq&)> selector;
    selector = [&](const multi_type_seq& irow)->bool
    {
        return hypoth[i++];
    };
    return do_score(selector);
#else // SCORE_MANY_TREES

    // For large tables, it is much more efficient to convert the
    // ensemble into a single tree, so that we iterate over the table
    // just once, instead of once-per-tree.
    if (1 == ensemble.size()) {
        return this->operator()(ensemble.begin()->get_tree());
    }

    combo_tree tr;
    combo_tree::pre_order_iterator head;
    head = tr.set_head(combo::id::logical_or);

    for (const scored_combo_tree& sct : ensemble)
        tr.append_child(head, sct.get_tree().begin());

    return this->operator()(tr);
#endif // SCORE_MANY_TREES
}

/// scorer, suitable for use with boosting, where the members of the
/// ensemble sometimes make mistakes. The mistakes are avoided by 
/// requiring the ensemble to cast a minimum vote, before a row is
/// truely considered to be selected.  The minimum vote is called the
/// "bias" in the code below.
behavioral_score precision_bscore::bias_selection(const scored_combo_tree_set& ensemble) const
{
    // Step 1: If any tree in the ensemble picks a row, that row is
    // picked with appropriate weight.
    std::vector<double> hypoth(_size, 0.0);
    for (const scored_combo_tree& sct: ensemble) {
        const combo_tree& tr = sct.get_tree();
        score_t trweight = sct.get_weight();

        interpreter_visitor iv(tr);
        auto interpret_tr = boost::apply_visitor(iv);

        size_t i=0;
        for (const CTable::value_type& io_row : _wrk_ctable) {
            // io_row.first = input vector
            const auto& irow = io_row.first;

            if (interpret_tr(irow.get_variant()) == id::logical_true) {
                hypoth[i] += trweight;
            }
            i++;
        }
    }

    // Step 2: find the worst wrong answer; that is our bias.
    size_t i=0;
    double bias = 0.0;
    for (const CTable::value_type& io_row : _wrk_ctable) {
        const CTable::counter_t& orow = io_row.second;

        // sumo will be negative if it should not be selected.
        // We look at the negative ones, because they are wrong...
        double sumo = sum_outputs(orow);
        if (0.0 > sumo and bias < hypoth[i]) bias = hypoth[i];
        i++;
    }
    logger().info() << "Precision ensemble bias: " << bias;

    // Step 3: tot up the active rows.
    i = 0;
    std::function<bool(const multi_type_seq&)> selector;
    selector = [&](const multi_type_seq& irow)->bool
    {
        return bias_scale * bias < hypoth[i++];
    };
    return do_score(selector);
}

score_t precision_bscore::get_error(const combo_tree& tr) const
{
    OC_ASSERT (exact_experts,
        "Precision scorer: inexact experts not yet supported!");

    // We want the flat, unweighted score!
    _return_weighted_score = false;
    behavioral_score bs(operator()(tr));
    _return_weighted_score = true;

    // We only accumulate up to the _size, and ignore the penalties on
    // the end!
    double accum = 0.0;
    for (size_t i=0; i<_size; i++) accum += bs[i];

    // perfect score has accum = 0.5.
    return 0.5 - accum;
}

behavioral_score precision_bscore::best_possible_bscore() const
{
    // For each row, compute the maximum precision it can get.
    // Typically, this is 0 or 1 for nondegenerate boolean tables.
    // Also store the sumo and total, so that they don't need to be
    // recomputed later.  Note that this routine could be performance
    // critical if used as a fitness function for feature selection
    // (which is the case).
    typedef std::multimap<contin_t,           // precision
                          std::pair<contin_t, // sum_outputs
                                    count_t>  // total count
                          > max_precisions_t;
    max_precisions_t max_precisions;
    for (CTable::const_iterator it = _wrk_ctable.begin();
         it != _wrk_ctable.end(); ++it)
    {
        const CTable::counter_t& c = it->second;
        count_t sumo = sum_outputs(c);
        count_t total = c.total_count();
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
    count_t active = 0.0;
    score_t sao = 0.0;
    score_t best_sc = very_worst_score;
    score_t best_precision = 0.0;
    score_t best_activation = 0.0;
    score_t best_activation_penalty = 0.0;

    reverse_foreach (const auto& mpv, max_precisions) {
        sao += mpv.second.first;
        active += mpv.second.second;

        // By using (tp-fp)/2 the sum of all the per-row contributions
        // is offset by -1/2 from the precision, as proved below
        //
        // 1/2 * (tp - fp) / (tp + fp)
        // = 1/2 * (tp - tp + tp - fp) / (tp + fp)
        // = 1/2 * ((tp + tp) / (tp + fp) - (tp + fp) / (tp + fp))
        // = 1/2 * (2*tp / (tp + fp) - 1)
        // = precision - 1/2
        //
        // So before adding the recall penalty we add 0.5 to
        // compensate for that
        score_t precision = (sao / active) + 0.5;
        score_t activation = active / _ctable_weight;
        score_t activation_penalty = get_activation_penalty(activation);
        score_t sc = precision + activation_penalty;

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
        if (_ctable_weight * min_activation <= active)
            break;
    }

    logger().debug("Precision scorer, best score = %f", best_sc);
    logger().debug("precision at best score = %f", best_precision);
    logger().debug("activation at best score = %f", best_activation);
    logger().debug("activation penalty at best score = %f", best_activation_penalty);

    // @todo it's not really the best bscore but rather the best score
    // That's because there are potentially many bscores that are the
    // best possible.
    return {best_sc};
}

// The worst possible score is to fail to identify any of the positive
// rows. This renders TP = 0.  Even if the table is compressed and 
// bizarrely weighted, we cannot do worse than that.
behavioral_score precision_bscore::worst_possible_bscore() const
{
    return {0.0};
}

// Note that the logarithm is always negative, so this method always
// returns a value that is zero or negative.  Note that it returns
// -inf if activation is 0.0 -- but that's OK, it won't hurt anything.
score_t precision_bscore::get_activation_penalty(score_t activation) const
{
    score_t dst = 0.0;
    if (activation < min_activation)
        dst = 1.0 - activation / min_activation;

    if (max_activation < activation)
        dst = (activation - max_activation) / (1.0 - max_activation);

    // logger().fine("activation penalty = %f", dst);
    return activation_pressure * log(1.0 - dst);
}

score_t precision_bscore::min_improv() const
{
    return 1.0 / _ctable_weight;
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
                                    count_t> // total count
                          > precision_to_count_t;
    precision_to_count_t ptc;
    for (CTable::const_iterator it = _wrk_ctable.begin();
         it != _wrk_ctable.end(); ++it)
    {
        const CTable::counter_t& c = it->second;
        count_t total = c.total_count();
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
    count_t active = 0.0;
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
        if (_ctable_weight * min_activation <= active)
            break;
    }
    return tr;
}

void precision_bscore::reset_weights()
{
    wnorm = 1.0;
    if (_return_weighted_score)
        _weights = std::vector<double>(_size, 1.0);
    else
        _weights = std::vector<double>();
}

void precision_bscore::update_weights(const std::vector<double>& rew)
{
    OC_ASSERT(_return_weighted_score,
        "Unexpected use of weights in the bscorer!");

    OC_ASSERT(rew.size() == _size,
        "Unexpected size of weight array!");

    double znorm = 0.0;
    for (size_t i = 0; i < _size; i++) {
        // Knock down the weights of the selected rows. The goal here
        // is to more-or-less keep them knocked down. Yes, due to
        // renorm (below), they'll slowly drift back up; I guess that's
        // OK. But whatever we do, we don't want to let later updates
        // push them up.
        _weights[i] *= wnorm;
        if (rew[i] < 1.0) _weights[i] *= rew[i];
        znorm += _weights[i];
    }

    // Normalization: sum of weights must equal _size
    // This is not really correct, but it keeps us in the general
    // area of having each row have an average weight of 1.0.
    znorm = ((double) _size) / znorm;
    wnorm = 1.0 / znorm;
    for (size_t i=0; i<_size; i++) _weights[i] *= znorm;
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
    _complexity_coef = 0.0;
    // Both p==0.0 and p==0.5 are singularity points in the Occam's
    // razor formula for discrete outputs (see the explanation in the
    // comment above ctruth_table_bscore)
    if (p > 0.0f and p < 0.5f)
        _complexity_coef = discrete_complexity_coef(alphabet_size, p)
            / ctable_usize;     // normalized by the size of the table
                                // because the precision is normalized
                                // as well

    logger().info() << "Precision scorer, noise = " << p
                    << " alphabest size = " << alphabet_size
                    << " complexity ratio = " << 1.0/_complexity_coef;
}

void precision_conj_bscore::set_complexity_coef(score_t ratio)
{
    _complexity_coef = 0.0;
    if (ratio > 0)
        _complexity_coef = 1.0 / ratio;

    logger().info() << "Precision scorer, complexity ratio = " << 1.0f/_complexity_coef;
}

behavioral_score precision_conj_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;

    // compute active and sum of all active outputs
    count_t active = 0.0;  // total weight of active outputs by tr
    score_t sao = 0.0;     // sum of all active outputs (in the boolean case)
    interpreter_visitor iv(tr);
    auto interpret_tr = boost::apply_visitor(iv);
    for (const CTable::value_type& vct : ctable) {
        // vct.first = input vector
        // vct.second = counter of outputs
        if (interpret_tr(vct.first.get_variant()) == id::logical_true) {
            double sumo = sum_outputs(vct.second);
            count_t totalc = vct.second.total_count();
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
    bs.push_back(precision);

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
    bs.push_back(conj_n_penalty);

    if (logger().isFineEnabled())
        logger().fine("precision = %f  conj_n=%u  conj_n penalty=%e",
                     precision, conj_n, conj_n_penalty);

    log_candidate_bscore(tr, bs);
    return bs;
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

} // ~namespace moses
} // ~namespace opencog
