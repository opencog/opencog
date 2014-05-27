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

#include "precision_bscore.h"

#include <boost/range/irange.hpp>

#include <opencog/comboreduct/table/table_io.h>

namespace opencog { namespace moses {

using namespace std;

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

void precision_bscore::set_complexity_coef(score_t ratio)
{
    _complexity_coef = 0.0;
    if (ratio > 0)
        _complexity_coef = 1.0 / ratio;

    logger().info() << "Precision scorer, complexity ratio = " << 1.0f/_complexity_coef;
}

behavioral_score precision_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;

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
            bs.push_back(sumo);
        }

        if (active > 0) {
            // normalize all components by active
            score_t iac = 1.0 / active; // inverse of activity to be faster
            for (auto& v : bs) v *= iac;

            // tp = true positive
            // fp = false positive
            //
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
            bs.push_back(0.5);
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

        bs.push_back(precision);
    }

    if (0 < active)
        precision = (sao / active + 0.5) / max_output;

    // For boolean tables, activation sum of true and false positives
    // i.e. the sum of all positives.   For contin tables, the activation
    // is likewise: the number of rows for which the combo tree returned
    // true (positive).
    score_t activation = (score_t)active / ctable_usize;
    score_t activation_penalty = get_activation_penalty(activation);
    bs.push_back(activation_penalty);
    if (logger().isFineEnabled())
        logger().fine("precision = %f  activation=%f  activation penalty=%e",
                      precision, activation, activation_penalty);

    log_candidate_bscore(tr, bs);
    return bs;
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
