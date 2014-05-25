/*
 * opencog/learning/moses/scoring/precision_bscore.h
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
#ifndef _PRECISION_BSCORE_H
#define _PRECISION_BSCORE_H

#include "scoring_base.h"

namespace opencog { namespace moses {

/**
 * Fitness function for maximizing binary precision, that is, for
 * minimizing false positives.   This scorer works for both boolean
 * regression and contin regression.  For contin regression, the
 * learned combo program still returns a boolean T/F, which is then
 * used to sum the values of the contin output value.  That is, if
 * the combo returns T for a given row, then we add the contin value
 * in that row.  If the combo returns false for that row, we do nothing.
 *
 * For contin tables, if 'positive' is false, then the negative of
 * the contin value is used for all calculations.
 *
 * http://en.wikipedia.org/wiki/Accuracy_and_precision#In_binary_classification
 *
 * This bscore has 2 components:
 *
 * 1) The precision.  That is, the number of true positives divided
 *    by the sum of true and false positives.  If the paramter 'positive'
 *    is set to false, then the 'negative predictive value' is computed
 *    (i.e. maximing the true negatives, minimizing false negatives).
 *
 * 2) a penalty ensuring that at least some true positives are found,
 *    as otherwise one can get perfect precision by always answering
 *    false (and thus there would be zero false poitives).  The penalty
 *    is explained below.
 *
 * The scorer counts the total number of rows for which the combo program
 * returned 'true'. This is defined as the 'activation'.  For boolean tables,
 * the activation is just the number of true positives plus the number of
 * false positives.
 *
 * Note that an activation of zero corresponds to perfect precision:
 * there were no false positives.  Thus, to get reasonable results,
 * one wants to learn a function that predicts at least a few true
 * postives, i.e. has an activation greater than zero.  This is done
 * by applying a penalty when the activation is outside of the range
 * of an interval [min_activation, max_activation].  See the
 * description of discriminating_bscore::get_threshold_penalty() for
 * details.
 *
 * If worst_norm is true, then the percision is divided by the absolute
 * average of the negative lower (resp. positive upper if
 * this->positive is false) decile or less. If there is no negative
 * (resp. positive if this->positive is false) values then it is not
 * normalized. (??)
 *
 * If substract_neg_target is true then the negation of the target (in
 * the boolean case) count for -1/2 instead of 0 and 1/2 instead of
 * 1. In other words the fitness to maximize is:
 *
 * 1/2 * (tp - fp) / (tp + fp)
 *
 * where tp and fp stand for true positive and false positive
 * respectively. This is actually equivalent to
 *
 * 1/2 * (tp - tp + tp - fp) / (tp + fp)
 * = 1/2 * (tp + tp) / (tp + fp) - (tp + fp) / (tp + fp)
 * = 1/2 * 2*tp / (tp + fp) - 1
 * = precision - 1/2
 *
 * One might ask then why use that fitness function instead of
 * precision as they are equivalent up to an additive constant. The
 * reason is because the bscore will look differently, in such case
 * when an element of the bscore is 0 it will likely correspond to no
 * activity. Active data points contributing to a precision above half
 * will have positive values, active data points contributing to a
 * precision below half will have negative values, active data points
 * contributing to precision of exactly 0.5 will have null values
 * (like inactive points, but it's rather unlikely).
 *
 * XXX This class should be reworked to derive from
 * discriminating_bscore.  This would allow us to get rid of duplicate
 * code for sec_complexity_coef() and for get_activation_penalty() and
 * min_improv() and sum_outputs(). TODO In fact everything could be
 * replaced, if it were not for the worst_deciles stuff.
 */
struct precision_bscore : public bscore_base
{
    precision_bscore(const CTable& _ctable,
                     float penalty = 1.0f,
                     float min_activation = 0.5f,
                     float max_activation = 1.0f,
                     bool positive = true,
                     bool worst_norm = false);

    behavioral_score operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    /**
     * Filter the table with all permitted idxs (the complementary
     * with [0..arity).
     */
    void ignore_idxs(const std::set<arity_t>&) const;

    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float stddev);

    /**
     * This is a experimental feature, we generate a massive combo
     * tree that is supposed to maximize the precision (keeping
     * activation within acceptable boundaries). The plan is that
     * because that combo tree is possibly (even certainly) overfit we
     * evolve it to prune from it what contributes the least (via
     * setting an adequate complexity penalty).
     *
     * The combo tree is a boolean formula, more specifically a
     * disjunctive normal form, representing a decision tree
     * maximizing the precision.
     *
     * Each active row corresponds to a conjunctive clauses where each
     * literal is in a input value (positive if the input is
     * id::logical_true, negative if the input is id::logical_false).
     *
     * The active rows are determined identically as in
     * precision_bscore::best_possible_bscore(). That is the rows are
     * sorted according to their individual precision, the first ones
     * till the min activation is reached are active.
     */
    combo_tree gen_canonical_best_candidate() const;

protected:
    const CTable& orig_ctable;  // ref to the original table

    // table actually used for the evaluation. It is mutable because
    // we want to be able to change it to ignore some features (it may
    // speed-up evaluation)
    mutable CTable wrk_ctable;

    // for debugging, keep that around till we fix best_possible_bscore
    // mutable CTable fully_filtered_ctable;

    size_t ctable_usize;   // uncompressed size of ctable
    score_t min_activation, max_activation;
    score_t max_output; // max output one gets (1 in case it is
                        // boolean). This is used to normalized the
                        // precision in case the output isn't boolean.
    score_t penalty;
    bool positive, worst_norm;

    // if enabled then each datapoint is an entry in the bscore (its
    // part contributing to the precision, and the activation penalty
    // is the last one).
    // WARNING: worst_score isn't supported then
    bool precision_full_bscore;

    type_node output_type;

private:
    score_t get_activation_penalty(score_t activation) const;

    // function to calculate the total weight of the observations
    // associated to an input vector
    std::function<score_t(const CTable::counter_t&)> sum_outputs;
};

/**
 * A bit like precision but tries to maximize the number conjunctions
 * after precision. It blatantly assumes that the domain is boolean.
 */
struct precision_conj_bscore : public bscore_base
{
    precision_conj_bscore(const CTable& _ctable, float hardness,
                          bool positive = true);

    behavioral_score operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float stddev);

protected:
    const CTable& ctable;

    size_t ctable_usize;   // uncompressed size of ctable
    float hardness;
    bool positive;

private:
    // function to calculate the total weight of the observations
    // associated to an input vector
    std::function<score_t(const CTable::counter_t&)> sum_outputs;
};

} // ~namespace moses
} // ~namespace opencog

#endif
