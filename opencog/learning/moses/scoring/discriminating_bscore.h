/*
 * opencog/learning/moses/scoring/discriminating_bscore.h
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
#ifndef _DISCRIMINATING_BSCORE_H
#define _DISCRIMINATING_BSCORE_H

#include "scoring_base.h"

namespace opencog { namespace moses {

/// Perform summations commonly used in binary discriminators.
/// These may be used to obtain precision, recall, specificty,
/// sensitivity, fall-out, F-score, etc.
///
/// The count() method performs the actual counting.  It zeroes out
/// the eight sums/counts below, before starting.  Once it returns,
/// the eight sums/counts are all valid for the peviously given combo
/// tree.
///
/// For boolean problems, the *_sum and *_count values are identical.
/// For contin problems, they are not.
/// Note that ctable_usize == number of rows in the uncompressed table,
/// it is equal to:  true_positive_count + false_positive_count +
/// true_negative_count + false_negative_count.
struct discriminator
{
    discriminator(const CTable&);

    struct d_counts {
        d_counts();
        score_t true_positive_sum;
        score_t false_positive_sum;
        score_t positive_count;

        score_t true_negative_sum;
        score_t false_negative_sum;
        score_t negative_count;
    };
    d_counts count(const combo_tree&) const;

    // Like count but do the counting for each datapoint (each row of
    // the ctable). This is useful for finer grain bscore (see
    // variable discriminator_bscore::full_bscore)
    std::vector<d_counts> counts(const combo_tree&) const;

protected:
    CTable _ctable;
    type_node _output_type;
    score_t _true_total;        // total number of Ts in the ctable
    score_t _false_total;       // total number of Fs in the ctable

    // Regarding the 2 functions below (sum_pos and sum_neg):
    //
    // When the output type is contin then the notion of
    // positive/negative is fuzzy, with degree corresponding to the
    // weight of the contin value.

    // give a ctable's row return the sum of true positives
    std::function<score_t(const CTable::counter_t&)> sum_true;
    // give a ctable's row return the sum of false positives
    std::function<score_t(const CTable::counter_t&)> sum_false;
};

/**
 * discriminating_bscore -- Base class for precision, recall,
 * senstivity, specificty, F-score, etc. type discriminator scorers.
 * Provides all the generic, common functions such scorer might need.
 */
struct discriminating_bscore : public bscore_base, discriminator
{
    discriminating_bscore(const CTable& _ctable,
                  float min_threshold = 0.5f,
                  float max_threshold = 1.0,
                  float hardness = 1.0f);

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    virtual behavioral_score best_possible_bscore() const;
    virtual score_t min_improv() const;

    /// Over-ride the default complexity setters, since our scoring is
    /// normalized to 1.0, unlike other scorers.
    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float stddev);

protected:
    //* The two functions below are used to implement a generic
    //* best_possible_bscore() method.  They should return values
    //* the two conjugate classification dimensions.  For example,
    //* one might return precision, and the other recall.  The 'fixed'
    //* quantity is the one meant to be kept above a given threshold,
    //* while the 'variable' one is to be maximized (while keeping the
    //* other above the threshold).
    virtual score_t get_fixed(score_t pos, score_t neg, unsigned cnt) const = 0;
    virtual score_t get_variable(score_t pos, score_t neg, unsigned cnt) const = 0;
    /**
     * This is a base class for scorers that try to maximize one quantity
     * while holding another constant; or rather, attempting to hold 
     * another constant.  This may be done by applying a penalty when
     * the held quantity wanders out of bounds.  This methods computes
     * such a penalty, based on the min and max thresholds provided
     * in the constructor, as well as the "hardness" with which the
     * penalty should be applied.  The penatly takes the form:
     *
     *    hardness * log(1 - dst(value, [min_threshold, max_threshold])) 
     *
     * where dst(x, I) is defined as being zero on the interval I and ramping
     * up to one if x lies outside the interval I.  The penalty
     * approaches -infty as the value approaches 0.0 or 1.0. Values less
     * than zero or greater than one are invalid.
     *
     *                            {  1 - x/x_min         if x < x_min 
     *    dst(x, [x_min,x_max]) = {    0                 if x_min < x < x_max
     *                            { (x-x_max)/(1-x_max)  if x_max < x
     *
     * Note that the logarithm is negative, and thus the total score derating
     * is negative.
     */
    score_t get_threshold_penalty(score_t) const;
    size_t _ctable_usize;
    score_t _max_output;
    score_t _min_output;
    float _min_threshold;
    float _max_threshold;
    float _hardness;

    // if enabled then each datapoint is an entry in the bscore
    // corresponding to its contribution to the variable score
    // component, and the last element of the bscore correspong to the
    // fix score component (like when disabled).
    //
    // All the datapoint contributions should sum up to the overall
    // variable score
    bool _full_bscore;
};

/**
 * recall_bscore -- scorer that attempts to maximize recall, while
 * holding precision at or above a minimum acceptable value.
 */
struct recall_bscore : public discriminating_bscore
{
    recall_bscore(const CTable& _ctable,
                  float min_precision = 0.8f,
                  float max_precision = 1.0f,
                  float hardness = 1.0f);

    behavioral_score operator()(const combo_tree& tr) const;

protected:
    virtual score_t get_fixed(score_t pos, score_t neg, unsigned cnt) const;
    virtual score_t get_variable(score_t pos, score_t neg, unsigned cnt) const;
};

/**
 * prerec_bscore -- scorer that attempts to maximize precision, while
 * holding recall at or above a minimum acceptable value.
 */
struct prerec_bscore : public discriminating_bscore
{
    prerec_bscore(const CTable& _ctable,
                  float min_recall = 0.5f,
                  float max_recall = 1.0f,
                  float hardness = 1.0f);

    behavioral_score operator()(const combo_tree& tr) const;

protected:
    virtual score_t get_fixed(score_t pos, score_t neg, unsigned cnt) const;
    virtual score_t get_variable(score_t pos, score_t neg, unsigned cnt) const;
};

/**
 * bep_bscore -- scorer that attempts to maximize "BEP", the 
 * "precision-recall break-even point"; i.e. the arithmetic average
 * of the precision and recall.  To avoid solutions that optimize
 * for very high precision or very high recall, but not both, the
 * scorer attempts to keep the absolute value between precision and
 * recall less than the given bound.
 */
struct bep_bscore : public discriminating_bscore
{
    bep_bscore(const CTable& _ctable,
               float min_diff = 0.0f,
               float max_diff = 0.5f,
               float hardness = 1.0f);

    behavioral_score operator()(const combo_tree& tr) const;

protected:
    virtual score_t get_fixed(score_t pos, score_t neg, unsigned cnt) const;
    virtual score_t get_variable(score_t pos, score_t neg, unsigned cnt) const;
};

/**
 * f_one_bscore -- scorer that attempts to maximize the F_1 score,
 * harmonic mean of the precision and recall scores.
 */
struct f_one_bscore : public discriminating_bscore
{
    f_one_bscore(const CTable& _ctable);
    behavioral_score operator()(const combo_tree& tr) const;

protected:
    virtual score_t get_fixed(score_t pos, score_t neg, unsigned cnt) const;
    virtual score_t get_variable(score_t pos, score_t neg, unsigned cnt) const;
};

} // ~namespace moses
} // ~namespace opencog

#endif
