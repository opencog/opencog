/*
 * opencog/learning/moses/scoring/scoring.h
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
#ifndef _MOSES_SCORING_H
#define _MOSES_SCORING_H

#include <iostream>
#include <fstream>
#include <functional>

#include <boost/range/numeric.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/weighted_skewness.hpp>

#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/functional.h>
#include <opencog/util/KLD.h>

#include <opencog/comboreduct/table/table.h>

#include "../moses/using.h"
#include "../moses/types.h"
#include "../representation/representation.h"

namespace opencog { namespace moses {

// Abstract scoring function class to implement
struct cscore_base : public unary_function<combo_tree, composite_score>
{
    // Evaluate the candidate tr
    virtual composite_score operator()(const combo_tree& tr) const = 0;

    // Return the best possible score achievable with that fitness
    // function. This is useful in order to stop running MOSES when
    // the best possible score is reached. If not overloaded it will
    // return best_score (constant defined under
    // opencog/learning/moses/moses/types.h)
    virtual score_t best_possible_score() const { return very_best_score; }

    // Return the minimum value considered for improvement (by
    // default return 0)
    virtual score_t min_improv() const { return 0.0; }

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0). The method provided by default does nothing
    // (no speed-up).
    //
    // There is also another case. When a new deme is spawn if some
    // features are ignored then the best_possible_score might be
    // lower, it's good to compute it in order to stop deme search. If
    // ignore_idxs is set then best_possible_score() can be recalled
    // to get thta new ma score value.
    virtual void ignore_idxs(const set<arity_t>&) const {}

    virtual ~cscore_base(){}
};

// Abstract bscoring function class to implement
struct bscore_base : public unary_function<combo_tree, penalized_bscore>
{
    bscore_base() : occam(false), complexity_coef(0.0) {};
    virtual ~bscore_base() {};

    // Evaluate the candidate tr
    virtual penalized_bscore operator()(const combo_tree& tr) const = 0;

    // Return the best possible bscore achievable with that fitness
    // function. This is useful in order to stop running MOSES when
    // the best possible score is reached
    virtual behavioral_score best_possible_bscore() const = 0;

    // Return the minimum value considered for improvement (by defaut
    // return 0)
    virtual score_t min_improv() const { return 0.0; }

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0). The method provided by default does nothing
    // (no speed-up).
    virtual void ignore_idxs(const set<arity_t>&) const {}

    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float p);

protected:
    bool occam; // If true, then Occam's razor is taken into account.
    score_t complexity_coef;
};

/**
 * Composite score calculated from the behavioral score.
 *
 * The score is calculated as the sum of the bscore over all features:
 *      score = sum_f BScore(f) + penalty
 *
 * This is a "minor" helper class, and exists for two reasons:
 * 1) avoids some redundancy of having the summation in many places
 * 2) Helps with keeping the score-caching code cleaner.
 *
 * TODO: could be detemplatized, it's only instantiated with
 * bscore_base.
 */
template<typename PBScorer>
struct bscore_based_cscore : public cscore_base
{
    bscore_based_cscore(const PBScorer& sr) : _pbscorer(sr) {}

    composite_score operator()(const combo_tree& tr) const
    {
        try {
            penalized_bscore pbs = _pbscorer(tr);
            return operator()(pbs, tree_complexity(tr));
        }
        catch (EvalException& ee)
        {
            // Exceptions are raised when operands are out of their
            // valid domain (negative input log or division by zero),
            // or outputs a value which is not representable (too
            // large exp or log). The error is logged as level fine
            // because its happens very often when learning continuous
            // functions, and it gets too much in the way if logged at
            // a lower level.
            logger().fine()
               << "The following candidate: " << tr << "\n"
               << "has failed to be evaluated, "
               << "raising the following exception: "
               << ee.get_message() << " " << ee.get_vertex();

            return worst_composite_score;
        }
    }

    // Hmmm, this could be static, actually ... 
    composite_score operator()(const penalized_bscore& pbs,
                               complexity_t cpxy) const
    {
        const behavioral_score &bs = pbs.first;
        score_t res = boost::accumulate(bs, 0.0);

        if (logger().isFineEnabled()) {
            logger().fine() << "bscore_based_cscore: " << res
                            << " complexity: " << cpxy
                            << " complexity penalty: " << pbs.second;
        }

        return composite_score(res, cpxy, pbs.second, 0.0);
    }


    // Returns the best score reachable for that problem. Used as
    // termination condition.
    score_t best_possible_score() const
    {
        return boost::accumulate(_pbscorer.best_possible_bscore(), 0.0);
    }

    // Return the minimum value considered for improvement
    score_t min_improv() const
    {
        return _pbscorer.min_improv();
    }

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0).
    void ignore_idxs(const set<arity_t>& idxs) const
    {
        _pbscorer.ignore_idxs(idxs);
    }

    const PBScorer& _pbscorer;
};

/**
 * Behavioral scorer defined by multiple behavioral scoring functions.
 * This is done when the problem to solve is defined in terms of multiple
 * problems. For now the multiple scores have the same type as defined
 * by the template argument BScorer.
 */
struct multibscore_based_bscore : public bscore_base
{
    typedef boost::ptr_vector<bscore_base> BScorerSeq;
    
    // ctors
    multibscore_based_bscore(const BScorerSeq& bscorers) : _bscorers(bscorers) {}

    // main operator
    penalized_bscore operator()(const combo_tree& tr) const;

    behavioral_score best_possible_bscore() const;

    // return the min of all min_improv
    score_t min_improv() const;

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0).
    void ignore_idxs(const set<arity_t>&) const;

protected:
    const BScorerSeq& _bscorers;
};

/**
 * Each feature corresponds to an input tuple, 0 if the output of the
 * candidate matches the output of the intended function, -1
 * otherwise.
 */
struct logical_bscore : public bscore_base
{
    template<typename Func>
    logical_bscore(const Func& func, int a)
            : target(func, a), arity(a) {}
    logical_bscore(const combo_tree& tr, int a)
            : target(tr, a), arity(a) {}

    penalized_bscore operator()(const combo_tree& tr) const;

    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

protected:
    complete_truth_table target;
    int arity;
};

/// Used to define the complexity scoring component given that p is the
/// probability of having an observation being wrong (see the comment
/// regarding ctruth_table_bscore for more information).
///
/// Note: this returns NEGATIVE values.
score_t discrete_complexity_coef(unsigned alphabet_size, double p);

/// Used to define the complexity scoring component given that stdev is
/// the standard deviation of the noise of the we're trying to predict
/// output (see the comment regarding contin_bscore for more information).
///
/// Note: this returns NEGATIVE values.
score_t contin_complexity_coef(unsigned alphabet_size, double stdev);

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

    penalized_bscore operator()(const combo_tree& tr) const;

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

    penalized_bscore operator()(const combo_tree& tr) const;

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

    penalized_bscore operator()(const combo_tree& tr) const;

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
    penalized_bscore operator()(const combo_tree& tr) const;

protected:
    virtual score_t get_fixed(score_t pos, score_t neg, unsigned cnt) const;
    virtual score_t get_variable(score_t pos, score_t neg, unsigned cnt) const;
};

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

    penalized_bscore operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    /**
     * Filter the table with all permitted idxs (the complementary
     * with [0..arity).
     */
    void ignore_idxs(const set<arity_t>&) const;

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

    penalized_bscore operator()(const combo_tree& tr) const;

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
        
/**
 * Fitness function based on discretization of the output. If the
 * classes match the bscore element is 0, or -1 otherwise. If
 * @weighted_average is true then each element of the bscore is
 * weighted so that each class overall has the same weight in the
 * scoring function.
 *
 * The Occam's razor function is identical to ctruth_table_bscore
 */
struct discretize_contin_bscore : public bscore_base
{
    discretize_contin_bscore(const OTable& ot, const ITable& it,
                             const vector<contin_t>& thres,
                             bool weighted_average);

    // @todo when switching to gcc 4.6 use constructor delagation to
    // simplify that
    // discretize_contin_bscore(const Table& table,
    //                          const vector<contin_t>& thres,
    //                          bool weighted_average,
    //                          float alphabet_size, float p);

    penalized_bscore operator()(const combo_tree& tr) const;

    // The best possible bscore is a vector of zeros. That's probably
    // not quite true, because there could be duplicated inputs, but
    // that's acceptable for now.
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

protected:
    OTable target;
    ITable cit;
    vector<contin_t> thresholds;
    bool weighted_accuracy;     // Whether the bscore is weighted to
                                // deal with unbalanced data.

    // Return the index of the class of value v.
    size_t class_idx(contin_t v) const;
    // Like class_idx but assume that the value v is within the class
    // [l_idx, u_idx)
    size_t class_idx_within(contin_t v, size_t l_idx, size_t u_idx) const;

    vector<size_t> classes;       // classes of the output, alligned with target

    // Weight of each class, so that each one weighs as much as the
    // others, even in case of unbalance sampling. Specifically:
    // weights[i] = s / (n * c_i) where s is the sample size, n the
    // number of classes and c_i the number of samples for class i.
    vector<score_t> weights;
};

/**
 * Behavioral scoring function minimizing residual errors.
 *
 * The first elements of the bscore correspond to the minus squared
 * errors. The last element is optional and corresponds to a program
 * size penalty.
 *
 * The math justifying the program size penalty equations is based on
 * the following thread
 * http://groups.google.com/group/opencog-news/browse_thread/thread/b7704419e082c6f1
 *
 * It turns out this is closely related to the Akaike Information
 * Criterion, see in particular
 * http://en.wikipedia.org/wiki/Akaike_information_criterion#Relevance_to_chi-squared_fitting
 *
 * Here's a summary:
 * Let M == model (the combo program being learned)
 * Let D == data (the table of values being modelled)
 * Let P(..) == probability
 * Let dP(..) == probability density
 *
 * According to Bayes
 *
 *    dP(M|D) = dP(D|M) * P(M) / P(D)
 *
 * Now let's consider the log likelihood of M knowing D.  Since D is
 * constant we can ignore P(D), so:
 *
 *    LL(M) = log(dP(D|M)) + log(P(M))
 *
 * Assume the output of M on input x has a Guassian noise of mean M(x)
 * and variance v, so dP(D|M) (the density probability)
 *
 *   dP(D|M) = Prod_{x\in D} (2*Pi*v)^(-1/2) exp(-(M(x)-D(x))^2/(2*v))
 *
 * Assume
 *    P(M) = |A|^-|M|
 * where |A| is the alphabet size.
 *
 * After simplification we can get the following log-likelihood of dP(M|D)
 *    -|M|*log(|A|)*2*v - Sum_{x\in D} (M(x)-D(x))^2
 *
 * Each datum corresponds to a feature of the bscore.
 *
 *    |M|*log(|A|)*2*v corresponds to an additional feature when v > 0
 */
struct contin_bscore : public bscore_base
{
    enum err_function_type {
        squared_error,
        abs_error
    };

    void init(err_function_type eft = squared_error)
    {
        switch (eft) {
        case squared_error:
            err_func = [](contin_t y1, contin_t y2) { return sq(y1 - y2); };
            break;
        case abs_error:
            err_func = [](contin_t y1, contin_t y2) { return std::abs(y1 - y2); };
            break;
        default:
            OC_ASSERT(false);
        }
    };

    template<typename Scoring>
    contin_bscore(const Scoring& score, const ITable& r,
                  err_function_type eft = squared_error)
        : target(score, r), cti(r)
    {
        init(eft);
    }

    contin_bscore(const OTable& t, const ITable& r,
                  err_function_type eft = squared_error)
        : target(t), cti(r)
    {
        init(eft);
    }

    contin_bscore(const Table& table,
                  err_function_type eft = squared_error)
        : target(table.otable), cti(table.itable) {
        init(eft);
    }

    penalized_bscore operator()(const combo_tree& tr) const;

    // The best possible bscore is a vector of zeros. That's probably
    // not quite true, because there could be duplicated inputs, but
    // that's acceptable for now.
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    virtual void set_complexity_coef(unsigned alphabet_size, float stddev);
    using bscore_base::set_complexity_coef; // Avoid hiding/shadowing

protected:
    OTable target;
    ITable cti;

private:
    // for a given data point calculate the error of the target
    // compared to the candidate output
    std::function<score_t(contin_t, contin_t)> err_func;
};

/**
 * ctruth_table_bscore -- compute behavioral score for boolean truth tables.
 *
 * The CTable ctt holds the "compressed" data table, consisting of
 * rows of input (independent) variables, and a single output
 * (dependent) variable. Scoring is performed by evaluating the
 * combo tree for each input row, and comparing the evaluation results
 * to the output column.
 *
 * The first elements correspond to the minus absolute errors (0 if
 * the booleans fit, -1 if they don't). The last element is optional
 * and corresponds to a program size penalty.
 *
 * Regarding the program size penalty, instead of considering a
 * standard deviation of the output, the probability p that one datum
 * is wrong is used.  Note that we expect p to be small; certainly
 * much less than 1/2 (as p=1/2 means half our data is wrong!)
 *
 * To summarize, the end result is the log-likelihood LL(M) that the
 * model M is correct:
 *
 *     LL(M) = -|M|*log(|A|) + |D_ne| log(p/(1-p)) + |D| log(1-p)
 *
 * which may be re-interpreted as a score:
 *
 *     score(M) = - [ LL(M) - |D| log(1-p) ] / log(p/(1-p))
 *              = -|D_ne| + |M|*log|A| / log(p/(1-p))
 *              = -|D_ne| - |M| |C_coef|
 *
 * where |D_ne| is the number of outputs that are incorrect, and |M| is
 * the complexity of the model, and |A| is the alphabets size employed
 * in the model.
 *
 * The coeffcient |C_coef| can be fixed in several different ways, and
 * not just relying on estimates of p and |A|. It is set by calling
 * bscore_base::set_complexity_coeff()
 *
 * -------------------------------------------------------------------
 * To summarize, if any of the entries in the data table are incorrect,
 * with some probability p, this is equivalent to raising the score by
 * an amount proportional to the model complexity.
 * -------------------------------------------------------------------
 *
 * The details were originally reported in this thread,
 * http://groups.google.com/group/opencog/browse_thread/thread/a4771ecf63d38df
 * and are restated, cleaned up, below:
 *
 * M is the model (the combo program)
 * D is the data, a table of n inputs i_k and one output o
 *     i_1 ... i_n o
 *
 * where i_k is the k'th input and o the output.  We want to assess
 * P(M|D) and, in particular, maximize it, as it is the fitness function.
 * According to Bayes
 *
 *     P(M|D) = P(D|M) * P(M) / P(D)
 *
 * Consider the log likelihood of M knowing D.  Since D is constant,
 * we can ignore P(D), so:
 *
 *     LL(M) = log(P(D|M)) + log(P(M))
 *
 * Assume each output of M on input x has probability p to be wrong.  So,
 *
 *     P(D|M) = Prod_{x\in D} [p*(M(x) != D(x)) + (1-p)*(M(x) == D(x))]
 *
 * where D(x) the observed result given input x.  Then,
 *
 *     log P(D|M) = Sum_{x\in D} log[p*(M(x) != D(x)) + (1-p)*(M(x) == D(x))]
 *
 * Let D = D_eq \cup D_ne  where D_eq and D_ne are the sets
 *
 *     D_eq = {x \in D | M(x) == D(x) }
 *     D_ne = {x \in D | M(x) != D(x) }
 *
 * Then
 *
 *     log P(D|M) = Sum_{x\in D_ne} log(p) + Sum_{x\in D_eq} log(1-p)
 *                = |D_ne| log(p) + |D_eq| log(1-p)
 *                = |D_ne| log(p) + |D| log(1-p) - |D_ne| log(1-p)
 *                = |D_ne| log(p/(1-p)) + |D| log(1-p)
 *
 * Here, |D| is simply the size of set D, etc.  Assuming that p is
 * small, i.e. much less than one, then, to second order in p:
 *
 *    log(1-p) = -p + p^2/2 + O(p^3)
 *
 * So:
 *
 *    log P(D|M) = |D_ne| log(p) - p (|D| - |D_ne|) + O(p^2)
 *
 * Next, assume P(M) is distributed according to Solomonoff's Universal
 * Distribution, approximated by (for now)
 *
 *     P(M) = |A|^-|M|
 *          = exp(-|M|*log(|A|))
 *
 * where A is the alphabet of the model, and |M| is the complexity of
 * the model.  Putting it all together, the log-likelihood of M is:
 *
 *     LL(M) = -|M|*log(|A|) + |D_ne| log(p/(1-p)) + |D| log(1-p)
 *
 * To get an expression usable for the score, just bring out the |D_ne|
 * by dividing by -log(p/(1-p)), to get
 *
 *     score(M) = - [ LL(M) - |D| log(1-p) ] / log(p/(1-p))
 *              = -|D_ne| + |M|*log|A| / log(p/(1-p))
 *              = -|D_ne| - |M| |C_coef|
 *
 * Note that, since p<1, that log(p) is negative, and so the second
 * term is negative.  It can be understood as a "complexity penalty".
 */
struct ctruth_table_bscore : public bscore_base
{
    template<typename Func>
    ctruth_table_bscore(const Func& func,
                        arity_t arity,
                        int nsamples = -1)
        : ctable(func, arity, nsamples)
    {}
    ctruth_table_bscore(const CTable& _ctt) : ctable(_ctt) {}

    penalized_bscore operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

protected:
    CTable ctable;
};

/**
 * Like ctruth_table_bscore, but for enums.
 * That is, the output column of the table is assumed to be enum-valued.
 * The only "tricky" thing here is to correctly count the number of
 * wrong answers in a compressed enum table.
 *
 * Do not use this scorer with "cond" conditionals returning enum.
 * It will work, but moses cannot learn efficiently with this scorer.
 * That is because the "straight" scorring it performs does not provide
 * any hints as to when a given predicate is correct or not.  Use the
 * enum_graded_bascore instead; see there for further explanation.
 *
 * The CTable ctt holds the "compressed" data table, consisting of
 * rows of input (independent) variables, and a single output
 * (dependent) variable. Scoring is performed by evaluating the
 * combo tree for each input row, and comparing the evaluation results
 * to the output column.
 *
 * The first elements correspond to the minus absolute errors (0 if
 * the enums match, -1 if they don't). The last element is optional
 * and corresponds to a program size penalty.
 *
 * Regarding the program size penalty, see ctruth_bscore, above.
 */
struct enum_table_bscore : public bscore_base
{
    enum_table_bscore(const CTable& _ctt) : ctable(_ctt) {}

    penalized_bscore operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    virtual score_t min_improv() const;

protected:
    CTable ctable;
};

/**
 * Like enum_table_bscore, but promotes accuracy of the first predicate.
 * This scorer assumes that the output column of the table is
 * enum-valued, and that the tree to be scored is a "cond" conditional.
 *
 * Recall: cond conditionals have the structure:
 *   cond(pred_1 val_1 pred_2 val_2 ... pred_n val_n else_val)
 *
 * The goal of this scorer is to find predicates that act like filters:
 * that is, the first pedicate of a condition statement must never be
 * inaccurate (must never mis-identify its consequent; must never issue
 * a false positive). The scorer accomplises this by adding a penalty
 * to the score whenever the first predicate is inaccurate.  The penalty
 * amount is user-adjustable.
 *
 * The enum_graded_bscore acheives the same effect as this scorer, but
 * employs a superior algorithm: it promotes the accuracy of all
 * predicates.  You probably want to use that one instead.  See the
 * explanation there for more details.
 */
struct enum_filter_bscore : public enum_table_bscore
{
    enum_filter_bscore(const CTable& _ctt)
        : enum_table_bscore(_ctt), punish(1.0)
    {}

    penalized_bscore operator()(const combo_tree& tr) const;

    score_t punish;
};

/**
 * Like enum_filter_bscore, but encourages accuracy of all predicates.
 * This scorer assumes that the output column of the table is
 * enum-valued, and that the tree to be scored is a "cond" conditional.
 *
 * Recall: cond conditionals have the structure:
 *   cond(pred_1 val_1 pred_2 val_2 ... pred_n val_n else_val)
 *
 * This scorer makes up for a deficiency with the enum_table_scorer
 * with regards learning cond's.  Consider, for example, two different
 * cond statements being evaluated on a single row.  Case A: pred_1
 * evaluates to true, but val_1 gives the wrong answer.  Case B:
 * pred_1 evaluates to false, thus 'feinting ignorance' of what to do,
 * or 'punting', to the rest of the cond chain to sort it out. And
 * suppose that the rest of the cond chain returns the wrong answer.
 * If we were to score these equally, then MOSES cannot tell them apart,
 * an exerts no selection pressure.  However, case B has a more accurate
 * pred_1, in that it is not making any false-positive identiications.
 *
 * This scorer will assign a higher score to case B than to case A,
 * thus encouraging a more accurate pred_1.  Likewise, whenever these
 * two cases arise for pred_2..pred_n, this scorer will also encourage
 * the more accurate case B.
 *
 * The enum_filter_bscore scorer also gave a higher score to case B, but
 * only for pred_1; it did not distinguish the two cases for the other
 * predicates in the chain.
 *
 * The value of 'grading' should lie between 0.0 and 1.0. Setting it to
 * 1.0 recovers the behaviour of enum_table_score, exactly.  Values
 * between 0.5 and 0.8 probably work best; but this needs experimental
 * measurement.
 */
struct enum_graded_bscore : public enum_table_bscore
{
    enum_graded_bscore(const CTable& _ctt)
        : enum_table_bscore(_ctt), grading(0.9)
    {}

    penalized_bscore operator()(const combo_tree& tr) const;

    virtual score_t min_improv() const;

    score_t grading;
protected:
    score_t graded_complexity(combo_tree::iterator) const;
};

/** 
 * Much like enum_graded_score, above, except this does not grade a
 * predicate if it is ineffective. 
 * 
 * A predicate is considered to be "ineffective" if it evaluates to
 * false for every row on the table.  Ineffective predicates just
 * waste time and space, and therefore, there score should not be
 * rewarded by grading (since the grading reward is stronger than
 * the complexity punishment).
 */
struct enum_effective_bscore : public enum_graded_bscore
{
    enum_effective_bscore(const CTable& _ctt)
        : enum_graded_bscore(_ctt), _ctable_usize(_ctt.uncompressed_size())
    {}

    penalized_bscore operator()(const combo_tree& tr) const;
protected:
    size_t _ctable_usize;
};

// Bscore to find interesting predicates. 
//
// The "predicate" that is found is a combo tree program that selects
// only certain rows out of the data table.  The data table is assumed
// to have one contin-valued output column, and boolean-valued input
// columns. The predicate efectively selects a bunch of outputs. The
// distribution of the resulting outputs is then examined; this
// distribution is deemed "interesting" by maximizing a linear
// combination of the following measures:
//
//    1) The Kullback Leibler divergence between the distribution
//       of the output values of the entire dataset, and the distribution
//       of the output values of the rows selected by the predicate.
//    2) The (absolute or relative) difference in skewness of the two
//       distributions.
//    3) The standardized Mann-Whitney U statistic on the selected outputs.
//    4) The product of #2 and #3.
//    5) Whether the fraction of rows that were welected (the "activation")
//       is with a desired range.
//
// All those measures are weighted; any that have null weight are
// disabled (it isn't computed and isn't pushed in the bscore).
//
// The predicate can be positive (we retain the rows/outputs when the
// predicate is true), or negative (we retain the rows/outputs when the
// predicate is false).
struct interesting_predicate_bscore : public bscore_base
{
    typedef score_t weight_t;
    typedef Counter<contin_t, contin_t> counter_t;
    typedef Counter<contin_t, contin_t> pdf_t;
    typedef boost::accumulators::accumulator_set<contin_t,
                                                 boost::accumulators::stats<
                      boost::accumulators::tag::weighted_skewness
                                                     >, contin_t> accumulator_t;

    interesting_predicate_bscore(const CTable& ctable,
                                 weight_t kld_weight = 1.0,
                                 weight_t skewness_weight = 1.0,
                                 weight_t stdU_weight = 1.0,
                                 weight_t skew_U_weight = 1.0,
                                 score_t min_activation = 0.0,
                                 score_t max_activation = 1.0,
                                 score_t penalty = 1.0,
                                 bool positive = true,
                                 bool abs_skewness = false,
                                 bool decompose_kld = false);
    penalized_bscore operator()(const combo_tree& tr) const;

    // the KLD has no upper boundary so the best of possible score is
    // the maximum value a behavioral_score can represent
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    virtual void set_complexity_coef(unsigned alphabet_size, float p);
    using bscore_base::set_complexity_coef; // Avoid hiding/shadowing

protected:

    counter_t _counter; // counter of the unconditioned distribution
    pdf_t _pdf;     // pdf of the unconditioned distribution
    mutable KLDS<contin_t> _klds; /// @todo dangerous: not thread safe!!!
    CTable _ctable;
    contin_t _skewness;   // skewness of the unconditioned distribution

    // weights of the various features
    weight_t _kld_w;
    weight_t _skewness_w;
    bool _abs_skewness;
    weight_t _stdU_w;
    weight_t _skew_U_w;
    score_t _min_activation, _max_activation;
    score_t _penalty;
    bool _positive;
    // If true then each component of the computation of KLD
    // corresponds to an element of the bscore. Otherwise the whole
    // KLD occupies just one bscore element
    bool _decompose_kld;

private:
    score_t get_activation_penalty(score_t activation) const;
};

struct iscorer_base : public unary_function<instance, composite_score>
{
    virtual composite_score operator()(const instance&) const = 0;
    virtual ~iscorer_base() {}
};

/**
 * Mostly for testing the optimization algos.  Returns minus the
 * hamming distance of the candidate to a given target instance and
 * constant null complexity.
 */
struct distance_based_scorer : public iscorer_base
{
    distance_based_scorer(const field_set& _fs,
                          const instance& _target_inst)
        : fs(_fs), target_inst(_target_inst) {}

    composite_score operator()(const instance& inst) const
    {
        score_t sc = -fs.hamming_distance(target_inst, inst);
        // Logger
        if (logger().isFineEnabled()) {
            logger().fine() << "distance_based_scorer - Evaluate instance: "
                            << fs.stream(inst) << "\n"
                            << "Score = " << sc << std::endl;
        }
        // ~Logger
        return composite_score(sc, 0, 0, 0);
    }

protected:
    const field_set& fs;
    const instance& target_inst;
};

struct complexity_based_scorer : public iscorer_base
{
    complexity_based_scorer(const cscore_base& s, representation& rep, bool reduce)
        : _cscorer(s), _rep(rep), _reduce(reduce) {}

    composite_score operator()(const instance& inst) const
    {
        if (logger().isFineEnabled()) {
            logger().fine() << "complexity_based_scorer - Evaluate instance: "
                            << _rep.fields().stream(inst);
        }

        try {
            combo_tree tr = _rep.get_candidate(inst, _reduce);
            return _cscorer(tr);
        } catch (...) {
            combo_tree raw_tr = _rep.get_candidate(inst, false);
            combo_tree red_tr = _rep.get_candidate(inst, true);
            logger().warn() << "The following instance could not be evaluated: "
                            << _rep.fields().stream(inst)
                            << "\nUnreduced tree: " << raw_tr
                            << "\nreduced tree: "<< red_tr;
        }
        return worst_composite_score;
    }

protected:
    const cscore_base& _cscorer;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
};

} //~namespace moses
} //~namespace opencog

#endif
