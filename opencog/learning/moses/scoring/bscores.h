/*
 * opencog/learning/moses/scoring/bscores.h
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
#ifndef _BSCORES_H
#define _BSCORES_H

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

#include "scoring_base.h"
#include "../moses/types.h"
#include "../representation/representation.h"

namespace opencog { namespace moses {

/**
 * Each feature corresponds to an input tuple, 0 if the output of the
 * candidate matches the output of the intended function, -1
 * otherwise.
 */
struct logical_bscore : public bscore_base
{
    template<typename Func>
    logical_bscore(const Func& func, int a)
            : _target(func, a), _arity(a) {}
    logical_bscore(const combo_tree& tr, int a)
            : _target(tr, a), _arity(a) {}

    behavioral_score operator()(const combo_tree& tr) const;

    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

protected:
    complete_truth_table _target;
    int _arity;
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

    behavioral_score operator()(const combo_tree& tr) const;

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

    behavioral_score operator()(const combo_tree& tr) const;

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

    behavioral_score operator()(const combo_tree& tr) const;

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

    behavioral_score operator()(const combo_tree& tr) const;

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

    behavioral_score operator()(const combo_tree& tr) const;

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

    behavioral_score operator()(const combo_tree&) const;

    virtual score_t min_improv() const;
    virtual complexity_t get_complexity(const combo_tree&) const;

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

    behavioral_score operator()(const combo_tree& tr) const;
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
    behavioral_score operator()(const combo_tree& tr) const;

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

// ============================================================================    

struct cluster_bscore : public bscore_base
{
    cluster_bscore(const ITable&);

    behavioral_score operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

protected:
    ITable _itable;
};

} //~namespace moses
} //~namespace opencog

#endif
