/*
 * opencog/learning/moses/moses/scoring.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks, Nil Geisweiller
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
#include <boost/range/algorithm/transform.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/weighted_skewness.hpp>

#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/functional.h>
#include <opencog/util/KLD.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>
#include <opencog/comboreduct/reduct/meta_rules.h>

#include "using.h"
#include "../representation/representation.h"
#include "types.h"

namespace opencog { namespace moses {

typedef float fitness_t; /// @todo is that really useful?

// Abstract scoring function class to implement
struct score_base : public unary_function<combo_tree, score_t>
{
    // Evaluate the candidate tr
    virtual score_t operator()(const combo_tree& tr) const = 0;

    // Return the best possible score achievable with that fitness
    // function. This is useful in order to stop running MOSES when
    // the best possible score is reached.
    virtual score_t best_possible_score() const = 0;

    // Return the minimum value considered for improvement
    virtual score_t min_improv() const = 0;
};

// Abstract bscoring function class to implement
struct bscore_base : public unary_function<combo_tree, behavioral_score>
{
    // Evaluate the candidate tr
    virtual behavioral_score operator()(const combo_tree& tr) const = 0;

    // Return the best possible bscore achievable with that fitness
    // function. This is useful in order to stop running MOSES when
    // the best possible score is reached
    virtual behavioral_score best_possible_bscore() const = 0;

    // Return the minimum value considered for improvement
    virtual score_t min_improv() const = 0;
};

/**
 * score calculated based on the behavioral score. Useful to avoid
 * redundancy of code and computation in case there is a cache over
 * bscore. The score is calculated as the sum of the bscore over all
 * features, that is:
 * score = sum_f BScore(f),
 */
/// @todo Inheriting that class from score_base raises a compile error
/// because in moses_exec.h some code attempts to use BScore that does
/// does not contain best_possible_bscore(). Specifically, the
/// BScoreACache and ScoreACache typedefs do this. However, this begs
/// a question: why do we need a base class anyway, if we're not going
/// to need it? ??
/// The answer:
/// First we do use it in other bscores such as
/// multiscore_based_bscore.
/// Second after fixing the cache API (that is having the function
/// being cached being inherited by the cache that compile error will
/// go away and we can have bscore_based_score inherit it)

template<typename BScore>
struct bscore_based_score : public unary_function<combo_tree, score_t>
{
    bscore_based_score(const BScore& bs) : bscore(bs) {}
    score_t operator()(const combo_tree& tr) const
    {
        try {
            behavioral_score bs = bscore(tr);
            score_t res = boost::accumulate(bs, 0.0);

            if (logger().isFineEnabled()) {
                stringstream ss;
                ss << "bscore_based_score: " << res;
                ss << " for candidate: " << tr;
                logger().fine(ss.str());
            }

            return res;
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
            stringstream ss;
            ss << "The following candidate: " << tr << "\n";
            ss << "has failed to be evaluated, "
               << "raising the following exception: "
               << ee.get_message() << " " << ee.get_vertex();
            logger().fine(ss.str());

            return get_score(worst_composite_score);
        }
    }

    // Returns the best score reachable for that problem. Used as
    // termination condition.
    score_t best_possible_score() const
    {
        return boost::accumulate(bscore.best_possible_bscore(), 0.0);
    }

    // Return the minimum value considered for improvement
    score_t min_improv() const
    {
        return bscore.min_improv();
    }

    const BScore& bscore;
};

/**
 * Bscore defined by multiple scoring functions. This is done when the
 * problem to solve is defined in terms of multiple problems. For now
 * the multiple scores have the same type as defined by the template
 * argument Score.
 */
template<typename Score>
struct multiscore_based_bscore : public bscore_base
{
    typedef boost::ptr_vector<Score> ScoreSeq;

    // ctors
    multiscore_based_bscore(const ScoreSeq& scores_) : scores(scores_) {}

    // main operator
    behavioral_score operator()(const combo_tree& tr) const
    {
        behavioral_score bs(scores.size());
        boost::transform(scores, bs.begin(), [&](const Score& sc){return sc(tr);});
        return bs;
    }

    behavioral_score best_possible_bscore() const
    {
        behavioral_score bs;
        foreach(const Score& sc, scores) {
            bs.push_back(sc.best_possible_score());
        }
        return bs;
    }

    // return the min of all min_improv
    score_t min_improv() const
    {
        score_t res = best_score;
        foreach(const Score& s, scores)
            res = min(res, s.min_improv());
        return res;
    }

    ScoreSeq scores;
};

/**
 * Bscore defined by multiple behavioral scoring functions. This is
 * done when the problem to solve is defined in terms of multiple
 * problems. For now the multiple scores have the same type as defined
 * by the template argument Score.
 */
template<typename BScore>
struct multibscore_based_bscore : public bscore_base
{
    typedef boost::ptr_vector<BScore> BScoreSeq;

    // ctors
    multibscore_based_bscore(const BScoreSeq& bscores_) : bscores(bscores_) {}

    // main operator
    behavioral_score operator()(const combo_tree& tr) const
    {
        behavioral_score bs;
        foreach(const BScore& bsc, bscores)
            boost::push_back(bs, bsc(tr));
        return bs;
    }

    behavioral_score best_possible_bscore() const
    {
        behavioral_score bs;
        foreach(const BScore& bsc, bscores)
            boost::push_back(bs, bsc.best_possible_bscore());
        return bs;
    }

    // return the min of all min_improv
    score_t min_improv() const
    {
        score_t res = best_score;
        foreach(const BScore& bs, bscores)
            res = min(res, bs.min_improv());
        return res;
    }

    BScoreSeq bscores;
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

    behavioral_score operator()(const combo_tree& tr) const;

    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    void set_complexity_coef(unsigned alphabet_size, float p);
    void set_complexity_coef(score_t complexity_ratio);

private:
    complete_truth_table target;
    int arity;

    bool occam; // If true, then Occam's razor is taken into account.
    score_t complexity_coef;
};

// Used to define the complexity scoring component given that p is the
// probability of having an observation being wrong (see the comment
// regarding ctruth_table_bscore for more information).
score_t discrete_complexity_coef(unsigned alphabet_size, double p);

// Used to define the complexity scoring component given that stdev is
// the standard deviation of the noise of the we're trying to predict
// output (see the comment regarding contin_bscore for more
// information).
score_t contin_complexity_coef(unsigned alphabet_size, double stdev);

/**
 * Fitness function based on binary precision
 * http://en.wikipedia.org/wiki/Accuracy_and_precision#In_binary_classification
 *
 * This bscore has 3 components:
 *
 * 1) the precision (or the negative predictive value if positive is
 * false). Note that candidates are active when they output true (even
 * if they try to maximize negative predictive value);
 *
 * 2) a penalty depending on whether the constraint explained below is
 * met;
 *
 * 3) and possibly the occam's razor.
 *
 * There's a constraint that the activation must be within the
 * interval [min_activation, max_activation]. If the constraint is not
 * met and penality is non null then the second component of the
 * bscore takes the value:
 * log( (1 - dst(activation, [min_activation, max_activation])) ^ penalty )
 * where dst(x, I) is defined as
 * dst(x, I) = max(min(I.min - x, 0) / I.min, min(x - I.max,0)/(1 - I.max))
 *
 * If the CTable output type is contin instead of boolean then the hit
 * count is replaced by the sum of the outputs (or minus that sum if
 * this->positive == false)
 *
 * If worst_norm is true then the percision is divided by the absolute
 * average of the negative lower (resp. positive upper if
 * this->positive is false) decile or less. If there is no negative
 * (resp. positive if this->positive is false) values then it is not
 * normalized.
 */
struct precision_bscore : public bscore_base
{
    precision_bscore(const CTable& _ctable,
                     float alphabet_size, float p,
                     float min_activation, float max_activation,
                     float penalty,
                     bool positive = true,
                     bool worst_norm = false);

    behavioral_score operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    CTable ctable;
    unsigned ctable_usize;                  // uncompressed size of ctable
    bool occam; // If true, then Occam's razor is taken into account.
    score_t complexity_coef;
    score_t min_activation, max_activation;
    score_t max_precision; // uppper bound of the maximum denormalized
                           // precision for that CTable
    score_t penalty;
    bool positive, worst_norm;

private:
    score_t get_activation_penalty(score_t activation) const;
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
                             bool weighted_average,
                             float alphabet_size, float p);

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

    OTable target;
    ITable cit;
    vector<contin_t> thresholds;
    bool weighted_accuracy;     // Whether the bscore is weighted to
                                // deal with unbalanced data.
    bool occam;                 // Whether Occam's razor is enabled
    score_t complexity_coef;

protected:
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

    void init(unsigned alphabet_size, float stdev,
              err_function_type eft = squared_error) {
        occam = stdev > 0;
        set_complexity_coef(alphabet_size, stdev);
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
                  float alphabet_size, float stdev,
                  err_function_type eft = squared_error)
        : target(score, r), cti(r) {
        init(alphabet_size, stdev, eft);
    }

    contin_bscore(const OTable& t, const ITable& r,
                  float alphabet_size, float stdev,
                  err_function_type eft = squared_error)
        : target(t), cti(r) {
        init(alphabet_size, stdev, eft);
    }

    contin_bscore(const Table& table,
                  float alphabet_size, float stdev,
                  err_function_type eft = squared_error)
        : target(table.otable), cti(table.itable) {
        init(alphabet_size, stdev, eft);
    }

    behavioral_score operator()(const combo_tree& tr) const;

    // The best possible bscore is a vector of zeros. That's probably
    // not quite true, because there could be duplicated inputs, but
    // that's acceptable for now.
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    OTable target;
    ITable cti;
    bool occam;
    score_t complexity_coef;

private:
    void set_complexity_coef(unsigned alphabet_size, float stdev);

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
 *
 * where |D_ne| is the number of outputs that are incorrect, and |M| is
 * the complexity of the model, and |A| is the alphabets size employed
 * in the model.
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
 */
struct ctruth_table_bscore : public bscore_base
{
    template<typename Func>
    ctruth_table_bscore(const Func& func, arity_t arity,
                        unsigned alphabet_size, float p, int nsamples = -1)
        : ctable(func, arity, nsamples)
    {
        set_complexity_coef(alphabet_size, p);
    }
    ctruth_table_bscore(const CTable& _ctt, unsigned alphabet_size, float p);

    behavioral_score operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

    void set_complexity_coef(score_t complexity_ratio);
    void set_complexity_coef(unsigned alphabet_size, float p);

private:

    CTable ctable;
    bool occam; // If true, then Occam's razor is taken into account.
    score_t complexity_coef;
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
    template<typename Func>
    enum_table_bscore(const Func& func, arity_t arity,
                      unsigned alphabet_size, float p, int nsamples = -1)
        : ctable(func, arity, nsamples)
    {
        set_complexity_coef(alphabet_size, p);
    }

    enum_table_bscore(const CTable& _ctt, unsigned alphabet_size, float p)
        : ctable(_ctt)
    {
        set_complexity_coef(alphabet_size, p);
    }

    behavioral_score operator()(const combo_tree& tr) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;

    score_t min_improv() const;

protected:
    void set_complexity_coef(unsigned alphabet_size, float p);

    CTable ctable;
    bool occam; // If true, then Occam's razor is taken into account.
    score_t complexity_coef;
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
    template<typename Func>
    enum_filter_bscore(const Func& func, arity_t arity,
                      float alphabet_size, float p, int nsamples = -1)
        : enum_table_bscore(func, arity, alphabet_size, p, nsamples),
          punish(1.0)
    {}

    enum_filter_bscore(const CTable& _ctt, float alphabet_size, float p)
        : enum_table_bscore(_ctt, alphabet_size, p),
          punish(1.0)
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
    template<typename Func>
    enum_graded_bscore(const Func& func, arity_t arity,
                      float alphabet_size, float p, int nsamples = -1)
        : enum_table_bscore(func, arity, alphabet_size, p, nsamples),
          grading(0.8)
    {}

    enum_graded_bscore(const CTable& _ctt, float alphabet_size, float p)
        : enum_table_bscore(_ctt, alphabet_size, p),
          grading(0.8)
    {}

    behavioral_score operator()(const combo_tree& tr) const;

    score_t grading;
};

// Bscore to find interesting predicates. Interestingness is measured
// in terms of several features such as
//
//    1) the Kullback Leibler divergence between the distribution
// output of the dataset and the distribution over the output filtered
// in by the program (when the predicate is true).
//    2) the (absolute or relative) difference in skewness of the 2 distributions
//    3) the standardized Mann-Whitney U statistic
//    4) the product of #2 and #3
//    5) the whether the activation is with a desired range
//
// All those features are weighted, any one with null weight is
// disabled (it isn't computed and isn't pushed in the bscore).
//
// the predicate can be positive (we retain the outputs when the
// predicate is true), or negative (we retain the outputs when the
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
                                 float alphabet_size, float stdev,
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

    counter_t counter; // counter of the unconditioned distribution
    pdf_t pdf;     // pdf of the unconditioned distribution
    mutable KLDS<contin_t> klds; /// @todo dangerous: not thread safe!!!
    CTable ctable;
    bool occam;
    score_t complexity_coef;
    contin_t skewness;   // skewness of the unconditioned distribution

    // weights of the various features
    weight_t kld_w;
    weight_t skewness_w;
    bool abs_skewness;
    weight_t stdU_w;
    weight_t skew_U_w;
    score_t min_activation, max_activation;
    score_t penalty;
    bool positive;
    // If true then each component of the computation of KLD
    // corresponds to an element of the bscore. Otherwise the whole
    // KLD occupies just one bscore element
    bool decompose_kld;

private:
    void set_complexity_coef(float alphabet_size, float stdev);
    score_t get_activation_penalty(score_t activation) const;
};

// For testing only
struct dummy_score : public unary_function<combo_tree, score_t>
{
    score_t operator()(const combo_tree& tr) const {
        return score_t();
    }
};

// For testing only
struct dummy_bscore : public unary_function<combo_tree, behavioral_score>
{
    behavioral_score operator()(const combo_tree& tr) const {
        return behavioral_score();
    }
};

/**
 * Mostly for testing the optimization algos.  Returns minus the
 * hamming distance of the candidate to a given target instance and
 * constant null complexity.
 */
struct distance_based_scorer : public unary_function<instance,
                                                     composite_score>
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
        return composite_score(sc, 0);
    }

protected:
    const field_set& fs;
    const instance& target_inst;
};

template<typename Scoring>
struct complexity_based_scorer : public unary_function<instance,
                                                       composite_score>
{
    complexity_based_scorer(const Scoring& s, representation& rep, bool reduce)
        : score(s), _rep(rep), _reduce(reduce) {}

    composite_score operator()(const instance& inst) const
    {
        using namespace reduct;

        // Logger
        if (logger().isFineEnabled()) {
            logger().fine() << "complexity_based_scorer - Evaluate instance: "
                            << _rep.fields().stream(inst);
        }
        // ~Logger

        try {
            combo_tree tr = _rep.get_candidate(inst, _reduce);
            return composite_score(score(tr), tree_complexity(tr));
        } catch (...) {
            logger().debug() << "Warning: The following instance has failed to be evaluated: "
                             << _rep.fields().stream(inst);
            return worst_composite_score;
        }
    }

protected:
    const Scoring& score;
    representation& _rep;
    bool _reduce; // whether the exemplar is reduced before being
                  // evaluated, this may be advantagous if Scoring is
                  // also a cache
};

} //~namespace moses
} //~namespace opencog

#endif
