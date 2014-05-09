/*
 * opencog/learning/moses/scoring/scoring_base.h
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
#ifndef _MOSES_SCORING_BASE_H
#define _MOSES_SCORING_BASE_H

#include <iostream>
#include <fstream>
#include <functional>

#include <boost/accumulators/accumulators.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/range/algorithm/min_element.hpp>

#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/functional.h>
#include <opencog/util/KLD.h>

#include <opencog/comboreduct/table/table.h>

#include "../moses/types.h"
#include "../representation/representation.h"

namespace opencog { namespace moses {

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
    virtual void ignore_idxs(const std::set<arity_t>&) const {}

    virtual ~cscore_base(){}
};

// Abstract bscoring function class to implement
struct bscore_base : public unary_function<combo_tree, penalized_bscore>
{
    bscore_base() : _occam(false), _complexity_coef(0.0) {};
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
    virtual void ignore_idxs(const std::set<arity_t>&) const {}

    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float p);

protected:
    bool _occam; // If true, then Occam's razor is taken into account.
    score_t _complexity_coef;
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
    void ignore_idxs(const std::set<arity_t>& idxs) const
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
    void ignore_idxs(const std::set<arity_t>&) const;

protected:
    const BScorerSeq& _bscorers;
};

// helper to log a combo_tree and its behavioral score
static inline void log_candidate_pbscore(const combo_tree& tr,
                                         const penalized_bscore& pbs)
{
    if (logger().isFineEnabled())
        logger().fine() << "Evaluate candidate: " << tr << "\n"
                        << "\tBScored: " << pbs;
}

} //~namespace moses
} //~namespace opencog

#endif
