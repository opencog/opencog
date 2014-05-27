/*
 * opencog/learning/moses/scoring/scoring_base.h
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

/// Abstract base class for obtaining the composite score.
struct cscore_base : public unary_function<combo_tree, composite_score>
{
    /// Evaluate the candidate combo_tree tr
    virtual composite_score operator()(const combo_tree& tr) const = 0;

    /// Return the best possible score achievable with this fitness
    /// function. This is useful for stopping MOSES when  the best
    /// possible score has been reached. If not overloaded, it will
    /// return very_best_score (which is a constant defined in
    /// opencog/learning/moses/moses/types.h)
    virtual score_t best_possible_score() const { return very_best_score; }

    /// Return the minimum value considered for improvementa.
    /// Return 0 by default.
    virtual score_t min_improv() const { return 0.0; }

    /// Indicate a set of features that should be ignored during scoring,
    /// The features are indicated as indexes, starting from 0.
    ///
    /// The primary use-case is for speeding up fitness evaluation. (???)
    ///
    /// Note that the best_possible_score may depend on the set of
    /// ignored features. Thus, the best_possible_score() method should
    /// be called only after the ignore idex have been set.
    virtual void ignore_idxs(const std::set<arity_t>&) const {}

    virtual ~cscore_base(){}
};

/// Abstract base class for behavioral scoring
struct bscore_base : public unary_function<combo_tree, behavioral_score>
{
    bscore_base() : _complexity_coef(0.0) {};
    virtual ~bscore_base() {};

    /// Return the behavioral score for the candidate combo_tree
    virtual behavioral_score operator()(const combo_tree&) const = 0;

    /// Return the best possible bscore achievable with this fitness
    /// function. This is useful for stopping MOSES when the best
    /// possible score has been reached.
    virtual behavioral_score best_possible_bscore() const = 0;

    /// Return the smallest change in the score which can be considered
    /// to be an improvement over the previous score. This is useful for
    /// avoiding local maxima which have a very flat top. That is, where
    /// all combo trees in the same local maximum have almost exactly
    /// the same score, and so scores improve by very small amounts
    /// during the search.  In such cases, one can save a lot of CPU
    /// time by terminating the search when the imrpovements are smaller
    /// than the min_improv(). Returns 0.0 by default.
    virtual score_t min_improv() const { return 0.0; }

    /// Indicate a set of features that should be ignored during scoring,
    /// The features are indicated as indexes, starting from 0.
    ///
    /// The primary use-case is for speeding up fitness evaluation. (???)
    ///
    /// Note that the best_possible_score may depend on the set of
    /// ignored features. Thus, the best_possible_score() method should
    /// be called only after the ignore idex have been set.
    virtual void ignore_idxs(const std::set<arity_t>&) const {}

    /// Get the appropriate complexity measure for the indicated combo
    /// tree. By default, this is the tree complexity, although it may
    /// depend on the scorer.
    virtual complexity_t get_complexity(const combo_tree& tr) const
    {
        return tree_complexity(tr);
    }

    /// Return the complexity coefficient.  This is used to obtain the
    /// complexity penalty for the score, which is meant to be computed
    /// as penalty = get_complexity_coef() * get_complexity(tree);
    /// This is done in two steps like this, because different scorers
    /// use a different scale, and so the complexity needs to be rescaled.
    /// Furthermore, different scorers also have a different notion of
    /// complexity. This is the opportunity to make adjustments for each
    /// case.
    virtual score_t get_complexity_coef() const { return _complexity_coef; }

    /// Store a complexity coefficient with the scorerer.  This is
    /// done to work around the fact that different kinds of scorers
    /// normalize their scores in different ways, and so the way that
    /// the penalties are scaled should differ as well.
    /// Strictly speaking, the user could just specify a different
    /// scale on the moses command line, but perhaps this inflicts too
    /// much effort on the user.  Thus, we maintain a "suggested" scaling
    /// here.
    virtual void set_complexity_coef(score_t complexity_ratio);
    virtual void set_complexity_coef(unsigned alphabet_size, float p);

protected:
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
 */
class behave_cscore : public cscore_base
{
public:
    behave_cscore(const bscore_base& sr) : _bscorer(sr) {}

    composite_score operator()(const combo_tree& tr) const;

    /// Returns the best score reachable for this problem. Used as
    /// termination condition.
    score_t best_possible_score() const
    {
        return boost::accumulate(_bscorer.best_possible_bscore(), 0.0);
    }

    /// Return the minimum value considered for improvement.
    score_t min_improv() const
    {
        return _bscorer.min_improv();
    }

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0).
    void ignore_idxs(const std::set<arity_t>& idxs) const
    {
        _bscorer.ignore_idxs(idxs);
    }

private:
    const bscore_base& _bscorer;
};

/**
 * Composite scorer defined by multiple behavioral scoring functions.
 * This is done when the problem to solve is defined in terms of multiple
 * problems.  Much like the above, but accumulated multiple behavioral
 * scores.
 */
class multibehave_cscore : public cscore_base
{
public:
    typedef boost::ptr_vector<bscore_base> BScorerSeq;
    
    /// ctor
    multibehave_cscore(const BScorerSeq& bscorers) : _bscorers(bscorers) {}

    /// Main entry point
    composite_score operator()(const combo_tree& tr) const;

    /// Returns the best score reachable for the problems. Used as
    /// termination condition.
    score_t best_possible_score() const;

    /// Return the minimum value considered for improvement.
    /// This will be the the min of all min_improv.
    score_t min_improv() const;

    /// In case the fitness function can be sped-up when certain
    /// features are ignored. The features are indicated as set of
    /// indices (from 0).
    void ignore_idxs(const std::set<arity_t>&) const;


protected:
    const BScorerSeq& _bscorers;
};


/**
 * Behavioral scorer defined by multiple behavioral scoring functions.
 * This is done when the problem to solve is defined in terms of multiple
 * problems.
XXX FIXME TODO  This class is ... wrong, and is a temporary placeholder.
It needs to be eventually be removed.  The primary problem is that bscores
should not be linearly combined in this way, since that results in diversity
measurements that are inappropriately weighted.  That is, different bscorers
may have a different conception of what "diversity" means. This is especially
the case for the enum_ scorers, which have a grading as each possible enum is
matched.  That is, notions of diversity are analogous to notions of complexity;
they make sense only within the context of the actual scorer.

The correct answer is to move the diversity calculations out of the metapop
and into bscore_base.  The combination of multiple bscorers into one place
would then happen in multibehave, above.  However, this is a big tearup of
the code, so I'm putting this off for a litttle while.
 */
struct multibscore_based_bscore : public bscore_base
{
    typedef boost::ptr_vector<bscore_base> BScorerSeq;
    
    // ctors
    multibscore_based_bscore(const BScorerSeq& bscorers) : _bscorers(bscorers) {}

    // main operator
    behavioral_score operator()(const combo_tree& tr) const;

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
static inline void log_candidate_bscore(const combo_tree& tr,
                                        const behavioral_score& bs)
{
    if (logger().isFineEnabled())
        logger().fine() << "Evaluate candidate: " << tr << "\n"
                        << "\tBScored: " << bs;
}

} //~namespace moses
} //~namespace opencog

#endif
