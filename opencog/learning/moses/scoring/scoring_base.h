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

#include "../moses/types.h"
#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace moses {

using combo::combo_tree;
using combo::arity_t;

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

/// Abstract base class for behavioral scoring.
/// A behavioral score is a vector of scores, one per sample of a dataset.
struct bscore_base : public std::unary_function<combo_tree, behavioral_score>
{
    bscore_base() : _complexity_coef(0.0), _size(0) {};
    virtual ~bscore_base() {};

    /// Return the behavioral score for the candidate combo_tree
    virtual behavioral_score operator()(const combo_tree&) const = 0;

    /// Return the size (length) of the behavioral_score that operator()
    /// above would return.
    virtual size_t size() const { return _size; }

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
    ///
    /// XXX I don't get it ... if we are going to ignore indexes,
    /// shouldn't we ignore them when building the combo trees?  And
    /// if a combo tree does make use of an index, how can one possibly
    /// "ignore" it ??  This just does not seem correct to me...
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
    size_t _size;
};

/// Abstract base class for summing behavioral scores.
struct ascore_base : public std::unary_function<combo_tree, composite_score>
{
    /// Sum up the behavioral score
    virtual score_t operator()(const behavioral_score&) const = 0;

    virtual ~ascore_base(){}
};

/// Simplest ascore, it just total up the bscore.
struct simple_ascore : ascore_base
{
    /// Sum up the behavioral score
    virtual score_t operator()(const behavioral_score&) const;
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
