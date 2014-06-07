/*
 * opencog/learning/moses/scoring/behave_cscore.h
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
#ifndef _MOSES_BEHAVE_CSCORE_H
#define _MOSES_BEHAVE_CSCORE_H

#include "scoring_base.h"

namespace opencog { namespace moses {

/**
 * Composite score calculated from the behavioral score.
 *
 * The score is calculated as the sum of the bscore over all rows:
 *      score = sum_i BScore(i) + penalty
 *
 * We're calling each element in the bscore a "row" because it typically
 * corresponds to a row of an input table, during supervised training.
 *
 * This is a "minor" helper class, and exists for three reasons:
 * 1) avoids some redundancy of having the summation in many places
 * 2) Helps with keeping the score-caching code cleaner.
 * 3) When boosting, the summation above is no longer just a simple sum.
 */
class behave_cscore : public cscore_base
{
public:
    behave_cscore(const bscore_base& sr) : _bscorer(sr) {}

    composite_score operator()(const combo_tree& tr) const;

    /// Returns the best score reachable for this problem. Used as
    /// termination condition.
    score_t best_possible_score() const;

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


} //~namespace moses
} //~namespace opencog

#endif // _MOSES_BEHAVE_CSCORE_H
