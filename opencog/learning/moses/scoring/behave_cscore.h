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

#include <opencog/util/lru_cache.h>

#include "scoring_base.h"

namespace opencog { namespace moses {

/**
 * Composite score calculated from the behavioral score.
 *
 * The score is calculated as the sum of the bscore over all samples:
 *      score = sum_x BScore(x) + penalty
 *
 * Each element in the bscore typically corresponds to a sample in a
 * supervised training set, that is, a row of a table contianing the
 * training data.
 *
 * This is a "minor" helper class, and exists for three reasons:
 * 1) avoids some redundancy of having the summation in many places
 * 2) Helps with keeping the score-caching code cleaner.
 * 3) When boosting, the summation above is no longer just a simple sum.
 */
class behave_cscore 
{
public:
    behave_cscore(const bscore_base& b, ascore_base& a)
        : _bscorer(b), _ascorer(a) {}

    behavioral_score get_bscore(const combo_tree&) const;
    behavioral_score get_bscore(const scored_combo_tree_set&) const;
    composite_score get_cscore(const combo_tree&);
    composite_score get_cscore(const scored_combo_tree_set&);

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

    ascore_base& get_ascorer() const { return _ascorer; }

private:
    const bscore_base& _bscorer;
    ascore_base& _ascorer;

#ifdef LATER
    prr_cache_threaded<cscore_base> score_cache(initial_cache_size, c_scorer,
                                                 "composite scores");
#endif

};


} //~namespace moses
} //~namespace opencog

#endif // _MOSES_BEHAVE_CSCORE_H
