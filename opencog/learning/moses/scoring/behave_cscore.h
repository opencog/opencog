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
    behave_cscore(bscore_base& b, size_t initial_cache_size=0);

    behavioral_score get_bscore(const combo_tree&) const;
    behavioral_score get_bscore(const scored_combo_tree_set&) const;
    composite_score get_cscore(const combo_tree&);
    composite_score get_cscore(const scored_combo_tree_set&);

    /// Returns the best score reachable for this problem. Used as
    /// termination condition.
    score_t best_possible_score() const;

    /// Returns the worst score reachable for this problem. Used to
    /// compute the scoring error during boosting.
    score_t worst_possible_score() const;

    /// Return the minimum value considered for improvement.
    score_t min_improv() const
    {
        return _bscorer.min_improv();
    }

    /// In table-based scorers, fitness function evaluation can be sped
    /// up when unused features are ignored.  The unused features must
    /// not subsequently appear in the combo tree to be scored.  Calling
    /// this with the empty set restores all features. The features are
    /// indicated as set of indices (from 0).
    void ignore_cols(const std::set<arity_t>& idxs) const
    {
        _bscorer.ignore_cols(idxs);
    }

    // In case one wants to evaluate the fitness on a subset of the
    // data, one can provide a set of row indexes to ignore
    void ignore_rows(const std::set<unsigned>& idxs) const
    {
        _bscorer.ignore_rows(idxs);
    }

    // Like ignore_rows but consider timestamps instead of indexes
    void ignore_rows_at_times(const std::set<TTable::value_type>& timestamps) const
    {
        _bscorer.ignore_rows_at_times(timestamps);
    }

    // Return the uncompressed size of the CTable
    unsigned get_ctable_usize() const
    {
        return _bscorer.get_ctable_usize();
    }

    // Return the original CTable
    const CTable& get_ctable() const {
        return _bscorer.get_ctable();
    }

private:
    bscore_base& _bscorer;

    // Below follows some assorted infrastructure to allow composite
    // scoress for trees to be cached.
    struct wrapper : public std::unary_function<combo_tree, composite_score>
    {
        composite_score operator()(const combo_tree&) const;
        behave_cscore* self;
    };

    bool _have_cache;
    wrapper _wrapper;
    prr_cache_threaded<wrapper> _cscore_cache;
    composite_score get_cscore_nocache(const combo_tree&);

public:
    // weird hack for subsample scoring...
    bscore_base& get_bscorer() { return _bscorer; }
};


} //~namespace moses
} //~namespace opencog

#endif // _MOSES_BEHAVE_CSCORE_H
