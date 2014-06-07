/*
 * opencog/learning/moses/scoring/boosting_cscore.h
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
#ifndef _MOSES_BOOST_CSCORE_H
#define _MOSES_BOOST_CSCORE_H

#include <vector>

#include "scoring_base.h"

namespace opencog { namespace moses {

/**
 * Composite score calculated from the behavioral score, using boosting.
 *
 * The score is calculated as a weighted sum of the bscore over all rows:
 *      score = sum_i weight(i) * BScore(i) + penalty
 *
 * We're calling each element in the bscore a "row" because it typically
 * corresponds to a row of an input table, during supervised training.
 * The weight is an adjustment, computed by applying a standard AdaBoost-
 * style algorithm.
 */
class boosting_cscore : public cscore_base
{
public:
    boosting_cscore(const bscore_base& sr) : _bscorer(sr) {}

    composite_score operator()(const combo_tree& tr) const;

    /// Returns the best score reachable for this problem. Used as
    /// termination condition.
    score_t best_possible_score() const;

    /// Return the minimum value considered for improvement.
    score_t min_improv() const;

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0).
    void ignore_idxs(const std::set<arity_t>& idxs) const
    {
        _bscorer.ignore_idxs(idxs);
    }

private:
    const bscore_base& _bscorer;
    std::vector<double> _weights;
};


} //~namespace moses
} //~namespace opencog

#endif // _MOSES_BOOST_CSCORE_H
