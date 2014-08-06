/*
 * opencog/learning/moses/scoring/select_bscore.h
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
#ifndef _SELECT_BSCORE_H
#define _SELECT_BSCORE_H

#include <opencog/comboreduct/table/table.h>
#include "scoring_base.h"

namespace opencog { namespace moses {

using combo::CTable;
using combo::count_t;

/**
 * Fitness function for selecting a range of rows from a continuous-
 * valued distribution.  The range of rows selected are expressed as
 * percentiles. That is, if the input table has N rows, then rows that
 * are selected will be rows N*lower_percentile through N*upper_percentile,
 * where the rows are numbered in ascending rank.  That is, this scorer
 * will rank (sort) the rows according to the output column, and then
 * select those only in the indicated range.  If 'positive' is false,
 * then the rows are ranked in reverse order.
 *
 * The 'hardness' indicates just how sharp it selection will be TBD.
 * Not yet implemented.
 */
struct select_bscore : public bscore_ctable_base
{
    select_bscore(const CTable& ctable,
                  double lower_percentile = 0.8f,
                  double upper_percentile = 0.9f,
                  double hardness = 1.0f,
                  bool positive = true);

    behavioral_score operator()(const combo_tree& tr) const;
    behavioral_score operator()(const scored_combo_tree_set&) const;
    score_t get_error(const behavioral_score&) const;

    // Return the best possible bscore. Used as one of the
    // termination conditions (when the best bscore is reached).
    behavioral_score best_possible_bscore() const;
    behavioral_score worst_possible_bscore() const;

    score_t min_improv() const;

protected:
    void set_best_possible_bscore();
    behavioral_score _best_possible_score;

    bool _positive;
    score_t _lower_bound;
    score_t _upper_bound;
};


} // ~namespace moses
} // ~namespace opencog

#endif
