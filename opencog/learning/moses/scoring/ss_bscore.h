/*
 * opencog/learning/moses/scoring/ss_bscore.h
 *
 * Copyright (C) 2014 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#ifndef _MOSES_SS_BSCORE_H
#define _MOSES_SS_BSCORE_H

#include "scoring_base.h"

namespace opencog { namespace moses {

using combo::combo_tree;
using combo::arity_t;
using combo::CTable;
using combo::TTable;

/**
 * Subsampling behavioral score. Takes a bscore and turn it into
 * subsampling bscore.
 */
struct ss_bscore : public bscore_base
{
    // ctors
    ss_bscore(const bscore_base& bscorer, unsigned n_subsamples = 0,
              float low_dev_pressure = 0.0, bool by_time = true);

    // main operator
    behavioral_score operator()(const combo_tree& tr) const;

    behavioral_score best_possible_bscore() const;

    // return the min of all min_improv
    score_t min_improv() const;

    // In case the fitness function can be sped-up when certain
    // features are ignored. The features are indicated as set of
    // indices (from 0).
    void ignore_cols(const std::set<arity_t>&) const;

    // In case one wants to evaluate the fitness on a subset of the
    // data, one can provide a set of row indexes to ignore
    void ignore_rows(const std::set<unsigned>&) const;

    // Like ignore_rows but consider timestamps instead of indexes
    void ignore_rows_at_times(const std::set<TTable::value_type>&) const;

    // Return the uncompressed size of the CTable
    unsigned get_ctable_usize() const;

    // Return the original CTable
    const CTable& get_ctable() const;

protected:
    const bscore_base& _bscorer;
    unsigned _n_subsamples;
    float _low_dev_pressure;
    bool _by_time;
    std::set<TTable::value_type> _timestamps;
};

} //~namespace moses
} //~namespace opencog

#endif
