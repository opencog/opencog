/*
 * opencog/learning/moses/moses/select_bscore.cc
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

#include "select_bscore.h"

namespace opencog { namespace moses {

using namespace combo;


select_bscore::select_bscore(const CTable& ctable,
                             double lower_percentile,
                             double upper_percentile,
                             double hardness,
                             bool positive) :
    bscore_ctable_base(ctable)
{
    OC_ASSERT(id::contin_type == _wrk_ctable.get_output_type(),
        "The selection scorer can only be used with contin-valued tables!");

    // Verify that the bounds are sane
    OC_ASSERT((0.0 <= lower_percentile) and
              (lower_percentile < 1.0) and
              (0.0 < upper_percentile) and
              (upper_percentile <= 1.0) and
              (0.0 <= hardness) and
              (hardness <= 1.0),
        "Selection scorer, invalid bounds.");

    std::vector<score_t> ranked_out;

    for (const CTable::value_type& io_row : ctable) {
        const auto& orow = io_row.second;
        for (const CTable::counter_t::value_type& tcv : orow) {
            score_t val = get_contin(tcv.first);
            if (!positive) val = -val;
            for (unsigned i = 0; i < tcv.second; i++) {
                ranked_out.push_back(val);
            }
        }
    }

    std::sort(ranked_out.begin(), ranked_out.end());
}


behavioral_score select_bscore::operator()(const combo_tree& tr) const
{
    behavioral_score bs;

    return bs;
}

behavioral_score select_bscore::best_possible_bscore() const
{
    behavioral_score bs;

    return bs;
}


score_t select_bscore::min_improv() const
{
    return 1.0 / _ctable_usize;
}

} // ~namespace moses
} // ~namespace opencog
