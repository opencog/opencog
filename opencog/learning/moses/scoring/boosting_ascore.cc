/*
 * opencog/learning/moses/scoring/boosting_ascore.cc
 *
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Linas Vepstas
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

#include <math.h>

#include "boosting_ascore.h"

namespace opencog { namespace moses {

boosting_ascore::boosting_ascore(size_t sz)
{
    _weights = std::vector<double>(sz, 1.0);
    _size = sz;
}

score_t boosting_ascore::operator()(const behavioral_score& bs) const
{
    OC_ASSERT(bs.size() == _size, "Unexpected behavioral score size");

#define M_OE (1.0 / M_E)

    score_t res = 0.0;
    for (size_t i=0; i<_size; i++) {
        // res += _weights[i] * bs[i];
        // Not the above, but the below...
        //
        // XXX FIXME for right now, this particular scorer implements
        // the 'discerete AdaBoost' error function, viz: we assume that
        // bs[i] = 0 or 1 (as is the case for the boolean-valued problems)
        // where bs[i] = 0 for wrong answer, bs[i] = 1 for right answer.
        // The error function then uses e=2.71828 for right answers, and
        // 1/e for wrong answers.
        //
        // we need to create other kinds of scorers for other problems
        // e.g brownboost, maybe logitboost, certainly gentleboost. But
        // for now this is discrete, exactly solvable adaboost.

        // Note: bs is 0 when the answer is right, -1 when wrong.
        // We want a larger value here when wrong, smaller when right,
        // because this is measuring the total error.
        res += _weights[i] * ((-bs[i] > 0.5) ? M_OE : M_E);
    }

    // To be consistent with moses, the total score should be negative too.
    res = -res;
    return res;
}


} // ~namespace moses
} // ~namespace opencog
