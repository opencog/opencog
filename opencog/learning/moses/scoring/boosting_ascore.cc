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

#include "boosting_ascore.h"

namespace opencog { namespace moses {

boosting_ascore::boosting_ascore(size_t sz)
{
    double norm = 1.0 / (double) sz;
    _weights = std::vector<double>(sz, norm);
    _size = sz;
}

score_t boosting_ascore::operator()(const behavioral_score& bs) const
{
    OC_ASSERT(bs.size() == _size, "Unexpected behavioral score size");

    score_t res = 0.0;
    for (size_t i=0; i<_size; i++) {
        res += _weights[i] * bs[i];
    }
    return res;
}


} // ~namespace moses
} // ~namespace opencog
