/*
 * opencog/learning/moses/moses/types.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#include "types.h"
#include "complexity.h"

namespace opencog { namespace moses {

const composite_score worst_composite_score=
    std::make_pair(worst_score,worst_complexity);

/**
 * Non standard definition of greater than between 2 composite scores.
 * In that definition nan on the score is never greater than anything.
 */
bool cscore_gt(const composite_score& l_csc, const composite_score& r_csc) {
    return !isnan(l_csc) && (isnan(r_csc) || l_csc > r_csc);
}
bool cscore_ge(const composite_score& l_csc, const composite_score& r_csc) {
    return cscore_gt(l_csc, r_csc) || l_csc == r_csc;
}

// XXX TODO: Expose this as a user-settable flag.
int composite_score::weight = 4;

} // ~namespace moses
} // ~namespace opencog
