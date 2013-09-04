/** argument.h --- 
 *
 * Copyright (C) 2012 OpenCog Foundation
 *
 * Author: Nil Geisweiller, Moshe Looks
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


#ifndef _COMBO_ARGUMENT_H
#define _COMBO_ARGUMENT_H

#include "common_def.h"
#include <boost/operators.hpp>

namespace opencog { namespace combo {

/**
 * class argument
 *
 *  Represents the index of an input variable of a function coded as a
 *  combo tree.  For boolean arguments, a negative value corresponds to
 *  the negated literal (i.e. not of the argument).  For example,
 *  idx == -3 represents the literal NOT($3) where $3 is the third
 *  argument of the function.
 *
 *  idx == 0 is invalid.
 */
class argument
    : boost::less_than_comparable<argument>, // generate >, <= and >= given <
      boost::equality_comparable<argument>   // generate != given ==
{
public:
    arity_t idx;

    explicit argument(arity_t i) : idx(i) {
        OC_ASSERT(idx != 0, "idx should be different than zero.");
    }

    void negate() {
        idx = -idx;
    }
    bool is_negated() const {
        return idx < 0;
    }

    bool operator<(const argument& rhs) const {
        static opencog::absolute_value_order<int> comp;
        return comp(idx, rhs.idx);
    }

    bool operator==(const argument& rhs) const {
        return idx == rhs.idx;
    }

    arity_t abs_idx() const {
        return idx < 0 ? -idx : idx;
    }
    const static arity_t idx_to_abs_idx_from_zero(arity_t other_idx) {
        return (other_idx < 0 ? -other_idx : other_idx) - 1;
    }
    const static arity_t idx_from_zero_to_idx(arity_t idx_from_zero) {
        return idx_from_zero + 1;;
    }
    /// Returns 0 for argument of idx 1 or -1, 1 for idx 2 or -2, and so on
    arity_t abs_idx_from_zero() const {
        return idx_to_abs_idx_from_zero(idx);
    }
    /// Check if the index of this arg is in the possible range given arity a.
    /// Formally, idx is in the range of a if :
    /// if idx==0 then it is not valid
    /// else if a>0 (that is, no arg_list) then idx<=a
    /// else if a<0 then idx < -a
    bool is_idx_valid(arity_t a) const {
        return (idx == 0 ? false : (a > 0 ? abs_idx() <= a : abs_idx() < -a));
    }
};

} // ~namespace combo
} // ~namespace opencog

#endif // _COMBO_ARGUMENT_H
