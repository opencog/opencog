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


#ifndef _OPENCOG_ARGUMENT_H
#define _OPENCOG_ARGUMENT_H

#include "common_def.h"

namespace opencog { namespace combo {

/*
  class argument
    represents the index, attribute idx, of an input variable
    of a function coded into a combo tree. In the case of a boolean
    argument a negative value corresponds to a negative literal.
    idx == 0 is invalide.
    For example idx == -3 represents the literal NOT($3) where $3 is the
    third argument of the function.
*/
class argument
{
public:
    explicit argument(arity_t i) : idx(i) {
        OC_ASSERT(idx != 0, "idx should be different than zero.");
    }
    arity_t idx;

    void negate() {
        idx = -idx;
    }
    bool is_negated() const {
        return idx < 0;
    }
    bool operator<(argument rhs) const {
        static opencog::absolute_value_order<int> comp;
        return comp(idx, rhs.idx);
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
    // returns 0 for argument of idx 1 or -1, 1 for idx 2 or -2, and so on
    arity_t abs_idx_from_zero() const {
        return idx_to_abs_idx_from_zero(idx);
    }
    // check if idx is in the possible range given arity a
    // formally idx is in the range of a:
    // if idx==0 then it is not valid anyway
    // else if a>0 (that is no arg_list) then idx<=a
    // else if a<0 then idx<-a
    bool is_idx_valid(arity_t a) const {
        return (idx == 0 ? false : (a > 0 ? abs_idx() <= a : abs_idx() < -a));
    }
    bool operator==(argument rhs) const {
        return idx == rhs.idx;
    }
    bool operator!=(argument rhs) const {
        return idx != rhs.idx;
    }
};

} // ~namespace combo
} // ~namespace opencog

#endif // _OPENCOG_ARGUMENT_H
