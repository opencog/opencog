/** metapop_params.cc ---
 *
 * Copyright (C) 2013 OpenCog Foundation
 *
 * Author: Nil Geisweiller
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

#include "metapop_params.h"

namespace opencog { namespace moses {

diversity_parameters::diversity_parameters()
    : pressure(0.0),
      exponent(-1.0),       // max
      normalize(true)       // sum or mean (default mean)
{
    set_dst(p_norm, 2.0 /* Euclidean */);
    set_dst2dp(inverse);
}

void diversity_parameters::set_dst(diversity_parameters::dst_enum_t de,
                                   diversity_parameters::dp_t p)
{
    switch(de) {
    case p_norm:
        dst = [p](const std::vector<score_t>& lhs,
                  const std::vector<score_t>& rhs) {
            return p_norm_distance(lhs, rhs, p);
        };
        break;
    case tanimoto:
        dst = [](const std::vector<score_t>& lhs,
                 const std::vector<score_t>& rhs) {
            return tanimoto_distance<std::vector<score_t>, dp_t>(lhs, rhs);
        };
        break;
    case angular:
        dst = [](const std::vector<score_t>& lhs,
                 const std::vector<score_t>& rhs) {
            return angular_distance<std::vector<score_t>, dp_t>(lhs, rhs);
        };
        break;
    default:
        OC_ASSERT(false);
    }
}

void diversity_parameters::set_dst2dp(diversity_parameters::dst2dp_enum_t d2de)
{
    dst2dp_type = d2de;
    switch(d2de) {
    case inverse:
        dst2dp = [&](dp_t dst) { return pressure / (1 + dst); };
        break;
    case complement:
        dst2dp = [&](dp_t dst) { return pressure * (1 - dst); };
        break;
    case pthpower:
        dst2dp = [&](dp_t dst) { return pow(dst, pressure); };
        break;
    default: {
        std::stringstream ss;
        ss << "diversity_parameters::set_dst2dp error: no case for " << d2de;
        OC_ASSERT(false, ss.str());
    }
    }
}

} // ~namespace moses
} // ~namespace opencog
