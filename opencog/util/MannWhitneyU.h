/* MannWhitneyU.h --- 
 *
 * Copyright (C) 2012 Nil Geisweiller
 *
 * Author: Nil Geisweiller <nilg@desktop>
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

#ifndef _OPENCOG_MANNWHITNEYU_H
#define _OPENCOG_MANNWHITNEYU_H

#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/map.hpp>

#include <opencog/util/Counter.h>
#include <opencog/util/ranking.h>

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

using boost::adaptors::map_values;

/**
 * return the Mann-Whitney U (U_2 to be precise) given 2 distributions
 * described by Counters. If n2, the size of c2, is not provided it is
 * automatically calculated.
 */
template<typename Key, typename FloatT>
FloatT MannWhitneyU(const Counter<Key, FloatT>& c1,
                    const Counter<Key, FloatT>& c2,
                    FloatT n2 = -1) {
    typedef Counter<Key, FloatT> counter_t;
    counter_t c = c1 + c2;
    counter_t r = ranking(c);
    FloatT sum2 = 0;
    for (const auto& v : c2)
        sum2 += r[v.first];
    if (n2 < 0)
        n2 = boost::accumulate(c2 | map_values, 0);
    return sum2 - n2*(n2+1)/2;
}

/**
 * return the standardized Mann-Whitney U given 2 distributions
 * described by Counters
 */
template<typename Key, typename FloatT>
FloatT standardizedMannWhitneyU(const Counter<Key, FloatT>& c1,
                                const Counter<Key, FloatT>& c2,
                                FloatT n1 = -1, FloatT n2 = -1) {
    if(n1 < 0)
        n1 = boost::accumulate(c1 | map_values, 0);
    if(n2 < 0)
        n2 = boost::accumulate(c2 | map_values, 0);
    FloatT U = MannWhitneyU(c1, c2, n2),
        mU = n1*n2/2,
        sU = sqrt(mU*(n1+n2+1)/6);
    return (U - mU) / sU;
}

/** @}*/
} // ~namespace opencog

#endif // _OPENCOG_MANNWHITNEYU_H
