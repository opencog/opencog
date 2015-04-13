/* random.h --- 
 *
 * Copyright (C) 2010 Novamente LLC
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


#ifndef _OPENCOG_RANDOM_H
#define _OPENCOG_RANDOM_H

#include <boost/numeric/conversion/cast.hpp>

#include "RandGen.h"
#include "mt19937ar.h"
#include "numeric.h"

/**
 * \file random.h
 *
 * This file contains a collection of random generators based on RandGen
 */

namespace opencog {
/** \addtogroup grp_cogutil
 *  @{
 */

//! Choose an element of the set s randomly, with uniform distribution.
//! \warning it is assumed that s is non-empty
template<typename T, typename C>
const T& randset(const std::set<T, C>& s, RandGen& rng = randGen())
{
    OC_ASSERT(!s.empty());
    return *std::next(s.begin(), rng.randint(s.size()));
}

//! Choose an element of the set s, with uniform distribution, 
//! and remove it
template<typename T>
T randset_erase(std::set<T>& s, RandGen& rng = randGen())
{
    OC_ASSERT(!s.empty());
    auto it = std::next(s.begin(), rng.randint(s.size()));
    T val = *it;
    s.erase(it);
    return val;
}

//! Return a random number sampled according to a Gaussian distribution.
//! If the number falls out of the range of T then it is automatically
//! truncated.
template<typename T>
T gaussian_rand(T mean, T std_dev, RandGen& rng = randGen())
{
    double val = mean + std_dev *
        std::sqrt(-2 * std::log(rng.randDoubleOneExcluded())) * 
        std::cos(2 * PI * rng.randDoubleOneExcluded());
    T res;
    try {
        res = boost::numeric_cast<T>(val);
    } catch(boost::numeric::positive_overflow&) {
        res = std::numeric_limits<T>::max();
    } catch(boost::numeric::negative_overflow&) {
        res = std::numeric_limits<T>::min();
    }
    return res;
}

//! linear biased random bool, b in [0,1] when b tends to 1 the result
//! tends to be true
static inline bool biased_randbool(float b, RandGen& rng = randGen())
{
    return b > rng.randfloat();
}

/** @}*/
} // ~namespace opencog

#endif // _OPENCOG_RANDOM_H
