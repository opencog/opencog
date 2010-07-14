/** random.h --- 
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

#include "RandGen.h"
#include <opencog/util/numeric.h>

/**
 * This file contains a collection of random generators based on RandGen
 */

namespace opencog {

//choose uniformly randomly an element of the set s
//WARNING : it is assumed that s is non-empty
template<typename T> T randset(const std::set<T>& s, RandGen& rng)
{
    OC_ASSERT(!s.empty(), "numeric - std::set should be empty.");
    int chosen_int = rng.randint(s.size());
    typename std::set<T>::const_iterator s_it = s.begin();
    //can be optimized maybe
    for (int si = 0; si < chosen_int; si++)
        ++s_it;
    return *s_it;
}

double gaussian_rand(double std_dev, double mean, RandGen& rng) {
    double res = mean + std_dev *
        std::sqrt(-2 * std::log(rng.randDoubleOneExcluded())) * 
        std::cos(2 * PI * rng.randDoubleOneExcluded());
    return res;
}

//linear biased random bool, b in [0,1]
//when b tends to 1 the result tends to be true
bool biased_randbool(float b, RandGen& rng) {
    return b > rng.randfloat();
}

} // ~namespace opencog

#endif // _OPENCOG_RANDOM_H
