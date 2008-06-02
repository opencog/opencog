/*
 * src/Util/RandGen.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#ifndef _RAND_GEN_H_
#define _RAND_GEN_H_

#include <set>
#include "exceptions.h"

namespace opencog
{

class RandGen
{

public:

    virtual ~RandGen() {}

    // random int between 0 and max rand number.
    virtual int randint() = 0;

    //random float in [0,1]
    virtual float randfloat();

    //random double in [0,1]
    virtual double randdouble();

    //random double in [0,1)
    virtual double randDoubleOneExcluded();

    //random int in [0,n)
    virtual int randint(int n);

    // return -1 or 1 randonly
    virtual int randPositiveNegative();

    // Random int from a gaussian distribution. Neg numbers are clipped to 0
    virtual unsigned int pos_gaussian_rand(unsigned int std_dev, unsigned int mean);

    //random boolean
    virtual bool randbool();

};

//choose uniformly randomly an element of the set s
//WARNING : it is assumed that s is non-empty
template<typename T> T randset(const std::set<T>& s, RandGen& rng)
{
    cassert(TRACE_INFO, !s.empty(), "numeric - std::set should be empty.");
    int chosen_int = rng.randint(s.size());
    typename std::set<T>::const_iterator s_it = s.begin();
    //can be optimized maybe
    for (int si = 0; si < chosen_int; si++) ++s_it;
    return *s_it;
}

} // namespace

#endif /* _RAND_GEN_H_ */
