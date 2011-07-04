/*
 * opencog/learning/moses/eda/replacement.h
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
#ifndef _EDA_REPLACE_H
#define _EDA_REPLACE_H

#include <opencog/util/exceptions.h>
#include <opencog/util/lazy_random_selector.h>

#include "using.h"
#include "field_set.h"
#include "scoring.h"

namespace opencog { 
namespace eda {

//note that NewInst only models InputIterator

struct replace_the_worst {
    template<typename NewInst, typename Dst>
    void operator()(NewInst from,NewInst to, Dst from_dst, Dst to_dst) const {
        OC_ASSERT(
                         distance(from, to) <= distance(from_dst, to_dst),
                         "Distance from -> to greater than distance from_dst -> to_dst.");
        nth_element(from_dst, from_dst + distance(from, to), to_dst);
        copy(from, to, from_dst);
    }
};


struct rtr_replacement {
    rtr_replacement(const field_set& fs, int ws, RandGen& _rng)
        : window_size(ws), _fields(&fs), rng(_rng) { }

    template<typename NewInst, typename Dst>
    void operator()(NewInst from,NewInst to, Dst from_dst, Dst to_dst) const {
        OC_ASSERT(window_size <= distance(from_dst, to_dst),
                         "windows size greater than distance from_dst -> to_dst.");

        for (;from != to;++from)
            operator()(*from, from_dst, to_dst);
    }

    template<typename Dst, typename ScoreT>
    void operator()(const scored_instance<ScoreT>& inst,
                    Dst from_dst, Dst to_dst) const {
        lazy_random_selector select(distance(from_dst, to_dst), rng);
        Dst closest = from_dst + select();
        int closest_distance = _fields->hamming_distance(inst, *closest);
        for (int i = 1;i < window_size;++i) {
            Dst candidate = from_dst + select();
            int distance = _fields->hamming_distance(inst, *candidate);
            if (distance < closest_distance) {
                closest_distance = distance;
                closest = candidate;
            }
        }
        if (*closest < inst)
            *closest = inst;
    }

    int window_size;
protected:
    const field_set* _fields;
    RandGen& rng;
};

} // ~namespace eda
} // ~namespace opencog

#endif
