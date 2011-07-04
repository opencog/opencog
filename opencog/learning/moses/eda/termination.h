/*
 * opencog/learning/moses/eda/termination.h
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
#ifndef _EDA_TERMINATION_H
#define _EDA_TERMINATION_H

#include <algorithm>
#include <limits>

namespace opencog { 
namespace eda {

template<typename ScoreT>
struct terminate_if_gte {
    terminate_if_gte(const ScoreT& b) : bound(b) { }

    template<typename It>
    bool operator()(It from,It to) const {
        return (*std::max_element(from, to) >= bound);
    }
protected:
    ScoreT bound;
};

template<typename ScoreT>
struct terminate_if_gte_or_no_improv {
    terminate_if_gte_or_no_improv(const ScoreT& b, int n) :
            bound(b), n_gen(n), at(-1) { }

    template<typename It>
    bool operator()(It from,It to) const {
        ScoreT res = std::max_element(from, to)->second;
        if (res > best_seen || at == -1) {
            best_seen = res;
            at = 0;
            return (res >= bound);
        }
        return (at++ > n_gen || best_seen >= bound);
    }
protected:
    ScoreT bound;
    mutable ScoreT best_seen;
    int n_gen;
    mutable int at;
};

} // ~namespace eda
} // ~namespace opencog

#endif
