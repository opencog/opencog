/*
 * opencog/learning/moses/example-progs/scoring_iterators.h
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
#ifndef _MOSES_SCORING_ITERATORS_H
#define _MOSES_SCORING_ITERATORS_H

#include <opencog/util/dorepeat.h>
#include <opencog/util/numeric.h> // needed for pow2
#include <opencog/comboreduct/combo/vertex.h> // needed for contin_t

namespace opencog { namespace moses {

using namespace opencog::combo;

// Base functor for functors taking an iterator range of value type and result T
template<typename T>
struct iterator_function {
    typedef T argument_type;
    typedef T result_type;
};
        
// even_parity(x1, ..., xn) = true iff (int)x1 + ... + (int)xn is even
// where n is the arity of even_parity
struct even_parity : public iterator_function<bool>
{
    // [from, to( corresponds to the sequence of inputs of the
    // function, the result corresponds to its output
    template<typename It>
    bool operator()(It from,It to) const {
        bool parity = true;
        while (from != to)
            parity ^= *from++;
        return parity;
    }
};

// disjunction(x1, ..., xn) = true iff there exists i such that xi is true
struct disjunction : public iterator_function<bool>
{
    // [from, to( corresponds to the sequence of inputs of the
    // function, the result corresponds to its output
    template<typename It>
    bool operator()(It from,It to) const {
        while (from != to)
            if (*from++)
                return true;
        return false;
    }
};

// multiplex(a1, ..., an, d1, ..., dm) = 1 iff m = 2^n and di = 1 if i
// is the address of the string bit described by a1, ..., an.
struct multiplex  : public iterator_function<bool>
{
    multiplex(unsigned int n) : arity(n) { }
    unsigned int arity;
    // [from, to( corresponds to the sequence of inputs of the
    // function, the result corresponds to its output
    template<typename It>
    bool operator()(It from, It to) const {
        // calculate address
        unsigned int addr = 0;
        for(unsigned int i = 0; i < arity; ++i)
            if(*from++)
                addr += pow2(i);
        // return the input corresonding to that address
        return *(from+addr);
    }
};

// majority(x1, ..., xn) = 0 iff n/2 or more arguments are false
struct majority : public iterator_function<bool>
{
    majority(unsigned int n) : arity(n) { }
    unsigned int arity;
    // [from, to( corresponds to the sequence of inputs of the
    // function, the result corresponds to its output
    template<typename It>
    bool operator()(It from, It to) const {
        return (unsigned int)std::count(from, to, true) > arity / 2;
    }
};

// simple function : f(x)_o = sum_{i={1,o}} x^i
// that is for instance:
// f(x)_3 = x+x^2+x^3
// f(x)_2 = x+x^2
// f(x)_1 = x
// f(x)_0 = 0
struct simple_symbolic_regression : public iterator_function<contin_t>
{
    simple_symbolic_regression(int o = 4) : order(o) { }
    int order;
    template<typename It>
    contin_t operator()(It from, It to) const {
        contin_t res = 0;
        dorepeat(order)
            res = (res + 1) * get_contin(*from);
        return res;
    }
};

} // ~namespace moses
} // ~namespace opencog

#endif
