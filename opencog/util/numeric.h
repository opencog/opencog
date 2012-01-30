/*
 * opencog/util/numeric.h
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#ifndef _OPENCOG_NUMERIC_H
#define _OPENCOG_NUMERIC_H

#include <stdint.h>
#include <cmath>
#include <ctime>
#include <vector>
#include <cstdlib>
#include <climits>
#include <limits>
#include <numeric>
#include <set>
#include <map>

#include <boost/math/special_functions.hpp>

#include "exceptions.h"
#include "oc_assert.h"
#include "foreach.h"

#include "iostreamContainer.h"

#define PI 3.141592653589793
#define EXPONENTIAL 2.71828182845905

#ifdef WIN32
#include <numeric>
#else
#include <ext/numeric>
#endif

namespace opencog
{

using std::numeric_limits;
using boost::math::isnan;
using boost::math::isfinite;
using boost::math::isinf;
using boost::math::isnormal;

#ifndef WIN32
// This needs to be changed for non-gcc. Note however that so far it
// has been useless as most of the time you just need pow2 or sq
// defined below in that header
using __gnu_cxx::power;
using __gnu_cxx::iota;
#endif

const double EPSILON = 1e-6; // default error when comparing 2 floats
const double PROB_EPSILON = 1e-127; // error when comparing 2 probabilities

// absolute_value_order
//   codes the following order, for T == int, -1,1,-2,2,-3,3,...
template<typename T>
struct absolute_value_order {
    bool operator()(T a,T b) const {
        return (a == -b) ? a < b : std::abs(a) < std::abs(b);
    }
};
template<typename T>
struct absolute_value_equality {
    bool operator()(T a,T b) const {
        return (a == b || a == -b);
    }
};

// the following functions are adapted from the bit twiddling hacks page:
// http://graphics.stanford.edu/~seander/bithacks.html
// 32-bit version is faster, than the others
// but beware, you need to check the number of bits yourself
// awkward phrasing cuz c++ doesn't allow partial specialization of functions
//
// a few possibilities were tried; gcc has a built-in function called
// __builtin_popcount which is somewhat slower. Using a lookup table might
// be somwhat faster under some circumstances, but was avoided because it
// might hog the fast memory ...
template<int> struct bits {
    template<typename T>
    static inline unsigned int count(T v);
};
template<> struct bits<32> {
    template<typename T>
    static inline unsigned int count(T v) {
        v = v - ((v >> 1) & 0x55555555);                       // reuse v as temp
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);        // temp
        return ((v + (v >> 4) & 0xF0F0F0F) * 0x1010101) >> 24; // count
    }
    template<typename T>
    static inline void interleave(T& v) {
        static const unsigned int B[] = {0x55555555, 0x33333333, 0x0F0F0F0F, 0x00FF00FF};
        static const unsigned int S[] = {1, 2, 4, 8};

        T upperbits = (v >> 16);
        upperbits = (upperbits | (upperbits << S[3])) & B[3];
        upperbits = (upperbits | (upperbits << S[2])) & B[2];
        upperbits = (upperbits | (upperbits << S[1])) & B[1];
        upperbits = (upperbits | (upperbits << S[0])) & B[0];
        v &= 0xFFFF; //take the lower 16 bits
        v = (v | (v << S[3])) & B[3];
        v = (v | (v << S[2])) & B[2];
        v = (v | (v << S[1])) & B[1];
        v = (v | (v << S[0])) & B[0];
        v = v | (upperbits << 1);
    }
};
template<> struct bits<64> {
    template<typename T>
    static inline unsigned int count(T v) {
        v = v - ((v >> 1) & (T)~(T)0 / 3);                         // temp
        v = (v & (T)~(T)0 / 15 * 3) + ((v >> 2) & (T)~(T)0 / 15 * 3);      // temp
        v = (v + (v >> 4)) & (T)~(T)0 / 255 * 15;                  // temp
        return ((T)(v * ((T)~(T)0 / 255)) >> (sizeof(v) - 1) * CHAR_BIT);
    }
    template<typename T>
    static inline void interleave(T& v) {
        unsigned int tmp1 = ((v >> 48) << 16);
        tmp1 |= (v << 32) >> 48;
        bits<32>::interleave(tmp1);
        unsigned int tmp2 = ((v >> 16) & 0xFFFF0000);
        tmp2 |= (v & 0xFFFF);
        bits<32>::interleave(tmp2);
        v = (T(tmp1) << 32 | T(tmp2));
    }
};
template<> struct bits<128> {
    template<typename T>
    static inline unsigned int count(T v) {
        v = v - ((v >> 1) & (T)~(T)0 / 3);                         // temp
        v = (v & (T)~(T)0 / 15 * 3) + ((v >> 2) & (T)~(T)0 / 15 * 3);      // temp
        v = (v + (v >> 4)) & (T)~(T)0 / 255 * 15;                  // temp
        return ((T)(v * ((T)~(T)0 / 255)) >> (sizeof(v) - 1) * CHAR_BIT);
    }
};

// count_bits will work up to 128-bits
template<typename T>
inline unsigned int count_bits32(T v)
{
    v = v - ((v >> 1) & 0x55555555);                    // reuse input as temp
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);     // temp
    return ((v + (v >> 4) & 0xF0F0F0F) * 0x1010101) >> 24; // count
}
template<typename T>
inline unsigned int count_bits(T v)
{
    v = v - ((v >> 1) & (T)~(T)0 / 3);                         // temp
    v = (v & (T)~(T)0 / 15 * 3) + ((v >> 2) & (T)~(T)0 / 15 * 3);      // temp
    v = (v + (v >> 4)) & (T)~(T)0 / 255 * 15;                  // temp
    return ((T)(v * ((T)~(T)0 / 255)) >> (sizeof(v) - 1) * CHAR_BIT);
}

// return p the smallest power of 2 so that p >= x. So for instance:
//   next_power_of_two(1) = 1
//   next_power_of_two(2) = 2
//   next_power_of_two(3) = 4
inline size_t next_power_of_two(size_t x)
{
    OC_ASSERT(x > 0);
    x--;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    x++;
    return x;
}

inline unsigned int integer_log2(size_t v)
{
    static const int MultiplyDeBruijnBitPosition[32] = {
        0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
        31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
    };

    v |= v >> 1; // first round down to power of 2
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v = (v >> 1) + 1;
    return MultiplyDeBruijnBitPosition[static_cast<uint32_t>(v * 0x077CB531UL) >> 27];
}

template<typename FloatT> FloatT log2(FloatT x)
{
    return std::log(x) / std::log(static_cast<FloatT>(2));
}

// return the smaller exponent in base 2. This is used to know how
// many bits should be used to pack a certain number of values. So
// for instance
//    nbits_to_pack(2) = 1,
//    nbits_to_pack(3) = 2,
//    nbits_to_pack(50) = 8
inline unsigned int nbits_to_pack(size_t multy)
{
    OC_ASSERT(multy > 0);
    return next_power_of_two(integer_log2(multy -1) + 1);
}

//sums of natural logarithms (for a particular floating-point type)
template<typename FloatT>
FloatT logsum(size_t n)
{
    static std::vector<FloatT> sums;
    if (n >= sums.size()) {
        int to = next_power_of_two(n) + 2;
        sums.reserve(to);
        if (sums.empty()) {
            sums.push_back(0);
            sums.push_back(0);
        }

        FloatT v = sums.back();
        for (int i = sums.size();i < to;++i) {
            v += log(FloatT(i - 1));
            sums.push_back(v);
        }
    }
    return sums[n];
}

// returns true iff abs(x - y) <= epsilon
template<typename FloatT> bool isWithin(FloatT x, FloatT y, FloatT epsilon) {
    return std::abs(x - y) <= epsilon;
}

// compare 2 FloatT with precision epsilon,
// note that, unlike isWithin, the precision adapts with the scale of x and y
template<typename FloatT> bool isApproxEq(FloatT x, FloatT y, FloatT epsilon) {
    FloatT diff = std::abs(x - y);
    FloatT amp = std::abs(x + y);
    if (amp*amp > epsilon)
        return diff <= epsilon * amp;
    else return diff <= epsilon;
}

// compare 2 FloatT with precision EPSILON
// note that, unlike isWithin, the precision adapts with the scale of x and y
template<typename FloatT> bool isApproxEq(FloatT x, FloatT y)
{
    return isApproxEq(x, y, static_cast<FloatT>(EPSILON));
}

// useful for entropy
template<typename FloatT> FloatT weightInformation(FloatT p)
{
    return p > PROB_EPSILON? -p * opencog::log2(p) : 0;
}

// compute the binary entropy of probability p
template<typename FloatT> FloatT binaryEntropy(FloatT p)
{
    OC_ASSERT(p >= 0 && p <= 1,
              "probability %f is not between 0 and 1", p);
    return weightInformation(p) + weightInformation(1.0 - p);
}

/**
 * Compute entropy of a probability distribution described by (from, to[.
 * Specifically it computes
 *
 * - Sum_i p_i log2(p_i)
 * 
 * where the p_i are values pointed by (from, to[, and Sum_i p_i == 1.0
 */
template<typename It> double entropy(It from, It to) {
    double res = 0;
    for(; from != to; ++from)
        res += weightInformation(*from);
    return res;
}
// helper
template<typename C>
double entropy(const C& c) {
    return entropy(c.begin(), c.end());
}

// compute the smallest divisor of n
template<typename IntT> IntT smallest_divisor(IntT n) {
    OC_ASSERT(n > 0, "n must be supperior than 0");
    if(n<3)
        return n;
    else {
        bool found_divisor = false;
        IntT i = 2;
        for(; i*i <= n && !found_divisor; i++) {
            found_divisor = n%i==0;
        }
        if(found_divisor)
            return i-1;
        else return n;
    }
}

// calculate the square of x
template<typename T> T sq(T x) { return x*x; }

// check if x isn't too high and return 2^x
template<typename OutInt> OutInt pow2(unsigned int x) {
    OC_ASSERT(8*sizeof(OutInt) - (numeric_limits<OutInt>::is_signed?1:0) > x);
    return static_cast<OutInt>(1) << x;
}
inline unsigned int pow2(unsigned int x) { return pow2<unsigned int>(x); }

} // ~namespace opencog

#endif // _OPENCOG_NUMERIC_H
