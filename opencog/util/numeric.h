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

#include <algorithm> // for std::max
#include <cmath>
#include <climits>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <vector>

#include <boost/range/numeric.hpp>

#include "exceptions.h"
#include "oc_assert.h"
#include "foreach.h"

#include "iostreamContainer.h"

/** \addtogroup grp_cogutil
 *  @{
 */

// These and many other constants are in <math.h>
#define PI          M_PI  // 3.1415926535897932384626433832795029L
#define EXPONENTIAL M_E   // 2.7182818284590452353602874713526625L

#ifdef WIN32
#include <numeric>
#else
#include <ext/numeric>
#endif

namespace opencog
{

using std::numeric_limits;
using std::isnan;
using std::isfinite;
using std::isinf;

using std::iota;

#ifndef WIN32
// This needs to be changed for non-gcc. Note however that so far it
// has been useless as most of the time you just need pow2 or sq
// defined below in that header
using __gnu_cxx::power;
#endif

const double EPSILON = 1e-6; // default error when comparing 2 floats
const double SMALL_EPSILON = 1e-32;
const double PROB_EPSILON = 1e-127; // error when comparing 2 probabilities

//! absolute_value_order
//!   codes the following order, for T == int, -1,1,-2,2,-3,3,...
template<typename T>
struct absolute_value_order
{
    bool operator()(T a,T b) const {
        return (a == -b) ? a < b : std::abs(a) < std::abs(b);
    }
};

template<typename T>
struct absolute_value_equality
{
    bool operator()(T a,T b) const {
        return (a == b || a == -b);
    }
};

/** @name Bithacks
 *
 * The following functions are adapted from the bit twiddling hacks page:
 * http://graphics.stanford.edu/~seander/bithacks.html
 * 32-bit version is faster, than the others
 * but beware, you need to check the number of bits yourself
 * awkward phrasing cuz c++ doesn't allow partial specialization of functions
 *
 * a few possibilities were tried; gcc has a built-in function called
 * __builtin_popcount which is somewhat slower. Using a lookup table might
 * be somwhat faster under some circumstances, but was avoided because it
 * might hog the fast memory ...
 */
///@{

template<int> struct bits
{
    template<typename T>
    static inline unsigned int count(T v);
};

template<> struct bits<32>
{
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

template<> struct bits<64>
{
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

template<> struct bits<128>
{
    template<typename T>
    static inline unsigned int count(T v) {
        v = v - ((v >> 1) & (T)~(T)0 / 3);                         // temp
        v = (v & (T)~(T)0 / 15 * 3) + ((v >> 2) & (T)~(T)0 / 15 * 3);      // temp
        v = (v + (v >> 4)) & (T)~(T)0 / 255 * 15;                  // temp
        return ((T)(v * ((T)~(T)0 / 255)) >> (sizeof(v) - 1) * CHAR_BIT);
    }
};

//! count_bits will work up to 128-bits
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

//! return p the smallest power of 2 so that p >= x
/// So for instance:
///   - next_power_of_two(1) = 1
///   - next_power_of_two(2) = 2
///   - next_power_of_two(3) = 4
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

//! return the smaller exponent in base 2. 
/// This is used to know how
/// many bits should be used to pack a certain number of values. So
/// for instance
///    - nbits_to_pack(2) = 1,
///    - nbits_to_pack(3) = 2,
///    - nbits_to_pack(50) = 8
inline unsigned int nbits_to_pack(size_t multy)
{
    OC_ASSERT(multy > 0);
    return next_power_of_two(integer_log2(multy -1) + 1);
}

//! sums of natural logarithms (for a particular floating-point type)
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

//! return the number of digits (in base 10 by default) of an integer
template<typename Int> Int ndigits(Int x, Int base = 10) {
    Int nd = 0;
    while (x != 0) { x /= base; nd++; }
    return nd;
}

//! returns true iff x >= min and x <= max
template<typename FloatT> bool isBetween(FloatT x, FloatT min_, FloatT max_)
{
    return x >= min_ && x <= max_;
}

//! returns true iff abs(x - y) <= epsilon
template<typename FloatT> bool isWithin(FloatT x, FloatT y, FloatT epsilon)
{
    return std::abs(x - y) <= epsilon;
}

//! compare 2 FloatT with precision epsilon,
/// note that, unlike isWithin, the precision adapts with the scale of x and y
template<typename FloatT> bool isApproxEq(FloatT x, FloatT y, FloatT epsilon)
{
    FloatT diff = std::abs(x - y);
    FloatT amp = std::abs(x + y);
    if (amp*amp > epsilon)
        return diff <= epsilon * amp;
    else return diff <= epsilon;
}

//! compare 2 FloatT with precision EPSILON
/// note that, unlike isWithin, the precision adapts with the scale of x and y
template<typename FloatT> bool isApproxEq(FloatT x, FloatT y)
{
    return isApproxEq(x, y, static_cast<FloatT>(EPSILON));
}

/**
 * Return x bounded by [l, u], that is it returns max(l, min(u, x))
 *
 * I'm not sure the name 'bound' is right...
 */
template<typename Float>
Float bound(Float x, Float l, Float u)
{
    return std::max(l, std::min(u, x));
}
    
//! useful for entropy
template<typename FloatT> FloatT weightInformation(FloatT p)
{
    return p > PROB_EPSILON? -p * opencog::log2(p) : 0;
}

//! compute the binary entropy of probability p
template<typename FloatT> FloatT binaryEntropy(FloatT p)
{
    OC_ASSERT(p >= 0 && p <= 1,
              "binaryEntropy: probability %f is not between 0 and 1", p);
    return weightInformation(p) + weightInformation(1.0 - p);
}

/**
 * Compute entropy of an n-ary probability distribution, with the
 * probabilities pointed at by iterators (from, to[.
 * Specifically it computes
 *
 * - Sum_i p_i log_2(p_i)
 * 
 * where the p_i are values pointed by (from, to[, 
 * It is assumed that Sum_i p_i == 1.0
 * That is, std::accumulate(from, to, 0) == 1.0
 */
template<typename It> double entropy(It from, It to)
{
    double res = 0;
    for(; from != to; ++from)
        res += weightInformation(*from);
    return res;
}

//! helper
template<typename C>
double entropy(const C& c)
{
    return entropy(c.begin(), c.end());
}

//! compute the smallest divisor of n
template<typename IntT> IntT smallest_divisor(IntT n)
{
    OC_ASSERT(n > 0, "smallest_divisor: n must be superior than 0");
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

//! calculate the square of x
template<typename T> T sq(T x) { return x*x; }

//! check if x isn't too high and return 2^x
template<typename OutInt> OutInt pow2(unsigned int x)
{
    OC_ASSERT(8*sizeof(OutInt) - (numeric_limits<OutInt>::is_signed?1:0) > x,
              "pow2: Amount to shift is out of range.");
    return static_cast<OutInt>(1) << x;
}
inline unsigned int pow2(unsigned int x) { return pow2<unsigned int>(x); }

//! Generalized mean http://en.wikipedia.org/wiki/Generalized_mean
template<typename It, typename Float>
Float generalized_mean(It from, It to, Float p = 1.0)
{
    Float pow_sum =
        std::accumulate(from, to, 0.0,
                        [&](Float l, Float r) { return l + pow(r, p); });
    return pow(pow_sum / std::distance(from, to), 1.0 / p);
}
template<typename C, typename Float>
Float generalized_mean(const C& c, Float p = 1.0)
{
    return generalized_mean(c.begin(), c.end(), p);
}

///@}


/// Compute the distance between two vectors, using the p-norm.  For
/// p=2, this is the usual Eucliden distance, and for p=1, this is the
/// Manhattan distance, and for p=0 or negative, this is the maximum
/// difference for one element.
template<typename Vec, typename Float>
Float p_norm_distance(const Vec& a, const Vec& b, Float p=1.0)
{
    OC_ASSERT (a.size() == b.size(),
               "Cannot compare unequal-sized vectors!  %d %d\n",
               a.size(), b.size());

    typename Vec::const_iterator ia = a.begin(), ib = b.begin();

    Float sum = 0.0;
    // Special case Manhattan distance.
    if (1.0 == p) {
        for (; ia != a.end(); ia++, ib++)
            sum += fabs (*ia - *ib);
        return sum;
    }
    // Special case Euclidean distance.
    if (2.0 == p) {
        for (; ia != a.end(); ia++, ib++)
            sum += sq (*ia - *ib);
        return sqrt(sum);
    }
    // Special case max difference
    if (0.0 >= p) {
        for (; ia != a.end(); ia++, ib++) {
            Float diff = fabs (*ia - *ib);
            if (sum < diff) sum = diff;
        }
        return sum;
    }

    // General case.
    for (; ia != a.end(); ia++, ib++) {
        Float diff = fabs (*ia - *ib);
        if (0.0 < diff)
            sum += pow(diff, p);
    }
    return pow(sum, 1.0/p);
}

/**
 * Return the Tanimoto distance, a continuous extension of the Jaccard
 * distance, between 2 vector.
 *
 * See http://en.wikipedia.org/wiki/Jaccard_index
 *
 * More specifically we use
 *
 * 1 - f(a,b)
 *
 * where
 *
 * f(a,b) = (sum_i a_i * b_i) / (sum_i a_i^2 + sum_i b_i^2 - sum_i a_i * b_i)
 *
 * If a and b are binary vectors then this corresponds to the Jaccard
 * distance. Otherwise, anything goes, it's not even a true metric
 * anymore, but it could be useful anyway.
 *
 * Warning: if a and b have negative elements then the result could be
 * negative. In that case you might prefer to use the angular
 * distance.
 *
 * Indeed the Tanimoto distance is considered as a generalization of
 * the Jaccard distance over multisets (where the weights are the
 * number of occurences, therefore never negative). See
 * http://en.wikipedia.org/wiki/Talk%3AJaccard_index
 */
template<typename Vec, typename Float>
Float tanimoto_distance(const Vec& a, const Vec& b)
{
    OC_ASSERT (a.size() == b.size(),
               "Cannot compare unequal-sized vectors!  %d %d\n",
               a.size(), b.size());

    Float ab = boost::inner_product(a, b, Float(0)),
        aa = boost::inner_product(a, a, Float(0)),
        bb = boost::inner_product(b, b, Float(0)),
        numerator = aa + bb - ab;

    if (numerator >= Float(SMALL_EPSILON))
        return 1 - (ab / numerator);
    else
        return 0;
}

/**
 * Return the angular distance between 2 vectors.
 *
 * See http://en.wikipedia.org/wiki/Cosine_similarity (Also, according
 * to Linas this is also equivalent to Fubini-Study metric, Fisher
 * information metric, and Kullbeck-Liebler divergence but they look
 * so differently so that anyone can hardly see it).
 *
 * Specifically it computes
 *
 * f(a,b) = alpha*cos^{-1}((sum_i a_i * b_i) /
 *                         (sqrt(sum_i a_i^2) * sqrt(sum_i b_i^2))) / pi
 *
 * with alpha = 1 where vector coefficients may be positive or negative
 * or   alpha = 2 where the vector coefficients are always positive.
 *
 * If pos_n_neg == true then alpha = 1, otherwise alpha = 2.
 */
template<typename Vec, typename Float>
Float angular_distance(const Vec& a, const Vec& b, bool pos_n_neg = true)
{
    OC_ASSERT (a.size() == b.size(),
               "Cannot compare unequal-sized vectors!  %d %d\n",
               a.size(), b.size());

    Float ab = boost::inner_product(a, b, Float(0)),
        aa = boost::inner_product(a, a, Float(0)),
        bb = boost::inner_product(b, b, Float(0)),
        numerator = sqrt(aa * bb);
    
    if (numerator >= Float(SMALL_EPSILON)) {
        // in case of rounding error
        Float r = bound(ab / numerator, Float(-1), Float(1));
        return (pos_n_neg ? 1 : 2) * acos(r) / PI;
    }
    else
        return 0;
}

} // ~namespace opencog
/** @}*/

#endif // _OPENCOG_NUMERIC_H
