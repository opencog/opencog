/*
 * opencog/learning/moses/example-progs/scoring_functions.h
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
#ifndef _EXAMPLE_SCORING_FUNCTIONS_H
#define _EXAMPLE_SCORING_FUNCTIONS_H

#include <cmath>
#include <boost/lexical_cast.hpp>

#include <opencog/util/exceptions.h>
#include <opencog/util/numeric.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/oc_assert.h>

#include "../representation/field_set.h"

using namespace opencog;
using namespace moses;

// Example scoring functions.
//
// These scoring functions all implement "toy problems" that typical
// optimization algorithms are expected to do well in solving. Thus,
// for example, "one_max" just counts the number of bits set in a
// bit-string.
//
// These functions take an instance as an argument, and score that
// instance for fitness.  Recall that an "instance" is a string of bits
// that encode a set of knob settings; an instance may encode discrete,
// continuous, or string variables.
//
// Recall that the C++ std::unary_fuinction<> template is jus a trick
// to make a C++ class behave as if it were a function, so that it can
// be used anywhere a function is used.


// Return, as the score, the total number of bits set in the instance.
struct one_max : public unary_function<instance, int>
{
    int operator()(const instance& inst) const
    {
        // This operates directly on the packed_t of the instance.
        //
        // boost::make_transform_iterator is a kind of pullback or
        // pushforward kind of thing, it creates a new iterator, such
        // that the new iterator applies the function count_bits() when
        // the iterator is dereferences.  Not sure, but I think
        // make_transform_iterator is kind-of-like a "monad functor".
        return accumulate
               (make_transform_iterator(inst.begin(),
                                        count_bits<packed_t>),
                make_transform_iterator(inst.end(),
                                        count_bits<packed_t>), 0);
    }
};


// Return, as the score, the sum total settings of all discrete knob
// settings in the instance.
struct n_max : public unary_function<instance, int>
{
    n_max(const field_set& fs) : fields(fs) {}
    int operator()(const instance& inst) const
    {
        return accumulate(fields.begin_disc(inst), fields.end_disc(inst), 0);
    }
    const field_set& fields;
};

// Return, as the score, the sum total of all continuous knob settings
// in the instance.
struct contin_max : public unary_function<instance, contin_t>
{
    contin_max(const field_set& fs) : fields(fs) {}
    contin_t operator()(const instance& inst) const
    {
        return accumulate(fields.begin_contin(inst), fields.end_contin(inst),
                          contin_t(0));
    }
    const field_set& fields;
};

// Return, as the score, the absolute difference (the "lp_1" distance)
// between the instance, and a vector of fixed, randomly generated values.
//
// That is, a vector of continuous values, bounded between a min and max,
// are randomly generated. The values of the continuous variables in the
// instance are compared to these random values, taking the absolute value,
// and summed over, thus returning the "lp_1" distance between the instance,
// and the random vector.
//
struct contin_uniform : public unary_function<instance, contin_t>
{
    contin_uniform(const field_set& fs, contin_t minval, contin_t maxval)
        : fields(fs), target(fs.n_contin_fields())
    {
        generate(target.begin(), target.end(),
                 bind(std::plus<contin_t>(),
                      bind(std::multiplies<contin_t>(),
                           bind(&RandGen::randdouble, boost::ref(randGen())),
                           maxval - minval), minval));
    }

    contin_t operator()(const instance& inst) const
    {
        contin_t res = 0;
        field_set::const_contin_iterator it1 = fields.begin_contin(inst);
        for (vector<contin_t>::const_iterator it2 = target.begin();
                it2 != target.end();++it1, ++it2)
            res -= fabs((*it1) - (*it2));
        return res;
    }
    const field_set& fields;
    vector<contin_t> target;
};

// Return, as the score, minus the sum of the squares of all
// continuous knob settings in the instance.
//
struct sphere : public unary_function<instance, contin_t>
{
    sphere(const field_set& fs) : fields(fs) {}
    contin_t operator()(const instance& inst) const {
        contin_t res = 0;
        for (field_set::const_contin_iterator it = fields.begin_contin(inst);
                it != fields.end_contin(inst);++it) {
            contin_t v = *it;
            res -= (v * v);
        }
        return res;
    }
    const field_set& fields;
};

// Return, as the score, the sum of pairs of numbers organized into
// terms.
//
// Recall that an "term variable" is a knob whose values are trees
// of arbitrary strings. In this case, this function insists that all
// these strings are exactly two characters long, and that each
// character is an ASCII digit. This scoring function then goes over
// all term knobs in the instance, pulls out these two digits, and
// adds them together.  The sum of all of these is the returned score.
struct termmax: public unary_function<instance, contin_t>
{
    termmax(const field_set& fs) : fields(fs) {}
    contin_t operator()(const instance& inst) const
    {
        contin_t res = 0;
        for (field_set::const_term_iterator it = fields.begin_term(inst);
                it != fields.end_term(inst);++it) {
            term_t s = *it;
            OC_ASSERT(s.length() == 2,
                             "term_t length should be equals to two");
            int a = boost::lexical_cast<int>(s[0]);
            int b = boost::lexical_cast<int>(s[1]);
            res += a + b;
        }
        return res;
    }
    const field_set& fields;
};

#endif
