/*
 * opencog/learning/moses/eda/scoring_functions.h
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
#ifndef _EDA_SCORING_FUNCTIONS_H
#define _EDA_SCORING_FUNCTIONS_H

#include <cmath>
#include <boost/lexical_cast.hpp>

#include <opencog/util/exceptions.h>
#include <opencog/util/numeric.h>
#include <opencog/util/RandGen.h>
#include <opencog/util/oc_assert.h>

#include "../representation/field_set.h"

namespace opencog { 
namespace moses {

struct one_max : public unary_function<instance, int> {
    int operator()(const instance& inst) const {
        //operates directly on packed_t
        return accumulate
               (make_transform_iterator(inst.begin(),
                                        count_bits<packed_t>),
                make_transform_iterator(inst.end(),
                                        count_bits<packed_t>), 0);
    }
};

struct n_max : public unary_function<instance, int> {
    n_max(const field_set& fs) : fields(fs) {}
    int operator()(const instance& inst) const {
        return accumulate(fields.begin_disc(inst), fields.end_disc(inst), 0);
    }
    const field_set& fields;
};

struct contin_max : public unary_function<instance, contin_t> {
    contin_max(const field_set& fs) : fields(fs) {}
    contin_t operator()(const instance& inst) const {
        return accumulate(fields.begin_contin(inst), fields.end_contin(inst),
                          contin_t(0));
    }
    const field_set& fields;
};

struct contin_uniform : public unary_function<instance, contin_t> {
    contin_uniform(const field_set& fs, contin_t minval, contin_t maxval,
                   RandGen& rng)
        : fields(fs), target(fs.n_contin()) {
        generate(target.begin(), target.end(),
                 bind(std::plus<contin_t>(),
                      bind(std::multiplies<contin_t>(),
                           bind(&RandGen::randdouble, ref(rng)),
                           maxval - minval), minval));
    }
    contin_t operator()(const instance& inst) const {
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

struct sphere : public unary_function<instance, contin_t> {
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

struct ontomax: public unary_function<instance, contin_t> {
    ontomax(const field_set& fs) : fields(fs) {}
    contin_t operator()(const instance& inst) const {
        contin_t res = 0;
        for (field_set::const_onto_iterator it = fields.begin_onto(inst);
                it != fields.end_onto(inst);++it) {
            onto_t s = *it;
            OC_ASSERT(s.length() == 2,
                             "onto_t length should be equals to two");
            int a = boost::lexical_cast<int>(s[0]);
            int b = boost::lexical_cast<int>(s[1]);
            res += a + b;
        }
        return res;
    }
    const field_set& fields;
};

} // ~namespace moses
} // ~namespace opencog

#endif
