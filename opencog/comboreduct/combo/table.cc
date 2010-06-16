/** table.cc --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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

#include "table.h"
#include <opencog/util/numeric.h>

using namespace combo;
using opencog::sqr;

truth_table::size_type
truth_table::hamming_distance(const truth_table& other) const
{
    OC_ASSERT(other.size() == size(),
                      "truth_tables size should be the same.");

    size_type res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += (*x++ != *y++);
    return res;
}

RndNumTable::RndNumTable(int sample_count, int arity, opencog::RandGen& rng, 
                         double max_randvalue , double min_randvalue )
{
    //populate the matrix
    for (int i = 0; i < sample_count; ++i) {
        contin_vector cv;
        for (int j = 0; j < arity; ++j)
        //   cv.push_back(rng.randdouble()*2.0 - 1.0); //TODO : rescale wrt
        cv.push_back((max_randvalue - min_randvalue) * rng.randdouble() + min_randvalue); 
        // input interval
        push_back(cv);
    }
}

contin_table::contin_table(const combo_tree& t, const RndNumTable& rnt,
                           opencog::RandGen& rng)
{
    for (const_cm_it i = rnt.begin(); i != rnt.end(); ++i) {
        int arg = 1;
        for (const_cv_it j = (*i).begin(); j != (*i).end(); ++j, ++arg)
            binding(arg) = *j;
        //assumption : all inputs of t are contin_t
        vertex res = eval_throws(rng, t);
        OC_ASSERT(is_contin(res), "res must be contin");
        push_back(get_contin(res));
    }
}

bool contin_table::operator==(const contin_table& ct) const
{
    if (ct.size() == size()) {
        const_cv_it ct_i = ct.begin();
        for (const_cv_it i = begin(); i != end(); ++i, ++ct_i) {
            if (!isApproxEq(*i, *ct_i))
                return false;
        }
        return true;
    } else return false;
}


contin_t contin_table::abs_distance(const contin_table& other) const
{
    OC_ASSERT(other.size() == size(),
              "contin_tables should have the same size.");

    contin_t res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += fabs(*x++ -*y++);
    return res;
}

contin_t contin_table::sum_squared_error(const contin_table& other) const
{
    OC_ASSERT(other.size() == size(),
              "contin_tables should have the same size.");

    contin_t res = 0;
    for (const_iterator x = begin(), y = other.begin();x != end();)
        res += sqr(*x++ -*y++);
    return res;
}
