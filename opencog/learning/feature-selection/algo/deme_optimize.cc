/** algo/deme_optimize.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
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

#include <opencog/learning/moses/optimization/optimization.h>

#include "deme_optimize.h"

namespace opencog {
    
using namespace std;

instance initial_instance(const feature_selection_parameters& fs_params,
                          const field_set& fields,
                          const vector<string>& ilabels)
{
    feature_set init_features = initial_features(ilabels, fs_params);
    instance res(fields.packed_width());
    for (size_t idx : init_features)
        *(fields.begin_bit(res) + idx) = true;
    return res;
}


} // ~namespace opencog
