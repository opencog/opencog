/** scorers/moses_optim.cc --- 
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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

#include "moses_optim.h"

namespace opencog {

std::set<arity_t> get_feature_set(const field_set& fields,
                                  const instance& inst)
{
    std::set<arity_t> fs;
    field_set::const_bit_iterator bit = fields.begin_bit(inst);
    for(arity_t i = 0; bit != fields.end_bit(inst); ++bit, ++i)
        if(*bit)
            fs.insert(i);
    return fs;
}

} // ~namespace opencog
