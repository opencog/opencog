/** algo/random.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings LLC
 * Copyright (C) 2014 Aidyia Limited
 *
 * Author: Nil Geisweiller
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#include <boost/range/irange.hpp>

#include "random.h"

namespace opencog {
    
feature_set_pop random_select_feature_sets(const CTable& ctable,
                                           const feature_selection_parameters& fs_params)
{
    auto ir = boost::irange(0, ctable.get_arity());
    feature_set all_features(ir.begin(), ir.end());
    feature_set fs = random_selection(all_features,
                                      fs_params.target_size);
    feature_set_pop::value_type sfs(0, fs);
    return {sfs};
}

} // ~namespace opencog
