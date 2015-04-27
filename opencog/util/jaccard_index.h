/** jaccard_index.h --- 
 *
 * Copyright (C) 2014 OpenCog Foundation
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

#ifndef _OPENCOG_JACCARD_INDEX_H
#define _OPENCOG_JACCARD_INDEX_H

#include <opencog/util/algorithm.h>

namespace opencog {

/**
 * Calculate the Jaccard index (see
 * http://en.wikipedia.org/wiki/Jaccard_index) of 2 sts.
 */

template<typename Set>
float jaccardIndex(const Set& s1, const Set& s2) {
    return (float)set_intersection(s1, s2).size()
        / (float)set_union(s1, s2).size();
}

}

#endif // _OPENCOG_JACCARD_INDEX_H
