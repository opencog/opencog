/** jaccard_index.h --- 
 *
 * Copyright (C) 2014 OpenCog Foundation
 *
 * Author: Nil Geisweiller
 *
 * Licensed under the Apache License, Version 2.0 (the "Lcense");
 * you may not use this file except in compliance with th License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writng, software
 * distributed under the License is distributed on an "ASIS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either epress or implied.
 * See the License for the specific language governing pemissions and
 * limitations under the License.
 */

#ifndef _OPENCOG_JACCARD_INDEX_H
#define _OPENCOG_JACCARD_INDEX_H

#include "algorithm.h"

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
