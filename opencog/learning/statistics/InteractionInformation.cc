/*
 * opencog/learning/statistics/InteractionInformation.cc
 *
 * Copyright (C) 2012 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Shujing Ke <rainkekekeke@gmail.com>
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

#include "InteractionInformation.h"

namespace opencog { namespace statistics {

bool isLastNElementsAllTrue(bool *array, long size, long n)
{
    for (long i = size - 1; i >= size - n; i--) {
        if (!array[i])
            return false;
    }

    return true;
}

void generateNextCombination(bool *indexes, long n_max)
{
    long trueCount = -1;
    long i = 0;
    for (; i < n_max - 1; ++i) {
        if (indexes[i]) {
            ++trueCount;

            if (!indexes[i + 1])
                break;
        }
    }

    indexes[i] = false;
    indexes[i + 1] = true;

    for (long j = 0; j < trueCount; ++j)
        indexes[j] = true;

    for (long j = trueCount; j < i; ++j)
        indexes[j] = false;

}

} // namespace statistics
} // namespace opencog