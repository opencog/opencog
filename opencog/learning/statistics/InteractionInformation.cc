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

#include <sstream>
#include <boost/lexical_cast.hpp>
#include "InteractionInformation.h"

namespace opencog { namespace statistics {

void sortIntVector(std::vector<int> &v, bool isDec)
{
    int tmp;
    long n = v.size();
    for (int i = 0; i < n; ++i)
        for (int j = i; j < n - 1; ++j) {
            if ((isDec && (v[j] > v[j + 1])) ||
                ((!isDec) && (v[j] < v[j + 1]))) {
                tmp = v[j];
                v[j] = v[j + 1];
                v[j + 1] = tmp;
            }
        }

}

bool isLastNElementsAllTrue(bool *array, long size, int n)
{
    for (long i = size - 1; i >= size - n; i--) {
        if (!array[i])
            return false;
    }

    return true;
}

void generateNextCombination(bool *indexes, long n_max)
{
    int trueCount = -1;
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

    for (int j = 0; j < trueCount; ++j)
        indexes[j] = true;

    for (int j = trueCount; j < i; ++j)
        indexes[j] = false;

}

std::string makeKeyStringFromIndexes(
        std::vector<int> &onePieceOfData, bool *array, long size)
{
    std::string key = "";
    for (int i = 0; i < size; ++i) {
        if (array[i]) {
            if (key != "")
                key += "-";

            char buf[16];
            sprintf(buf, "%d", onePieceOfData[i]);
            key += std::string(buf);
        }
    }

    return key;
}

std::vector<int> makeAddressVectorFromString(std::string str, char splitter)
{
    std::vector<int> result;
    std::stringstream ss(str);
    std::string sub_str;
    while (getline(ss, sub_str, splitter))
        result.push_back(boost::lexical_cast<int>(sub_str));

    return result;
}

} // namespace statistics
} // namespace opencog