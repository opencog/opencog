/*
 * opencog/learning/statistics/Probability.h
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

#ifndef _OPENCOG_STATISTICS_PROBABILITY_H
#define _OPENCOG_STATISTICS_PROBABILITY_H

#include <map>
#include <iterator>
#include "DataProvider.h"

namespace opencog { namespace statistics {

class Probability
{
public:
 template<typename Metadata>
 inline static void calculateProbabilities(DataProvider<Metadata> &provider)
 {
     std::map<std::vector<long>, StatisticData>::iterator it;

     for (long n = 1; n <= provider.n_gram; ++n) {
         for (it = provider.mDataMaps[n].begin();
              it != provider.mDataMaps[n].end(); ++it) {
             StatisticData &pieceData = it->second;
             pieceData.probability =
                     pieceData.count /
                     (static_cast<float> (provider.mRawDataNumbers[n]));
         }
     }
 }
};

} // namespace statistics
} // namespace opencog

#endif // _OPENCOG_STATISTICS_PROBABILITY_H
