/*
 * opencog/learning/statistics/InteractionInformation.h
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

#ifndef _OPENCOG_STATISTICS_INTERACTION_INFORMATION_H
#define _OPENCOG_STATISTICS_INTERACTION_INFORMATION_H

#include <string>
#include <vector>
#include <map>
#include <iterator>
#include "DataProvider.h"

namespace opencog { namespace statistics {

 void sortIntVector(std::vector<int> &v, bool isDec = true);
 bool isLastNElementsAllTrue(bool* array, long size, int n);
 void generateNextCombination(bool *indexes, long n_max);
 std::string makeKeyStringFromIndexes(
         std::vector<int> &onePieceOfData, bool* array, long size);
 std::vector<int> makeAddressVectorFromString(
         std::string str, char splitter = '-');

class InteractionInformation
{
public:
 template<typename Metadata>
 inline static float calculateInteractionInformation(
         std::vector<int> onePieceOfData, DataProvider<Metadata>* data)
 {
     if (! data->isOrderDependent)
         sortIntVector(onePieceOfData);

     long n_max = onePieceOfData.size();
     bool sign = true;
     float interactionInfo = 0.0f;
     float tmpSum;
     std::map<std::string,StatisticData>::iterator it;
     std::string key;

     bool indexes[n_max];

     for (int n_gram = 1; n_gram <= n_max; n_gram ++)
     {
         tmpSum = 0.0f;

         // Use the binary method to generate all combinations:

         // generate the first combination
         for (int i = 0; i < n_gram; ++ i)
             indexes[i] = true;

         for (int i = n_gram; i < n_max; ++ i)
             indexes[i] = false;

         while (true)
         {
             key = makeKeyStringFromIndexes(onePieceOfData, indexes, n_max);

             it = data->mDataMaps[n_gram].find(key);

             if (it != data->mDataMaps[n_gram].end())
             {
                 StatisticData& pieceData = it->second;
                 tmpSum += pieceData.entropy;
             }

             if (isLastNElementsAllTrue(indexes, n_max, n_gram))
                 break;

             generateNextCombination(indexes, n_max);

         }

         if (sign)
             interactionInfo += tmpSum;
         else
             interactionInfo -= tmpSum;

         sign = !sign;
     }

     return interactionInfo;
 }

 template<typename Metadata>
 inline static float calculateInteractionInformation(
         std::string onePieceOfData, DataProvider<Metadata>* data)
 {
     return calculateInteractionInformation(
             makeAddressVectorFromString(onePieceOfData), data
     );
 }

 template<typename Metadata>
 inline static void calculateInteractionInformations(
         DataProvider<Metadata>* data)
 {
     std::map<std::string,StatisticData>::iterator it;

     for (int n = 1; n <= data->n_gram; ++n )
     {
         for( it = data->mDataMaps[n].begin(); it != data->mDataMaps[n].end(); ++it)
         {
             StatisticData& pieceData = it->second;
             pieceData.interactionInformation = calculateInteractionInformation(
                             (std::string)(it->first), data);
         }

     }
 }
};

} // namespace statistics
} // namespace opencog

#endif //_OPENCOG_STATISTICS_INTERACTION_INFORMATION_H
