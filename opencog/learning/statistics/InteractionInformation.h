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

 bool isLastNElementsAllTrue(bool* array, long size, long n);
 void generateNextCombination(bool *indexes, long n_max);

class InteractionInformation
{
public:
 template<typename Metadata>
 inline static float calculateInteractionInformation(
         std::vector<Metadata> &onePieceOfData,
         DataProvider<Metadata> &provider)
 {
     return calculateInteractionInformation(onePieceOfData, provider, -1);
 }

 template<typename Metadata>
 inline static float calculateInteractionInformation(
         std::vector<Metadata> &onePieceOfData,
         DataProvider<Metadata> &provider,
         long n_max_limit)
 {
     if (!provider.isOrderDependent)
         std::sort(onePieceOfData.begin(), onePieceOfData.end());

     long n_max = n_max_limit;
     if(n_max == -1)
         n_max = onePieceOfData.size();

     bool sign = true;
     float interactionInfo = 0.0f;
     float tmpSum;
     std::map<std::vector<long>, StatisticData>::iterator it;

     bool indexes[n_max];

     for (long n_gram = 1; n_gram <= n_max; n_gram ++)
     {
         tmpSum = 0.0f;

         // Use the binary method to generate all combinations:

         // generate the first combination
         for (long i = 0; i < n_gram; ++ i)
             indexes[i] = true;

         for (long i = n_gram; i < n_max; ++ i)
             indexes[i] = false;

         while (true)
         {
             std::vector<long> key = provider.makeKeyFromData(indexes, onePieceOfData);
             it = provider.mDataMaps[n_gram].find(key);
             if (it != provider.mDataMaps[n_gram].end())
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
 inline static float calculateInteractionInformationFromKey(
         std::vector<long> &onePieceOfData, DataProvider<Metadata> &provider)
 {
     return calculateInteractionInformationFromKey(onePieceOfData, provider, -1);
 }

 template<typename Metadata>
 inline static float calculateInteractionInformationFromKey(
         std::vector<long> &onePieceOfData,
         DataProvider<Metadata> &provider,
         long n_max_limit)
 {
     std::vector<Metadata> data = provider.makeDataFromKey(onePieceOfData);
     return calculateInteractionInformation(data, provider, n_max_limit);
 }

 template<typename Metadata>
 inline static void calculateInteractionInformations(
         DataProvider<Metadata> &provider)
 {
     std::map<std::vector<long> ,StatisticData>::iterator it;

     for (long n = 1; n <= provider.n_gram; ++n )
     {
         for(it = provider.mDataMaps[n].begin();
             it != provider.mDataMaps[n].end();
             ++it)
         {
             StatisticData& pieceData = it->second;
             std::vector<long> onePieceOfData = (std::vector<long>)it->first;
             pieceData.interactionInformation =
                     calculateInteractionInformationFromKey(
                             onePieceOfData, provider, -1
                     );
         }

     }
 }
};

} // namespace statistics
} // namespace opencog

#endif //_OPENCOG_STATISTICS_INTERACTION_INFORMATION_H
