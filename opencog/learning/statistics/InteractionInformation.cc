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
#include <map>
#include <iterator>
#include <math.h>
#include <stdio.h>
#include <sstream>


using namespace opencog::statistics;
using namespace std;

void sortIntVector(vector<int> &v, bool isDec = true)
{
    int tmp;
    int n = v.size();
    for (int i = 0; i < n; ++i)
        for (int j = i; j < n-1; ++j)
    {
        if ((isDec && (v[j] > v[j+1])) || ((!isDec) && (v[j] < v[j+1])) )
        {
            tmp = v[j];
            v[j] = v[j+1];
            v[j+1] = tmp;
        }
    }

}

bool isLastNElementsAllTrue(bool* array, int size, int n)
{
    for (int i = size - 1; i >= size - n; i --)
    {
        if (! array[i])
            return false;
    }

    return true;
}

void generateNextCombination(bool *&indexes, int n_max)
{
    int trueCount = -1;
    int i = 0;
    for (; i < n_max - 1; ++ i)
    {
        if (indexes[i]   )
        {
            ++ trueCount;

            if (! indexes[i+1])
                break;
        }
    }

    indexes[i] = false;
    indexes[i+1] = true;

    for (int j = 0; j < trueCount; ++ j)
        indexes[j] = true;

    for (int j = trueCount; j < i; ++ j)
        indexes[j] = false;

}

string makeKeyStringFromIndexes(vector<int> &onePieceOfData, bool* array, int size)
{
    string key = "";
    for (int i = 0; i < size; ++ i)
    {
        if (array[i])
        {
            if (key != "")
                key += "-";

            char buf[8];
            sprintf(buf,"%d",onePieceOfData[i]);
            key += string(buf);
        }
    }

    return key;
}

vector<int> makeAddressVectorFromString(string str, char spliter = '-')
{
    vector<int> result;
    stringstream ss(str);
    string sub_str;
    while(getline(ss,sub_str,spliter))
        result.push_back(atoi(sub_str.c_str()));

    return result;
}

template<typename Metadata>
float InteractionInformation::calculateInteractionInformation(string onePieceOfData, DataProvider<Metadata>* data)
{
    return calculateInteractionInformation(makeAddressVectorFromString(onePieceOfData),data);
}

template<typename Metadata>
float InteractionInformation::calculateInteractionInformation(vector<int> onePieceOfData, DataProvider<Metadata>* data)
{
    if (! data->isOrderDependent)
    {
        sortIntVector(onePieceOfData);
    }

    int n_max = onePieceOfData.size();
    bool sign = true;
    float interactionInfo = 0.0f;
    float tmpSum;
    map<string,StatisticData>::iterator it;
    string key;
    StatisticData& pieceData;

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
                pieceData = (StatisticData)(it->second);
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
void InteractionInformation::calculateInteractionInformations(DataProvider<Metadata>* data)
{
    map<string,StatisticData>::iterator it;

    for (int n = 1; n <= data->n_gram; ++n )
    {
        for( it = data->mDataMaps[n].begin(); it != data->mDataMaps[n].end(); ++it)
        {
            StatisticData& pieceData = (StatisticData)(it->second);
            pieceData.interactionInformation = calculateInteractionInformation((string)(it->first), data);
        }

    }
}

