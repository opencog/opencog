/*
 * opencog/learning/statistics/DataProvider.cc
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

#include "DataProvider.h"
#include <iterator>

using namespace opencog::statistics;


template<typename Metadata>
DataProvider<Metadata>::DataProvider(int _n_gram, bool _isOrderDependent)
    :n_gram(_n_gram),isOrderDependent(_isOrderDependent)
{
    mDataMaps = new map<string,int>[n_gram + 1];

    mRawDataNumbers = new int[n_gram + 1];
    for (int i= 0; i < n_gram + 1; i ++)
        mRawDataNumbers[i] = 0;
}


template<typename Metadata>
DataProvider<Metadata>::~DataProvider()
{
    delete mDataMaps;
    delete mRawDataNumbers;
}

template<typename Metadata>
bool DataProvider<Metadata>::addOneMetaData(Metadata meta)
{
    typename set<Metadata>::iterator it;
    it = mMetaDataSet.find(meta);
    if (it == mMetaDataSet.end())
    {
        mMetaDataSet.insert(meta);
        return true;
    }

    return false;
}

template<typename Metadata>
void DataProvider<Metadata>::addOneRawDataCount(int _n_gram, Metadata oneRawData[],int countNum )
{
    Metadata* rawData[_n_gram];
    typename set<Metadata>::iterator it;

    for (int i = 0; i < _n_gram; i++)
    {
        it = mMetaDataSet.find(oneRawData[i]);
        if (it == mMetaDataSet.end())
        {
            it = mMetaDataSet.insert(oneRawData[i]).first;
        }

        rawData[i] = it;
    }

    _addOneRawDataCount(_n_gram, rawData, countNum);
}

template<typename Metadata>
void DataProvider<Metadata>::_addOneRawDataCount(int _n_gram, Metadata* &oneRawData,int countNum )
{
    map<string,StatisticData>::iterator it;
    string key = makeStringFromData(_n_gram, oneRawData);
    it = mDataMaps[_n_gram].find(key);

    if (it == mDataMaps[_n_gram].end())
    {
        // add new pair to the map
        StatisticData newData(countNum);

        mDataMaps[_n_gram].insert(map<string,StatisticData>::value_type(key,newData));
    }
    else
    {
        // this key already exists in the map, just add the countNum
        (StatisticData)(it->second).count += countNum;
    }

    mRawDataNumbers[_n_gram] += countNum;
}

template<typename Metadata>
vector<string> DataProvider<Metadata>::makeStringVectorFromData(int _n_gram, Metadata oneRawData[])
{
    vector<string> empty;
    vector<string> result;

    typename set<Metadata>::iterator it;

    for (int i = 0; i < _n_gram; i++)
    {
        it = mMetaDataSet.find(oneRawData[i]);
        if (it == mMetaDataSet.end())
        {
            return empty;
        }

        result.push_back(string(itoa(it)));
    }

    return result;
}

template<typename Metadata>
vector<int> DataProvider<Metadata>::makeAddressVectorFromData(int _n_gram, Metadata oneRawData[])
{
    vector<int> empty;
    vector<int> result;

    typename set<Metadata>::iterator it;

    for (int i = 0; i < _n_gram; i++)
    {
        it = mMetaDataSet.find(oneRawData[i]);
        if (it == mMetaDataSet.end())
        {
            return empty;
        }

        result.push_back(it);
    }

    return result;
}



template<typename Metadata>
string DataProvider<Metadata>::_makeStringFromData(int _n_gram, Metadata* &oneRawData)
{
    // if the is not OrderDependent, we sort the data adresses ascending
    if (! this->isOrderDependent)
        sort(_n_gram, oneRawData);

    string result = "";

    for (int i =0; i < _n_gram; i ++)
    {
        if (i != 0)
            result += "-";

        result += string(itoa(oneRawData[i]));
    }

    return result;
}

template<typename Metadata>
void DataProvider<Metadata>:: sort(int n, Metadata* &oneRawData)
{
    Metadata &tmp;
    for (int i = 0; i < n; ++i)
        for (int j = i; j < n-1; ++j)
    {
        if (oneRawData[j] > oneRawData[j+1] )
        {
            tmp = oneRawData[j];
            oneRawData[j] = oneRawData[j+1];
            oneRawData[j+1] = tmp;
        }
    }

}


