/*
 * opencog/learning/statistics/DataProvider.h
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

#ifndef _OPENCOG_STATISTICS_DATAPROVIDER_H
#define _OPENCOG_STATISTICS_DATAPROVIDER_H

#include <sstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <iterator>
#include <boost/lexical_cast.hpp>

namespace opencog { namespace statistics {

struct StatisticData
{
public:
    int count;
    float probability;
    float entropy;
    float interactionInformation;

public:
    StatisticData(int _count,
                  float _probability,
                  float _entropy,
                  float _interactionInformation)
    {
        count = _count;
        probability = _probability;
        entropy = _entropy;
        interactionInformation = _interactionInformation;
    }

    StatisticData(int _count)
    {
        count = _count;
        probability = 0.0f;
        entropy = 0.0f;
        interactionInformation = 0.0f;
    }
};

// The DataProvider construct the raw data sets for statistics module
// If the Metadata is a class or struct, not a original C++ data type,
// you should implement the followings in your Metadata class:
// constructor, copy constructor, destructor, operator ==, !=, <, = .
template<typename Metadata>
class DataProvider
{
public:

    // Save all the Metadata. Each Metadata is a piece of one-gram data.
    std::set<Metadata> mMetaDataSet;

    // n-gram data maps:
    int n_gram;

    // map array, the array size = n_gram + 1 (index starts from 1)
    // the key is string type, the string contains the address of the Metadata:
    // for 1-gram, the string is the address toString
    // for 2-gram, the string is "address1-address2"
    // for n-gram, the string is "address1-address2-...-addressn"
    // use - as the separator
    std::map<std::string, StatisticData> *mDataMaps;

    // An array to save the total data counts for each n-gram raw data map.
    // The array size = n_gram + 1 (index starts from 1)
    // the total data numbers for each gram of raw data map.
    // e.g. there are 3435 datas for 3-gram raw data map, which is not the
    // size of the 3-gram raw data map, it is the sum of all the count number
    // of each data in the 3-gram raw data map
    int *mRawDataNumbers;

    // Does the permutation order of the metadatas in one pattern matter?
    // like whether "a-b-c" is considered to be the same with "b-c-a".
    bool isOrderDependent;

    DataProvider(int _n_gram, bool _isOrderDependent) :
            n_gram(_n_gram), isOrderDependent(_isOrderDependent)
    {
        mDataMaps = new std::map<std::string, StatisticData>[n_gram + 1];

        mRawDataNumbers = new int[n_gram + 1];
        for (int i = 0; i < n_gram + 1; i++)
            mRawDataNumbers[i] = 0;
    }

    ~DataProvider()
    {
        delete[] mDataMaps;
        delete[] mRawDataNumbers;
    }

    // add one piece of meta data into the mMetaDataSet
    inline bool addOneMetaData(Metadata meta)
    {
        typename std::set<Metadata>::iterator it;
        it = mMetaDataSet.find(meta);
        if (it == mMetaDataSet.end()) {
            mMetaDataSet.insert(meta);
            return true;
        }
        return false;
    }

    // add the count number of one piece of raw data to mDataMaps.
    // if there is already a piece of this raw data in the mDataMaps as a key,
    // just add the countNum.
    // if there is not, add a new pair to mDataMaps.
    inline void addOneRawDataCount(int _n_gram,
                                   Metadata *oneRawData,
                                   int countNum)
    {
        Metadata* rawData[_n_gram];
        typename std::set<Metadata>::iterator it;

        for (int i = 0; i < _n_gram; i++) {
            it = mMetaDataSet.find(oneRawData[i]);
            if (it == mMetaDataSet.end()) {
                it = mMetaDataSet.insert(oneRawData[i]).first;
            }
            rawData[i] = (Metadata*) &(*it);
        }

        _addOneRawDataCount(_n_gram, rawData, countNum);
    }

    // Note: oneRawData is the array of the raw data
    inline std::vector<std::string> makeStringVectorFromData (
            int _n_gram,
            Metadata *oneRawData)
    {
        unsigned int hex_address = 0;
        std::vector<std::string> empty;
        std::vector<std::string> result;

        typename std::set<Metadata>::iterator it;

        for (int i = 0; i < _n_gram; i++) {
            it = mMetaDataSet.find(oneRawData[i]);
            if (it == mMetaDataSet.end()) {
                return empty;
            }

            Metadata* ret_ptr = (Metadata*) &(*it);
            std::stringstream ss;
            ss << std::hex << ret_ptr;
            ss >> hex_address;
            result.push_back(boost::lexical_cast<std::string>(hex_address));
        }

        return result;
    }

    // Note: oneRawData is the array of the raw data
    inline std::vector<int> makeAddressVectorFromData(
            int _n_gram,
            Metadata *oneRawData)
    {
        unsigned int hex_address = 0;
        std::vector<int> empty;
        std::vector<int> result;

        typename std::set<Metadata>::iterator it;

        for (int i = 0; i < _n_gram; i++) {
            it = mMetaDataSet.find(oneRawData[i]);
            if (it == mMetaDataSet.end()) {
                return empty;
            }

            Metadata* ret_ptr = (Metadata*) &(*it);

            std::stringstream ss;
            ss << std::hex << ret_ptr;
            ss >> hex_address;
            result.push_back(static_cast<int>(hex_address));
        }

        return result;
    }

    // TODO:
    void saveRawDataToFiles(std::string folderName) { }

    void loadRawDataFromFiles(std::string folderName) { }

    void saveResultsToFiles(std::string folderName) { }

protected:
    // Note: the address of oneRawData should be the one saved in mMetaDataSet,
    // not any metadata from outside
    inline std::string _makeStringFromData(int _n_gram,
                                           Metadata* *oneRawData)
    {
        // if the is not OrderDependent, we sort the data addresses ascending
        if (!this->isOrderDependent)
            this->sort(_n_gram, oneRawData);

        std::string result = "";

        unsigned int hex_address = 0;
        for (int i = 0; i < _n_gram; i++) {
            if (i != 0)
                result += "-";

            std::stringstream ss;
            ss << std::hex << oneRawData[i];
            ss >> hex_address;
            result +=boost::lexical_cast<std::string>(hex_address);
        }

        return result;
    }

    // Note: the address of oneRawData should be the one saved in mMetaDataSet,
    // not any metadata from outside
    // add the count number of one piece of raw data to mDataMaps.
    // if there is already a piece of this raw data in the mDataMaps as a key,
    // just add the countNum.
    // if there is not, add a new pair to mDataMaps.
    inline void _addOneRawDataCount(int _n_gram,
                                    Metadata* *oneRawData,
                                    int countNum)
    {
        std::map<std::string, StatisticData>::iterator it;
        std::string key = _makeStringFromData(_n_gram, oneRawData);
        it = mDataMaps[_n_gram].find(key);

        if (it == mDataMaps[_n_gram].end()) {
            // add new pair to the map
            StatisticData newData(countNum);

            mDataMaps[_n_gram].insert(
                    std::map<std::string, StatisticData>
                    ::value_type(key, newData)
            );
        }
        else {
            // this key already exists in the map, just add the countNum
            (it->second).count += countNum;
        }

        mRawDataNumbers[_n_gram] += countNum;
    }

    inline void sort(int n, Metadata **oneRawData) {
        Metadata *tmp;
        for (int i = 0; i < n; ++i)
            for (int j = i; j < n - 1; ++j)
                if (oneRawData[j] > oneRawData[j + 1]) {
                    tmp = oneRawData[j];
                    oneRawData[j] = oneRawData[j + 1];
                    oneRawData[j + 1] = tmp;
                }
    }
};

} // namespace statistics
} // namespace opencog

#endif //_OPENCOG_STATISTICS_DATAPROVIDER_H

