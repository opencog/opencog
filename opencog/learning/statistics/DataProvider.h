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

#include <cstdint>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <iterator>
#include <algorithm>

namespace opencog { namespace statistics {

struct StatisticData
{
public:
    uint64_t count;
    float probability;
    float entropy;
    float interactionInformation;

public:
    StatisticData(uint64_t _count,
                  float _probability,
                  float _entropy,
                  float _interactionInformation)
    {
        count = _count;
        probability = _probability;
        entropy = _entropy;
        interactionInformation = _interactionInformation;
    }

    StatisticData(uint64_t _count)
    {
        count = _count;
        probability = 0.0f;
        entropy = 0.0f;
        interactionInformation = 0.0f;
    }

    StatisticData(unsigned int _count)
    {
        count = _count;
        probability = 0.0f;
        entropy = 0.0f;
        interactionInformation = 0.0f;
    }
};

template<typename Metadata>
class MetaDataContainer
{
public:
    MetaDataContainer()
    {
        mDataSetWithKey = new std::map<long, Metadata>();
        mDataSetWithValue = new std::map<Metadata, long>();
    }

    ~MetaDataContainer()
    {
        delete mDataSetWithValue;
        delete mDataSetWithKey;
    }

    inline long size()
    {
        return mDataSetWithKey->size();
    }

    inline long getKey(Metadata &meta)
    {
        typename std::map<Metadata, long>::iterator it;
        it = mDataSetWithValue->find(meta);
        if (it == mDataSetWithValue->end()) {
            return -1;
        }
        return it->second;
    }

    inline Metadata* getValue(long key)
    {
        typename std::map<long, Metadata>::iterator it;
        it = mDataSetWithKey->find(key);
        if (it == mDataSetWithKey->end()) {
            return NULL;
        }
        return &(it->second);
    }

    inline Metadata* find(Metadata &meta)
    {
        long key = getKey(meta);
        return getValue(key);
    }

    inline bool insert(Metadata &meta)
    {
        if(find(meta))
            return false;

        ++mMetaDataMapIndex;
        mDataSetWithKey->insert(std::make_pair(mMetaDataMapIndex, meta));
        mDataSetWithValue->insert(std::make_pair(meta, mMetaDataMapIndex));
        return true;
    }

    inline std::string print_meta_data_set()
    {
        std::ostringstream stringStream;

        stringStream << "" << std::endl;
        stringStream << ">All Data Set<" << std::endl;

        typename std::map<long, Metadata>::iterator it;
        it = mDataSetWithKey->begin();
        for ( ; it != mDataSetWithKey->end(); ++it)
            stringStream << it->first << ": " << it->second << " " << std::endl;

        stringStream << ">All Data Set End<" << std::endl;
        return stringStream.str();
    }

protected:
    // Save all the Metadata. Each Metadata is a piece of one-gram data.
    // It also saves an unique key of Metadata.
    std::map<long, Metadata> *mDataSetWithKey;
    std::map<Metadata, long> *mDataSetWithValue;

    long mMetaDataMapIndex = -1;
};

// The DataProvider construct the raw data sets for statistics module
// If the Metadata is a class or struct, not a original C++ data type,
// you should implement the followings in your Metadata class:
// constructor, copy constructor, destructor, operator ==, !=, <, = .
template<typename Metadata>
class DataProvider
{
public:
    // n-gram data maps:
    int n_gram;

    // Manages all Metadata. Each Metadata is a piece of one-gram data.
    // It also saves an unique key of Metadata.
    MetaDataContainer<Metadata> *mDataSet;

    // map array, the array size = n_gram + 1 (index starts from 1)
    // the key is vector<int> type, contains the unique key of the Metadata:
    // for 1-gram, the vector contains one unique key
    // for 2-gram, the vector contains [key1, key2]
    // for n-gram, the vector contains [key1, key2, ..., keyN]
    std::map<std::vector<long>, StatisticData> *mDataMaps;

    // An array to save the total data counts for each n-gram raw data map.
    // The array size = n_gram + 1 (index starts from 1)
    // the total data numbers for each gram of raw data map.
    // e.g. there are 3435 datas for 3-gram raw data map, which is not the
    // size of the 3-gram raw data map, it is the sum of all the count number
    // of each data in the 3-gram raw data map
    uint64_t *mRawDataNumbers;

    // Does the permutation order of the metadatas in one pattern matter?
    // like whether "a-b-c" is considered to be the same with "b-c-a".
    bool isOrderDependent;

    DataProvider(int _n_gram, bool _isOrderDependent) :
            n_gram(_n_gram), isOrderDependent(_isOrderDependent)
    {
        mDataSet = new MetaDataContainer<Metadata>();
        mDataMaps = new std::map<std::vector<long>, StatisticData>[n_gram + 1];

        mRawDataNumbers = new uint64_t[n_gram + 1];
        for (int i = 0; i < n_gram + 1; i++)
            mRawDataNumbers[i] = 0;
    }

    ~DataProvider()
    {
        delete[] mDataMaps;
        delete[] mRawDataNumbers;
        delete mDataSet;
    }

    // add one piece of meta data into the mMetaDataSet
    inline bool addOneMetaData(Metadata meta)
    {
        return this->mDataSet->insert(meta);
    }

    // add the count number of one piece of raw data to mDataMaps.
    // if there is already a piece of this raw data in the mDataMaps as a key,
    // just add the countNum.
    // if there is not, add a new pair to mDataMaps.
    inline void addOneRawDataCount(std::vector<Metadata> &oneRawData,
                                   unsigned int countNum)
    {
        std::vector<Metadata> rawData;

        for (auto& mdata : oneRawData){
            Metadata* data = mDataSet->find(mdata);
            if(data == NULL) {
                data = &mdata;
                mDataSet->insert(*data);
            }
            rawData.push_back(*data);
        }
        _addOneRawDataCount(rawData, countNum);
    }

    inline std::vector<long> makeKeyFromData(std::vector<Metadata> &oneRawData)
    {
        return makeKeyFromData(NULL, oneRawData);
    }

    // Note: the address of oneRawData should be the one saved in mMetaDataSet,
    // not any metadata from outside
    inline std::vector<long> makeKeyFromData(bool combination_array[],
                                             std::vector<Metadata> &oneRawData)
    {
        // if the is not OrderDependent, we sort the data addresses ascending
        if (!this->isOrderDependent)
            std::sort(oneRawData.begin(), oneRawData.end());

        std::vector<long> result;

        if (combination_array) {
            int i = 0;
            for (auto& mdata : oneRawData) {
                if (combination_array[i])
                    result.push_back(mDataSet->getKey(mdata));
                ++i;
            }
        } else {
            result.reserve(oneRawData.size());
            for (auto& mdata : oneRawData)
                result.push_back(mDataSet->getKey(mdata));
        }

        return result;
    }

    // not any metadata from outside
    inline std::vector<Metadata> makeDataFromKey(std::vector<long> &indexes)
    {
        std::vector<Metadata> result;
        result.reserve(indexes.size());
        for (auto& index : indexes)
            result.push_back(*(mDataSet->getValue(index)));

        return result;
    }

    // TODO:
    void saveRawDataToFiles(std::string folderName) { }

    void loadRawDataFromFiles(std::string folderName) { }

    void saveResultsToFiles(std::string folderName) { }

    inline std::string print_data_map()
    {
        std::ostringstream stringStream;

        stringStream << "" << std::endl;
        stringStream << ">Data Map<" << std::endl;
        for (long i = 0; i <= n_gram; i++){
            stringStream << i << "-gram: " << std::endl;
            std::map<std::vector<long>, StatisticData> map = mDataMaps[i];
            std::map<std::vector<long>, StatisticData>::iterator it_2 = map.begin();
            for ( ; it_2 != map.end(); ++it_2){
                stringStream << "> key: ";
                for (std::vector<long>::const_iterator j = it_2->first.begin(); j != it_2->first.end(); ++j)
                    stringStream << *j << "-";
                stringStream << std::endl;
                stringStream << "> count: " << it_2->second.count << ", " << std::endl;
                stringStream << "> probability: " << it_2->second.probability << ", " << std::endl;
                stringStream << "> entropy: " << it_2->second.entropy << ", " << std::endl;
                stringStream << "> interactionInformation: " << it_2->second.interactionInformation << std::endl;
                stringStream << "> ---" << std::endl;
            }
            stringStream << "-----";
        }
        stringStream << ">Data Map End<" << std::endl;
        return stringStream.str();
    }
protected:
    // Note: the address of oneRawData should be the one saved in mMetaDataSet,
    // not any metadata from outside
    // add the count number of one piece of raw data to mDataMaps.
    // if there is already a piece of this raw data in the mDataMaps as a key,
    // just add the countNum.
    // if there is not, add a new pair to mDataMaps.
    inline void _addOneRawDataCount(std::vector<Metadata> &oneRawData,
                                    unsigned int countNum)
    {
        long n_gram = oneRawData.size();
        std::map<std::vector<long>, StatisticData>::iterator it;
        std::vector<long> key = makeKeyFromData(oneRawData);
        it = mDataMaps[n_gram].find(key);

        if (it == mDataMaps[n_gram].end()) {
            // add new pair to the map
            StatisticData newData(countNum);
            mDataMaps[n_gram].insert(
                    std::map<std::vector<long>, StatisticData>
                    ::value_type(key, newData)
            );
        }
        else {
            // this key already exists in the map, just add the countNum
            (it->second).count += countNum;
        }
        mRawDataNumbers[n_gram] += countNum;
    }
};

} // namespace statistics
} // namespace opencog

#endif //_OPENCOG_STATISTICS_DATAPROVIDER_H
