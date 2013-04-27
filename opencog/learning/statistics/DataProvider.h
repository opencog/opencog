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

#include <string>
#include <map>
#include <set>
#include <vector>

using namespace std;

namespace opencog {
 namespace statistics {

 struct StatisticData
 {
 public:
     int    count;
     float  probability;
     float  entropy;
     float  interactionInformation;

 public:
     StatisticData(int _count, float _probability, float _entropy, float _interactionInformation)
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

 // The DataProvider contruct the raw data sets for statistics module
 // If the Metadata is a class or struct, not a original C++ data type, you should implement the followings in your Metadata class:
 // constructor, copy constructor, destructor, operator ==, !=, <, = .
template<typename Metadata>
 class DataProvider
 {
    public:

     // Save all the Metadata. Each Metadata is a piece of one-gram data.
     set<Metadata> mMetaDataSet;

     // n-gram data maps:

     int n_gram;

     // map array, the array size = n_gram
     // the key is string type, the string contains the address of the Metadata:
     // for 1-gram, the string is the address toString
     // for 2-gram, the string is "address1-address2"
     // for n-gram, the string is "address1-address2-...-addressn"
     // use - as the separator
     map<string,StatisticData> *mDataMaps;

     // An array to save the total data counts for each n-gram raw data map.
     // the total data numbers for each gram of raw data map.
     // e.g. there are 3435 datas for 3-gram raw data map, which is not the size of the 3-gram raw data map,
     // it is the sum of all the count number of each data in the 3-gram raw data map
     int* mRawDataNumbers;

     // Does the permutation order of the metadatas in one pattern matter?
     // like whether "a-b-c" is considered to be the same with "b-c-a"
     bool isOrderDependent;

     DataProvider(int _n_gram, bool _isOrderDependent);

     ~DataProvider();

     // add one piece of meta data into the mMetaDataSet
     bool addOneMetaData(Metadata meta);

     // add the count number of one piece of raw data to mDataMaps.
     // if there is already a piece of this raw data in the mDataMaps as a key, just add the countNum
     // if there is not, add a new pair to mDataMaps
     void addOneRawDataCount(int _n_gram, Metadata oneRawData[],int countNum );


     // Note: oneRawData is the array of the raw data
     vector<string> makeStringVectorFromData(int n_gram, Metadata* oneRawData);

     // Note: oneRawData is the array of the raw data
     vector<int> makeAddressVectorFromData(int n_gram, Metadata* oneRawData);


     // TODO:
     void saveRawDataToFiles(string folderName){}
     void loadRawDataFromFiles(string folderName){}
     void saveResultsToFiles(string folderName){}

 protected:
     // Note: the address of oneRawData should be the one saved in mMetaDataSet, not any metadata from outside
     string _makeStringFromData(int n_gram, Metadata* &oneRawData);

     // Note: the address of oneRawData should be the one saved in mMetaDataSet, not any metadata from outside
     // add the count number of one piece of raw data to mDataMaps.
     // if there is already a piece of this raw data in the mDataMaps as a key, just add the countNum
     // if there is not, add a new pair to mDataMaps
     void _addOneRawDataCount(int n_gram, Metadata* &oneRawData,int countNum = 1);

     void sort(int n, Metadata* &oneRawData);

 };

 } //  namespace statistics

} // namespace opencog


#endif //_OPENCOG_STATISTICS_DATAPROVIDER_H

