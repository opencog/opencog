from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.map cimport map as cmap

cdef extern from "opencog/learning/statistics/DataProvider.h" namespace "opencog::statistics":
    cdef cppclass StatisticData:
        int count
        float probability
        float entropy
        float interactionInformation
        StatisticData(int) except +
        StatisticData(int, float, float, float) except +

    cdef cppclass MetaDataContainer[Metadata]:
        long size()

    # Interface of C++ DataProvider class.
    cdef cppclass DataProvider[Metadata]:
        int n_gram
        bint isOrderDependent
        MetaDataContainer[Metadata] *mDataSet
        cmap[vector[long], StatisticData] *mDataMaps

        DataProvider(int, bool) except +
        bint addOneMetaData(Metadata)
        void addOneRawDataCount(vector[Metadata] &oneRawData, int countNum)
        vector[long] makeKeyFromData(vector[Metadata] &oneRawData)
        vector[long] makeKeyFromData(bool[] combination_array, vector[Metadata] &oneRawData)
        vector[Metadata] makeDataFromKey(vector[long] &indexes)
        string print_data_map()

cdef extern from "opencog/learning/statistics/Probability.h" namespace "opencog::statistics::Probability":
    void calculateProbabilities(DataProvider[long] &provider)

cdef extern from "opencog/learning/statistics/Entropy.h" namespace "opencog::statistics::Entropy":
    void calculateEntropies(DataProvider[long] &provider)
