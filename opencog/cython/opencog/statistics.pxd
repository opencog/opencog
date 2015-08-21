from libcpp cimport bool as cppbool
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.map cimport map as cmap

cdef extern from "opencog/learning/statistics/DataProvider.h" \
    namespace "opencog::statistics":
    # noinspection PyPep8Naming
    # Interface of C++ StatisticData struct.
    cdef cppclass StatisticData:
        int count
        float probability
        float entropy
        float interactionInformation
        StatisticData(int) except +
        StatisticData(int, float, float, float) except +

    # Interface of C++ MetaDataContainer class.
    cdef cppclass MetaDataContainer[Metadata]:
        long size()

    # noinspection PyPep8Naming, PyUnresolvedReferences
    # Interface of C++ DataProvider class.
    cdef cppclass DataProvider[Metadata]:
        int n_gram
        bint isOrderDependent
        MetaDataContainer[Metadata] *mDataSet
        cmap[vector[long], StatisticData] *mDataMaps

        DataProvider(int, cppbool) except +
        bint addOneMetaData(Metadata)
        void addOneRawDataCount(vector[Metadata] &oneRawData, int countNum)
        vector[long] makeKeyFromData(vector[Metadata] &oneRawData)
        vector[long] makeKeyFromData(cppbool[] combination_array,
                                     vector[Metadata] &oneRawData)
        vector[Metadata] makeDataFromKey(vector[long] &indexes)
        string print_data_map()

cdef extern from "opencog/learning/statistics/Probability.h" \
    namespace "opencog::statistics::Probability":
    # noinspection PyPep8Naming
    # Interface of C++ template method in Probability class.
    void calculateProbabilities(DataProvider[long] &provider)

cdef extern from "opencog/learning/statistics/Entropy.h" \
    namespace "opencog::statistics::Entropy":
    # noinspection PyPep8Naming
    # Interface of C++ template method in Entropy class.
    void calculateEntropies(DataProvider[long] &provider)

cdef extern from "opencog/learning/statistics/InteractionInformation.h" \
    namespace "opencog::statistics::InteractionInformation":
    # noinspection PyPep8Naming
    # Interface of C++ template method in InteractionInformation class.
    float calculateInteractionInformation(vector[long] &onePieceOfData,
                                          DataProvider[long] &provider)
    # noinspection PyPep8Naming
    # Interface of C++ template method in InteractionInformation class.
    void calculateInteractionInformations(DataProvider[long] &provider)
