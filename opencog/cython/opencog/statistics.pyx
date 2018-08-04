from cython cimport sizeof
from cython.operator cimport dereference as deref, preincrement as inc

from libc.stdlib cimport malloc, free
from libcpp cimport bool as cppbool
from libcpp.vector cimport vector
from libcpp.set cimport set
from libcpp.map cimport map

from opencog.atomspace import Handle

cdef class PyStatisticData:
    """ C++ StatisticData wrapper class.
    """
    cdef StatisticData *thisptr
    def __cinit__(self, unsigned int count, probability=None, entropy=None,
                  interaction_information=None):
        if probability is None:
            self.thisptr = new StatisticData(count)
        else:
            self.thisptr = new StatisticData(count, probability, entropy,
                                             interaction_information)
    def __str__(self):
        return "count: {0:d} " \
               "probability: {1:f} " \
               "entropy: {2:f} " \
               "interactionInformation: {3:f}".format(
            self.count,
            self.probability,
            self.entropy,
            self.interaction_information
        )

    property count:
        def __get__(self): return self.thisptr.count
        def __set__(self, count): self.thisptr.count = count

    property probability:
        def __get__(self): return self.thisptr.probability
        def __set__(self, probability): self.thisptr.probability = probability

    property entropy:
        def __get__(self): return self.thisptr.entropy
        def __set__(self, entropy): self.thisptr.entropy = entropy

    property interaction_information:
        def __get__(self): return self.thisptr.interactionInformation
        def __set__(self, interaction_information):
            self.thisptr.interactionInformation = interaction_information


cdef class PyDataProvider:
    """ C++ DataProvider wrapper class.

    This class wraps the C++ DataProvider class, but this wrapper class supports
    only python long type, since cython binding doesn't support the
    *declaration* of template class now.

    TODO: Change class to support template.
    TODO: Remove the redundant converting code. Cython can convert 'python list'
    <-> 'C++ vector' automatically but can't use now since it has bug.
    (Redefinition error,
    same as https://gist.github.com/mattjj/15f28177d68238659386)
    """
    cdef DataProvider[long] *thisptr

    def __cinit__(self, n_gram, is_order_dependent):
        self.thisptr = new DataProvider[long](n_gram, is_order_dependent)

    def __dealloc__(self):
        del self.thisptr

    def add_one_metadata(self, metadata):
        return self.thisptr.addOneMetaData(metadata)

    def add_one_rawdata_count(self, one_rawdata, count_num):
        cdef vector[long] v
        for item in one_rawdata:
            v.push_back(item)
        self.thisptr.addOneRawDataCount(v, count_num)

    def make_key_from_data(self, one_rawdata, combination_array=None):
        cdef cppbool *bool_array
        cdef vector[long] v

        for item in one_rawdata:
            v.push_back(item)

        if combination_array is None:
            result = self.thisptr.makeKeyFromData(v)
        else:
            bool_array = <cppbool *>malloc(
                len(combination_array) * sizeof(cppbool)
            )

            if bool_array is NULL:
                raise MemoryError()

            for i in xrange(len(combination_array)):
                if combination_array[i]:
                    bool_array[i] = True
                else:
                    bool_array[i] = False

            result = self.thisptr.makeKeyFromData(bool_array, v)
            free(bool_array)

        l = list()
        for item in result:
            l.append(item)
        return l

    def make_data_from_key(self, indexes):
        cdef vector[long] v
        for item in indexes:
            v.push_back(item)
        return self.thisptr.makeDataFromKey(v)

    def print_data_map(self):
        return self.thisptr.print_data_map().c_str()

    def dataset_size(self):
        return deref(self.thisptr.mDataSet).size()

    def datamap_find(self, one_rawdata):
        cdef vector[long] v
        for item in self.make_key_from_data(one_rawdata):
            v.push_back(item)

        cdef map[vector[long], StatisticData].iterator it
        it = self.thisptr.mDataMaps[v.size()].find(v)
        if it == self.thisptr.mDataMaps[v.size()].end():
            return None
        else:
            return PyStatisticData(
                int(deref(it).second.count),
                float(deref(it).second.probability),
                float(deref(it).second.entropy),
                float(deref(it).second.interactionInformation)
            )

    property n_gram:
        def __get__(self): return self.thisptr.n_gram
        def __set__(self, n_gram): self.thisptr.n_gram = n_gram

    property is_order_dependent:
        def __get__(self): return self.thisptr.isOrderDependent
        def __set__(self, is_order_dependent):
            self.thisptr.isOrderDependent = is_order_dependent

cdef class PyProbability:
    """ C++ Probability wrapper class.

    This class wraps the C++ Probability class, but this wrapper class supports
    only python long type, since cython binding doesn't support the
    *declaration* of template class now.

    TODO: Change class to support template.
    """
    @classmethod
    def calculate_probabilities(cls, PyDataProvider provider):
        cdef DataProvider[long] *thisptr = provider.thisptr
        calculateProbabilities(deref(thisptr))

cdef class PyEntropy:
    """ C++ Entropy wrapper class.

    This class wraps the C++ Entropy class, but this wrapper class supports
    only python long type, since cython binding doesn't support the
    *declaration* of template class now.

    TODO: Change class to support template.
    """
    @classmethod
    def calculate_entropies(cls, PyDataProvider provider):
        cdef DataProvider[long] *thisptr = provider.thisptr
        calculateEntropies(deref(thisptr))

cdef class PyInteractionInformation:
    """ C++ InteractionInformation wrapper class.

    This class wraps the C++ InteractionInformation class, but this wrapper
    class supports only python long type, since cython binding doesn't support
    the *declaration* of template class now.

    TODO: Change class to support template.
    TODO: Remove the redundant converting code. Cython can convert 'python list'
    <-> 'C++ vector' automatically but can't use now since it has bug.
    (Redefinition error,
    same as https://gist.github.com/mattjj/15f28177d68238659386)
    """
    @classmethod
    def calculate_interaction_information(cls, one_piece_of_data,
                                          PyDataProvider provider,
                                          n_max_limit=-1):
        cdef vector[long] v
        for item in one_piece_of_data:
            v.push_back(item)
        cdef DataProvider[long] *thisptr = provider.thisptr
        return calculateInteractionInformation(v, deref(thisptr), n_max_limit)

    @classmethod
    def calculate_interaction_informations(cls, PyDataProvider provider):
        cdef DataProvider[long] *thisptr = provider.thisptr
        calculateInteractionInformations(deref(thisptr))


class PyDataProviderAtom:
    """ Python DataProvider class for Atom.

    This class wraps the Python DataProvider wrapper class.
    """
    def __init__(self, n_gram, is_order_dependent):
        self.provider = PyDataProvider(n_gram, is_order_dependent)

    def add_one_metadata(self, atom):
        return self.provider.add_one_metadata(atom.h)

    def add_one_rawdata_count(self, one_rawdata, count_num):
        long_vector = list()
        for atom in one_rawdata:
            long_vector.append(atom.h)

        self.provider.add_one_rawdata_count(long_vector, count_num)

    def make_key_from_data(self, one_rawdata, combination_array=None):
        long_vector = list()
        for atom in one_rawdata:
            long_vector.append(atom.h)

        if combination_array is None:
            return self.provider.make_key_from_data(long_vector)
        else:
            return self.provider.make_key_from_data(long_vector,
                                                    combination_array)

    def make_data_from_key(self, atomspace, indexes):
        long_vector = list()
        for index in indexes:
            long_vector.append(index)

        ret_vector = self.provider.make_data_from_key(long_vector)
        result = list()
        for handle in ret_vector:
            result.append(atomspace[Handle(handle)])

        return result

    def print_data_map(self):
        return self.provider.print_data_map()

    def dataset_size(self):
        return self.provider.dataset_size()

    def datamap_find(self, one_rawdata):
        long_vector = list()
        for atom in one_rawdata:
            long_vector.append(atom.h)
        return self.provider.datamap_find(long_vector)

    @property
    def n_gram(self):
        return self.provider.n_gram

    @property
    def is_order_dependent(self):
        return self.provider.is_order_dependent

class PyProbabilityAtom:
    """ Python Probability class for Atom.

    This class wraps the Python Probability wrapper class.
    """
    def calculate_probabilities(self, provider_atom):
        PyProbability.calculate_probabilities(provider_atom.provider)

class PyEntropyAtom:
    """ Python Entropy class for Atom.

    This class wraps the Python Entropy wrapper class.
    """
    def calculate_entropies(self, provider_atom):
        PyEntropy.calculate_entropies(provider_atom.provider)

class PyInteractionInformationAtom:
    """ Python InteractionInformation class for Atom.

    This class wraps the Python InteractionInformation wrapper class.
    """
    def calculate_interaction_information(self, one_piece_of_data,
                                          provider_atom, n_max_limit=-1):
        long_vector = list()
        for atom in one_piece_of_data:
            long_vector.append(atom.h)

        return PyInteractionInformation.calculate_interaction_information(
            long_vector, provider_atom.provider, n_max_limit
        )
    def calculate_interaction_informations(self, provider_atom):
        PyInteractionInformation.calculate_interaction_informations(
            provider_atom.provider
        )
