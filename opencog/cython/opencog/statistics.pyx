from cython cimport sizeof
from libc.stdlib cimport malloc, free
from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

from opencog.atomspace import Handle


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

    def __cinit__(self, _n_gram, _isOrderDependent):
        self.thisptr = new DataProvider[long](_n_gram, _isOrderDependent)

    def __dealloc__(self):
        del self.thisptr

    def addOneMetaData(self, meta_data):
        return self.thisptr.addOneMetaData(meta_data)

    def addOneRawDataCount(self, oneRawData, countNum):
        cdef vector[long] v
        for item in oneRawData:
            v.push_back(item)
        self.thisptr.addOneRawDataCount(v, countNum)

    def makeKeyFromData(self, oneRawData, combination_array=None):
        cdef bool *bool_array
        cdef vector[long] v

        for item in oneRawData:
            v.push_back(item)

        if combination_array is None:
            result = self.thisptr.makeKeyFromData(v)
        else:
            bool_array = <bool *>malloc(len(combination_array)*sizeof(bool))

            if bool_array is NULL:
                raise MemoryError()

            for i in xrange(len(combination_array)):
                if combination_array[i]:
                    bool_array[i] = True
                else:
                    bool_array[i] = False
                print str(combination_array[i])

            result = self.thisptr.makeKeyFromData(bool_array, v)
            free(bool_array)

        l = list()
        for item in result:
            l.append(item)
        return l

    def makeDataFromKey(self, indexes):
        cdef vector[long] v
        for item in indexes:
            v.push_back(item)

        return self.thisptr.makeDataFromKey(v)

    def print_data_map(self):
        return self.thisptr.print_data_map().c_str()

    property n_gram:
        def __get__(self): return self.thisptr.n_gram
        def __set__(self, n_gram): self.thisptr.n_gram = n_gram

    property isOrderDependent:
        def __get__(self): return self.thisptr.isOrderDependent
        def __set__(self, isOrderDependent):
            self.thisptr.isOrderDependent = isOrderDependent

    property mDataSetSize:
        def __get__(self): return deref(self.thisptr.mDataSet).size()

    property mDataMapsSize:
        def __get__(self): return deref(self.thisptr.mDataMaps).size()


class PyDataProviderAtom:
    """ Python DataProvider class for Atom.

    This class wraps the Python DataProvider wrapper class.
    """
    def __init__(self, _n_gram, _isOrderDependent):
        self.provider = PyDataProvider(_n_gram, _isOrderDependent)

    def addOneMetaData(self, atom):
        return self.provider.addOneMetaData(atom.h.value())

    def addOneRawDataCount(self, oneRawData, countNum):
        long_vector = list()
        for atom in oneRawData:
            long_vector.append(atom.h.value())

        self.provider.addOneRawDataCount(long_vector, countNum)

    def makeKeyFromData(self, oneRawData, combination_array=None):
        long_vector = list()
        for atom in oneRawData:
            long_vector.append(atom.h.value())

        if combination_array is None:
            return self.provider.makeKeyFromData(long_vector)
        else:
            return self.provider.makeKeyFromData(long_vector, combination_array)

    def makeDataFromKey(self, atomspace, indexes):
        long_vector = list()
        for index in indexes:
            long_vector.append(index)

        ret_vector = self.provider.makeDataFromKey(long_vector)
        result = list()
        for handle in ret_vector:
            result.append(atomspace[Handle(handle)])

        return result

    def print_data_map(self):
        return self.provider.print_data_map()

    @property
    def n_gram(self):
        return self.provider.n_gram

    @property
    def isOrderDependent(self):
        return self.provider.isOrderDependent

    @property
    def mDataSetSize(self):
        return self.provider.mDataSetSize

    @property
    def mDataMapsSize(self):
        return self.provider.mDataMapsSize
