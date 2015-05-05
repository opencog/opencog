from libcpp.vector cimport vector
from libcpp.list cimport list as cpplist


cdef extern from "Python.h":
    # Tacky hack to pass atomspace pointer to AtomSpace ctor.
    cdef void* PyLong_AsVoidPtr(object)

    # Needed to return truth value pointers to C++ callers.
    cdef object PyLong_FromVoidPtr(void *p)


# Basic wrapping for std::string conversion.
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()
        int size()


# Basic wrapping for back_insert_iterator conversion.
cdef extern from "<vector>" namespace "std":
    cdef cppclass output_iterator "back_insert_iterator<vector<opencog::Handle> >"
    cdef output_iterator back_inserter(vector[cHandle])


# Basic OpenCog types
# ClassServer
ctypedef short Type

cdef extern from "opencog/atomspace/ClassServer.h" namespace "opencog":
    cdef cppclass cClassServer "opencog::ClassServer":
        bint isNode(Type t)
        bint isLink(Type t)
        bint isA(Type t, Type t)

        bint isDefined(string typename)
        Type getType(string typename)
        string getTypeName(Type t)
        Type getNumberOfClasses()
    cdef cClassServer classserver()

cdef extern from "opencog/atomspace/atom_types.h" namespace "opencog":
    cdef Type NOTYPE

# Handle
ctypedef public long UUID

cdef extern from "opencog/atomspace/Handle.h" namespace "opencog":
    cdef cppclass cHandle "opencog::Handle":
        cHandle()
        cHandle(UUID)
        UUID value()
        bint operator==(cHandle h)
        bint operator!=(cHandle h)
        bint operator<(cHandle h)
        bint operator>(cHandle h)
        bint operator<=(cHandle h)
        bint operator>=(cHandle h)
        cHandle UNDEFINED
# HandleSeq
    cdef cppclass cHandleSeq "opencog::HandleSeq"

cdef class Handle:
    cdef cHandle *h

cdef class Atom:
    cdef Handle handle
    cdef AtomSpace atomspace
    cdef object _atom_type
    cdef object _name
    cdef object _outgoing


# AtomSpace

cdef extern from "opencog/atomspace/AtomSpace.h" namespace "opencog":
    cdef cppclass cAtomSpace "opencog::AtomSpace":
        AtomSpace()

        cHandle getHandle(Type t, string s)
        cHandle getHandle(Type t, vector[cHandle])

        bint isValidHandle(cHandle h)
        int getSize()
        Type getType(cHandle h)

        void clear()

cdef AtomSpace_factory(cAtomSpace *to_wrap)

cdef class AtomSpace:
    cdef cAtomSpace *atomspace
    cdef bint owns_atomspace

# SpaceServer
cdef extern from "opencog/spacetime/SpaceServer.h" namespace "opencog":
    cdef cppclass cSpaceServer "opencog::SpaceServer":
        SpaceServer()

# TimeServer
cdef extern from "opencog/spacetime/TimeServer.h" namespace "opencog":
    cdef cppclass cTimeServer "opencog::TimeServer":
        TimeServer()


cdef extern from "opencog/atomutils/AtomUtils.h" namespace "opencog":
    # C++: 
    #   
    #   HandleSeq get_predicates(const Handle& target, 
    #                     Type predicateType=PREDICATE_NODE,
    #                     bool subClasses=true)
    #   void finalize_opencog();
    #   void configuration_load(const char* configFile);
    #
    cdef vector[cHandle] c_get_predicates "get_predicates" (cHandle& target, Type t, bint subclass)
    cdef vector[cHandle] c_get_predicates_for "get_predicates_for" (cHandle& target, cHandle& predicate)

