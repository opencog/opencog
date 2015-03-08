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


### TruthValue
ctypedef double count_t
ctypedef float confidence_t
ctypedef float strength_t

cdef extern from "opencog/atomspace/TruthValue.h" namespace "opencog":
    cdef cppclass tv_ptr "std::shared_ptr<opencog::TruthValue>":
        tv_ptr()
        tv_ptr(tv_ptr copy)
        tv_ptr(cTruthValue* fun)
        tv_ptr(cSimpleTruthValue* fun)
        cTruthValue* get()

    cdef cppclass cTruthValue "opencog::TruthValue":
        strength_t getMean()
        confidence_t getConfidence()
        count_t getCount()
        tv_ptr DEFAULT_TV()
        bint isNullTv()
        string toString()
        bint operator==(cTruthValue h)
        bint operator!=(cTruthValue h)

cdef extern from "opencog/atomspace/SimpleTruthValue.h" namespace "opencog":
    cdef cppclass cSimpleTruthValue "opencog::SimpleTruthValue":
        cSimpleTruthValue(float, float)
        strength_t getMean()
        confidence_t getConfidence()
        count_t getCount()
        count_t confidenceToCount(float)
        confidence_t countToConfidence(float)
        tv_ptr DEFAULT_TV()
        string toString()
        bint operator==(cTruthValue h)
        bint operator!=(cTruthValue h)


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
        cHandle(void*)
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

        cHandle addNode(Type t, string s) except +
        cHandle addNode(Type t, string s, tv_ptr tvn) except +

        cHandle addPrefixedNode(Type t, string s) except +

        cHandle addLink(Type t, vector[cHandle]) except +
        cHandle addLink(Type t, vector[cHandle], tv_ptr tvn) except +

        cHandle getHandle(Type t, string s)
        cHandle getHandle(Type t, vector[cHandle])

        bint isValidHandle(cHandle h)
        int getSize()
        string getName(cHandle h)
        Type getType(cHandle h)
        tv_ptr getTV(cHandle h)
        void setTV(cHandle h, tv_ptr tvn)

        vector[cHandle] getOutgoing(cHandle h)
        bint isSource(cHandle h, cHandle source)
        vector[cHandle] getIncoming(cHandle h)

        # these should alias the proper types for sti/lti/vlti
        short getSTI(cHandle h)
        short getLTI(cHandle h)
        bint getVLTI(cHandle h)
        void setSTI(cHandle h, short)
        void setLTI(cHandle h, short)
        void incVLTI(cHandle h)
        void decVLTI(cHandle h)

        string atomAsString(cHandle h, bint)

        # ==== query methods ====
        # get by type
        output_iterator getHandlesByType(output_iterator, Type t, bint subclass)
        # get by name
        output_iterator getHandlesByName(output_iterator, string& name, Type t, bint subclass)
        # get by target types
        output_iterator getHandleSet(output_iterator,Type t,Type target,bint subclass,bint target_subclass)
        # get by target handle
        output_iterator getHandleSet(output_iterator,cHandle& h,Type t,bint subclass)
        # get by STI range
        output_iterator getHandlesByAV(output_iterator, short lowerBound, short upperBound)
        output_iterator getHandlesByAV(output_iterator, short lowerBound)
        # get from AttentionalFocus
        output_iterator getHandleSetInAttentionalFocus(output_iterator)

        # vector[chandle].iterator getHandlesByName(output_iterator, Type t, string name, bint subclass)
        # vector[chandle].iterator getHandlesByType(output_iterator, Type t, xxx bint subclass)

        void clear()
        bint removeAtom(cHandle h, bint recursive) 

        void print_list "print" ()

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

