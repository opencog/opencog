from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref

# basic wrapping for std::string conversion
cdef extern from "<string>" namespace "std":
    cdef cppclass string:
        string()
        string(char *)
        char * c_str()

# Basic OpenCog types
ctypedef int UUID
ctypedef int Type

# Handle
cdef extern from "opencog/atomspace/Handle.h" namespace "opencog":
    cdef cppclass cHandle "opencog::Handle":
        cHandle()
        cHandle(UUID)
        UUID value()
        bint operator==(cHandle h)
        cHandle UNDEFINED


cdef class Handle:
    cdef cHandle *h
    def __cinit__(self, h):
        self.h=new cHandle(h)
    def __dealloc__(self):
        del self.h
    def value(self):
        return self.h.value()
    def __richcmp__(Handle h1, Handle h2, int op):
        if op == 2: # ==
            return h1.h.value() == h2.h.value()
        elif op == 3: # !=
            return h1.h.value() != h2.h.value()
        elif op == 4: # >
            return h1.h.value() > h2.h.value()
        elif op == 0: # <
            return h1.h.value() < h2.h.value()
        elif op == 1: # <=
            return h1.h.value() <= h2.h.value()
        elif op == 5: # >=
            return h1.h.value() >= h2.h.value()
    def __str__(self):
        return "<UUID:" + str(self.h.value()) + ">"

# TruthValue
cdef extern from "opencog/atomspace/TruthValue.h" namespace "opencog":
    cdef cppclass cTruthValue "opencog::TruthValue":
        getMean()
        getConfidence()
        getCount()
        cTruthValue DEFAULT_TV()

cdef extern from "opencog/atomspace/SimpleTruthValue.h" namespace "opencog":
    cdef cppclass cSimpleTruthValue "opencog::SimpleTruthValue":
        cSimpleTruthValue(float, float)
        getMean()
        getConfidence()
        getCount()
        cTruthValue DEFAULT_TV()

cdef class TruthValue:
    cdef cTruthValue *cobj
    def __cinit__(self):
        self.cobj = <cTruthValue*> new cSimpleTruthValue(0.5,0.5)
    def __dealloc__(self):
        del self.cobj

# HandleSeq
#ctypedef vector[cHandle] HandleSeq

# TimeServer
cdef extern from "opencog/atomspace/TimeServer.h" namespace "opencog":
    cdef cppclass cTimeServer "opencog::TimeServer":
        TimeServer()

cdef class TimeServer:
    cdef cTimeServer *timeserver

    def __cinit__(self):
        #self.timeserver = &timeserver
        pass

    def __dealloc__(self):
        # Don't do anything because the AtomSpace takes care of cleaning up
        pass
# AtomSpace

cdef extern from "opencog/atomspace/AtomSpace.h" namespace "opencog":
    cdef cppclass cAtomSpace "opencog::AtomSpace":
        AtomSpace()

        cHandle addNode(Type t, string s)
        cHandle addNode(Type t, string s, cTruthValue tvn)

        cHandle addPrefixedNode(Type t, string s)
        cHandle addPrefixedNode(Type t, string s, cTruthValue tvn)

        cHandle addLink(Type t, vector[cHandle])
        cHandle addLink(Type t, vector[cHandle], cTruthValue tvn)

        int getSize()

        cTimeServer getTimeServer()
        void print_list "print" ()

cdef class AtomSpace:
    cdef cAtomSpace *atomspace
    cdef cTimeServer *timeserver

    # TODO how do we do a copy constructor that shares the AtomSpaceAsync?
    def __cinit__(self):
        self.atomspace = new cAtomSpace()

    def __dealloc__(self):
        del self.atomspace

    def add_node(self,Type t,n,TruthValue tv=None):
        # convert to string
        py_byte_string = n.encode('UTF-8')
        # create temporary cpp string
        cdef string *name = new string(py_byte_string)
        cdef cHandle result
        if tv is None:
            # get handle
            result = self.atomspace.addNode(t,deref(name))
        else:
            result = self.atomspace.addNode(t,deref(name),deref(tv.cobj))
        # delete temporary string
        del name
        if result == result.UNDEFINED: return None
        return Handle(result.value());

    def add_link(self,Type t,outgoing):
        # create temporary cpp vector
        cdef vector[cHandle] o_vect
        for h in outgoing:
            o_vect.push_back(deref((<Handle>h).h))
        # get handle
        cdef cHandle result = self.atomspace.addLink(t,o_vect)
        # delete temporary vector
        if result == result.UNDEFINED: return None
        return Handle(result.value());

    def size(self):
        return self.atomspace.getSize()

    #def get_time_server(self):
        #timeserver = &self.atomspace.getTimeServer()
        #return TimeServer(timeserver)

    def print_list(self):
        self.atomspace.print_list()


# SpaceServer
cdef extern from "opencog/atomspace/SpaceServer.h" namespace "opencog":
    cdef cppclass cSpaceServer "opencog::SpaceServer":
        SpaceServer()

cdef class SpaceServer:
    cdef cSpaceServer *spaceserver

    def __init__(self):
        #self.spaceserver = &spaceserver
        pass

    def __dealloc__(self):
        # Don't do anything because the AtomSpace takes care of cleaning up
        pass


