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
        bint operator!=(cHandle h)
        bint operator<(cHandle h)
        bint operator>(cHandle h)
        bint operator<=(cHandle h)
        bint operator>=(cHandle h)
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
            return deref(h1.h) == deref(h2.h)
        elif op == 3: # !=
            return deref(h1.h) != deref(h2.h)
        elif op == 4: # >
            return deref(h1.h) > deref(h2.h)
        elif op == 0: # <
            return deref(h1.h) < deref(h2.h)
        elif op == 1: # <=
            return deref(h1.h) <= deref(h2.h)
        elif op == 5: # >=
            return deref(h1.h) >= deref(h2.h)
    def __str__(self):
        return "<UUID:" + str(self.h.value()) + ">"
    def is_undefined(self):
        if deref(self.h) == self.h.UNDEFINED: return True
        return False
    def is_valid(self):
        return self.atomspace.is_valid(self)

# TruthValue
ctypedef int count_t
ctypedef float confidence_t
ctypedef float strength_t

cdef extern from "opencog/atomspace/TruthValue.h" namespace "opencog":
    cdef cppclass cTruthValue "opencog::TruthValue":
        strength_t getMean()
        confidence_t getConfidence()
        count_t getCount()
        cTruthValue DEFAULT_TV()

cdef extern from "opencog/atomspace/SimpleTruthValue.h" namespace "opencog":
    cdef cppclass cSimpleTruthValue "opencog::SimpleTruthValue":
        cSimpleTruthValue(float, float)
        strength_t getMean()
        confidence_t getConfidence()
        count_t getCount()
        cTruthValue DEFAULT_TV()

cdef class TruthValue:
    cdef cTruthValue *cobj
    def __cinit__(self, float strength, float count):
        self.cobj = <cTruthValue*> new cSimpleTruthValue(strength,count)
    def __dealloc__(self):
        del self.cobj
    def mean(self):
        return deref(self.cobj).getMean()
    def confidence(self):
        return deref(self.cobj).getConfidence()
    def count(self):
        return deref(self.cobj).getCount()

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

        bint isValidHandle(cHandle h)
        int getSize()
        string getName(cHandle h)

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

    def add_node(self, Type t, atom_name, TruthValue tv=None, prefixed=False):
        """ Add Node to AtomSpace
        @todo support [0.5,0.5] format for TruthValue.
        @todo support type name for type.
        @returns handle referencing the newly created Atom
        """
        # convert to string
        py_byte_string = atom_name.encode('UTF-8')
        # create temporary cpp string
        cdef string *name = new string(py_byte_string)
        cdef cHandle result
        if prefixed:
            # prefixed nodes ALWAYS generate a new atom using atom_name
            # as the prefix
            if tv is None:
                # get handle
                result = self.atomspace.addPrefixedNode(t,deref(name))
            else:
                result = self.atomspace.addPrefixedNode(t,deref(name),deref(tv.cobj))
        else:
            if tv is None:
                # get handle
                result = self.atomspace.addNode(t,deref(name))
            else:
                result = self.atomspace.addNode(t,deref(name),deref(tv.cobj))
        # delete temporary string
        del name
        if result == result.UNDEFINED: return None
        return Handle(result.value());

    def add_link(self,Type t,outgoing,TruthValue tv=None):
        """ Add Link to AtomSpace
        @todo support [0.5,0.5] format for TruthValue.
        @todo support type name for type.
        @returns handle referencing the newly created Atom
        """
        # create temporary cpp vector
        cdef vector[cHandle] o_vect
        for h in outgoing:
            o_vect.push_back(deref((<Handle>h).h))
        cdef cHandle result
        if tv is None:
            # get handle
            result = self.atomspace.addLink(t,o_vect)
        else:
            result = self.atomspace.addLink(t,o_vect,deref(tv.cobj))
        if result == result.UNDEFINED: return None
        return Handle(result.value());

    def is_valid(self,h):
        """ Check whether the passed handle refers to an actual handle
        """
        try:
            assert isinstance(h,Handle)
        except AssertionError:
            # Try to convert to a Handle object
            try:
                uuid = int(h)
                h = Handle(uuid)
            except ValueError, TypeError:
                raise TypeError("Need UUID or Handle object")
        if self.atomspace.isValidHandle(deref((<Handle>h).h)):
            return True
        return False

    def size(self):
        return self.atomspace.getSize()

    #def get_time_server(self):
        #timeserver = &self.atomspace.getTimeServer()
        #return TimeServer(timeserver)

    def print_list(self):
        self.atomspace.print_list()

# Atom wrapper object, we should really do something similar in the
# core OpenCog API.
cdef class Atom:
    cdef Handle handle
    cdef AtomSpace atomspace
    def __init__(self,Handle h,AtomSpace a):
        self.handle = h
        self.atomspace = a
    def get_name(self):
        cdef string name
        name = self.atomspace.atomspace.getName(deref(self.handle.h))
        return name.c_str()

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


