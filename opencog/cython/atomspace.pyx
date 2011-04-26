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

### TruthValue
ctypedef int count_t
ctypedef float confidence_t
ctypedef float strength_t

cdef extern from "opencog/atomspace/TruthValue.h" namespace "opencog":
    cdef cppclass cTruthValue "opencog::TruthValue":
        strength_t getMean()
        confidence_t getConfidence()
        count_t getCount()
        cTruthValue DEFAULT_TV()
        string toString()
        bint operator==(cTruthValue h)

cdef extern from "opencog/atomspace/SimpleTruthValue.h" namespace "opencog":
    cdef cppclass cSimpleTruthValue "opencog::SimpleTruthValue":
        cSimpleTruthValue(float, float)
        strength_t getMean()
        confidence_t getConfidence()
        count_t getCount()
        cTruthValue DEFAULT_TV()
        string toString()
        bint operator==(cTruthValue h)

cdef extern from "boost/shared_ptr.hpp":

    cdef cppclass tv_ptr "boost::shared_ptr<opencog::TruthValue>":
        tv_ptr()
        tv_ptr(cTruthValue* fun)
        tv_ptr(cSimpleTruthValue* fun)
        cTruthValue* get()

cdef class TruthValue:
    """ The truth value represents the strength and confidence of
        a relationship or term. In OpenCog there are a number of TruthValue
        types, but as these involve additional complexity we focus primarily on
        the SimpleTruthValue type which allows strength and count

        @todo Support IndefiniteTruthValue, DistributionalTV, NullTV etc
    """
    # This stores a pointer to a smart pointer to the C++ TruthValue object
    # This indirection is unfortunately necessary because cython doesn't
    # allow C++ objects on the stack
    cdef tv_ptr *cobj

    def __cinit__(self, strength=0.0, count=0.0):
        # By default create a SimpleTruthValue
        self.cobj = new tv_ptr(new cSimpleTruthValue(strength,count))

    def __dealloc__(self):
        # This deletes the *smart pointer*, not the actual pointer
        del self.cobj

    def __getattr__(self,aname):
        if aname == "mean":
            return self._mean()
        elif aname == "confidence":
            return self._confidence()
        elif aname == "count":
            return self._count()

    cdef _mean(self):
        return self._ptr().getMean()

    cdef _confidence(self):
        return self._ptr().getConfidence()

    cdef _count(self):
        return self._ptr().getCount()

    def __richcmp__(TruthValue h1, TruthValue h2, int op):
        " @todo support the rest of the comparison operators"
        if op == 2: # ==
            return deref(h1._ptr()) == deref(h2._ptr())

    cdef cTruthValue* _ptr(self):
        return self.cobj.get()

    def __str__(self):
        return self._ptr().toString().c_str()


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
        tv_ptr getTV(cHandle h)
        void setTV(cHandle h, cTruthValue tvn)

        # these should alias the proper types for sti/lti/vlti
        short getSTI(cHandle h)
        short getLTI(cHandle h)
        bint getVLTI(cHandle h)
        void setSTI(cHandle h, short)
        void setLTI(cHandle h, short)
        void setVLTI(cHandle h, bint)

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
                result = self.atomspace.addPrefixedNode(t,deref(name),deref(<cTruthValue*>(tv._ptr())))
        else:
            if tv is None:
                # get handle
                result = self.atomspace.addNode(t,deref(name))
            else:
                result = self.atomspace.addNode(t,deref(name),deref(tv._ptr()))
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
            result = self.atomspace.addLink(t,o_vect,deref(tv._ptr()))
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
    def __getattr__(self,aname):
        if aname == "name":
            return self.__get_name()
        elif aname == "tv":
            return self.__get_tv()
        elif aname == "av":
            return self.__get_av()
    def __setattr__(self,aname,val):
        if aname == "name":
            raise ValueError("Atom name is immutable")
        elif aname == "tv":
            self.__set_tv(val)
        elif aname == "av":
            self.set_av(av_dict=val)
    def __get_name(self):
        cdef string name
        name = self.atomspace.atomspace.getName(deref(self.handle.h))
        return name.c_str()
    def __get_tv(self):
        cdef tv_ptr tv
        tv = self.atomspace.atomspace.getTV(deref(self.handle.h))
        return TruthValue(tv.get().getMean(),tv.get().getCount())
    def __set_tv(self,TruthValue val):
        self.atomspace.atomspace.setTV(deref(self.handle.h),deref(val._ptr()))
    def __get_av(self):
        # @todo this is the slow way. quicker way is to support the
        # AttentionValue object and get all values with one atomspace call
        sti = self.atomspace.atomspace.getSTI(deref(self.handle.h))
        lti = self.atomspace.atomspace.getLTI(deref(self.handle.h))
        vlti = self.atomspace.atomspace.getVLTI(deref(self.handle.h))
        return { "sti": sti, "lti": lti, "vlti": vlti }
    def set_av(self,sti=None,lti=None,vlti=None,av_dict=None):
        # @todo this is the slow way. quicker way is to support the
        # AttentionValue object and get all values with one atomspace call
        if av_dict:
            if "sti" in av_dict: sti = av_dict["sti"]
            if "lti" in av_dict: lti = av_dict["lti"]
            if "vlti" in av_dict: vlti = av_dict["vlti"]
        if sti: self.atomspace.atomspace.setSTI(deref(self.handle.h),sti)
        if lti: self.atomspace.atomspace.setLTI(deref(self.handle.h),lti)
        if vlti: self.atomspace.atomspace.setVLTI(deref(self.handle.h),vlti)


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


