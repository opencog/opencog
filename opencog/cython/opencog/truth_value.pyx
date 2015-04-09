from atomspace cimport cTruthValue, tv_ptr

cdef class TruthValue:
    """ The truth value represents the strength and confidence of
        a relationship or term. In OpenCog there are a number of TruthValue
        types, but as these involve additional complexity we focus primarily on
        the SimpleTruthValue type which allows strength and count

        @todo Support IndefiniteTruthValue, DistributionalTV, NullTV etc
    """
    # This stores a pointer to a smart pointer to the C++ TruthValue object
    # Declared in atomspace.pxd
    # cdef tv_ptr *cobj

    def __cinit__(self, strength=0.0, count=0.0):
        # By default create a SimpleTruthValue
        self.cobj = new tv_ptr(new cSimpleTruthValue(strength, count))

    def __dealloc__(self):
        # This deletes the *smart pointer*, not the actual pointer
        del self.cobj

    property mean:
        def __get__(self): return self._mean()

    property confidence:
        def __get__(self): return self._confidence()

    property count:
        def __get__(self): return self._count()

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
        
        raise ValueError, "TruthValue does not yet support most comparison operators"

    cdef cTruthValue* _ptr(self):
        return self.cobj.get()

    cdef tv_ptr* _tvptr(self):
        return self.cobj

    def truth_value_ptr_object(self):
        return PyLong_FromVoidPtr(<void*>self.cobj)

    def __str__(self):
        return self._ptr().toString().c_str()

    def confidence_to_count(self, float conf):
        return (<cSimpleTruthValue*>self._ptr()).confidenceToCount(conf)

    def count_to_confidence(self, float count):
        return (<cSimpleTruthValue*>self._ptr()).countToConfidence(count)
