from opencog.atomspace cimport cHandle, cAtomSpace

cdef extern from "opencog/cython/opencog/BindlinkStub.h" namespace "opencog":
    # C++: 
    #   Handle stub_bindlink(AtomSpace*, Handle);
    #
    cdef cHandle c_stub_bindlink "stub_bindlink" (cAtomSpace*, cHandle)


cdef extern from "opencog/query/BindLink.h" namespace "opencog":
    # C++: 
    #   Handle bindlink(AtomSpace*, Handle);
    #   Handle single_bindlink (AtomSpace*, Handle);
    #   Handle crisp_logic_bindlink(AtomSpace*, Handle);
    #   Handle pln_bindlink(AtomSpace*, Handle);
    #   Handle validate_bindlink(AtomSpace*, Handle);
    #
    cdef cHandle c_bindlink "bindlink" (cAtomSpace*, cHandle)
    cdef cHandle c_single_bindlink "single_bindlink" (cAtomSpace*, cHandle)
    cdef cHandle c_crisp_logic_bindlink "crisp_logic_bindlink" (cAtomSpace*,
                                                                cHandle)
    cdef cHandle c_pln_bindlink "pln_bindlink" (cAtomSpace*, cHandle)
    cdef cHandle c_validate_bindlink "validate_bindlink" (cAtomSpace*, cHandle)
