from opencog.atomspace cimport cHandle, cTruthValue, cAtomSpace

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
    #   TruthValuePtr satisfaction_link(AtomSpace*, Handle);
    #
    cdef cHandle c_bindlink "bindlink" (cAtomSpace*, cHandle)
    cdef cHandle c_single_bindlink "single_bindlink" (cAtomSpace*, cHandle)
    cdef cHandle c_crisp_logic_bindlink "crisp_logic_bindlink" (cAtomSpace*,
                                                                cHandle)
    cdef cHandle c_pln_bindlink "pln_bindlink" (cAtomSpace*, cHandle)
    cdef cTruthValue c_satisfaction_link "satisfaction_link" (cAtomSpace*, cHandle)
