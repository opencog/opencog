from opencog.atomspace cimport AtomSpace, Handle
from opencog.atomspace cimport cHandle, cAtomSpace
from cython.operator cimport dereference as deref


def stub_bindlink(AtomSpace atomspace, Handle handle):
    cdef cHandle c_result = c_stub_bindlink(atomspace.atomspace,
                                            deref(handle.h))
    cdef Handle result = Handle(c_result.value())
    return result

def bindlink(AtomSpace atomspace, Handle handle):
    cdef cHandle c_result = c_bindlink(atomspace.atomspace,
                                       deref(handle.h))
    cdef Handle result = Handle(c_result.value())
    return result

def single_bindlink(AtomSpace atomspace, Handle handle):
    cdef cHandle c_result = c_single_bindlink(atomspace.atomspace,
                                              deref(handle.h))
    cdef Handle result = Handle(c_result.value())
    return result

def crisp_logic_bindlink(AtomSpace atomspace, Handle handle):
    cdef cHandle c_result = c_crisp_logic_bindlink(atomspace.atomspace,
                                                   deref(handle.h))
    cdef Handle result = Handle(c_result.value())
    return result

def pln_bindlink(AtomSpace atomspace, Handle handle):
    cdef cHandle c_result = c_pln_bindlink(atomspace.atomspace, deref(handle.h))
    cdef Handle result = Handle(c_result.value())
    return result

def validate_bindlink(AtomSpace atomspace, Handle handle):
    cdef cHandle c_result = c_validate_bindlink(atomspace.atomspace,
                                                deref(handle.h))
    cdef Handle result = Handle(c_result.value())
    return result
