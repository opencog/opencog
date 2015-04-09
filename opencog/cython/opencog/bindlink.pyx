from opencog.atomspace cimport Atom, AtomSpace, Handle, TruthValue
from opencog.atomspace cimport cHandle, cAtomSpace, cTruthValue
from opencog.atomspace cimport tv_ptr, strength_t, count_t
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

def satisfaction_link(AtomSpace atomspace, Handle handle):
    cdef tv_ptr result_tv_ptr = c_satisfaction_link(atomspace.atomspace,
                                                 deref(handle.h))
    cdef cTruthValue* result_tv = result_tv_ptr.get()
    cdef strength_t strength = deref(result_tv).getMean()
    cdef count_t count = deref(result_tv).getCount()
    return TruthValue(strength, count)

def execute_atom(AtomSpace atomspace, Atom atom):
    cdef Handle atom_h = atom.h
    cdef cHandle result_c_handle = c_execute_atom(atomspace.atomspace,
                                                  deref(atom_h.h))
    cdef result_handle = Handle(result_c_handle.value())
    return Atom(result_handle, atomspace)

def evaluate_atom(AtomSpace atomspace, Atom atom):
    cdef Handle atom_h = atom.h
    cdef tv_ptr result_tv_ptr = c_evaluate_atom(atomspace.atomspace,
                                                deref(atom_h.h))
    cdef cTruthValue* result_tv = result_tv_ptr.get()
    cdef strength_t strength = deref(result_tv).getMean()
    cdef count_t count = deref(result_tv).getCount()
    return TruthValue(strength, count)
