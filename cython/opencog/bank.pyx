from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

from atomspace cimport *

# @todo use the guide here to separate out into a hierarchy
# http://wiki.cython.org/PackageHierarchy


cdef class AttentionBank:


    def __init__(self, aspace):
        attentionbank(aspace)

    def __dealloc__(self):
        attentionbank(<cAtomSpace*> PyLong_AsVoidPtr(0))

    def get_atoms_by_av(self, lower_bound, upper_bound=None):
        cdef vector[cHandle] handle_vector
        if upper_bound is not None:
            attentionbank().get_handles_by_AV(back_inserter(handle_vector),
                    lower_bound, upper_bound)
        else:
            attentionbank().get_handles_by_AV(back_inserter(handle_vector),
                    lower_bound)
        return convert_handle_seq_to_python_list(handle_vector, self)

    def get_atoms_in_attentional_focus(self):
        cdef vector[cHandle] handle_vector
        attentionbank().get_handle_set_in_attentional_focus(back_inserter(handle_vector))
        return convert_handle_seq_to_python_list(handle_vector, self)
