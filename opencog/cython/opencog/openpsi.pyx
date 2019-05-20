
from atomspace cimport *
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

cdef class OpenPsi:

    cdef AtomSpace _as
    def __cinit__(self, AtomSpace _as):
        self._as = _as

    def add_rule(self, context, Atom action, Atom goal, TruthValue stv, Atom category):
        cdef vector[cHandle] handle_vector
        for atom in context:
            if isinstance(atom, Atom):
                handle_vector.push_back(deref((<Atom>(atom)).handle))

        openPsi = new cOpenPsi()
        openPsi.c_add_rule(handle_vector,
                         deref(action.handle),
                         deref(goal.handle),
                         deref(stv._tvptr()),
                         deref(category.handle))