
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

        cdef cOpenPsi openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_add_rule(handle_vector,
                                                 deref(action.handle),
                                                 deref(goal.handle),
                                                 deref(stv._tvptr()),
                                                 deref(category.handle))

        return OpenPsiRule(self._as, Atom.createAtom(handle, self._as))

    def is_rule(self, Atom atom):
        return get_openpsi_scm().c_is_rule(deref(atom.handle))

    def get_categories(self):
        cdef cOpenPsi openPsi = get_openpsi_scm()
        cdef vector[cHandle] res_handles = openPsi.c_get_categories()
        list = []
        cdef vector[cHandle].iterator it = res_handles.begin()
        while it != res_handles.end():
            handle = deref(it)
            list.append(Atom.createAtom(handle, self._as))
            inc(it)
        return list

    def add_category(self, Atom new_category):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_add_category(deref(new_category.handle))
        return Atom.createAtom(handle, self._as)


cdef class OpenPsiRule:

    cdef AtomSpace _as
    cdef Atom rule

    def __cinit__(self, AtomSpace _as, Atom rule):
        self._as = _as
        self.rule = rule

    def get_rule_atom(self):
        return self.rule

    def get_context(self):
        openPsi = get_openpsi_scm()

        cdef vector[cHandle] res_handles = openPsi.c_get_context(deref(self.rule.handle))
        list = []
        cdef vector[cHandle].iterator it = res_handles.begin()
        while it != res_handles.end():
            handle = deref(it)
            list.append(Atom.createAtom(handle, self._as))
            inc(it)
        return list

    def get_goal(self):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_get_goal(deref(self.rule.handle))
        return Atom.createAtom(handle, self._as)

    def add_to_category(self, Atom category):
        openPsi = get_openpsi_scm()
        cdef cHandle handle = openPsi.c_add_to_category(deref(self.rule.handle),
                                                        deref(category.handle))
        return Atom.createAtom(handle, self._as)
