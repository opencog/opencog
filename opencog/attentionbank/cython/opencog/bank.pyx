from libcpp.vector cimport vector

from cython.operator cimport dereference as deref, preincrement as inc
from atomspace cimport AtomSpace, Atom


cdef vector_to_set(vector[cHandle] handle_vector):
    cdef cHandle c_handle
    cdef vector[cHandle].iterator it = handle_vector.begin()
    atoms = set()
    while it != handle_vector.end():
        c_handle = deref(it)
        atoms.add(Atom.createAtom(c_handle))
        inc(it)
    return atoms


cdef class AttentionBank:

    cdef AtomSpace _as

    def __cinit__(self, AtomSpace _as):
        self._as = _as
        attentionbank(_as.atomspace)

    def __dealloc__(self):
        attentionbank(NULL)

    def get_sti(self, Atom atom):
        return get_sti(deref(atom.handle))

    def set_sti(self, Atom atom, sti):
        attentionbank(self._as.atomspace).set_sti(deref(atom.handle), sti)

    def get_lti(self, Atom atom):
        return get_lti(deref(atom.handle))

    def set_lti(self, Atom atom, lti):
        attentionbank(self._as.atomspace).set_lti(deref(atom.handle), lti)

    def get_vlti(self, Atom atom):
        return get_vlti(deref(atom.handle))

    def inc_vlti(self, Atom atom):
        attentionbank(self._as.atomspace).inc_vlti(deref(atom.handle))

    def dec_vlti(self, Atom atom):
        attentionbank(self._as.atomspace).dec_vlti(deref(atom.handle))

    def set_av(self, Atom atom, sti, lti):
        attentionbank(self._as.atomspace).set_sti(deref(atom.handle), sti)
        attentionbank(self._as.atomspace).set_lti(deref(atom.handle), lti)

    def get_atoms_by_av(self, sti_lower_bound, sti_upper_bound = None):

        cdef av_type lower_bound = sti_lower_bound
        cdef av_type upper_bound = -1
        cdef vector[cHandle] handle_vector

        if sti_upper_bound is None:
            attentionbank(self._as.atomspace).get_handles_by_AV(back_inserter(handle_vector), lower_bound)
        else:
            upper_bound = sti_upper_bound
            attentionbank(self._as.atomspace).get_handles_by_AV(back_inserter(handle_vector), lower_bound, upper_bound)

        return vector_to_set(handle_vector)

    def get_atoms_in_attentional_focus(self):
        cdef vector[cHandle] handle_vector
        attentionbank(self._as.atomspace).get_handle_set_in_attentional_focus(back_inserter(handle_vector))
        return vector_to_set(handle_vector)


def af_bindlink(AtomSpace atomspace, Atom atom):
    if atom is None: raise ValueError("af_bindlink atom is: None")

    cdef cHandle c_result = c_af_bindlink(atomspace.atomspace, deref(atom.handle))
    return Atom.createAtom(c_result)

