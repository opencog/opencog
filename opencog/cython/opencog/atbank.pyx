from cython.operator cimport dereference as deref
from atomspace cimport AtomSpace, Atom

cdef class AttentionBank:

    cdef AtomSpace _as

    def __cinit__(self, AtomSpace _as):
        self._as = _as
        attentionbank(_as.atomspace)

    def set_sti(self, Atom atom, sti):
        attentionbank(self._as.atomspace).set_sti(deref(atom.handle), sti)

    def set_lti(self, Atom atom, lti):
        attentionbank(self._as.atomspace).set_lti(deref(atom.handle), lti)

    def inc_vlti(self, Atom atom):
        attentionbank(self._as.atomspace).inc_vlti(deref(atom.handle))

    def dec_vlti(self, Atom atom):
        attentionbank(self._as.atomspace).dec_vlti(deref(atom.handle))

    def set_av(self, Atom atom, sti, lti):
        attentionbank(self._as.atomspace).set_sti(deref(atom.handle), sti)
        attentionbank(self._as.atomspace).set_lti(deref(atom.handle), lti)

def af_bindlink(AtomSpace atomspace, Atom atom):
    if atom == None: raise ValueError("af_bindlink atom is: None")

    cdef cHandle c_result = c_af_bindlink(atomspace.atomspace, deref(atom.handle))
    return Atom.createAtom(c_result, atomspace)
