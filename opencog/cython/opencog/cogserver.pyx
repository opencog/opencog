include "agent.pyx"
include "request.pyx"

from opencog.cogserver cimport cAtomSpace, server_atomspace
from opencog.cogserver import AtomSpace
# from opencog.atomspace import AtomSpace

cdef AtomSpace get_server_atomspace():
    casp = server_atomspace()
    asp = AtomSpace()
    asp.atomspace = casp;
    asp.owns_atomspace = False
    return asp
