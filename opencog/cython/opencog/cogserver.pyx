include "agent.pyx"
include "request.pyx"

from opencog.cogserver cimport cAtomSpace, server_atomspace
from opencog.cogserver import AtomSpace

# For the below to work, we need to put the atomspace.pxd file where
# cython can find it.
# from opencog.atomspace import AtomSpace

def get_server_atomspace():
    casp = server_atomspace()
    asp = AtomSpace()
    asp.atomspace = casp;
    asp.owns_atomspace = False
    return asp
