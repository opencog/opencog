include "agent.pyx"
include "request.pyx"

from opencog.cogserver cimport cAtomSpace, server_atomspace

# For the below to work, we need to put the atomspace.pxd file where
# cython can find it (viz. /usr/local/include/opencog/cython)
from atomspace cimport AtomSpace

def get_server_atomspace():
    casp = server_atomspace()
    asp = AtomSpace()
    asp.atomspace = casp;
    asp.owns_atomspace = False
    return asp
