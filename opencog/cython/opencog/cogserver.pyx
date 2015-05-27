include "agent.pyx"
include "request.pyx"

# from opencog.cogserver cimport cAtomSpace, server_atomspace
from opencog.cogserver cimport server_atomspace

# For the below to work, we need to put the atomspace.pxd file where
# cython can find it (viz. /usr/local/include/opencog/cython)
from opencog.atomspace cimport AtomSpace_factory

def get_server_atomspace():
    casp = server_atomspace()
    asp = AtomSpace_factory(casp)
    return asp
