include "agent.pyx"
include "request.pyx"

# from opencog.cogserver cimport cAtomSpace, server_atomspace
from opencog.cogserver cimport server_atomspace

# For the below to work, we need to put the atomspace.pxd file where
# cython can find it (viz. /usr/local/include/opencog/cython)
# from opencog.atomspace cimport cAtomSpace
from opencog.atomspace import AtomSpace

def get_server_atomspace():
    # We have to pass the atomspace address as long integer,
    # as otherwise, I cannot figure out how to stop cython
    # from complaining that it
    # "Cannot convert 'cAtomSpace *' to Python object"
    iaddr = server_atomspace()
    asp = AtomSpace(iaddr)
    return asp
