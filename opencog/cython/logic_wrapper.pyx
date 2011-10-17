from libcpp.vector cimport vector
from libcpp.list cimport list as cpplist
from cython.operator cimport dereference as deref, preincrement as inc

## basic wrapping for std::string conversion
#cdef extern from "<string>" namespace "std":
#    cdef cppclass string:
#        string()
#        string(char *)
#        char * c_str()
#        int size()

# Automatically compile .pyx modules. util.py, tree.py and logic.py all work, but I'm not sure it's actually faster.
#import pyximport; pyximport.install()

from opencog.atomspace cimport cAtomSpace, AtomSpace_factory, cHandle, Handle, Atom

from logic import Chainer
from tree import tree_from_atom

cdef api python_pln_fc(cAtomSpace* c_space) with gil:
    space = AtomSpace_factory(c_space)
    
    c = Chainer(space)
    c.fc()


cdef api cHandle python_pln_bc(cAtomSpace* c_space, cHandle c_target) with gil:
    space = AtomSpace_factory(c_space)
    
    target_Handle = Handle(c_target.value())
    target_Atom = Atom(target_Handle, space)
    
    if target_Handle.value() < 0:
        print 'undefined handle'
        return c_target
    
    target = tree_from_atom(target_Atom)
    c = Chainer(space)
    result_Handle_list = c.bc(target)

#    # Create a C++ vector (optional; not really needed for PLNUTest)
#    cdef vector[cHandle] o_vect
#    for h in outgoing:
#        if isinstance(h,Handle):
#            o_vect.push_back(deref((<Handle>h).h))
#        elif isinstance(h,Atom):
#            o_vect.push_back(deref((<Handle>(h.h)).h))

    if not len(result_Handle_list):
        return cHandle(-1)

    cdef Handle result = result_Handle_list[0]
    
    cdef cHandle c_result = deref(result.h)
    
    return c_result
