from libcpp cimport bool
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

from atomspace cimport *

# @todo use the guide here to separate out into a hierarchy
# http://wiki.cython.org/PackageHierarchy
    
cdef api string get_path_as_string() with gil:
    import sys
    cdef bytes c_str = str(sys.path)
    return string(c_str)

cdef AtomSpace_factory(cAtomSpace *to_wrap):
    cdef AtomSpace instance = AtomSpace.__new__(AtomSpace)
    instance.atomspace = to_wrap
    instance.owns_atomspace = False
    return instance

cdef class AtomSpace:
    # these are defined in atomspace.pxd:
    #cdef cAtomSpace *atomspace
    #cdef cTimeServer *timeserver
    #cdef bint owns_atomspace

    # TODO how do we do a copy constructor that shares the AtomSpaceImpl?
    def __cinit__(self):
        self.owns_atomspace = False

    # A tacky hack to pass in a pointer to an atomspace from C++-land.
    # basically, pass an int, and cast it to the C++ pointer.  This
    # works, but is not very safe, and has a certain feeling of "ick"
    # about it.  But I can't find any better way.
    def __init__(self, long addr = 0):
        if (addr == 0) :
            self.atomspace = new cAtomSpace()
            self.owns_atomspace = True
        else :
            self.atomspace = <cAtomSpace*> PyLong_AsVoidPtr(addr)
            self.owns_atomspace = False

    def __dealloc__(self):
        if self.owns_atomspace:
            del self.atomspace

    def __richcmp__(as_1, as_2, int op):
        if not isinstance(as_1, AtomSpace) or not isinstance(as_2, AtomSpace):
            return NotImplemented
        cdef AtomSpace atomspace_1 = <AtomSpace>as_1
        cdef AtomSpace atomspace_2 = <AtomSpace>as_1

        cdef cAtomSpace* c_atomspace_1 = atomspace_1.atomspace
        cdef cAtomSpace* c_atomspace_2 = atomspace_2.atomspace
        
        is_equal = True
        if c_atomspace_1 != c_atomspace_2:
            is_equal = False
        if op == 2: # ==
            return is_equal
        elif op == 3: # !=
            return not is_equal

    def is_valid(self,h):
        """ Check whether the passed handle refers to an actual handle
        """
        try:
            assert isinstance(h,Handle)
        except AssertionError:
            # Try to convert to a Handle object
            try:
                uuid = int(h)
                h = Handle(uuid)
            except ValueError, TypeError:
                raise TypeError("Need UUID or Handle object")
        if self.atomspace.isValidHandle(deref((<Handle>h).h)):
            return True
        return False

    def clear(self):
        """ Remove all atoms from the AtomSpace """
        self.atomspace.clear()

    # Methods to make the atomspace act more like a standard Python container
    def __contains__(self,o):
        """ Custom checker to see if object is in AtomSpace """
        if isinstance(o, Handle):
            return self.is_valid(o)
        elif isinstance(o, Atom):
            return self.is_valid((<Atom>o).handle)

    def __len__(self):
        """ Return the number of atoms in the AtomSpace """
        return self.size()

    def __getitem__(self, key):
        """ Support self[handle] lookup to get an atom """
        if not isinstance(key, Handle):
            raise KeyError("Lookup only supported by opencog.atomspace.Handle")
        if key in self:
            return Atom(key,self)
        raise IndexError("No Atom with handle %s" % str(key))

    def size(self):
        """ Return the number of atoms in the AtomSpace """
        return self.atomspace.getSize()

    def get_type(self, Handle h):
        """ Get the Type of an Atom in the AtomSpace """
        return self.atomspace.getType(deref(h.h))


cdef api object py_atomspace(cAtomSpace *c_atomspace) with gil:
    cdef AtomSpace atomspace = AtomSpace_factory(c_atomspace)
    return atomspace

cdef api object py_atom(UUID uuid, object atomspace):
    cdef Handle temphandle = Handle(uuid)
    cdef Atom atom = Atom(temphandle, atomspace)
    return atom
