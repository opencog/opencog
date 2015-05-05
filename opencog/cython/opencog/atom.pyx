
# Atom wrapper object, we should really do something similar in the
# core OpenCog API.
cdef class Atom(object):

    def __init__(self,Handle h,AtomSpace a):
        self.handle = h
        # cache the results after first retrieval of
        # immutable properties
        self._atom_type = None
        self._name = None
        self._outgoing = None
        # Not really a cache ... an atom could be moved from one
        # atomspace to another (!)
        self.atomspace = a

    def __nonzero__(self):
        """ Allows boolean comparison, return false is handle is
        UNDEFINED or doesn't exist in AtomSpace """
        if self.handle:
            return self.atomspace.is_valid(self.handle)
        else: return False

    property atomspace:
        def __get__(self):
            return self.atomspace

    property h:
        def __get__(self): return self.handle
    
    def handle_uuid(self):
        return self.handle.value()

    def __richcmp__(a1_, a2_, int op):
        if not isinstance(a1_, Atom) or not isinstance(a2_, Atom):
            return NotImplemented
        cdef Atom a1 = a1_
        cdef Atom a2 = a2_
        
        is_equal = True
        if a1.atomspace != a2.atomspace:
            is_equal = False
        if is_equal:
            if a1.handle != a2.handle:
                is_equal = False
        if op == 2: # ==
            return is_equal
        elif op == 3: # !=
            return not is_equal
        #elif op == 4: # >
            #return deref(h1.h) > deref(h2.h)
        #elif op == 0: # <
            #return deref(h1.h) < deref(h2.h)
        #elif op == 1: # <=
            #return deref(h1.h) <= deref(h2.h)
        #elif op == 5: # >=
            #return deref(h1.h) >= deref(h2.h)

    # Necessary to prevent weirdness with RPyC
    def __cmp__(a1, a2):
        is_equal = (a1.atomspace == a2.atomspace and
                     a1.handle == a2.handle)
        if is_equal:
            return 0
        else:
            return -1

    def __hash__(a1):
        return hash(a1.h.value())
