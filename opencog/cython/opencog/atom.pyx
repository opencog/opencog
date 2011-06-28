
# Atom wrapper object, we should really do something similar in the
# core OpenCog API.
cdef class Atom(object):

    def __init__(self,Handle h,AtomSpace a):
        self.handle = h
        self.atomspace = a
        # cache the results after first retrieval of
        # immutable properties
        self._atom_type = None
        self._name = None
        self._outgoing = None

    def __nonzero__(self):
        """ Allows boolean comparison, return false is handle is
        UNDEFINED or doesn't exist in AtomSpace """
        if self.handle:
            return self.atomspace.is_valid(self.handle)
        else: return False

    property h:
        def __get__(self): return self.handle
    
    property name:
        def __get__(self):
            if self._name is None:
                self._name = self.atomspace.get_name(self.handle)
            return self._name

    property tv:
        def __get__(self):
            return self.atomspace.get_tv(self.handle)
        def __set__(self,val):
            self.atomspace.set_tv(self.handle,val)

    property av:
        def __get__(self):
            return self.atomspace.get_av(self.handle)
        def __set__(self,val):
            self.atomspace.set_av(self.handle,av_dict=val)

    property out:
        def __get__(self):
            if self._outgoing is None:
                self._outgoing = self.atomspace.get_outgoing(self.handle)
            return self._outgoing

    property arity:
        def __get__(self):
            return len(self.out)

    property incoming:
        def __get__(self):
            return self.atomspace.get_incoming(self.handle)

    property type:
        def __get__(self):
            if self._atom_type is None:
                self._atom_type = self.atomspace.get_type(self.handle)
            return self._atom_type

    property type_name:
        def __get__(self):
            return get_type_name(self.type)

    property t:
        def __get__(self):
            return self.type

    def is_source(self,Atom a):
        return self.atomspace.is_source(a.h,self.handle)

    def is_node(self):
        return is_a(self.t,types.Node)

    def is_link(self):
        return is_a(self.t,types.Link)

    def is_a(self,t):
        return is_a(self.t,t)

    def long_string(self):
        return self.atomspace.get_atom_string(self.handle,terse=False)

    def __str__(self):
        return self.atomspace.get_atom_string(self.handle,terse=True)

    def __repr__(self):
        return "Atom(%s,%s)" % (repr(self.handle),repr(self.atomspace))

    def __richcmp__(Atom a1, Atom a2, int op):
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

    def __hash__(a1):
        return hash(a1.h.value())
