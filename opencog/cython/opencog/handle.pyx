from cython.operator cimport dereference as deref

from atomspace cimport *


cdef class Handle:
    def __cinit__(self, h):
        self.h = new cHandle(h)
    def __dealloc__(self):
        del self.h
    def value(self):
        return self.h.value()
    def __richcmp__(Handle h1, Handle h2, int op):
        if op == 2: # ==
            return deref(h1.h) == deref(h2.h)
        elif op == 3: # !=
            return deref(h1.h) != deref(h2.h)
        elif op == 4: # >
            return deref(h1.h) > deref(h2.h)
        elif op == 0: # <
            return deref(h1.h) < deref(h2.h)
        elif op == 1: # <=
            return deref(h1.h) <= deref(h2.h)
        elif op == 5: # >=
            return deref(h1.h) >= deref(h2.h)
    def __str__(self):
        return "<UUID:" + str(self.h.value()) + ">"
    def __repr__(self):
        return "Handle(" + str(self.h.value()) + ")"
    def is_undefined(self):
        if deref(self.h) == self.h.UNDEFINED: return True
        return False
    def __nonzero__(self):
        """ Allows boolean comparison, return false is handle == UNDEFINED """
        return not self.is_undefined()
