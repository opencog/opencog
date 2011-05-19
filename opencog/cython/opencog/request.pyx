# it'd be nice to define the type of atomspace but I can't get this working
from opencog.cogserver cimport cRequest, string

cdef class Request:
    def __cinit__(self):
        pass

    def run(self, args=[], atomspace=None):
        self.send("This is the default python request.")

    def send(self, msg):
        self.c_obj.send(string(msg))

