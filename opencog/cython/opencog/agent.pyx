# it'd be nice to define the type of atomspace but I can't get this working
from opencog.cogserver cimport cAgent, stim_t, string

cdef class MindAgent:
    # TODO add a pointer to the Agent C++ object so that calls to stimulate atoms
    # can be made (among other things)

    def __cinit__(self):
        pass

    property frequency:
        def __get__(self):
            return self.c_obj.frequency()
        def __set__(self,int f):
            self.c_obj.setFrequency(f)

# These methods are not available until we have support for MindAgents running
# continuously in their own threads
#    def start(self, AtomSpace atomspace):
#        pass

#    def end(self):
#        pass

    def run(self, atomspace):
        print "Implement me in your MindAgent subclass: " + atomspace


