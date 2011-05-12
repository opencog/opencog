# it'd be nice to define the type of atomspace but I can't get this working
#from opencog.atomspace cimport AtomSpace

cdef class MindAgent:
    # TODO add a pointer to the Agent C++ object so that calls to stimulate atoms
    # can be made (among other things)

    def __cinit__(self):
        self._frequency = 1

    property frequency:
        def __get__(self):
            return self._frequency
        def __set__(self,f):
            self._frequency = f

# These methods are not available until we have support for MindAgents running
# continuously in their own threads
#    def start(self, AtomSpace atomspace):
#        pass

#    def end(self):
#        pass

    def run(self, atomspace):
        print "Implement me in your MindAgent subclass: " + atomspace


