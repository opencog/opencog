# it'd be nice to define the type of atomspace
from opencog.atomspace cimport cAtomSpace, AtomSpace_factory

cdef class MindAgent:
    
    # TODO add a pointer to the Agent C++ object so that calls to stimulate atoms
    # can be made (among other things)

    def get_frequency(self):
        """ Return 0 if the agent runs continuously, otherwise return
        the number of cognitive cycles between each call to "run"
        """
        return 1

# These methods are not available until we have support for MindAgents running
# continuously in their own threads
#    def start(self, AtomSpace atomspace):
#        pass

#    def end(self):
#        pass
    cdef api __run_wrap(self,cAtomSpace *c_atomspace):
        a = AtomSpace_factory(c_atomspace)
        self.run(a)

    def run(self, atomspace):
        print "Implement me in your MindAgent subclass: " + atomspace


