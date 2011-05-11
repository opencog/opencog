cdef class MindAgent:
    
    # TODO add a pointer to the Agent C++ object so that calls to stimulate atoms
    # can be made (among other things)

    def get_frequency(self):
        """ Return 0 if the agent runs continuously, otherwise return
        the number of cognitive cycles between each call to "run"
        """
        return 1

    def start(self, AtomSpace atomspace):
        pass

    def end(self):
        pass

    def run(self, AtomSpace atomspace):
        pass
