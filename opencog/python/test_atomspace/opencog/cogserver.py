__author__ = 'Keyvan'


class MindAgent(object):
    def run(self, atomspace):
        pass


class Request(object):
    def __init__(self):
        pass

    def run(self, args=[], atomspace=None):
        self.send("This is the default python request.")

    def send(self, msg):
        print str(msg)


class MindAgent:
    # TODO add a pointer to the Agent C++ object so that calls to stimulate atoms
    # can be made (among other things)

    def __init__(self):
        self.frequency = 0
        pass

# These methods are not available until we have support for MindAgents running
# continuously in their own threads
#    def start(self, AtomSpace atomspace):
#        pass

#    def end(self):
#        pass

    def run(self, atomspace):
        print "Implement me in your MindAgent subclass: " + atomspace


class Server(object):
    def __init__(self):
        self.mind_agents = set()

    def add_mind_agent(self, mind_agent):
        self.mind_agents.add(mind_agent)

    def run(self, atomspace):
        while(True):
            for mind_agent in self.mind_agents:
                mind_agent.run(atomspace)