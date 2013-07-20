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


class Server(object):
    def __init__(self):
        self.mind_agents = set()

    def add_mind_agent(self, mind_agent):
        self.mind_agents.add(mind_agent)

    def run(self, atomspace):
        while(True):
            for mind_agent in self.mind_agents:
                mind_agent.run(atomspace)