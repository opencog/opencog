__author__ = 'Keyvan'

from blending.agents import DummyBlendingAgent
from opencog.atomspace import AtomSpace
from opencog.cogserver import Server


if __name__ == '__main__':
    server = Server()
    server.add_mind_agent(DummyBlendingAgent())
    server.run(AtomSpace())