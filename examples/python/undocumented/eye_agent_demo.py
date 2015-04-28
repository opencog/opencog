from learning.bayesian_learning.agents import TheEyeAgent
from opencog.atomspace import AtomSpace
from opencog.cogserver import Server

__author__ = 'Keyvan'


if __name__ == '__main__':
    server = Server()
    server.add_mind_agent(TheEyeAgent())
    server.run(AtomSpace())