__author__ = 'Keyvan'

from blending.agents import DummyBlendingAgent
from opencog.atomspace import AtomSpace
from opencog.cogserver import Server
from load_scm_file import load_scm_file

# Check if git shows the branch
if __name__ == '__main__':
    server = Server()
    server.add_mind_agent(DummyBlendingAgent())
    a = AtomSpace()
    load_scm_file(a, '../examples/bat_man.scm')
    a.print_list()
    server.run(a)