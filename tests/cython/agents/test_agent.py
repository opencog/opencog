import opencog.cogserver
from opencog.atomspace import types

class TestAgent(opencog.cogserver.MindAgent):

    def __init__(self):
        self.counter = 1

    def run(self, atomspace):
        atomspace.add_node(types.ConceptNode, "testnode" + str(self.counter))
        print("I am running and the atomspace contains: ")
        print(atomspace.get_atoms_by_type(types.Node))
        self.counter += 1
